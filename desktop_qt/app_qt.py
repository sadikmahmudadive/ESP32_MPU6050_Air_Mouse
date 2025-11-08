import sys
import os
import json
import socket
import platform as py_platform  # alias to avoid clash with OpenGL.platform after GL imports
import subprocess
from datetime import datetime
from threading import Event
import time
import uuid
from collections import deque

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt, Signal, QTimer, QThread
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QSpinBox, QDoubleSpinBox, QCheckBox, QProgressBar, QTextEdit, QStatusBar, QMessageBox,
    QListWidget, QPlainTextEdit, QGroupBox, QFormLayout, QRadioButton
)
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import (
    glClearColor, glDisable, glViewport, glMatrixMode, glLoadIdentity, glOrtho,
    glClear, glBegin, glEnd, glVertex2f, glColor3f, glEnable,
    GL_COLOR_BUFFER_BIT, GL_DEPTH_TEST, GL_PROJECTION, GL_MODELVIEW,
    GL_LINE_STRIP, GL_LINE_LOOP, GL_QUADS, GL_LINE_SMOOTH
)
from OpenGL.GLU import *  # If specific GLU funcs needed later, consider narrowing this.

try:
    from pynput.mouse import Controller, Button
    PYNPUT_AVAILABLE = True
except Exception:
    PYNPUT_AVAILABLE = False

APP_NAME = "ESP Air Mouse — Qt"
CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'config.json')

# --- Gesture Recorder Implementation ---

class GestureRecorder(QtCore.QObject):
    """Stateful recorder for automatic gesture segmentation.

    Modes:
        tap: user triggers each capture manually.
        auto: motion threshold triggers start and quiet period triggers stop.
        batch: cycles labels automatically collecting target count.
    """
    segmentSaved = Signal(str, str)  # path, label
    segmentRejected = Signal(str)    # reason
    progressUpdated = Signal(dict)   # label->count mapping
    activeLabelChanged = Signal(str)
    statusMessage = Signal(str)

    def __init__(self, base_dir: str, sampling_hz: int = 100, window_len_s: float = 1.0,
                 pre_roll_s: float = 0.25, post_roll_s: float = 0.25,
                 accel_thresh_g: float = 1.2, gyro_thresh_dps: float = 120.0,
                 quiet_ms: int = 200, target_per_label: int = 30, parent=None):
        super().__init__(parent)
        self.base_dir = base_dir
        os.makedirs(self.base_dir, exist_ok=True)
        self.sampling_hz = sampling_hz
        self.window_len_s = window_len_s
        self.pre_roll_s = pre_roll_s
        self.post_roll_s = post_roll_s
        self.accel_thresh_g = accel_thresh_g
        self.gyro_thresh_dps = gyro_thresh_dps
        self.quiet_ms = quiet_ms
        self.target_per_label = target_per_label
        # dynamic state
        self.mode = 'tap'
        self.labels = []
        self.active_label = None
        self.label_counts = {}
        self.ring = deque(maxlen=int((window_len_s + pre_roll_s + post_roll_s) * sampling_hz * 3))
        self.capturing = False
        self.capture_start_index = None
        self.last_motion_ms = None
        self.quiet_start_ms = None
        self.batch_index = 0
        self.segment_seq = 0
        self.auto_enabled = False
        self.batch_enabled = False
        self.streaming_active = False  # external (firmware) streaming

    # --- Session control ---
    def start_session(self, labels: list[str], mode: str = 'tap'):
        if not labels:
            self.statusMessage.emit("No labels provided; session aborted.")
            return
        self.labels = list(dict.fromkeys([l.strip() for l in labels if l.strip()]))  # dedupe preserve order
        self.label_counts = {l: 0 for l in self.labels}
        self.mode = mode
        self.batch_enabled = (mode == 'batch')
        self.auto_enabled = (mode == 'auto') or self.batch_enabled
        self.batch_index = 0
        self.set_active_label(self.labels[0])
        self.segment_seq = 0
        self.statusMessage.emit(f"Session started in {self.mode} mode. Labels: {', '.join(self.labels)}")
        self.progressUpdated.emit(self.label_counts.copy())

    def stop_session(self):
        self.capturing = False
        self.statusMessage.emit("Session stopped.")

    def set_active_label(self, label: str):
        if label not in self.label_counts:
            return
        self.active_label = label
        self.activeLabelChanged.emit(label)

    def next_label(self):
        if not self.labels:
            return
        self.batch_index = (self.batch_index + 1) % len(self.labels)
        self.set_active_label(self.labels[self.batch_index])

    # --- Sample ingestion ---
    def feed_sample(self, ts_ms: int, ax: int, ay: int, az: int, gx: int, gy: int, gz: int):
        # Convert raw to engineering units (approx). Assuming accel raw ~ LSB per mg? Keep raw for now.
        self.ring.append((ts_ms, ax, ay, az, gx, gy, gz))
        if self.auto_enabled:
            self._auto_state_machine()

    # --- Manual trigger (tap) ---
    def manual_trigger(self):
        if self.active_label is None:
            self.statusMessage.emit("Set a label first.")
            return
        self._start_capture()
        # schedule finalize after window length + post roll
        QtCore.QTimer.singleShot(int((self.window_len_s + self.post_roll_s)*1000), self._finalize_capture)

    # --- Auto motion logic ---
    def _auto_state_machine(self):
        if self.active_label is None:
            return
        # Compute accel magnitude & gyro magnitude (simple)
        window = list(self.ring)[-1:]
        if not window:
            return
        ts_ms, ax, ay, az, gx, gy, gz = window[0]
        a_mag = (ax**2 + ay**2 + az**2) ** 0.5
        g_mag = (gx**2 + gy**2 + gz**2) ** 0.5
        now = ts_ms
        motion = (a_mag > self.accel_thresh_g*16384) or (g_mag > self.gyro_thresh_dps*131)  # assuming MPU6050 scale factors
        if motion:
            self.last_motion_ms = now
            self.quiet_start_ms = None
            if not self.capturing:
                self._start_capture()
        else:
            if self.capturing:
                if self.quiet_start_ms is None:
                    self.quiet_start_ms = now
                elif now - self.quiet_start_ms >= self.quiet_ms:
                    self._finalize_capture()

    # --- Capture control ---
    def _start_capture(self):
        if self.capturing:
            return
        self.capturing = True
        self.capture_start_index = len(self.ring) - 1  # index of last sample when started
        self.statusMessage.emit(f"Capture started for {self.active_label}")

    def _finalize_capture(self):
        if not self.capturing:
            return
        self.capturing = False
        # Determine slice indices
        if not self.ring:
            self.segmentRejected.emit("Empty ring buffer")
            return
        end_index = len(self.ring) - 1
        # Pre-roll samples
        pre_samples = int(self.pre_roll_s * self.sampling_hz)
        post_samples = int(self.post_roll_s * self.sampling_hz)
        start_index = max(0, self.capture_start_index - pre_samples)
        slice_samples = list(self.ring)[start_index:end_index + 1 + post_samples]
        # Quality checks
        if len(slice_samples) < int(self.window_len_s * self.sampling_hz * 0.6):
            self.segmentRejected.emit("Too short")
            return
        # Basic variance check on accel magnitude
        a_mags = [((ax**2 + ay**2 + az**2) ** 0.5) for (_, ax, ay, az, gx, gy, gz) in slice_samples]
        if (max(a_mags) - min(a_mags)) < 500:  # raw threshold heuristic
            self.segmentRejected.emit("Low variance")
            return
        # Save segment CSV
        ts_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        seg_id = uuid.uuid4().hex[:8]
        self.segment_seq += 1
        fname = f"{self.active_label}_{ts_str}_{seg_id}_{self.segment_seq}.csv"
        path = os.path.join(self.base_dir, fname)
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write("timestamp_ms,ax,ay,az,gx,gy,gz,label,segment_id,sample_index\n")
                for i,(ts_ms, ax, ay, az, gx, gy, gz) in enumerate(slice_samples):
                    f.write(f"{ts_ms},{ax},{ay},{az},{gx},{gy},{gz},{self.active_label},{seg_id},{i}\n")
        except Exception as e:
            self.segmentRejected.emit(f"File error: {e}")
            return
        # Update counts
        self.label_counts[self.active_label] += 1
        self.segmentSaved.emit(path, self.active_label)
        self.progressUpdated.emit(self.label_counts.copy())
        self.statusMessage.emit(f"Saved segment {path}")
        # Batch advancement
        if self.batch_enabled and self.label_counts[self.active_label] >= self.target_per_label:
            # Advance label or finish
            remaining = [l for l,c in self.label_counts.items() if c < self.target_per_label]
            if remaining:
                # pick next remaining label
                for l in self.labels:
                    if self.label_counts[l] < self.target_per_label:
                        self.set_active_label(l)
                        break
            else:
                self.statusMessage.emit("Batch collection complete.")
                self.stop_session()


class UdpListener(QThread):
    logSignal = Signal(str)
    batterySignal = Signal(float, int)  # volts, percent
    packetSignal = Signal(int, int, int, int, str)  # seq, dx, dy, buttons, sender
    keySignal = Signal(int)  # keyId: 1=F5,2=RIGHT,3=LEFT
    rawImuSignal = Signal(int, int, int, int, int, int)  # ax,ay,az,gx,gy,gz

    def __init__(self, port: int, parent=None):
        super().__init__(parent)
        self.port = port
        self.stop_event = Event()

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind(("0.0.0.0", self.port))
        except Exception as e:
            self.logSignal.emit(f"UDP bind failed: {type(e).__name__} {e}")
            return
        sock.settimeout(1.0)
        while not self.stop_event.is_set():
            try:
                data, addr = sock.recvfrom(64)
            except socket.timeout:
                continue
            except Exception as e:
                self.logSignal.emit(f"UDP receive error: {e}")
                break
            if not data:
                continue
            # Battery packet (0xB0)
            if data[0] == 0xB0 and len(data) >= 4:
                mv = data[2] | (data[3] << 8)
                volts = mv / 1000.0
                pct = int(data[4]) if len(data) >= 5 else -1
                self.batterySignal.emit(volts, pct)
                continue
            # Mouse packet (0xA5)
            if len(data) >= 6 and data[0] == 0xA5:
                seq = data[1]
                dx = int.from_bytes(data[2:3], 'little', signed=True)
                dy = int.from_bytes(data[3:4], 'little', signed=True)
                buttons = data[4]
                self.packetSignal.emit(seq, dx, dy, buttons, addr[0])
                continue
            # Key packet (0xC0): 0xC0, seq, keyId, 0
            if len(data) >= 3 and data[0] == 0xC0:
                keyId = int(data[2])
                self.keySignal.emit(keyId)
                continue
            # Raw IMU packet (0xD0): 0xD0, seq, ax,ay,az,gx,gy,gz (int16 LE)
            if len(data) >= 14 and data[0] == 0xD0:
                def s16(lo, hi):
                    v = lo | (hi << 8)
                    return v - 65536 if v & 0x8000 else v
                ax = s16(data[2], data[3])
                ay = s16(data[4], data[5])
                az = s16(data[6], data[7])
                gx = s16(data[8], data[9])
                gy = s16(data[10], data[11])
                gz = s16(data[12], data[13])
                self.rawImuSignal.emit(ax, ay, az, gx, gy, gz)
                continue
        try:
            sock.close()
        except Exception:
            pass

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(800, 520)

        self.mouse = Controller() if PYNPUT_AVAILABLE else None
        self.left_down = False
        self.prev_buttons = 0
        self.packet_count = 0
        self.last_packet_at = None
        self.last_sender = None

        self.udp_thread: UdpListener | None = None
        self.udp_port = 5555
        self.recenter_edges = True
        self.edge_margin = 2
        self.show_batt_percent = True  # toggle: True -> show percent, False -> show volts

        self._load_settings()

        # Build UI
        self._build_ui()
        self._init_virtual_desktop()
        # Motion history for GL visual
        from collections import deque
        self.motion_hist = deque(maxlen=400)
        self.batt_volts = 0.0
        self.batt_pct = -1

        # Timers
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(1000)

    def _build_ui(self):
        central = QWidget()
        vroot = QVBoxLayout(central)

        # Tabs
        tabs = QTabWidget()
        self.tabs = tabs
        vroot.addWidget(tabs)

        # Mouse tab
        mouse_tab = QWidget(); tabs.addTab(mouse_tab, "Mouse")
        mv = QVBoxLayout(mouse_tab)

        top = QHBoxLayout()
        self.listen_btn = QPushButton("Start Wi‑Fi listener")
        self.listen_btn.clicked.connect(self.toggle_listener)
        top.addWidget(self.listen_btn)
        top.addWidget(QLabel("UDP Port:"))
        self.port_spin = QSpinBox(); self.port_spin.setRange(1, 65535); self.port_spin.setValue(self.udp_port)
        top.addWidget(self.port_spin)
        apply_btn = QPushButton("Apply"); apply_btn.clicked.connect(self.apply_port)
        top.addWidget(apply_btn)
        top.addStretch(1)
        mv.addLayout(top)

        # Battery row
        brow = QHBoxLayout()
        brow.addWidget(QLabel("Battery:"))
        self.batt_label = QLabel("--")
        brow.addWidget(self.batt_label)
        self.batt_bar = QProgressBar(); self.batt_bar.setRange(0, 100); self.batt_bar.setValue(0); self.batt_bar.setFixedWidth(140)
        brow.addWidget(self.batt_bar)
        brow.addStretch(1)
        mv.addLayout(brow)

        # Mouse log
        self.mouse_log = QTextEdit(); self.mouse_log.setReadOnly(True); self.mouse_log.setFixedHeight(140)
        mv.addWidget(self.mouse_log)

        # Diagnostics tab
        diag_tab = QWidget(); tabs.addTab(diag_tab, "Diagnostics")
        dv = QVBoxLayout(diag_tab)
        dtop = QHBoxLayout()
        diag_btn = QPushButton("Run diagnostics"); diag_btn.clicked.connect(self.run_diagnostics)
        clear_btn = QPushButton("Clear log"); clear_btn.clicked.connect(lambda: self.log_box.clear())
        dtop.addWidget(diag_btn); dtop.addWidget(clear_btn); dtop.addStretch(1)
        dv.addLayout(dtop)
        self.log_box = QTextEdit(); self.log_box.setReadOnly(True)
        dv.addWidget(self.log_box)
        # Toast label for key events
        self.toast_label = QLabel("")
        self.toast_label.setStyleSheet("color:#fff;background:#444;padding:4px;border-radius:4px;")
        self.toast_label.setVisible(False)
        dv.addWidget(self.toast_label)

        # BLE tab
        ble_tab = QWidget(); tabs.addTab(ble_tab, "BLE")
        bl = QVBoxLayout(ble_tab)
        btop = QHBoxLayout()
        self.ble_scan_btn = QPushButton("Scan BLE")
        self.ble_scan_btn.clicked.connect(self.ble_scan)
        btop.addWidget(self.ble_scan_btn)
        self.ble_connect_btn = QPushButton("Connect")
        self.ble_connect_btn.clicked.connect(self.ble_connect)
        btop.addWidget(self.ble_connect_btn)
        self.ble_disconnect_btn = QPushButton("Disconnect")
        self.ble_disconnect_btn.clicked.connect(self.ble_disconnect)
        btop.addWidget(self.ble_disconnect_btn)
        btop.addStretch(1)
        bl.addLayout(btop)
        self.ble_list = QListWidget(); bl.addWidget(self.ble_list)

        # Visual (OpenGL) tab
        visual_tab = QWidget(); tabs.addTab(visual_tab, "Visual")
        vv = QVBoxLayout(visual_tab)
        self.gl_widget = VisualWidget(self)
        vv.addWidget(self.gl_widget)

        # Settings tab
        settings_tab = QWidget(); tabs.addTab(settings_tab, "Settings")
        sv = QVBoxLayout(settings_tab)
        self.recenter_cb = QCheckBox("Recenter at edges (avoid boundary lock)"); self.recenter_cb.setChecked(self.recenter_edges)
        self.recenter_cb.stateChanged.connect(self.on_recenter_toggle)
        sv.addWidget(self.recenter_cb)
        self.batt_toggle_cb = QCheckBox("Show battery percent (uncheck to show volts)")
        self.batt_toggle_cb.setChecked(self.show_batt_percent)
        self.batt_toggle_cb.stateChanged.connect(self.on_batt_toggle)
        sv.addWidget(self.batt_toggle_cb)
        sv.addStretch(1)

        # ML tab (enhanced automatic recorder)
        ml_tab = QWidget(); tabs.addTab(ml_tab, "ML")
        mlv = QVBoxLayout(ml_tab)

        # Mode & label area
        mode_row = QHBoxLayout()
        self.mode_tap_btn = QRadioButton("Tap")
        self.mode_auto_btn = QRadioButton("Auto Motion")
        self.mode_batch_btn = QRadioButton("Batch")
        self.mode_tap_btn.setChecked(True)
        mode_row.addWidget(QLabel("Mode:"))
        mode_row.addWidget(self.mode_tap_btn)
        mode_row.addWidget(self.mode_auto_btn)
        mode_row.addWidget(self.mode_batch_btn)
        mode_row.addStretch(1)
        mlv.addLayout(mode_row)

        label_row = QHBoxLayout()
        self.ml_label_edit = QLineEdit(); self.ml_label_edit.setPlaceholderText("active label (e.g. flick_left)")
        label_row.addWidget(self.ml_label_edit)
        self.ml_set_label_btn = QPushButton("Set Active")
        self.ml_set_label_btn.clicked.connect(self._ml_set_label)
        label_row.addWidget(self.ml_set_label_btn)
        self.batch_labels_edit = QPlainTextEdit()
        self.batch_labels_edit.setPlaceholderText("Label list (one per line) for batch mode")
        self.batch_labels_edit.setFixedHeight(80)
        mlv.addWidget(self.batch_labels_edit)
        mlv.addLayout(label_row)

        # Controls row
        ctrl_row = QHBoxLayout()
        self.session_start_btn = QPushButton("Start Session")
        self.session_start_btn.clicked.connect(self._ml_start_session)
        ctrl_row.addWidget(self.session_start_btn)
        self.quick_auto_btn = QPushButton("Quick Auto Collect")
        self.quick_auto_btn.setStyleSheet("font-weight:600")
        self.quick_auto_btn.clicked.connect(self._ml_quick_auto)
        ctrl_row.addWidget(self.quick_auto_btn)
        self.session_stop_btn = QPushButton("Stop Session")
        self.session_stop_btn.clicked.connect(self._ml_stop_session)
        ctrl_row.addWidget(self.session_stop_btn)
        self.tap_capture_btn = QPushButton("Capture (Space)")
        self.tap_capture_btn.clicked.connect(self._ml_manual_capture)
        ctrl_row.addWidget(self.tap_capture_btn)
        ctrl_row.addStretch(1)
        mlv.addLayout(ctrl_row)

        # Advanced settings
        adv_box = QGroupBox("Advanced Settings")
        adv_box.setCheckable(True); adv_box.setChecked(False)
        form = QFormLayout(adv_box)
        self.win_len_spin = QDoubleSpinBox(); self.win_len_spin.setRange(0.2, 3.0); self.win_len_spin.setSingleStep(0.1); self.win_len_spin.setValue(1.0)
        form.addRow("Window (s)", self.win_len_spin)
        self.pre_roll_spin = QDoubleSpinBox(); self.pre_roll_spin.setRange(0.0, 1.0); self.pre_roll_spin.setSingleStep(0.05); self.pre_roll_spin.setValue(0.25)
        form.addRow("Pre-roll (s)", self.pre_roll_spin)
        self.post_roll_spin = QDoubleSpinBox(); self.post_roll_spin.setRange(0.0, 1.0); self.post_roll_spin.setSingleStep(0.05); self.post_roll_spin.setValue(0.25)
        form.addRow("Post-roll (s)", self.post_roll_spin)
        self.accel_thresh_spin = QDoubleSpinBox(); self.accel_thresh_spin.setRange(0.2, 5.0); self.accel_thresh_spin.setValue(1.2)
        form.addRow("Accel thresh (g)", self.accel_thresh_spin)
        self.gyro_thresh_spin = QDoubleSpinBox(); self.gyro_thresh_spin.setRange(20.0, 1000.0); self.gyro_thresh_spin.setValue(120.0)
        form.addRow("Gyro thresh (dps)", self.gyro_thresh_spin)
        self.quiet_ms_spin = QSpinBox(); self.quiet_ms_spin.setRange(50, 2000); self.quiet_ms_spin.setValue(200)
        form.addRow("Quiet time (ms)", self.quiet_ms_spin)
        self.target_per_label_spin = QSpinBox(); self.target_per_label_spin.setRange(1, 500); self.target_per_label_spin.setValue(30)
        form.addRow("Target per label", self.target_per_label_spin)
        mlv.addWidget(adv_box)

        # Status & progress
        self.ml_status = QLabel("Session: idle")
        mlv.addWidget(self.ml_status)
        self.progress_box = QTextEdit(); self.progress_box.setReadOnly(True); self.progress_box.setFixedHeight(120)
        mlv.addWidget(self.progress_box)
        self.ml_note = QLabel("Segments saved under ml_logs/ (one gesture per file). Raw IMU header 0xD0.")
        nf = self.ml_note.font(); nf.setPointSize(9); self.ml_note.setFont(nf)
        mlv.addWidget(self.ml_note)
        mlv.addStretch(1)

        # Instantiate recorder (lazy; created on session start with current params)
        self.recorder = None

        self.setCentralWidget(central)
        self.status = QStatusBar(); self.setStatusBar(self.status)

    # Hotkeys for ML recorder convenience
    def keyPressEvent(self, event: QtGui.QKeyEvent):
        key = event.key()
        if key == Qt.Key_Space:
            if hasattr(self, 'tap_capture_btn') and self.tap_capture_btn.isEnabled():
                self._ml_manual_capture()
                event.accept(); return
        if key == Qt.Key_Escape:
            self._ml_stop_session(); event.accept(); return
        # Numeric label shortcuts (1..9)
        if Qt.Key_1 <= key <= Qt.Key_9:
            idx = key - Qt.Key_1
            if self.recorder and self.recorder.labels:
                if idx < len(self.recorder.labels):
                    self.recorder.set_active_label(self.recorder.labels[idx])
                    event.accept(); return
        # C to cycle next label
        if key == Qt.Key_C:
            if self.recorder:
                self.recorder.next_label(); event.accept(); return
        super().keyPressEvent(event)

    # Logging helpers
    def log(self, msg: str):
        self.log_box.append(msg)

    def log_mouse(self, msg: str):
        self.mouse_log.append(msg)

    # Settings
    def _load_settings(self):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
            self.udp_port = int(data.get('udp_port', self.udp_port))
            self.recenter_edges = bool(data.get('recenter_edges', self.recenter_edges))
            self.show_batt_percent = bool(data.get('show_batt_percent', self.show_batt_percent))
        except Exception:
            pass

    def _save_settings(self):
        try:
            data = {
                'udp_port': int(self.udp_port),
                'recenter_edges': bool(self.recenter_edges),
                'show_batt_percent': bool(self.show_batt_percent),
            }
            with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except Exception:
            pass

    # --- ML logging controls ---
    # --- New ML recorder handlers ---
    def _ml_set_label(self):
        label = self.ml_label_edit.text().strip()
        if not label:
            QMessageBox.information(self, "ML", "Enter a label")
            return
        if self.recorder:
            self.recorder.set_active_label(label)
        self.ml_status.setText(f"Active label: {label}")

    def _ml_start_session(self):
        mode = 'tap'
        if self.mode_auto_btn.isChecked():
            mode = 'auto'
        elif self.mode_batch_btn.isChecked():
            mode = 'batch'
        labels_text = self.batch_labels_edit.toPlainText().strip()
        labels = [self.ml_label_edit.text().strip()] if mode != 'batch' else [l.strip() for l in labels_text.splitlines() if l.strip()]
        if mode != 'batch' and not labels[0]:
            QMessageBox.information(self, "ML", "Set an active label before starting session")
            return
        base = os.path.join(os.path.dirname(__file__), 'ml_logs')
        self.recorder = GestureRecorder(
            base_dir=base,
            sampling_hz=100,
            window_len_s=self.win_len_spin.value(),
            pre_roll_s=self.pre_roll_spin.value(),
            post_roll_s=self.post_roll_spin.value(),
            accel_thresh_g=self.accel_thresh_spin.value(),
            gyro_thresh_dps=self.gyro_thresh_spin.value(),
            quiet_ms=self.quiet_ms_spin.value(),
            target_per_label=self.target_per_label_spin.value()
        )
        # Connect recorder signals
        self.recorder.segmentSaved.connect(self._on_segment_saved)
        self.recorder.segmentRejected.connect(self._on_segment_rejected)
        self.recorder.progressUpdated.connect(self._on_progress)
        self.recorder.activeLabelChanged.connect(self._on_active_label_changed)
        self.recorder.statusMessage.connect(self._on_recorder_status)
        self.recorder.start_session(labels, mode)
        # Ensure UDP listener is running
        if not (self.udp_thread and self.udp_thread.isRunning()):
            self.toggle_listener()
        self._send_stream_cmd(start=True)
        self.ml_status.setText(f"Session running ({mode})")
        self.log(f"Recorder session started mode={mode} labels={labels}")
        # UX: disable tap button in batch/auto
        self.tap_capture_btn.setEnabled(mode == 'tap')

    def _ml_stop_session(self):
        if self.recorder:
            self.recorder.stop_session()
        self._send_stream_cmd(start=False)
        self.ml_status.setText("Session: idle")

    def _ml_manual_capture(self):
        if not self.recorder:
            QMessageBox.information(self, "ML", "Start a session first")
            return
        if self.mode_tap_btn.isChecked():
            self.recorder.manual_trigger()
        else:
            # force finalize if currently capturing
            if self.recorder.capturing:
                self.recorder._finalize_capture()
            else:
                self.recorder.manual_trigger()

    def _on_segment_saved(self, path: str, label: str):
        self.log(f"Segment saved: {os.path.basename(path)} label={label}")
        self._show_toast(f"Saved: {label}")

    def _on_segment_rejected(self, reason: str):
        self.log(f"Segment rejected: {reason}")

    def _on_progress(self, mapping: dict):
        lines = [f"{k}: {v}" for k,v in mapping.items()]
        self.progress_box.setPlainText("\n".join(lines))

    def _on_recorder_status(self, msg: str):
        self.log("[Recorder] " + msg)
        if msg:
            self._show_toast(msg)

    def _on_active_label_changed(self, label: str):
        self.ml_status.setText(f"Active label: {label}")
        self.ml_label_edit.setText(label)
        # Show a concise prompt for the next gesture
        if self.recorder:
            count = self.recorder.label_counts.get(label, 0)
            tgt = self.recorder.target_per_label if self.recorder.batch_enabled else None
            suffix = f" ({count}/{tgt})" if tgt is not None else ""
            self._show_toast(f"Do gesture: {label}{suffix}")

    def _ml_quick_auto(self):
        # One-click: use labels from list, start batch session
        labels_text = self.batch_labels_edit.toPlainText().strip()
        labels = [l.strip() for l in labels_text.splitlines() if l.strip()]
        if not labels:
            QMessageBox.information(self, "ML", "Enter labels (one per line) in the list, then Quick Auto Collect")
            return
        self.mode_batch_btn.setChecked(True)
        self.ml_label_edit.setText(labels[0])
        self._ml_start_session()

    def _send_stream_cmd(self, start: bool):
        cmd = 0x01 if start else 0x02
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            pkt = bytes([0xD2, cmd])
            sock.sendto(pkt, ("255.255.255.255", int(self.udp_port)))
            sock.close()
        except Exception as e:
            self.log(f"Failed to send stream cmd {cmd}: {e}")

    # Virtual desktop bounds
    def _init_virtual_desktop(self):
        # Union of all screens
        screens = QtGui.QGuiApplication.screens()
        if not screens:
            self.vx, self.vy, self.vw, self.vh = 0, 0, 1920, 1080
            return
        rect = screens[0].geometry()
        xmin = rect.left(); ymin = rect.top(); xmax = rect.right(); ymax = rect.bottom()
        for s in screens[1:]:
            r = s.geometry()
            xmin = min(xmin, r.left()); ymin = min(ymin, r.top())
            xmax = max(xmax, r.right()); ymax = max(ymax, r.bottom())
        self.vx, self.vy = xmin, ymin
        self.vw, self.vh = (xmax - xmin + 1), (ymax - ymin + 1)

    # Diagnostics
    def run_diagnostics(self):
        ssid = self._windows_ssid()
        ip = self._primary_ip()
        self.log(f"OS: {py_platform.system()} {py_platform.release()}")
        self.log(f"Virtual desktop: ({self.vx},{self.vy}) {self.vw}x{self.vh}")
        self.log(f"Primary IP: {ip or 'unknown'}  SSID: {ssid or 'unknown'}")

    def _primary_ip(self):
        ip = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            try:
                ip = socket.gethostbyname(socket.gethostname())
            except Exception:
                ip = None
        return ip

    def _windows_ssid(self):
        if py_platform.system() != 'Windows':
            return None
        try:
            out = subprocess.check_output(["netsh", "wlan", "show", "interfaces"], encoding='utf-8', errors='ignore')
            for line in out.splitlines():
                line = line.strip()
                if line.startswith("SSID") and not line.startswith("SSID BSSID"):
                    parts = line.split(':', 1)
                    if len(parts) == 2:
                        return parts[1].strip()
        except Exception:
            return None
        return None

    # Listener controls
    def toggle_listener(self):
        if self.udp_thread and self.udp_thread.isRunning():
            self.listen_btn.setText("Start Wi‑Fi listener")
            self.udp_thread.stop_event.set()
            self.udp_thread.wait(1000)
            self.udp_thread = None
            self.log("Stopped UDP listener")
        else:
            self.udp_thread = UdpListener(self.udp_port)
            self.udp_thread.logSignal.connect(self.log)
            self.udp_thread.batterySignal.connect(self.on_battery)
            self.udp_thread.packetSignal.connect(self.on_packet)
            self.udp_thread.keySignal.connect(self.on_key)
            self.udp_thread.rawImuSignal.connect(self.on_raw_imu)
            self.udp_thread.start()
            self.listen_btn.setText("Stop Wi‑Fi listener")
            self.log(f"Started UDP listener on port {self.udp_port}")

    def apply_port(self):
        new_port = int(self.port_spin.value())
        restart = self.udp_thread and self.udp_thread.isRunning()
        if restart:
            self.toggle_listener()
        self.udp_port = new_port
        self._save_settings()
        if restart:
            self.toggle_listener()

    # UI updates
    def _update_status(self):
        ssid = self._windows_ssid()
        ip = self._primary_ip()
        last = self.last_packet_at.strftime('%H:%M:%S') if self.last_packet_at else '—'
        sender = self.last_sender or '—'
        text = f"UDP:{self.udp_port}  Packets:{self.packet_count}  Last:{last} from {sender}  SSID:{ssid or 'unknown'}  IP:{ip or 'unknown'}"
        self.status.showMessage(text)

    def on_recenter_toggle(self, state):
        self.recenter_edges = (state == Qt.Checked)
        self._save_settings()

    def on_batt_toggle(self, state):
        self.show_batt_percent = (state == Qt.Checked)
        self._save_settings()
        self.gl_widget.update()

    # Packet handlers
    def on_battery(self, volts: float, pct: int):
        # Update stored values first
        self.batt_volts = volts; self.batt_pct = pct
        if pct >= 0:
            self.batt_bar.setValue(pct)
        else:
            # Piecewise LiPo 1S estimation fallback (if firmware didn't send percent)
            # Matches firmware curve for consistency
            points = [
                (3.50,0),(3.55,3),(3.58,6),(3.60,9),(3.63,12),(3.66,16),(3.69,20),(3.71,24),(3.73,28),(3.75,33),
                (3.78,38),(3.80,43),(3.83,49),(3.85,54),(3.87,58),(3.89,62),(3.91,66),(3.94,70),(3.96,73),(3.98,76),
                (4.00,80),(4.03,84),(4.06,88),(4.09,92),(4.12,95),(4.15,97),(4.18,99),(4.20,100)
            ]
            if volts <= points[0][0]:
                est_pct = 0
            elif volts >= points[-1][0]:
                est_pct = 100
            else:
                est_pct = 0
                for i in range(1, len(points)):
                    if volts <= points[i][0]:
                        v0,p0 = points[i-1]; v1,p1 = points[i]
                        t = (volts - v0)/(v1 - v0)
                        est_pct = int(p0 + t*(p1 - p0) + 0.5)
                        break
            self.batt_bar.setValue(est_pct)
            pct = est_pct
        # Update label depending on toggle
        if self.show_batt_percent:
            self.batt_label.setText(f"{pct}%")
        else:
            self.batt_label.setText(f"{volts:.2f} V")
        self.log(f"Battery: {volts:.2f}V {pct}%")
        self.gl_widget.update()

    def on_packet(self, seq: int, dx: int, dy: int, buttons: int, sender: str):
        self.packet_count += 1
        self.last_packet_at = datetime.now()
        self.last_sender = sender
        self.log_mouse(f"UDP seq={seq} dx={dx} dy={dy} btn=0x{buttons:02x}")
        if not self.mouse:
            return
        # Relative motion with edge handling across virtual desktop
        try:
            x, y = self.mouse.position
            nx = x + dx
            ny = y + dy
            vx, vy, vw, vh = self.vx, self.vy, self.vw, self.vh
            # Clamp
            clx = max(vx, min(vx + vw - 1, nx))
            cly = max(vy, min(vy + vh - 1, ny))
            eff_dx = int(clx - x)
            eff_dy = int(cly - y)
            near_edge = (nx <= vx + self.edge_margin or nx >= vx + vw - 1 - self.edge_margin or ny <= vy + self.edge_margin or ny >= vy + vh - 1 - self.edge_margin)
            if self.recenter_edges and near_edge:
                # Recenter to middle
                cx = vx + vw // 2
                cy = vy + vh // 2
                QtGui.QCursor.setPos(cx, cy)
            else:
                if eff_dx != 0 or eff_dy != 0:
                    self.mouse.move(eff_dx, eff_dy)
        except Exception:
            # Fallback raw move
            try:
                self.mouse.move(dx, dy)
            except Exception:
                pass

        # Button handling: left stateful, right/middle edge-trigger
        left = bool(buttons & 0x01); right = bool(buttons & 0x02); back = bool(buttons & 0x04)
        if left != self.left_down:
            if left:
                self.mouse.press(Button.left)
            else:
                self.mouse.release(Button.left)
            self.left_down = left
        # Edge triggers
        prev = self.prev_buttons
        if right and not (prev & 0x02):
            self.mouse.press(Button.right); self.mouse.release(Button.right)
        if back and not (prev & 0x04):
            self.mouse.press(Button.middle); self.mouse.release(Button.middle)
        self.prev_buttons = buttons
        # Record motion for GL
        self.motion_hist.append((dx, dy))
        self.gl_widget.update()
    # Raw IMU handler (for logging)
    def on_raw_imu(self, ax: int, ay: int, az: int, gx: int, gy: int, gz: int):
        # Feed sample to recorder if active; timestamp in ms monotonic
        if self.recorder:
            ts_ms = int(time.time() * 1000)
            self.recorder.feed_sample(ts_ms, ax, ay, az, gx, gy, gz)
        # Could add live preview later


    # Key packet handler: inject presentation keys
    def on_key(self, keyId: int):
        # keyId: 1=F5(start), 2=RIGHT(next), 3=LEFT(prev)
        if not PYNPUT_AVAILABLE:
            self.log("pynput not available: cannot inject keys")
            return
        try:
            from pynput.keyboard import Controller as KbController, Key
            kb = KbController()
            if keyId == 1:
                # F5 to start presentation (PowerPoint/Keynote; for Google Slides use F5 or Ctrl+F5 depending)
                kb.press(Key.f5); kb.release(Key.f5)
                self._show_toast("Presentation: START (F5)")
            elif keyId == 2:
                kb.press(Key.right); kb.release(Key.right)
                self._show_toast("Presentation: NEXT")
            elif keyId == 3:
                kb.press(Key.left); kb.release(Key.left)
                self._show_toast("Presentation: PREV")
            elif keyId == 4:
                kb.press(Key.esc); kb.release(Key.esc)
                self._show_toast("Presentation: END (Esc)")
            else:
                self.log(f"Unknown keyId {keyId}")
        except Exception as e:
            self.log(f"Key inject failed: {e}")

    def _show_toast(self, text: str):
        self.toast_label.setText(text)
        self.toast_label.setVisible(True)
        self.toast_label.setStyleSheet("color:#fff;background:#333;padding:4px 8px;border-radius:6px;font-weight:500;")
        # Auto hide after 1.5s
        QtCore.QTimer.singleShot(1500, lambda: self.toast_label.setVisible(False))

    # BLE (basic scan/connect — Windows note: OS may auto-claim HID; this is for diagnostics)
    def ble_scan(self):
        from bleak import BleakScanner
        self.ble_scan_btn.setEnabled(False)
        self.ble_list.clear()
        def _scan():
            try:
                devices = BleakScanner.discover(timeout=6.0)
                return devices
            except Exception as e:
                return e
        def _done(res):
            self.ble_scan_btn.setEnabled(True)
            if isinstance(res, Exception):
                QMessageBox.warning(self, "BLE", f"Scan failed: {res}")
                return
            for d in res:
                name = getattr(d, 'name', None) or '<unknown>'
                addr = getattr(d, 'address', '<no-addr>')
                self.ble_list.addItem(f"{name} [{addr}]")
        # Run in thread pool to avoid blocking UI
        pool = QtCore.QThreadPool.globalInstance()
        class Task(QtCore.QRunnable):
            def run(self2):
                res = _scan()
                QtCore.QMetaObject.invokeMethod(self, lambda: _done(res), Qt.QueuedConnection)
        pool.start(Task())

    def ble_connect(self):
        from bleak import BleakClient
        item = self.ble_list.currentItem()
        if not item:
            QMessageBox.information(self, "BLE", "Select a device first")
            return
        addr = item.text().split('[')[-1].strip(']')
        self.log(f"Connecting to {addr}...")
        self._ble_client = BleakClient(addr)
        pool = QtCore.QThreadPool.globalInstance()
        def _connect():
            try:
                self._ble_client.connect(timeout=8.0)
                return None
            except Exception as e:
                return e
        def _done(res):
            if res is None:
                QMessageBox.information(self, "BLE", "Connected")
            else:
                QMessageBox.warning(self, "BLE", f"Connect failed: {res}")
        class Task(QtCore.QRunnable):
            def run(self2):
                res = _connect()
                QtCore.QMetaObject.invokeMethod(self, lambda: _done(res), Qt.QueuedConnection)
        pool.start(Task())

    def ble_disconnect(self):
        cli = getattr(self, '_ble_client', None)
        if not cli:
            return
        try:
            cli.disconnect()
            QMessageBox.information(self, "BLE", "Disconnected")
        except Exception:
            pass

class VisualWidget(QOpenGLWidget):
    def __init__(self, main: 'MainWindow'):
        super().__init__(main)
        self.main = main
        self.setMinimumHeight(220)
        self._last_frame_time = None
        self._fps = 0.0
        self._frame_counter = 0
        self._fps_accum_ms = 0.0
        # Continuous repaint to keep FPS updating and maintain smooth visuals
        self._repaint_timer = QTimer(self)
        self._repaint_timer.timeout.connect(self.update)
        self._repaint_timer.start(16)  # ~60 FPS

    def initializeGL(self):
        glClearColor(0.08, 0.08, 0.10, 1.0)
        glDisable(GL_DEPTH_TEST)
        glEnable(GL_LINE_SMOOTH)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, w, h, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT)
        glLoadIdentity()
        w = self.width(); h = self.height()
        # FPS timing
        now_ms = QtCore.QTime.currentTime().msec() + QtCore.QTime.currentTime().second()*1000 + QtCore.QTime.currentTime().minute()*60000
        if self._last_frame_time is not None:
            dt = now_ms - self._last_frame_time
            if dt < 0: dt = 0
            self._fps_accum_ms += dt
            self._frame_counter += 1
            if self._fps_accum_ms >= 1000:
                self._fps = self._frame_counter * 1000.0 / self._fps_accum_ms
                self._fps_accum_ms = 0.0
                self._frame_counter = 0
        self._last_frame_time = now_ms
        # Draw motion trace as polyline centered
        cx, cy = w//2, h//2
        x, y = cx, cy
        hist = list(self.main.motion_hist)[-200:]
        glBegin(GL_LINE_STRIP)
        for i,(dx,dy) in enumerate(hist):
            # Progressive color fade (older points dimmer)
            age = i/len(hist) if hist else 0
            glColor3f(0.2*(1-age)+0.0, 0.7*(1-age)+0.1, 1.0*(1-age)+0.2)
            x = max(0, min(w-1, x + dx))
            y = max(0, min(h-1, y + dy))
            glVertex2f(x, y)
        glEnd()

        # Battery gauge (right side vertical bar) with gradient + low battery blink
        volts = self.main.batt_volts
        pct = self.main.batt_pct if self.main.batt_pct >= 0 else 0
        bw, bh = 24, int(h*0.6)
        bx, by = w - 40, (h - bh)//2
        # Outline
        glColor3f(0.55, 0.55, 0.55)
        glBegin(GL_LINE_LOOP)
        glVertex2f(bx, by)
        glVertex2f(bx+bw, by)
        glVertex2f(bx+bw, by+bh)
        glVertex2f(bx, by+bh)
        glEnd()
        # Compute gradient color (red->yellow->green)
        # Red (0%) -> Yellow (50%) -> Green (100%)
        rR,gR,bR = 0.90,0.15,0.10
        rY,gY,bY = 0.95,0.75,0.15
        rG,gG,bG = 0.15,0.85,0.35
        if pct <= 50:
            t = pct/50.0
            cr = rR + t*(rY - rR); cg = gR + t*(gY - gR); cb = bR + t*(bY - bR)
        else:
            t = (pct-50)/50.0
            cr = rY + t*(rG - rY); cg = gY + t*(gG - gY); cb = bY + t*(bG - bY)
        # Blink below 10%
        blink_on = True
        if pct < 10:
            ms = QtCore.QTime.currentTime().msec() + QtCore.QTime.currentTime().second()*1000
            blink_on = (ms % 800) < 400
        fill = int((pct/100.0) * (bh-4))
        if blink_on and fill > 0:
            glColor3f(cr, cg, cb)
            glBegin(GL_QUADS)
            glVertex2f(bx+2, by+bh-2)
            glVertex2f(bx+bw-2, by+bh-2)
            glVertex2f(bx+bw-2, by+bh-2-fill)
            glVertex2f(bx+2, by+bh-2-fill)
            glEnd()
        # Optional LOW indicator
        if pct < 10 and blink_on:
            painter_low = QtGui.QPainter(self)
            painter_low.setPen(QtGui.QColor(255,70,70))
            painter_low.drawText(bx-4, by+bh+14, "LOW")
            painter_low.end()
        # Text overlay
        painter = QtGui.QPainter(self)
        painter.setPen(QtGui.QColor(220,220,220))
        painter.drawText(10, 18, f"Battery: {volts:.2f}V {pct}%  FPS: {self._fps:.1f}")
        painter.drawText(10, 36, f"Trace points: {len(self.main.motion_hist)}")
        painter.end()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    ret = app.exec()
    # Clean stop
    try:
        if w.udp_thread and w.udp_thread.isRunning():
            w.udp_thread.stop_event.set()
            w.udp_thread.wait(1000)
    except Exception:
        pass
    sys.exit(ret)

if __name__ == '__main__':
    main()
