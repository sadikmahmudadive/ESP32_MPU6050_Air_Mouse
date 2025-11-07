import sys
import os
import json
import socket
import platform as py_platform  # alias to avoid clash with OpenGL.platform after GL imports
import subprocess
from datetime import datetime
from threading import Event

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt, Signal, QTimer, QThread
from PySide6.QtWidgets import QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit, QSpinBox, QCheckBox, QProgressBar, QTextEdit, QStatusBar, QMessageBox, QListWidget
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

class UdpListener(QThread):
    logSignal = Signal(str)
    batterySignal = Signal(float, int)  # volts, percent
    packetSignal = Signal(int, int, int, int, str)  # seq, dx, dy, buttons, sender

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
        sv.addStretch(1)

        self.setCentralWidget(central)
        self.status = QStatusBar(); self.setStatusBar(self.status)

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
        except Exception:
            pass

    def _save_settings(self):
        try:
            data = {
                'udp_port': int(self.udp_port),
                'recenter_edges': bool(self.recenter_edges),
            }
            with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except Exception:
            pass

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

    # Packet handlers
    def on_battery(self, volts: float, pct: int):
        self.batt_label.setText(f"{volts:.2f} V {pct if pct>=0 else 0}%")
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

        # Battery gauge (right side vertical bar)
        volts = self.main.batt_volts
        pct = self.main.batt_pct if self.main.batt_pct >= 0 else 0
        bw, bh = 24, int(h*0.6)
        bx, by = w - 40, (h - bh)//2
        glColor3f(0.45, 0.45, 0.45)
        glBegin(GL_LINE_LOOP)
        glVertex2f(bx, by)
        glVertex2f(bx+bw, by)
        glVertex2f(bx+bw, by+bh)
        glVertex2f(bx, by+bh)
        glEnd()
        fill = int((pct/100.0) * (bh-4))
        glColor3f(0.15, 0.85, 0.35)
        glBegin(GL_QUADS)
        glVertex2f(bx+2, by+bh-2)
        glVertex2f(bx+bw-2, by+bh-2)
        glVertex2f(bx+bw-2, by+bh-2-fill)
        glVertex2f(bx+2, by+bh-2-fill)
        glEnd()
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
