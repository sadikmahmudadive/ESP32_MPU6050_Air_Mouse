"""
Minimal desktop companion for "ESP Air 1" using Bleak and Tkinter.

This prototype scans for BLE devices, lists them, and can connect to a device named
"ESP Air 1". Once the firmware exposes configuration GATT characteristics, this
script can be extended to read/write them.

Run:
    python app.py

Dependencies in requirements.txt
"""
import asyncio
import threading
import traceback
import platform
import sys
import os
import json
import subprocess
from datetime import datetime
import tkinter as tk
from tkinter import scrolledtext, messagebox, ttk
import socket
from threading import Event
import ctypes
try:
    from pynput.mouse import Controller, Button
    PYNPUT_AVAILABLE = True
except Exception:
    PYNPUT_AVAILABLE = False

from bleak import BleakScanner, BleakClient

TARGET_NAME = "ESP Air 1"
# Placeholder characteristic UUIDs (replace after adding to firmware)
BATTERY_CHAR_UUID = "00002a19-0000-1000-8000-00805f9b34fb"  # standard Battery Level
# Example custom chars you may add to the firmware
CFG_CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"

class App:
    def __init__(self, root):
        self.root = root
        root.title("ESP Air 1 — Companion")

        # Top banner
        banner = tk.Label(root, text="ESP Air Mouse", font=("Segoe UI", 16, "bold"))
        banner.pack(fill=tk.X, pady=(4,2))

        notebook = ttk.Notebook(root)
        notebook.pack(fill=tk.BOTH, expand=True)

        # Tabs
        mouse_tab = ttk.Frame(notebook)
        diag_tab = ttk.Frame(notebook)
        settings_tab = ttk.Frame(notebook)
        notebook.add(mouse_tab, text="Mouse")
        notebook.add(diag_tab, text="Diagnostics")
        notebook.add(settings_tab, text="Settings")

        # Mouse tab contents
        top_row = ttk.Frame(mouse_tab)
        top_row.pack(fill=tk.X)
        self.scan_btn = ttk.Button(top_row, text="Scan BLE", command=self.scan)
        self.scan_btn.pack(side=tk.LEFT)
        self.wifi_listen_btn = ttk.Button(top_row, text="Start Wi‑Fi listener", command=self.toggle_wifi_listener)
        self.wifi_listen_btn.pack(side=tk.LEFT, padx=6)
        ttk.Label(top_row, text="UDP Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value="5555")
        self.port_entry = ttk.Entry(top_row, width=6, textvariable=self.port_var)
        self.port_entry.pack(side=tk.LEFT)
        ttk.Button(top_row, text="Apply", command=self.apply_port).pack(side=tk.LEFT, padx=(4,0))

        list_frame = ttk.Frame(mouse_tab)
        list_frame.pack(fill=tk.BOTH, expand=True, pady=(4,4))
        self.devices_listbox = tk.Listbox(list_frame, height=10)
        self.devices_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        list_scroll = ttk.Scrollbar(list_frame, command=self.devices_listbox.yview)
        list_scroll.pack(side=tk.LEFT, fill=tk.Y)
        self.devices_listbox.config(yscrollcommand=list_scroll.set)

        actions_row = ttk.Frame(mouse_tab)
        actions_row.pack(fill=tk.X)
        self.connect_btn = ttk.Button(actions_row, text="Connect BLE", command=self.connect)
        self.connect_btn.pack(side=tk.LEFT)
        self.disconnect_btn = ttk.Button(actions_row, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=4)
        self.read_batt_btn = ttk.Button(actions_row, text="Read BLE Battery", command=self.read_battery, state=tk.DISABLED)
        self.read_batt_btn.pack(side=tk.LEFT, padx=4)

        # Battery display
        batt_frame = ttk.Frame(mouse_tab)
        batt_frame.pack(fill=tk.X, pady=(4,2))
        ttk.Label(batt_frame, text="Battery:").pack(side=tk.LEFT)
        self.batt_var = tk.StringVar(value="--")
        self.batt_label = ttk.Label(batt_frame, textvariable=self.batt_var, width=12)
        self.batt_label.pack(side=tk.LEFT)
        self.batt_progress = ttk.Progressbar(batt_frame, length=120, mode='determinate')
        self.batt_progress.pack(side=tk.LEFT, padx=6)

        # Mouse log
        self.mouse_log = scrolledtext.ScrolledText(mouse_tab, height=8)
        self.mouse_log.pack(fill=tk.BOTH, expand=False)

        # Diagnostics tab
        diag_controls = ttk.Frame(diag_tab)
        diag_controls.pack(fill=tk.X)
        self.diag_btn = ttk.Button(diag_controls, text="Run diagnostics", command=self.dump_diagnostics)
        self.diag_btn.pack(side=tk.LEFT)
        self.clear_btn = ttk.Button(diag_controls, text="Clear log", command=self.clear_log)
        self.clear_btn.pack(side=tk.LEFT, padx=4)
        self.help_btn = ttk.Button(diag_controls, text="Help", command=self.show_help)
        self.help_btn.pack(side=tk.LEFT, padx=4)
        self.log_box = scrolledtext.ScrolledText(diag_tab, height=14)
        self.log_box.pack(fill=tk.BOTH, expand=True, pady=(4,4))

        # Settings tab
        st_frame = ttk.Frame(settings_tab)
        st_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.recenter_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(st_frame, text="Recenter at edges (avoid boundary lock)", variable=self.recenter_var, command=self._on_recenter_toggle).pack(anchor='w')

        # Connect buttons
        self.connect_btn = tk.Button(root, text="Connect to ESP Air 1", command=self.connect)
        self.connect_btn.pack(fill=tk.X)

        # Manual address connect (useful on Windows when OS has claimed the device)
        self.addr_frame = tk.Frame(root)
        self.addr_frame.pack(fill=tk.X)
        tk.Label(self.addr_frame, text="Address:").pack(side=tk.LEFT)
        self.addr_entry = tk.Entry(self.addr_frame)
        self.addr_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.addr_connect_btn = tk.Button(self.addr_frame, text="Connect by address", command=self.connect_by_address)
        # Status + network info at bottom (moved into __init__ correctly)
        self.status = ttk.Label(root, text="Status: idle")
        self.status.pack(fill=tk.X)
        self.netinfo_label = ttk.Label(root, text="", anchor='w', justify='left')
        self.netinfo_label.pack(fill=tk.X)
        # Initialize virtual desktop bounds for multi-monitor setups (Windows)
        self._init_virtual_desktop_bounds(root)
        # Edge behavior controls
        self.recenter_edges = False  # set True if you prefer recentering at edges
        self.edge_margin = 2

        # Runtime state
        self.client = None
        self.loop = asyncio.get_event_loop()
        self.found_devices = []
        self.udp_thread = None
        self.udp_stop_event = Event()
        self.udp_port = 5555
        self.mouse = Controller() if PYNPUT_AVAILABLE else None
        self.left_down = False
        self._prev_udp_buttons = 0
        self.packet_count = 0
        self.last_packet_at = None
        self.last_sender = None

        # Load settings if present
        self._load_settings()
        self.port_var.set(str(self.udp_port))
        # apply settings
        self.recenter_edges = bool(getattr(self, 'recenter_edges', True))
        self.recenter_var.set(self.recenter_edges)

        # Auto-start Wi‑Fi listener so the system connects wirelessly without manual steps
        self.root.after(100, self._auto_start_listener)
        # Periodic UI/status updates
        self.root.after(1000, self._update_status_loop)

    def log(self, *args):
        line = ' '.join(map(str,args))
        self.log_box.insert(tk.END, line + "\n")
        self.log_box.see(tk.END)
        # Also mirror packet logs in mouse tab if they are UDP packets
        if line.startswith("UDP pkt") or line.startswith("Battery"):
            self.mouse_log.insert(tk.END, line + "\n")
            self.mouse_log.see(tk.END)

    def clear_log(self):
        self.log_box.delete('1.0', tk.END)

    def show_help(self):
        msg = (
            "Wi‑Fi Mouse usage:\n\n"
            "Direct AP mode (no router):\n"
            "  1) Double‑tap RESET -> connect to ESP‑Air‑Setup -> http://192.168.4.1\n"
            "  2) Check 'Direct AP mode' (set password if desired) -> Save & Reboot\n"
            "  3) Connect PC to Wi‑Fi 'ESP‑Air‑Mouse' -> run this app\n\n"
            "Normal Wi‑Fi (via router):\n"
            "  1) Double‑tap RESET -> http://192.168.4.1 -> enter SSID/password -> Save & Reboot\n"
            "  2) Ensure PC and ESP32 are on same network -> run this app\n\n"
            "Left = click/drag, Right hold = scroll mode, Right tap = right‑click, Back = middle‑click."
        )
        messagebox.showinfo("ESP Air — Help", msg)

    def _auto_start_listener(self):
        try:
            if not (self.udp_thread and self.udp_thread.is_alive()):
                self.toggle_wifi_listener()
        except Exception:
            pass

    def scan(self):
        self.status.config(text="Status: scanning...")
        self.log("Scanning for BLE devices...")
        threading.Thread(target=self._scan_bg, daemon=True).start()

    def _scan_bg(self):
        # Increase discover timeout on some platforms (Windows advertising can be slow)
        devices = self.loop.run_until_complete(BleakScanner.discover(timeout=10.0))
        self.found_devices = devices
        self.devices_listbox.delete(0, tk.END)
        for d in devices:
            # RSSI may not be present on all platforms/bleak versions; fall back safely
            rssi = getattr(d, 'rssi', None)
            if rssi is None:
                # try metadata (some backends put RSSI there)
                try:
                    rssi = d.metadata.get('rssi') if d.metadata else None
                except Exception:
                    rssi = None
            rssi_str = f" RSSI={rssi}" if rssi is not None else ""
            name = d.name or '<unknown>'
            entry = f"{name} [{d.address}]{rssi_str}"
            self.devices_listbox.insert(tk.END, entry)
        # also log a short summary
        self.log(f"Scan complete: {len(devices)} device(s) found")
        self.status.config(text=f"Status: found {len(devices)} device(s)")

    def connect(self):
        # If a device is selected in the list, connect to it. Otherwise try to find the
        # device by TARGET_NAME as a fallback.
        sel = self.devices_listbox.curselection()
        if sel:
            idx = sel[0]
            addr = self.found_devices[idx].address
        else:
            addr = None
            for d in self.found_devices:
                if d.name and TARGET_NAME in d.name:
                    addr = d.address
                    break
        if not addr:
            messagebox.showwarning("Not found", f"No device selected and no device named {TARGET_NAME} found. Please scan and select or use address.")
            return
        self.status.config(text=f"Status: connecting to {addr}...")
        threading.Thread(target=self._connect_bg, args=(addr,), daemon=True).start()

    def connect_by_address(self):
        addr = self.addr_entry.get().strip()
        if not addr:
            messagebox.showwarning("No address", "Please enter a BLE address (e.g. AA:BB:CC:DD:EE:FF)")
            return
        self.status.config(text=f"Status: connecting to {addr}...")
        threading.Thread(target=self._connect_bg, args=(addr,), daemon=True).start()

    def _connect_bg(self, address):
        try:
            client = BleakClient(address)
            ok = self.loop.run_until_complete(client.connect())
            if ok:
                self.client = client
                self.status.config(text=f"Status: connected to {address}")
                self.disconnect_btn.config(state=tk.NORMAL)
                self.read_batt_btn.config(state=tk.NORMAL)
                self.log(f"Connected to {address}")
                return
            else:
                self.log("Connect failed: connect() returned False")
        except Exception as e:
            # Provide richer diagnostics for troubleshooting
            self.log("Connect failed:", type(e).__name__, str(e))
            try:
                self.log(traceback.format_exc())
            except Exception:
                pass

        # Fallback: try to locate the device object by address and connect using that.
        try:
            self.log("Attempting fallback: finding device by address...")
            # Increase timeout for fallback lookup on Windows; BLE advertising or scanning
            # can take longer, especially if the device is in low-power or intermittent mode.
            dev = self.loop.run_until_complete(BleakScanner.find_device_by_address(address, timeout=10.0))
            if dev is None:
                self.log(f"Fallback failed: Device with address {address} was not found.")
                # Helpful Windows-specific hint
                self.log("On Windows, ensure the device is disconnected/removed from system Bluetooth (Settings → Bluetooth & devices → Devices) before connecting from this app.")
                self.log(f"Platform: {platform.system()} {platform.release()}")
                self.status.config(text="Status: connect failed")
                return
            self.log(f"Found device object; attempting connect to {dev.address} ({dev.name})")
            client2 = BleakClient(dev)
            ok2 = self.loop.run_until_complete(client2.connect())
            if ok2:
                self.client = client2
                self.status.config(text=f"Status: connected to {dev.address}")
                self.disconnect_btn.config(state=tk.NORMAL)
                self.read_batt_btn.config(state=tk.NORMAL)
                self.log(f"Connected to {dev.address}")
                return
            else:
                self.log("Fallback connect() returned False")
        except Exception as e:
            self.log("Fallback connect failed:", type(e).__name__, str(e))
            try:
                self.log(traceback.format_exc())
            except Exception:
                pass

        self.status.config(text="Status: connect failed")

    def disconnect(self):
        if not self.client:
            return
        try:
            self.loop.run_until_complete(self.client.__aexit__(None, None, None))
        except Exception:
            pass
        self.client = None
        self.disconnect_btn.config(state=tk.DISABLED)
        self.read_batt_btn.config(state=tk.DISABLED)
        self.status.config(text="Status: disconnected")
        self.log("Disconnected")

    def read_battery(self):
        if not self.client:
            messagebox.showinfo("Not connected", "No device connected")
            return
        threading.Thread(target=self._read_batt_bg, daemon=True).start()

    def _read_batt_bg(self):
        try:
            data = self.loop.run_until_complete(self.client.read_gatt_char(BATTERY_CHAR_UUID))
            if data:
                level = int(data[0])
                self.log(f"Battery level: {level}%")
            else:
                self.log("Battery read returned no data")
        except Exception as e:
            self.log("Battery read failed:", e)

    def dump_diagnostics(self):
        """Start a background diagnostics dump: versions, platform, discovered devices.
        Writes results to desktop_app/diagnostics.txt and the GUI log."""
        threading.Thread(target=self._dump_diag_bg, daemon=True).start()

    def _dump_diag_bg(self):
        out_lines = []
        try:
            out_lines.append(f"Platform: {platform.platform()}")
            # Avoid backslash inside f-string expression (can raise SyntaxError in some contexts)
            pyver = sys.version.replace("\n", " ")
            out_lines.append(f"Python: {pyver}")
            try:
                import bleak
                bver = getattr(bleak, '__version__', 'unknown')
            except Exception:
                bver = 'not-installed'
            out_lines.append(f"bleak: {bver}")

            self.log("Starting diagnostics scan (8s)...")
            # keep scan a bit longer for diagnostics
            devices = self.loop.run_until_complete(BleakScanner.discover(timeout=8.0))
            out_lines.append(f"Scan found {len(devices)} device(s)")
            for d in devices:
                rssi = getattr(d, 'rssi', None)
                try:
                    meta = d.metadata if hasattr(d, 'metadata') else None
                except Exception:
                    meta = None
                out_lines.append(f"- {d.name or '<unknown>'} [{d.address}] RSSI={rssi} metadata={meta}")

        except Exception as e:
            out_lines.append(f"Diagnostics failed: {type(e).__name__} {e}")
            try:
                out_lines.append(traceback.format_exc())
            except Exception:
                pass

        # write to diagnostics file next to this script and log to GUI
        try:
            base = os.path.dirname(__file__)
            fn = os.path.join(base, 'diagnostics.txt')
            with open(fn, 'w', encoding='utf-8') as f:
                for l in out_lines:
                    f.write(l + "\n")
            self.log(f"Diagnostics written to: {fn}")
        except Exception as e:
            self.log("Failed to write diagnostics file:", type(e).__name__, str(e))

        for l in out_lines:
            self.log(l)

    def toggle_wifi_listener(self):
        if not PYNPUT_AVAILABLE:
            messagebox.showerror("Missing dependency", "pynput is required for Wi‑Fi mouse mode. Install requirements and restart the app.")
            return
        if self.udp_thread and self.udp_thread.is_alive():
            self.log("Stopping Wi‑Fi mouse listener...")
            self.udp_stop_event.set()
            self.wifi_listen_btn.config(text="Start Wi‑Fi mouse listener")
        else:
            self.udp_stop_event.clear()
            self.udp_thread = threading.Thread(target=self._udp_listener_bg, daemon=True)
            self.udp_thread.start()
            self.wifi_listen_btn.config(text="Stop Wi‑Fi mouse listener")
            self.log(f"Started UDP listener on port {self.udp_port}")

    def apply_port(self):
        try:
            new_port = int(self.port_var.get())
            if not (1 <= new_port <= 65535):
                raise ValueError()
        except Exception:
            messagebox.showerror("Invalid port", "Please enter a valid UDP port (1-65535).")
            return
        restart = self.udp_thread and self.udp_thread.is_alive()
        if restart:
            self.toggle_wifi_listener()  # stop
        self.udp_port = new_port
        self._save_settings()
        if restart:
            self.toggle_wifi_listener()  # start

    def _on_recenter_toggle(self):
        self.recenter_edges = bool(self.recenter_var.get())
        self._save_settings()

    def _init_virtual_desktop_bounds(self, root):
        # Determine virtual desktop rectangle across all monitors (Windows)
        try:
            user32 = ctypes.windll.user32
            # SM_XVIRTUALSCREEN=76, SM_YVIRTUALSCREEN=77, SM_CXVIRTUALSCREEN=78, SM_CYVIRTUALSCREEN=79
            vx = int(user32.GetSystemMetrics(76))
            vy = int(user32.GetSystemMetrics(77))
            vw = int(user32.GetSystemMetrics(78))
            vh = int(user32.GetSystemMetrics(79))
            if vw <= 0 or vh <= 0:
                raise RuntimeError("invalid virtual metrics")
            self.vx, self.vy, self.vw, self.vh = vx, vy, vw, vh
        except Exception:
            # Fallback to primary screen only
            try:
                self.vx, self.vy = 0, 0
                self.vw = root.winfo_screenwidth()
                self.vh = root.winfo_screenheight()
            except Exception:
                self.vx, self.vy, self.vw, self.vh = 0, 0, 1920, 1080

    def _udp_listener_bg(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind(("0.0.0.0", self.udp_port))
        except Exception as e:
            self.log("UDP bind failed:", type(e).__name__, e)
            return
        sock.settimeout(1.0)
        last_seq = None
        while not self.udp_stop_event.is_set():
            try:
                data, addr = sock.recvfrom(64)
            except socket.timeout:
                continue
            except Exception as e:
                self.log("UDP receive error:", e)
                break
            # Battery packet (0xB0) can be shorter; handle first to avoid length gating
            if data[0] == 0xB0:
                # Expect at least 4 bytes after header+seq for millivolts; legacy firmware used 4-byte packet total (header, seq, mvL, mvH)
                if len(data) >= 4:
                    mv = data[2] | (data[3] << 8)
                    volts = mv / 1000.0
                    fw_percent = None
                    # New firmware adds percent as 5th byte
                    if len(data) >= 5:
                        fw_percent = data[4]
                    if fw_percent is not None:
                        pct = max(0, min(100, fw_percent))
                        self.batt_var.set(f"{volts:.2f}V {pct:3d}%")
                        self.batt_progress['value'] = pct
                        self.log(f"Battery packet: {volts:.2f}V ({pct}%) [fw]")
                    else:
                        # Fallback estimation (typical LiPo 3.3V empty to 4.2V full)
                        est = (volts - 3.3) / (4.2 - 3.3)
                        est = max(0.0, min(1.0, est))
                        self.batt_var.set(f"{volts:.2f}V")
                        self.batt_progress['value'] = est * 100.0
                        self.log(f"Battery packet: {volts:.2f}V (~{est*100:.0f}%) [est]")
                continue
            # From here on we need a full mouse packet length
            if not data or len(data) < 6:
                continue
            if data[0] != 0xA5:
                continue
            seq = data[1]
            dx = int.from_bytes(data[2:3], byteorder='little', signed=True)
            dy = int.from_bytes(data[3:4], byteorder='little', signed=True)
            buttons = data[4]
            self.log(f"UDP pkt from {addr[0]} seq={seq} dx={dx} dy={dy} btns=0x{buttons:02x}")
            self.packet_count += 1
            self.last_packet_at = datetime.now()
            self.last_sender = addr[0]
            # apply mouse movement
            try:
                if dx != 0 or dy != 0:
                    # Multi-monitor aware edge handling
                    try:
                        x, y = self.mouse.position
                        vx = getattr(self, 'vx', 0)
                        vy = getattr(self, 'vy', 0)
                        vw = getattr(self, 'vw', 1920)
                        vh = getattr(self, 'vh', 1080)
                        nx = x + dx
                        ny = y + dy
                        # Hard clamp within virtual desktop
                        clamped_x = max(vx, min(vx + vw - 1, nx))
                        clamped_y = max(vy, min(vy + vh - 1, ny))
                        eff_dx = int(clamped_x - x)
                        eff_dy = int(clamped_y - y)
                        if self.recenter_edges and (nx <= vx + self.edge_margin or nx >= vx + vw - 1 - self.edge_margin or ny <= vy + self.edge_margin or ny >= vy + vh - 1 - self.edge_margin):
                            # Recenter strategy: jump back toward center to preserve relative motion continuity
                            center_x = vx + vw // 2
                            center_y = vy + vh // 2
                            self.mouse.position = (center_x, center_y)
                        else:
                            if eff_dx != 0 or eff_dy != 0:
                                self.mouse.move(eff_dx, eff_dy)
                    except Exception:
                        self.mouse.move(dx, dy)
                # Button handling:
                # - bit0 (left): treat as stateful press/release (supports drag)
                # - bit1 (right): treat as click on rising edge
                # - bit2 (back): treat as click on rising edge (mapped to middle click)
                left_state = bool(buttons & 0x01)
                if left_state != self.left_down:
                    if left_state:
                        self.mouse.press(Button.left)
                    else:
                        self.mouse.release(Button.left)
                    self.left_down = left_state

                # Edge detection for right/back to avoid repeats
                prev = self._prev_udp_buttons
                if (buttons & 0x02) and not (prev & 0x02):
                    self.mouse.press(Button.right); self.mouse.release(Button.right)
                if (buttons & 0x04) and not (prev & 0x04):
                    self.mouse.press(Button.middle); self.mouse.release(Button.middle)
                self._prev_udp_buttons = buttons
            except Exception as e:
                self.log("Mouse injection failed:", e)
        try:
            sock.close()
        except Exception:
            pass
        self.log("UDP listener stopped")

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
        if platform.system() != 'Windows':
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

    def _update_status_loop(self):
        ssid = self._windows_ssid()
        ip = self._primary_ip()
        listening = (self.udp_thread and self.udp_thread.is_alive())
        last = self.last_packet_at.strftime('%H:%M:%S') if self.last_packet_at else '—'
        sender = self.last_sender or '—'
        info = [
            f"Listener: {'ON' if listening else 'OFF'} (UDP {self.udp_port})",
            f"SSID: {ssid or 'unknown'}",
            f"Local IP: {ip or 'unknown'}",
            f"Packets: {self.packet_count}  Last @ {last} from {sender}",
        ]
        self.netinfo_label.config(text='  '.join(info))
        self.root.after(1000, self._update_status_loop)

    def _cfg_path(self):
        try:
            base = os.path.dirname(__file__)
        except Exception:
            base = '.'
        return os.path.join(base, 'config.json')

    def _load_settings(self):
        try:
            with open(self._cfg_path(), 'r', encoding='utf-8') as f:
                data = json.load(f)
            self.udp_port = int(data.get('udp_port', self.udp_port))
            self.recenter_edges = bool(data.get('recenter_edges', getattr(self, 'recenter_edges', False)))
        except Exception:
            pass

    def _save_settings(self):
        try:
            data = { 'udp_port': self.udp_port, 'recenter_edges': bool(getattr(self, 'recenter_edges', False)) }
            with open(self._cfg_path(), 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
        except Exception:
            pass


if __name__ == '__main__':
    root = tk.Tk()
    app = App(root)
    # Clean shutdown: stop UDP thread
    def _on_close():
        try:
            if app.udp_thread and app.udp_thread.is_alive():
                app.udp_stop_event.set()
        except Exception:
            pass
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", _on_close)
    root.mainloop()
