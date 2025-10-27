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
import tkinter as tk
from tkinter import scrolledtext, messagebox

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

        # Scan button
        self.scan_btn = tk.Button(root, text="Scan for devices", command=self.scan)
        self.scan_btn.pack(fill=tk.X)

        # Devices list (selectable) and a small log area
        list_frame = tk.Frame(root)
        list_frame.pack(fill=tk.BOTH, expand=True)
        self.devices_listbox = tk.Listbox(list_frame, height=12)
        self.devices_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        list_scroll = tk.Scrollbar(list_frame, command=self.devices_listbox.yview)
        list_scroll.pack(side=tk.LEFT, fill=tk.Y)
        self.devices_listbox.config(yscrollcommand=list_scroll.set)

        # Small log area
        self.log_box = scrolledtext.ScrolledText(root, height=6)
        self.log_box.pack(fill=tk.BOTH, expand=False)

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
        self.addr_connect_btn.pack(side=tk.LEFT)

        # Control buttons
        self.disconnect_btn = tk.Button(root, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(fill=tk.X)

        self.read_batt_btn = tk.Button(root, text="Read Battery (if available)", command=self.read_battery, state=tk.DISABLED)
        self.read_batt_btn.pack(fill=tk.X)

        self.status = tk.Label(root, text="Status: idle")
        self.status.pack(fill=tk.X)

        self.client = None
        self.loop = asyncio.get_event_loop()
        self.found_devices = []

    def log(self, *args):
        self.log_box.insert(tk.END, ' '.join(map(str,args)) + "\n")
        self.log_box.see(tk.END)

    def scan(self):
        self.status.config(text="Status: scanning...")
        self.log("Scanning for BLE devices...")
        threading.Thread(target=self._scan_bg, daemon=True).start()

    def _scan_bg(self):
        devices = self.loop.run_until_complete(BleakScanner.discover(timeout=5.0))
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
            self.log("Connect failed:", e)

        # Fallback: try to locate the device object by address and connect using that.
        try:
            self.log("Attempting fallback: finding device by address...")
            dev = self.loop.run_until_complete(BleakScanner.find_device_by_address(address, timeout=5.0))
            if dev is None:
                self.log(f"Fallback failed: Device with address {address} was not found.")
                self.log("On Windows, ensure the device is disconnected/removed from system Bluetooth before connecting from this app.")
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
            self.log("Fallback connect failed:", e)

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


if __name__ == '__main__':
    root = tk.Tk()
    app = App(root)
    root.mainloop()
