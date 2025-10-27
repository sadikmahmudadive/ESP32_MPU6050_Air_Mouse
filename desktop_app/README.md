ESP Air 1 — Desktop Companion (prototype)

This is a minimal desktop companion prototype that can scan for a BLE device named "ESP Air 1" and connect to it using Bleak (Python BLE library). It is intended as a starting point: once the firmware exposes custom GATT characteristics (for battery level, sensitivity, gesture enable/disable, etc.), this app can be extended to read/write those settings.

Requirements
- Python 3.8+
- Install dependencies:
  pip install -r requirements.txt

Run
  python app.py

What this prototype does
- Scans for BLE devices and lists those found
- If a device named "ESP Air 1" is found, you can connect to it and view basic information
- The GUI is intentionally minimal (Tkinter) to keep the prototype small and cross-platform

Next steps to integrate fully
1. Add a custom BLE GATT service to the ESP32 firmware exposing characteristics for:
   - Battery level (or use standard Battery Service)
   - Sensitivity (read/write)
   - Gesture enable/disable (read/write)
2. Update `app.py` to read/write those characteristics after connection
3. Add persistence (save user settings locally) and a nicer UI (PyQt or Electron) if desired

If you want I can:
- Add the BLE GATT service implementation to the firmware
- Extend this companion to read/write the exact characteristic UUIDs and provide sliders/buttons in the UI
- Create an Electron app instead of Python if you prefer web tech

