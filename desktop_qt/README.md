# ESP Air Mouse – Qt Desktop App

A PySide6-based version of the desktop companion providing:

- UDP mouse listener (Wi‑Fi mode) with multi-monitor edge handling
- Battery voltage & percent display
- Optional recenter-at-edges behavior
- Status panel with packet counts and network info
- OpenGL Visual tab (motion trace + battery gauge)
- BLE diagnostics tab (scan/connect/disconnect)
- FPS counter overlay (Visual tab)
- Refined 1S LiPo battery percent mapping (matches firmware curve)

## Install

```bash
python -m pip install -r requirements.txt
```

## Run

```bash
python app_qt.py
```

## Shortcuts / Features

- Live zero & axis mappings are handled on firmware side (button combos).
- Recenter at edges can be toggled in the Settings tab.

## Configuration

A `config.json` will be created in this folder storing:

```json
{
  "udp_port": 5555,
  "recenter_edges": true
}
```

## Future Enhancements

- Expand BLE to read battery characteristic & config GATT settings.
- Add adjustable sensitivity sliders and live accel curve preview.
- Persist motion trace metrics to file for analysis.
- Add speed/color encoding for motion trace (partial fade implemented).
