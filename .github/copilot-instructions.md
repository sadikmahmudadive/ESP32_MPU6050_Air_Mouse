# ESP32 Air Mouse - AI Agent Instructions

## Project Overview
This workspace contains a complete ecosystem for an ESP32-based Air Mouse using an MPU6050 sensor. It spans embedded firmware, a desktop companion application, and a machine learning pipeline for gesture recognition.

## Architecture & Components

### 1. Firmware (`src/`, `platformio.ini`)
- **Framework**: Arduino on ESP32 (PlatformIO).
- **Core Logic**: `src/main.cpp` handles sensor reading (MPU6050), sensor fusion (Tockn or DMP), and communication.
- **Modes**:
  - **Wi-Fi Mode** (Default): Sends UDP packets to a host.
  - **BLE Mode**: Acts as a standard HID mouse (controlled by `USE_BLE_MOUSE` flag in `platformio.ini`).
- **Provisioning**: Uses a custom "Double-Reset Detector" to enter a Captive Portal for Wi-Fi configuration.

### 2. Desktop Application (`desktop_qt/`)
- **Framework**: Python with PySide6 (Qt).
- **Entry Point**: `desktop_qt/app_qt.py`.
- **Role**: Listens for UDP packets from the ESP32, visualizes sensor data (OpenGL), handles "recenter" logic, and logs data for ML training.
- **Legacy**: `desktop_app/` is an older version; prefer `desktop_qt/` for new development.

### 3. Machine Learning (`ml/`)
- **Workflow**: Data Collection -> Training -> Export.
- **Tools**:
  - `train.py`: Trains Random Forest or TensorFlow models from CSV logs.
  - `live_predict.py`: Runs inference on live UDP streams.
- **Integration**: Generates `gesture_model_data.h` / `.tflite` for potential firmware integration (currently inference is primarily off-device or experimental).

## Key Workflows

### Firmware Development
- **Build**: `pio run`
- **Upload**: `pio run -t upload`
- **Monitor**: `pio device monitor`
- **Configuration**: Modify `platformio.ini` build flags (`-DUSE_WIFI_MOUSE=1`) to switch modes.

### Desktop App
- **Run**: `python desktop_qt/app_qt.py`
- **Config**: Settings are stored in `desktop_qt/config.json` (auto-generated).
- **Data Logging**: The app saves CSV logs to `desktop_qt/ml_logs/` for training.

### ML Pipeline
1. **Collect**: Use Desktop App to record gestures.
2. **Train**: `python ml/train.py --data-dir desktop_qt/ml_logs --out-dir ml/out_quick --model-type rf`
3. **Predict**: `python ml/live_predict.py --model ml/out_quick/rf_model.joblib`

## ML Architecture & Firmware Integration

### Model Architectures
- **Random Forest** (Default):
  - Uses `sklearn.ensemble.RandomForestClassifier` (200 estimators).
  - Best for PC-side inference due to size/complexity.
- **TensorFlow / TFLite** (Embedded-ready):
  - Architecture: `Input -> Dense(128, ReLU) -> Dropout(0.2) -> Dense(64, ReLU) -> Dense(Softmax)`.
  - Quantization: Supports INT8 quantization for ESP32 deployment.

### Firmware Integration Strategy
To run inference on the ESP32:
1. **Train & Export**: Run `train.py` with `--model-type tf --export-tflite gesture_model_int8.tflite --quantize`.
2. **Generate Header**: The script automatically creates `gesture_model_data.h` containing the model as a C byte array.
3. **Embed**:
   - Copy `gesture_model_data.h` to `lib/gesture_model/` (or `include/`).
   - Use `TensorFlowLite_ESP32` library to load the model from the byte array.
   - Feed raw IMU data (accelerometer/gyroscope) into the interpreter.

## Protocols & Conventions

### UDP Protocol (Port 5555)
- **Mouse Packet (`0xA5`)**: `[0xA5, seq, dx, dy, buttons, reserved]`
- **Raw IMU Packet (`0xD0`)**: `[0xD0, seq, ax_h, ax_l, ..., gz_l]` (Big Endian int16 for ax, ay, az, gx, gy, gz)

### Code Style
- **C++**: Arduino style, single `main.cpp` for simplicity, use `#ifdef` for feature flags.
- **Python**: PEP 8, use type hinting where helpful.

## Critical Files
- `src/main.cpp`: Main firmware logic.
- `platformio.ini`: Build configuration and library dependencies.
- `desktop_qt/app_qt.py`: Main desktop application logic.
- `ml/train.py`: ML training script.
