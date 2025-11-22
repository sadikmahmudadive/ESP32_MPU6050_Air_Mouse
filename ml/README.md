# ML tooling for ESP32 Air Mouse

This folder contains command-line tools to train, evaluate, and export gesture models used by the ESP32 Air Mouse project.

Files added:

- `train.py` — load CSV segments from `ml_logs/`, extract features, train models (RandomForest; optional TensorFlow dense), evaluate, and export TFLite and C header.
- `live_predict.py` — listen on UDP for raw IMU streaming packets (header `0xD0`) and run live predictions using an exported TFLite model or a saved sklearn model.
- `requirements.txt` — minimal Python dependencies. TensorFlow is optional for TFLite export.

Quick start (create virtualenv, install deps):

```bash
python -m venv .venv
.\.venv\Scripts\activate
python -m pip install -r requirements.txt
# If you want TensorFlow + TFLite export:
# python -m pip install tensorflow
```

Train a Random Forest and evaluate:

```bash
python train.py --data-dir ml_logs --out-dir . --model-type rf
```

Train TF model and export TFLite (requires TensorFlow):

```bash
python train.py --data-dir ml_logs --out-dir . --model-type tf --export-tflite gesture_model_int8.tflite --quantize
```

Run live prediction from device (listens UDP 5555 by default):

```bash
python live_predict.py --tflite model.tflite
```

If you want, I can also update the `gesture_training.ipynb` to call these scripts from notebook cells and add interactive visualizations. Request that if you'd like the notebook updated.