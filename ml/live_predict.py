#!/usr/bin/env python3
"""
Listen for raw IMU UDP packets (header 0xD0) and run live predictions using
an exported TFLite model or a saved sklearn model.

Usage:
  python live_predict.py --tflite model.tflite
  python live_predict.py --sklearn rf_model.joblib

The script collects a sliding window of samples (default 100 samples) and
computes the same features used by `train.py` to make predictions.
"""

import argparse
import socket
import struct
import time
import numpy as np
import pandas as pd
import joblib
from pathlib import Path

try:
    import tflite_runtime.interpreter as tflite_rt
    TFLITE_RUNTIME = True
except Exception:
    try:
        import tensorflow as tf
        TFLITE_RUNTIME = False
    except Exception:
        tf = None

# feature extraction reused (simpler copy)
from scipy.signal import welch

BANDS = [(0.5,2),(2,5),(5,10),(10,20)]


def extract_features_from_buffer(buf_df, sampling_hz=100.0):
    # buf_df: DataFrame with columns ax,ay,az,gx,gy,gz
    axes = ['ax','ay','az','gx','gy','gz']
    feats = {}
    for a in axes:
        arr = buf_df[a].values.astype(float)
        feats[f'{a}_mean'] = np.mean(arr)
        feats[f'{a}_std'] = np.std(arr)
        feats[f'{a}_min'] = np.min(arr)
        feats[f'{a}_max'] = np.max(arr)
        feats[f'{a}_median'] = np.median(arr)
        feats[f'{a}_p25'] = np.percentile(arr,25)
        feats[f'{a}_p75'] = np.percentile(arr,75)
    a_mag = np.sqrt(buf_df['ax']**2 + buf_df['ay']**2 + buf_df['az']**2)
    for name,arr in [('a_mag',a_mag)]:
        feats[f'{name}_mean'] = np.mean(arr)
        feats[f'{name}_std'] = np.std(arr)
        feats[f'{name}_min'] = np.min(arr)
        feats[f'{name}_max'] = np.max(arr)
        feats[f'{name}_median'] = np.median(arr)
        feats[f'{name}_p25'] = np.percentile(arr,25)
        feats[f'{name}_p75'] = np.percentile(arr,75)
    try:
        f, Pxx = welch(a_mag, fs=sampling_hz, nperseg=min(256, len(a_mag)))
        for (lo,hi) in BANDS:
            mask = (f >= lo) & (f < hi)
            energy = np.sum(Pxx[mask]) if np.any(mask) else 0.0
            feats[f'a_mag_energy_{int(lo)}_{int(hi)}'] = energy
    except Exception:
        for (lo,hi) in BANDS:
            feats[f'a_mag_energy_{int(lo)}_{int(hi)}'] = 0.0
    return pd.DataFrame([feats])


def load_tflite_model(path):
    if 'tf' in globals() and tf is not None:
        interpreter = tf.lite.Interpreter(model_path=path)
        interpreter.allocate_tensors()
        return ('tf', interpreter)
    else:
        # try tflite_runtime
        import tflite_runtime.interpreter as tflite_rt
        interpreter = tflite_rt.Interpreter(model_path=path)
        interpreter.allocate_tensors()
        return ('rt', interpreter)


def predict_tflite(interp, X):
    input_details = interp.get_input_details()
    output_details = interp.get_output_details()
    inp = X.astype(np.float32)
    # reshape if needed
    inp = inp.reshape((1, inp.shape[1]))
    interp.set_tensor(input_details[0]['index'], inp)
    interp.invoke()
    out = interp.get_tensor(output_details[0]['index'])
    # softmax/class probabilities
    if out.ndim == 2:
        idx = np.argmax(out[0])
        return idx, out[0]
    else:
        return None, out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--tflite', help='TFLite model path')
    p.add_argument('--sklearn', help='Sklearn joblib model path')
    p.add_argument('--port', type=int, default=5555, help='UDP listen port')
    p.add_argument('--window', type=int, default=100, help='Sliding window samples')
    args = p.parse_args()

    model_type = None
    tflite_model = None
    sk_model = None
    classes = None

    if args.tflite:
        tflite_model = load_tflite_model(args.tflite)
        model_type = 'tflite'
    elif args.sklearn:
        sk_model = joblib.load(args.sklearn)
        model_type = 'sklearn'
        if hasattr(sk_model, 'classes_'):
            classes = sk_model.classes_

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(1.0)

    print(f"Listening for IMU packets on UDP {args.port} (expect header 0xD0). Window={args.window}")
    # buffer for samples
    buf = []
    lastpred = None
    while True:
        try:
            data, addr = sock.recvfrom(256)
        except Exception:
            continue
        if not data:
            continue
        # Expect header 0xD0, seq, ax(lo,hi), ay, az, gx, gy, gz (int16 little-endian)
        if data[0] != 0xD0:
            continue
        if len(data) < 14:
            continue
        # parse int16 little-endian
        def s16(lo, hi):
            v = lo | (hi<<8)
            return v - 65536 if v & 0x8000 else v
        ax = s16(data[2], data[3])
        ay = s16(data[4], data[5])
        az = s16(data[6], data[7])
        gx = s16(data[8], data[9])
        gy = s16(data[10], data[11])
        gz = s16(data[12], data[13])
        # scale back to original units expected by train script (if train used mg & centi-deg/s, adapt accordingly)
        # Here we assume ax,ay,az are mg (accel*1000) and gx,gy,gz are centideg/s (gyro*100)
        # train.py expects accel in same raw units used during collection; if your CSV uses different units, adjust.
        sample = {'ax': ax/1000.0, 'ay': ay/1000.0, 'az': az/1000.0, 'gx': gx/100.0, 'gy': gy/100.0, 'gz': gz/100.0}
        buf.append(sample)
        if len(buf) > args.window:
            buf.pop(0)
        if len(buf) >= args.window:
            df = pd.DataFrame(buf)
            feats_df = extract_features_from_buffer(df)
            X = feats_df.values.astype(np.float32)
            if model_type == 'sklearn' and sk_model is not None:
                pred = sk_model.predict(feats_df.values)
                print(f"Pred: {pred[0]}")
            elif model_type == 'tflite' and tflite_model is not None:
                kind, interp = tflite_model
                # wrapper for both interpreters: both provide get_input_details/get_tensor
                idx, probs = predict_tflite(interp, X)
                print(f"TFLite pred idx={idx} probs={probs}")


if __name__ == '__main__':
    main()
