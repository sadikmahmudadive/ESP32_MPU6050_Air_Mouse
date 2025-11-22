#!/usr/bin/env python3
"""
Train and export gesture models for ESP32 Air Mouse.

Usage examples:
  python train.py --data-dir ml_logs --out-dir . --model-type rf
  python train.py --data-dir ml_logs --out-dir . --model-type tf --export-tflite gesture_model_int8.tflite --quantize

The script looks for CSV files under the data dir with columns:
  timestamp_ms,ax,ay,az,gx,gy,gz,label

It extracts time-domain and simple spectral features, trains a RandomForest by default,
plots a confusion matrix, and optionally trains a small TF model and exports TFLite.

"""

import argparse
import os
import glob
import json
import math
from pathlib import Path
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
import matplotlib.pyplot as plt
from scipy.signal import welch
import joblib

# Optional TF imports (lazy)
try:
    import tensorflow as tf
    TF_AVAILABLE = True
except Exception:
    TF_AVAILABLE = False

# Default sampling Hz; will try to read ml/gesture_meta.json
META_PATH = Path(__file__).with_name('gesture_meta.json')
DEFAULT_HZ = 100.0

# Frequency bands for energy computation (Hz)
BANDS = [(0.5,2),(2,5),(5,10),(10,20)]

# --- Augmentation utilities ---
def jitter(df, sigma=0.01):
    """Add Gaussian noise to accel & gyro columns (fractional relative noise).

    df: DataFrame with ax,ay,az,gx,gy,gz
    sigma: standard deviation as fraction of signal std
    """
    out = df.copy()
    for c in ['ax','ay','az','gx','gy','gz']:
        if c in out.columns:
            arr = out[c].astype(float).values
            noise = np.random.normal(0.0, max(1e-8, sigma * np.std(arr)), size=arr.shape)
            out[c] = arr + noise
    return out

def scale_time(df, accel_scale=1.0, gyro_scale=1.0):
    out = df.copy()
    for c in ['ax','ay','az']:
        if c in out.columns:
            out[c] = out[c].astype(float) * accel_scale
    for c in ['gx','gy','gz']:
        if c in out.columns:
            out[c] = out[c].astype(float) * gyro_scale
    return out

def time_warp(df, rate=1.0):
    """Speed up/slow down the sequence by simple linear interpolation.

    rate > 1.0 -> speed up (shorter), rate < 1.0 -> slow down (longer)
    """
    if rate == 1.0:
        return df.copy()
    n = len(df)
    if n < 3:
        return df.copy()
    old_idx = np.arange(n)
    new_n = max(3, int(n / rate))
    new_idx = np.linspace(0, n-1, new_n)
    out = pd.DataFrame()
    # Only interpolate numeric sensor/time columns; preserve non-numeric (e.g. label)
    numeric_cols = [c for c in ['timestamp_ms','ax','ay','az','gx','gy','gz'] if c in df.columns]
    for col in numeric_cols:
        out[col] = np.interp(new_idx, old_idx, df[col].astype(float).values)
    # Preserve non-numeric columns by repeating their first value (e.g. label)
    for col in df.columns:
        if col not in numeric_cols:
            out[col] = [df[col].iloc[0]] * len(out)
    return out

def random_crop(df, min_frac=0.6):
    """Randomly crop a contiguous sub-window and then resample back to original length."""
    n = len(df)
    if n < 4:
        return df.copy()
    min_len = max(2, int(n * min_frac))
    start = np.random.randint(0, n - min_len + 1)
    end = start + np.random.randint(min_len, n - start + 1)
    crop = df.iloc[start:end]
    # resample back to n using interpolation
    old_idx = np.linspace(0, 1, len(crop))
    new_idx = np.linspace(0, 1, n)
    # Interpolate only numeric columns
    numeric_cols = [c for c in ['timestamp_ms','ax','ay','az','gx','gy','gz'] if c in df.columns]
    out = pd.DataFrame()
    for c in numeric_cols:
        out[c] = np.interp(new_idx, old_idx, crop[c].astype(float).values)
    # Preserve non-numeric columns by repeating their first value (e.g. label)
    for c in df.columns:
        if c not in numeric_cols:
            out[c] = [df[c].iloc[0]] * n
    return out

def augment_segment(df, n_augment=4):
    """Produce a list of augmented DataFrames from a single segment."""
    res = []
    for i in range(n_augment):
        choice = np.random.choice(['jitter','scale','timewarp','crop','combo'], p=[0.25,0.2,0.2,0.2,0.15])
        if choice == 'jitter':
            out = jitter(df, sigma=0.02 * (1 + np.random.rand()))
        elif choice == 'scale':
            a_s = 1.0 + (np.random.rand() - 0.5) * 0.2
            g_s = 1.0 + (np.random.rand() - 0.5) * 0.2
            out = scale_time(df, accel_scale=a_s, gyro_scale=g_s)
        elif choice == 'timewarp':
            rate = 0.9 + np.random.rand() * 0.3
            out = time_warp(df, rate=rate)
        elif choice == 'crop':
            out = random_crop(df, min_frac=0.6 + np.random.rand()*0.3)
        else:
            # combo: jitter then small time warp
            out = jitter(df, sigma=0.015)
            out = time_warp(out, rate=0.95 + np.random.rand()*0.1)
        # Ensure label column is preserved if present in original
        if 'label' in df.columns and 'label' in out.columns:
            pass
        elif 'label' in df.columns:
            out['label'] = df['label'].iloc[0]
        res.append(out)
    return res



def load_meta():
    if META_PATH.exists():
        try:
            with open(META_PATH, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return None
    return None


def list_csv_files(data_dir):
    files = glob.glob(os.path.join(data_dir, '*.csv'))
    return files


def extract_features_from_segment(df, sampling_hz=100.0):
    # Expect columns: timestamp_ms,ax,ay,az,gx,gy,gz,label (label optional here)
    # Compute features per axis: mean, std, min, max, median, p25, p75
    axes = ['ax','ay','az','gx','gy','gz']
    feats = {}
    for a in axes:
        if a not in df.columns:
            feats.update({f'{a}_mean':0,'{a}_std':0})
            continue
        arr = df[a].values.astype(float)
        feats[f'{a}_mean'] = np.mean(arr)
        feats[f'{a}_std'] = np.std(arr)
        feats[f'{a}_min'] = np.min(arr)
        feats[f'{a}_max'] = np.max(arr)
        feats[f'{a}_median'] = np.median(arr)
        feats[f'{a}_p25'] = np.percentile(arr,25)
        feats[f'{a}_p75'] = np.percentile(arr,75)
    # magnitude features
    a_mag = np.sqrt(df['ax'].astype(float)**2 + df['ay'].astype(float)**2 + df['az'].astype(float)**2)
    g_mag = np.sqrt(df['gx'].astype(float)**2 + df['gy'].astype(float)**2 + df['gz'].astype(float)**2)
    for name,arr in [('a_mag',a_mag),('g_mag',g_mag)]:
        feats[f'{name}_mean'] = np.mean(arr)
        feats[f'{name}_std'] = np.std(arr)
        feats[f'{name}_min'] = np.min(arr)
        feats[f'{name}_max'] = np.max(arr)
        feats[f'{name}_median'] = np.median(arr)
        feats[f'{name}_p25'] = np.percentile(arr,25)
        feats[f'{name}_p75'] = np.percentile(arr,75)
    # band energies on a_mag
    try:
        f, Pxx = welch(a_mag, fs=sampling_hz, nperseg=min(256, len(a_mag)))
        for (lo,hi) in BANDS:
            mask = (f >= lo) & (f < hi)
            energy = np.sum(Pxx[mask]) if np.any(mask) else 0.0
            feats[f'a_mag_energy_{int(lo)}_{int(hi)}'] = energy
    except Exception:
        for (lo,hi) in BANDS:
            feats[f'a_mag_energy_{int(lo)}_{int(hi)}'] = 0.0
    return feats


def load_dataset(data_dir, sampling_hz=100.0):
    files = list_csv_files(data_dir)
    X = []
    y = []
    names = []
    for fn in files:
        try:
            df = pd.read_csv(fn)
        except Exception:
            continue
        if df.shape[0] < 3:
            continue
        # determine label
        lbl = None
        if 'label' in df.columns:
            lbls = df['label'].dropna().unique()
            if len(lbls) >= 1:
                lbl = str(lbls[0])
        # fallback to filename label parsing
        if not lbl:
            lbl = Path(fn).stem.split('_')[0]
        feats = extract_features_from_segment(df, sampling_hz=sampling_hz)
        X.append(feats)
        y.append(lbl)
        names.append(fn)
    if not X:
        raise RuntimeError(f"No valid CSV segments found in {data_dir}")
    Xdf = pd.DataFrame(X)
    return Xdf, np.array(y), names


def load_raw_segments(data_dir):
    """Return list of tuples (df,label,filename) for each raw CSV segment."""
    files = list_csv_files(data_dir)
    res = []
    for fn in files:
        try:
            df = pd.read_csv(fn)
        except Exception:
            continue
        if df.shape[0] < 3:
            continue
        lbl = None
        if 'label' in df.columns:
            lbls = df['label'].dropna().unique()
            if len(lbls) >= 1:
                lbl = str(lbls[0])
        if not lbl:
            lbl = Path(fn).stem.split('_')[0]
        res.append((df, lbl, fn))
    return res


def train_random_forest(X, y, out_dir, n_jobs=4):
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, stratify=y)
    clf = RandomForestClassifier(n_estimators=200, random_state=42, n_jobs=n_jobs)
    clf.fit(X_train, y_train)
    y_pred = clf.predict(X_test)
    print("RandomForest Classification Report:\n", classification_report(y_test, y_pred))
    cm = confusion_matrix(y_test, y_pred, labels=clf.classes_)
    plt.figure(figsize=(6,6))
    plt.imshow(cm, interpolation='nearest', cmap=plt.cm.Blues)
    plt.title('Confusion matrix (RandomForest)')
    plt.colorbar()
    ticks = np.arange(len(clf.classes_))
    plt.xticks(ticks, clf.classes_, rotation=45)
    plt.yticks(ticks, clf.classes_)
    plt.xlabel('Predicted'); plt.ylabel('True')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'confusion_rf.png'))
    # feature importances
    try:
        fi = clf.feature_importances_
        idx = np.argsort(fi)[-20:][::-1]
        names = X.columns.values
        plt.figure(figsize=(8,4))
        plt.barh(np.arange(len(idx)), fi[idx][::-1])
        plt.yticks(np.arange(len(idx)), names[idx][::-1])
        plt.title('Top feature importances (RF)')
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, 'feature_importances_rf.png'))
    except Exception:
        pass
    # Save model
    joblib.dump(clf, os.path.join(out_dir, 'rf_model.joblib'))
    print(f"Saved RandomForest model to {out_dir}/rf_model.joblib")
    return clf


# TensorFlow model helpers (optional)

def build_tf_model(input_dim, n_classes):
    model = tf.keras.Sequential([
        tf.keras.layers.InputLayer(input_shape=(input_dim,)),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(64, activation='relu'),
        tf.keras.layers.Dense(n_classes, activation='softmax')
    ])
    model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])
    return model


def train_tf_model(X, y, out_dir, epochs=50, batch_size=32):
    X_train, X_test, y_train, y_test = train_test_split(X.values.astype(np.float32), y, test_size=0.2, random_state=42, stratify=y)
    # encode labels
    classes, y_train_enc = np.unique(y_train, return_inverse=True)
    _, y_test_enc = np.unique(y_test, return_inverse=True)
    model = build_tf_model(X.shape[1], len(classes))
    model.fit(X_train, y_train_enc, validation_split=0.1, epochs=epochs, batch_size=batch_size)
    loss, acc = model.evaluate(X_test, y_test_enc)
    print(f"TF model test acc: {acc:.4f}")
    model.save(os.path.join(out_dir, 'tf_model'))
    print(f"Saved TF model to {out_dir}/tf_model")
    return model, classes, X_train


def export_tflite_from_tf(model_dir, out_path, representative_data=None, quantize=False):
    converter = tf.lite.TFLiteConverter.from_saved_model(model_dir)
    if quantize:
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        if representative_data is not None:
            converter.representative_dataset = lambda: (np.expand_dims(x,0) for x in representative_data)
        converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        converter.inference_input_type = tf.int8
        converter.inference_output_type = tf.int8
    tflite_model = converter.convert()
    with open(out_path, 'wb') as f:
        f.write(tflite_model)
    print(f"Wrote TFLite to {out_path}")
    return out_path


def c_header_from_file(bin_path, header_path, array_name='gesture_model_int8'):
    b = Path(bin_path).read_bytes()
    with open(header_path, 'w', encoding='utf-8') as f:
        f.write('#pragma once\n')
        f.write('#include <cstdint>\n\n')
        f.write(f'const unsigned int {array_name}_len = {len(b)};\n')
        f.write(f'const unsigned char {array_name}[] = {{\n')
        for i in range(0, len(b), 12):
            chunk = b[i:i+12]
            f.write('    ' + ', '.join(str(x) for x in chunk) + ',\n')
        f.write('};\n')
    print(f"Wrote C header to {header_path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--data-dir', default='ml_logs', help='Directory with CSV segment files')
    p.add_argument('--out-dir', default='.', help='Output directory for models and plots')
    p.add_argument('--model-type', choices=['rf','tf','both'], default='rf')
    p.add_argument('--export-tflite', default='', help='TFLite output filename (optional, requires TF)')
    p.add_argument('--quantize', action='store_true', help='If using TF, apply int8 quantization during TFLite export')
    p.add_argument('--augment', action='store_true', help='Apply data augmentation to raw segments before feature extraction')
    p.add_argument('--augment-per-sample', type=int, default=4, help='Number of augmented variants to produce per original sample')
    p.add_argument('--sampling-hz', type=float, default=0.0, help='Override sampling Hz (auto from gesture_meta.json if available)')
    args = p.parse_args()

    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)

    meta = load_meta()
    sampling_hz = args.sampling_hz if args.sampling_hz and args.sampling_hz>0 else (meta.get('sampling_hz') if meta else DEFAULT_HZ)
    try:
        sampling_hz = float(sampling_hz)
    except Exception:
        sampling_hz = DEFAULT_HZ

    print(f"Loading segments from {args.data_dir} (sampling_hz={sampling_hz})")
    if args.augment:
        raws = load_raw_segments(args.data_dir)
        X_list = []
        y_list = []
        for df, lbl, fn in raws:
            # original
            feats = extract_features_from_segment(df, sampling_hz=sampling_hz)
            X_list.append(feats)
            y_list.append(lbl)
            # augmented variants
            aug_list = augment_segment(df, n_augment=args.augment_per_sample)
            for a in aug_list:
                feats_a = extract_features_from_segment(a, sampling_hz=sampling_hz)
                X_list.append(feats_a)
                y_list.append(lbl)
        if not X_list:
            raise RuntimeError(f"No valid CSV segments found in {args.data_dir}")
        Xdf = pd.DataFrame(X_list)
        y = np.array(y_list)
        names = None
        print(f"Loaded {len(y)} (including augmentations), feature dim={Xdf.shape[1]}")
    else:
        Xdf, y, names = load_dataset(args.data_dir, sampling_hz=sampling_hz)
        print(f"Loaded {len(y)} segments, feature dim={Xdf.shape[1]}")

    # Simple label encoding for TF
    if args.model_type in ('rf','both'):
        rf = train_random_forest(Xdf, y, out_dir)
    else:
        rf = None

    if args.model_type in ('tf','both'):
        if not TF_AVAILABLE:
            print('TensorFlow not available; skipping TF model')
        else:
            model, classes, X_train = train_tf_model(Xdf, y, out_dir)
            if args.export_tflite:
                rep_samples = X_train[:100] if X_train is not None and X_train.shape[0] > 0 else None
                export_tflite_from_tf(os.path.join(out_dir,'tf_model'), args.export_tflite, representative_data=rep_samples, quantize=args.quantize)
                if args.quantize and args.export_tflite:
                    # also write C header
                    header_out = os.path.join(out_dir, Path(args.export_tflite).stem + '.h')
                    c_header_from_file(args.export_tflite, header_out)

    # If only RF model and user requested tflite, notify
    if rf is not None and args.export_tflite and TF_AVAILABLE:
        print('Note: to produce a TFLite model you must train a TF model (use --model-type tf or both).')


if __name__ == '__main__':
    main()
