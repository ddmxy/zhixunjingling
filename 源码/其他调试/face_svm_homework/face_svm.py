#!/usr/bin/env python3
"""
SVM face recognition homework demo.
Uses laptop webcam + HOG features + sklearn SVC.

Usage:
  python face_svm.py collect   # capture training images
  python face_svm.py train     # train SVM and print accuracy
  python face_svm.py detect      # real-time recognition
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import joblib
import numpy as np
from sklearn.metrics import classification_report
from sklearn.model_selection import train_test_split
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC

ROOT = Path(__file__).resolve().parent
DATA_DIR = ROOT / "data"
MODEL_DIR = ROOT / "model"
MODEL_PATH = MODEL_DIR / "face_svm.joblib"
CASCADE_PATH = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"

FACE_SIZE = (64, 64)
HOG = cv2.HOGDescriptor(_winSize=FACE_SIZE, _blockSize=(16, 16), _blockStride=(8, 8), _cellSize=(8, 8), _nbins=9)
PERSONS = {
    "1": "person_a",
    "2": "person_b",
    "3": "person_c",
}


def open_camera(index: int = 0) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


def detect_faces(gray: np.ndarray, cascade: cv2.CascadeClassifier) -> list[tuple[int, int, int, int]]:
    faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))
    return [(int(x), int(y), int(w), int(h)) for x, y, w, h in faces]


def extract_features(face_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, FACE_SIZE)
    hog = HOG.compute(resized).flatten()
    hist = cv2.calcHist([resized], [0], None, [32], [0, 256]).flatten()
    hist = hist / (hist.sum() + 1e-6)
    return np.hstack([hog, hist]).astype(np.float32)


def draw_help(frame: np.ndarray, lines: list[str]) -> None:
    y = 28
    for line in lines:
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        y += 28


def collect_samples(camera_index: int) -> None:
    cascade = cv2.CascadeClassifier(CASCADE_PATH)
    cap = open_camera(camera_index)
    counts = {name: len(list((DATA_DIR / name).glob("*.jpg"))) for name in PERSONS.values()}

    print("Collect mode")
    print("Keys: 1/2/3 save face for person_a/b/c | s save unknown | q quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detect_faces(gray, cascade)
        display = frame.copy()

        for x, y, w, h in faces:
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)

        info = [f"{name}: {counts[name]} imgs" for name in PERSONS.values()]
        draw_help(display, ["Collect training data", *info, "1/2/3=save person | s=unknown | q=quit"])
        cv2.imshow("SVM Face Collect", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        if not faces:
            continue

        x, y, w, h = max(faces, key=lambda item: item[2] * item[3])
        face = frame[y : y + h, x : x + w]

        if key in PERSONS:
            label = PERSONS[key]
        elif key == ord("s"):
            label = "unknown"
        else:
            continue

        out_dir = DATA_DIR / label
        out_dir.mkdir(parents=True, exist_ok=True)
        idx = len(list(out_dir.glob("*.jpg")))
        out_path = out_dir / f"{idx:04d}.jpg"
        cv2.imwrite(str(out_path), face)
        counts[label] = idx + 1
        print(f"saved {out_path}")

    cap.release()
    cv2.destroyAllWindows()


def load_dataset() -> tuple[np.ndarray, np.ndarray, list[str]]:
    images: list[np.ndarray] = []
    labels: list[str] = []

    if not DATA_DIR.exists():
        raise RuntimeError(f"No data folder: {DATA_DIR}. Run collect first.")

    for person_dir in sorted(DATA_DIR.iterdir()):
        if not person_dir.is_dir():
            continue
        for img_path in sorted(person_dir.glob("*.jpg")):
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            images.append(extract_features(img))
            labels.append(person_dir.name)

    if len(images) < 10:
        raise RuntimeError("Need at least 10 face images. Run collect and save more samples.")

    classes = sorted(set(labels))
    return np.vstack(images), np.array(labels), classes


def train_model() -> None:
    x, y, classes = load_dataset()
    x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42, stratify=y)

    model = Pipeline(
        [
            ("scaler", StandardScaler()),
            ("svm", SVC(kernel="rbf", C=10.0, gamma="scale", probability=True)),
        ]
    )
    model.fit(x_train, y_train)
    y_pred = model.predict(x_test)

    print("Classes:", classes)
    print(classification_report(y_test, y_pred, zero_division=0))

    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    joblib.dump({"model": model, "classes": classes}, MODEL_PATH)
    print(f"Model saved: {MODEL_PATH}")


def detect_live(camera_index: int) -> None:
    if not MODEL_PATH.exists():
        raise RuntimeError(f"Model not found: {MODEL_PATH}. Run train first.")

    bundle = joblib.load(MODEL_PATH)
    model: Pipeline = bundle["model"]
    cascade = cv2.CascadeClassifier(CASCADE_PATH)
    cap = open_camera(camera_index)

    print("Detect mode | q=quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detect_faces(gray, cascade)

        for x, y, w, h in faces:
            face = frame[y : y + h, x : x + w]
            feat = extract_features(face).reshape(1, -1)
            pred = model.predict(feat)[0]
            if hasattr(model.named_steps["svm"], "predict_proba"):
                prob = float(model.predict_proba(feat).max())
            else:
                prob = 0.0

            color = (0, 255, 0) if pred != "unknown" else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = f"{pred} {prob:.0%}"
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

        draw_help(frame, ["SVM face recognition", "green=known | red=unknown | q=quit"])
        cv2.imshow("SVM Face Detect", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser(description="SVM face recognition homework")
    parser.add_argument("mode", choices=["collect", "train", "detect"], help="run mode")
    parser.add_argument("--camera", type=int, default=0, help="camera index, default 0")
    args = parser.parse_args()

    if args.mode == "collect":
        collect_samples(args.camera)
    elif args.mode == "train":
        train_model()
    else:
        detect_live(args.camera)
    return 0


if __name__ == "__main__":
    sys.exit(main())
