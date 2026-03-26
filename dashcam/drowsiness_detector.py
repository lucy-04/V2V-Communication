#!/usr/bin/env python3
"""
V2V Drowsiness Detection — Smart Dashcam Simulator
====================================================
Captures webcam frames, detects driver face landmarks via dlib's 68-point
predictor, computes Eye Aspect Ratio (EAR), and sends ``DROWSY_ALERT`` over
serial to the ESP32 when the driver's eyes are closed for too long.

Usage:
    # With serial output to ESP32
    python drowsiness_detector.py --port /dev/tty.usbserial-0001 --baud 115200

    # Dry-run (no serial, console only)
    python drowsiness_detector.py --no-serial

    # Custom thresholds
    python drowsiness_detector.py --ear-threshold 0.22 --consec-frames 25
"""

import argparse
import time
import sys

import cv2
import dlib
import numpy as np
from scipy.spatial import distance as dist
import imutils
from imutils import face_utils

# ─────────────────────── Constants ────────────────────────────────────────────
# dlib 68-point landmark indices for left and right eyes
LEFT_EYE_IDX  = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
RIGHT_EYE_IDX = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]

# Colours for overlay (BGR)
GREEN  = (0, 255, 0)
RED    = (0, 0, 255)
YELLOW = (0, 255, 255)


# ─────────────────────── EAR Calculation ──────────────────────────────────────
def eye_aspect_ratio(eye: np.ndarray) -> float:
    """
    Compute the Eye Aspect Ratio (EAR) for a single eye.

    EAR = (||p2-p6|| + ||p3-p5||) / (2 * ||p1-p4||)

    Where p1..p6 are the 6 landmark points of the eye in order.
    A value near 0.3 means open; near 0.15 means closed.

    Args:
        eye: numpy array of shape (6, 2) containing the (x, y) coordinates
             of the 6 eye landmarks.

    Returns:
        The EAR as a float.
    """
    # Vertical distances
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])
    # Horizontal distance
    C = dist.euclidean(eye[0], eye[3])

    ear = (A + B) / (2.0 * C)
    return ear


# ─────────────────────── Serial Wrapper ───────────────────────────────────────
class SerialBridge:
    """
    Wraps pyserial to send DROWSY_ALERT to the ESP32.
    If ``--no-serial`` is passed, operates in dry-run mode (console only).
    """

    def __init__(self, port: str | None, baud: int, dry_run: bool = False):
        self.dry_run = dry_run
        self.ser = None

        if not dry_run and port:
            try:
                import serial
                self.ser = serial.Serial(port, baud, timeout=1)
                time.sleep(2)  # wait for ESP32 to reboot on serial connect
                print(f"[SERIAL] Connected to {port} @ {baud} baud")
            except Exception as e:
                print(f"[SERIAL] ERROR: Could not open {port}: {e}")
                print("[SERIAL] Falling back to dry-run mode.")
                self.dry_run = True

    def send_alert(self):
        """Send DROWSY_ALERT to the ESP32 (or print to console in dry-run)."""
        msg = "DROWSY_ALERT\n"
        if self.dry_run or self.ser is None:
            print(f"[DRY-RUN] Would send: {msg.strip()}")
        else:
            self.ser.write(msg.encode("utf-8"))
            self.ser.flush()
            print(f"[SERIAL] Sent: {msg.strip()}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# ─────────────────────── Main Loop ────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="V2V Drowsiness Detection — Smart Dashcam Simulator"
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="Serial port for ESP32 (e.g., /dev/tty.usbserial-0001 or COM3)"
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--no-serial", action="store_true",
        help="Dry-run mode: no serial output, console only"
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="Camera index (default: 0)"
    )
    parser.add_argument(
        "--ear-threshold", type=float, default=0.25,
        help="EAR threshold below which eyes are considered closed (default: 0.25)"
    )
    parser.add_argument(
        "--consec-frames", type=int, default=20,
        help="Number of consecutive frames below EAR threshold to trigger alert "
             "(default: 20, ~0.7s at 30fps)"
    )
    parser.add_argument(
        "--cooldown", type=float, default=5.0,
        help="Seconds to wait before re-triggering alert (default: 5.0)"
    )
    parser.add_argument(
        "--shape-predictor", type=str,
        default="shape_predictor_68_face_landmarks.dat",
        help="Path to dlib shape predictor model file"
    )
    args = parser.parse_args()

    # ── Load dlib models ──
    print("[INIT] Loading dlib face detector...")
    detector = dlib.get_frontal_face_detector()

    print(f"[INIT] Loading shape predictor: {args.shape_predictor}")
    try:
        predictor = dlib.shape_predictor(args.shape_predictor)
    except RuntimeError:
        print(
            f"\n[ERROR] Could not load '{args.shape_predictor}'.\n"
            "Download it from:\n"
            "  http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2\n"
            "Then extract and place it in the same directory as this script.\n"
        )
        sys.exit(1)

    # ── Open camera ──
    print(f"[INIT] Opening camera {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {args.camera}")
        sys.exit(1)

    # ── Serial bridge ──
    bridge = SerialBridge(
        port=args.port,
        baud=args.baud,
        dry_run=args.no_serial or args.port is None,
    )

    # ── State ──
    consec_counter = 0        # frames with EAR below threshold
    last_alert_time = 0.0     # timestamp of last DROWSY_ALERT sent
    alert_active = False

    print("[INIT] Drowsiness detector running. Press 'q' to quit.\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Frame capture failed, retrying...")
                continue

            frame = imutils.resize(frame, width=640)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect faces
            faces = detector(gray, 0)

            for face in faces:
                # Get 68 facial landmarks
                shape = predictor(gray, face)
                shape = face_utils.shape_to_np(shape)

                # Extract eye regions
                left_eye  = shape[LEFT_EYE_IDX[0]:LEFT_EYE_IDX[1]]
                right_eye = shape[RIGHT_EYE_IDX[0]:RIGHT_EYE_IDX[1]]

                # Compute EAR for both eyes and average
                left_ear  = eye_aspect_ratio(left_eye)
                right_ear = eye_aspect_ratio(right_eye)
                avg_ear   = (left_ear + right_ear) / 2.0

                # ── Draw eye contours ──
                left_hull  = cv2.convexHull(left_eye)
                right_hull = cv2.convexHull(right_eye)
                cv2.drawContours(frame, [left_hull],  -1, GREEN, 1)
                cv2.drawContours(frame, [right_hull], -1, GREEN, 1)

                # ── Drowsiness logic ──
                if avg_ear < args.ear_threshold:
                    consec_counter += 1

                    if consec_counter >= args.consec_frames:
                        now = time.time()
                        if now - last_alert_time > args.cooldown:
                            # ── TRIGGER ALERT ──
                            bridge.send_alert()
                            last_alert_time = now
                            alert_active = True
                        
                        # Visual warning on frame
                        cv2.putText(
                            frame, "*** DROWSINESS DETECTED ***",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, RED, 2
                        )
                else:
                    consec_counter = 0
                    alert_active = False

                # ── EAR overlay ──
                colour = RED if avg_ear < args.ear_threshold else GREEN
                cv2.putText(
                    frame, f"EAR: {avg_ear:.3f}",
                    (480, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, colour, 2
                )

            # Status bar
            status_colour = RED if alert_active else GREEN
            status_text = "ALERT!" if alert_active else "Monitoring"
            cv2.putText(
                frame, f"Status: {status_text}",
                (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_colour, 1
            )

            cv2.imshow("V2V Drowsiness Detector", frame)

            # Quit on 'q'
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\n[EXIT] Interrupted by user.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        bridge.close()
        print("[EXIT] Cleanup complete.")


if __name__ == "__main__":
    main()
