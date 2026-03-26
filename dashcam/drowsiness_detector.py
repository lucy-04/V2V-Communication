#!/usr/bin/env python3
"""
V2V Drowsiness Detection — Smart Dashcam Simulator
====================================================
Captures webcam frames, detects the DRIVER's face (rightmost-closest person)
via dlib's 68-point predictor, computes Eye Aspect Ratio (EAR), and sends
``DROWSY_ALERT`` over serial to the ESP32 when:
  1. The driver's eyes are closed for too many consecutive frames, OR
  2. The driver's head drops out of frame for > 4 seconds (face lost).

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
import threading

import cv2
import dlib
import numpy as np
from scipy.spatial import distance as dist
import imutils
from imutils import face_utils

# ─────────────────────── Constants ────────────────────────────────────────────
# dlib 68-point landmark indices
LEFT_EYE_IDX  = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
RIGHT_EYE_IDX = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]
NOSE_IDX      = face_utils.FACIAL_LANDMARKS_68_IDXS["nose"]
JAW_IDX       = face_utils.FACIAL_LANDMARKS_68_IDXS["jaw"]

# Colours for overlay (BGR)
GREEN  = (0, 255, 0)
RED    = (0, 0, 255)
YELLOW = (0, 255, 255)
CYAN   = (255, 255, 0)
ORANGE = (0, 165, 255)


# ─────────────────────── EAR Calculation ──────────────────────────────────────
def eye_aspect_ratio(eye: np.ndarray) -> float:
    """
    Compute the Eye Aspect Ratio (EAR) for a single eye.

    EAR = (||p2-p6|| + ||p3-p5||) / (2 * ||p1-p4||)

    A value near 0.3 means open; near 0.15 means closed.
    """
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])
    C = dist.euclidean(eye[0], eye[3])
    return (A + B) / (2.0 * C)


# ─────────────────────── Driver Selection ─────────────────────────────────────
def select_driver_face(faces, frame_width):
    """
    From a list of detected faces, select the DRIVER — defined as the
    rightmost AND closest (largest bounding-box area) face.

    Strategy:
      1. Score each face: score = (face_right_x / frame_width) * 0.6
                                + (face_area / max_area)       * 0.4
         Right-side bias (60%) + size/closeness bias (40%).
      2. Return the face with the highest score.

    This handles the dashcam being in the centre of the car: the driver is
    to the right of the camera and is the closest person to it.

    Args:
        faces: list of dlib rectangles from the face detector.
        frame_width: width of the frame in pixels.

    Returns:
        The single dlib rectangle for the driver, or None if no faces.
    """
    if len(faces) == 0:
        return None
    if len(faces) == 1:
        return faces[0]

    # Compute areas
    areas = []
    for f in faces:
        areas.append((f.right() - f.left()) * (f.bottom() - f.top()))
    max_area = max(areas) if max(areas) > 0 else 1

    best_score = -1.0
    best_face = None
    for f, area in zip(faces, areas):
        # Normalised right-edge position (0 = left edge, 1 = right edge)
        right_norm = f.right() / frame_width
        # Normalised area (larger = closer to camera)
        area_norm = area / max_area
        # Weighted score — favour right side + closeness
        score = right_norm * 0.6 + area_norm * 0.4
        if score > best_score:
            best_score = score
            best_face = f
    return best_face


# ─────────────────────── Serial Wrapper ───────────────────────────────────────
class SerialBridge:
    """
    Wraps pyserial to send commands to ESP32 and read back all debug output.
    A background thread continuously reads ESP32 serial output and prints it
    so you can see every value (GPS, TX, RX, CSMA, RISK, etc.) in one terminal.
    If ``--no-serial`` is passed, operates in dry-run mode (console only).
    """

    def __init__(self, port: str | None, baud: int, dry_run: bool = False):
        self.dry_run = dry_run
        self.ser = None
        self._reader_thread = None
        self._running = False

        if not dry_run and port:
            try:
                import serial
                self.ser = serial.Serial(port, baud, timeout=0.1)
                time.sleep(2)  # wait for ESP32 to reboot on serial connect
                print(f"[SERIAL] Connected to {port} @ {baud} baud")
                # Start background reader thread
                self._running = True
                self._reader_thread = threading.Thread(
                    target=self._read_loop, daemon=True
                )
                self._reader_thread.start()
            except Exception as e:
                print(f"[SERIAL] ERROR: Could not open {port}: {e}")
                print("[SERIAL] Falling back to dry-run mode.")
                self.dry_run = True

    def _read_loop(self):
        """Background thread: continuously reads and prints ESP32 serial output."""
        while self._running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="replace").rstrip()
                    if line:
                        print(f"[ESP32] {line}")
            except Exception:
                pass  # port closed or read error — silently continue

    def send(self, message: str):
        """Send a raw string to the ESP32 (with newline appended)."""
        msg = message + "\n"
        if self.dry_run or self.ser is None:
            print(f"[DRY-RUN] Would send: {message}")
        else:
            self.ser.write(msg.encode("utf-8"))
            self.ser.flush()
            print(f"[SERIAL] Sent: {message}")

    def send_alert(self):
        """Send DROWSY_ALERT to the ESP32."""
        self.send("DROWSY_ALERT")

    def send_cancel(self):
        """Send CANCEL_DROWSY to the ESP32."""
        self.send("CANCEL_DROWSY")

    def send_driver_ok(self):
        """Send DRIVER_OK to the ESP32 (cancels incap timer / SOS)."""
        self.send("DRIVER_OK")

    def close(self):
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
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
        "--head-down-timeout", type=float, default=4.0,
        help="Seconds the driver's face can be missing before triggering alert "
             "(default: 4.0)"
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
    consec_counter = 0              # consecutive frames with EAR below threshold
    last_alert_time = 0.0           # timestamp of last DROWSY_ALERT sent
    alert_active = False
    alert_reason = ""

    # Head-down / face-lost tracking
    driver_last_seen_time = time.time()   # last time we detected the driver's face
    head_down_alert_sent = False          # avoid repeating while head is still down

    print("[INIT] Drowsiness detector running. Press 'q' to quit.")
    print("[INIT] Driver = rightmost-closest face to the dashcam.")
    print("[INIT] Keyboard: 'q'=quit  'o'=DRIVER_OK  'c'=CANCEL_DROWSY")
    if not bridge.dry_run:
        print("[INIT] ESP32 output will appear with [ESP32] prefix.\n")
    else:
        print("")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Frame capture failed, retrying...")
                continue

            frame = imutils.resize(frame, width=640)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_h, frame_w = frame.shape[:2]
            now = time.time()

            # Detect all faces
            faces = detector(gray, 0)

            # ── Select only the DRIVER (rightmost + closest) ──
            driver_face = select_driver_face(faces, frame_w)

            # Draw faded boxes around non-driver faces (so user sees them ignored)
            for face in faces:
                if driver_face is not None and face == driver_face:
                    continue  # skip driver — drawn separately below
                cv2.rectangle(
                    frame,
                    (face.left(), face.top()),
                    (face.right(), face.bottom()),
                    (128, 128, 128), 1
                )
                cv2.putText(
                    frame, "passenger",
                    (face.left(), face.top() - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1
                )

            if driver_face is not None:
                # ── Driver detected — reset head-down timer ──
                driver_last_seen_time = now
                head_down_alert_sent = False

                # Draw driver bounding box
                cv2.rectangle(
                    frame,
                    (driver_face.left(), driver_face.top()),
                    (driver_face.right(), driver_face.bottom()),
                    CYAN, 2
                )
                cv2.putText(
                    frame, "DRIVER",
                    (driver_face.left(), driver_face.top() - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, CYAN, 2
                )

                # Get 68 facial landmarks for the driver only
                shape = predictor(gray, driver_face)
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

                # ── Drowsiness logic (eyes closed) ──
                if avg_ear < args.ear_threshold:
                    consec_counter += 1

                    if consec_counter >= args.consec_frames:
                        if now - last_alert_time > args.cooldown:
                            bridge.send_alert()
                            last_alert_time = now
                            alert_active = True
                            alert_reason = "EYES CLOSED"

                        cv2.putText(
                            frame, "*** DROWSINESS: EYES CLOSED ***",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, RED, 2
                        )
                else:
                    consec_counter = 0
                    if alert_reason == "EYES CLOSED":
                        alert_active = False
                        alert_reason = ""

                # ── EAR overlay ──
                colour = RED if avg_ear < args.ear_threshold else GREEN
                cv2.putText(
                    frame, f"EAR: {avg_ear:.3f}",
                    (480, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, colour, 2
                )

            else:
                # ── No driver face detected — possible head-down ──
                consec_counter = 0   # reset EAR counter since we can't see eyes
                elapsed_missing = now - driver_last_seen_time

                if elapsed_missing >= args.head_down_timeout:
                    # Driver's face has been missing for too long → head-down alert
                    if not head_down_alert_sent:
                        if now - last_alert_time > args.cooldown:
                            bridge.send_alert()
                            last_alert_time = now
                            head_down_alert_sent = True
                            alert_active = True
                            alert_reason = "HEAD DOWN"
                            print(f"[ALERT] Driver face lost for "
                                  f"{elapsed_missing:.1f}s — HEAD DOWN detected!")

                    cv2.putText(
                        frame, "*** DROWSINESS: HEAD DOWN ***",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, RED, 2
                    )

                elif elapsed_missing > 1.0:
                    # Warning: face missing but not yet at threshold
                    remaining = args.head_down_timeout - elapsed_missing
                    cv2.putText(
                        frame, f"Driver face lost ({remaining:.1f}s to alert)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, ORANGE, 2
                    )

            # ── Status bar ──
            if alert_active:
                status_text = f"ALERT: {alert_reason}"
                status_colour = RED
            else:
                num_faces = len(faces)
                driver_str = "driver locked" if driver_face else "no driver"
                status_text = f"Monitoring — {num_faces} face(s), {driver_str}"
                status_colour = GREEN

            cv2.putText(
                frame, f"Status: {status_text}",
                (10, frame_h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_colour, 1
            )

            cv2.imshow("V2V Drowsiness Detector", frame)

            # Quit on 'q', send DRIVER_OK on 'o', CANCEL_DROWSY on 'c'
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("o"):
                bridge.send_driver_ok()
            elif key == ord("c"):
                bridge.send_cancel()

    except KeyboardInterrupt:
        print("\n[EXIT] Interrupted by user.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        bridge.close()
        print("[EXIT] Cleanup complete.")


if __name__ == "__main__":
    main()
