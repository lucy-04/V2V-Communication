#!/usr/bin/env python3
"""
V2V Drowsiness Detection — Smart Dashcam Simulator
====================================================
Captures webcam frames, detects the DRIVER's face (rightmost-closest person)
via dlib's 68-point predictor, computes Eye Aspect Ratio (EAR), and sends
alerts over serial to the ESP32:

  1. Eyes closed for 3 seconds  → DROWSY_ALERT  (buzzer wake-up attempt)
  2. Head out of frame for 4 s  → DROWSY_ALERT  (head-down detection)
  3. Continuous sleep for 10 s  → SOS_TRIGGER   (emergency beacon + WhatsApp)

WhatsApp alert uses the free CallMeBot API.  To enable it:
  1. Save +34 644 52 74 88 in your contacts as "CallMeBot"
  2. Send "I allow callmebot to send me messages" to that number on WhatsApp
  3. You'll receive an API key — pass it with --whatsapp-apikey

Usage:
    python drowsiness_detector.py --port COM4 --baud 115200 --whatsapp-apikey 123456

    # Dry-run (no serial, no WhatsApp)
    python drowsiness_detector.py --no-serial

    # Custom thresholds
    python drowsiness_detector.py --ear-threshold 0.22 --closed-seconds 3.0 --sos-seconds 10.0
"""

import argparse
import time
import sys
import threading
import urllib.request
import urllib.parse

import cv2
import dlib
import numpy as np
from scipy.spatial import distance as dist
import imutils
from imutils import face_utils

# ─────────────────────── Constants ────────────────────────────────────────────
LEFT_EYE_IDX  = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
RIGHT_EYE_IDX = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]
NOSE_IDX      = face_utils.FACIAL_LANDMARKS_68_IDXS["nose"]
JAW_IDX       = face_utils.FACIAL_LANDMARKS_68_IDXS["jaw"]

GREEN  = (0, 255, 0)
RED    = (0, 0, 255)
YELLOW = (0, 255, 255)
CYAN   = (255, 255, 0)
ORANGE = (0, 165, 255)

EMERGENCY_PHONE = "+918800830233"


# ─────────────────────── EAR Calculation ──────────────────────────────────────
def eye_aspect_ratio(eye: np.ndarray) -> float:
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])
    C = dist.euclidean(eye[0], eye[3])
    return (A + B) / (2.0 * C)


# ─────────────────────── Driver Selection ─────────────────────────────────────
def select_driver_face(faces, frame_width):
    if len(faces) == 0:
        return None
    if len(faces) == 1:
        return faces[0]

    areas = []
    for f in faces:
        areas.append((f.right() - f.left()) * (f.bottom() - f.top()))
    max_area = max(areas) if max(areas) > 0 else 1

    best_score = -1.0
    best_face = None
    for f, area in zip(faces, areas):
        right_norm = f.right() / frame_width
        area_norm = area / max_area
        score = right_norm * 0.6 + area_norm * 0.4
        if score > best_score:
            best_score = score
            best_face = f
    return best_face


# ─────────────────────── WhatsApp SOS via CallMeBot ───────────────────────────
def send_whatsapp_sos(phone: str, apikey: str, lat: float = 0.0, lon: float = 0.0):
    """Send an emergency WhatsApp message via the free CallMeBot API."""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

    message = (
        "🚨 V2V EMERGENCY SOS 🚨\n"
        "Driver is INCAPACITATED!\n"
        f"Time: {timestamp}\n"
    )
    if lat != 0.0 or lon != 0.0:
        message += f"Location: {lat:.6f}, {lon:.6f}\n"
        message += f"Maps: https://maps.google.com/?q={lat},{lon}"
    else:
        message += "Location: GPS not available"

    encoded_msg = urllib.parse.quote(message)
    url = (
        f"https://api.callmebot.com/whatsapp.php"
        f"?phone={phone}&text={encoded_msg}&apikey={apikey}"
    )

    try:
        req = urllib.request.Request(url)
        response = urllib.request.urlopen(req, timeout=10)
        status = response.getcode()
        print(f"[WHATSAPP] SOS sent to {phone} (HTTP {status})")
        return True
    except Exception as e:
        print(f"[WHATSAPP] ERROR sending to {phone}: {e}")
        return False


# ─────────────────────── Serial Wrapper ───────────────────────────────────────
class SerialBridge:
    """
    Wraps pyserial to send commands to ESP32 and read back all output.
    Background thread reads ESP32 serial and extracts GPS coordinates.
    """

    def __init__(self, port: str | None, baud: int, dry_run: bool = False):
        self.dry_run = dry_run
        self.ser = None
        self._reader_thread = None
        self._running = False

        # Track latest GPS coordinates from ESP32 output
        self.last_lat = 0.0
        self.last_lon = 0.0

        if not dry_run and port:
            try:
                import serial
                self.ser = serial.Serial(port, baud, timeout=0.1)
                time.sleep(2)
                print(f"[SERIAL] Connected to {port} @ {baud} baud")
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
        """Background thread: reads ESP32 output and extracts GPS coordinates."""
        while self._running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="replace").rstrip()
                    if line:
                        print(f"[ESP32] {line}")
                        # Extract GPS coordinates from ESP32 output
                        if "LAT:" in line:
                            try:
                                self.last_lat = float(line.split("LAT:")[1].strip())
                            except (ValueError, IndexError):
                                pass
                        if "LON:" in line:
                            try:
                                self.last_lon = float(line.split("LON:")[1].strip())
                            except (ValueError, IndexError):
                                pass
            except Exception:
                pass

    def send(self, message: str):
        msg = message + "\n"
        if self.dry_run or self.ser is None:
            print(f"[DRY-RUN] Would send: {message}")
        else:
            self.ser.write(msg.encode("utf-8"))
            self.ser.flush()
            print(f"[SERIAL] Sent: {message}")

    def send_alert(self):
        self.send("DROWSY_ALERT")

    def send_cancel(self):
        self.send("CANCEL_DROWSY")

    def send_driver_ok(self):
        self.send("DRIVER_OK")

    def send_sos_trigger(self):
        """Directly trigger SOS mode on ESP32 — bypasses incap timer."""
        self.send("SOS_TRIGGER")

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
        help="Serial port for ESP32 (e.g., COM4 or /dev/ttyUSB0)"
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
        help="EAR below which eyes are considered closed (default: 0.25)"
    )
    parser.add_argument(
        "--closed-seconds", type=float, default=3.0,
        help="Seconds eyes must be closed to trigger DROWSY_ALERT (default: 3.0)"
    )
    parser.add_argument(
        "--sos-seconds", type=float, default=10.0,
        help="Seconds of continuous sleep to trigger SOS emergency (default: 10.0)"
    )
    parser.add_argument(
        "--cooldown", type=float, default=5.0,
        help="Cooldown between DROWSY_ALERTs (default: 5.0)"
    )
    parser.add_argument(
        "--head-down-timeout", type=float, default=4.0,
        help="Seconds without driver face before alert (default: 4.0)"
    )
    parser.add_argument(
        "--whatsapp-apikey", type=str, default=None,
        help="CallMeBot API key for WhatsApp SOS messages"
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
    eyes_closed_since = None        # timestamp when eyes first closed
    sleep_start_time = None         # tracks CONTINUOUS sleep (eyes or head-down)
    last_alert_time = 0.0           # timestamp of last DROWSY_ALERT
    alert_active = False
    alert_reason = ""
    sos_triggered = False           # True once SOS has been sent for this sleep episode

    # Head-down / face-lost tracking
    driver_last_seen_time = time.time()
    head_down_alert_sent = False

    # WhatsApp status
    if args.whatsapp_apikey:
        print(f"[INIT] WhatsApp SOS enabled → {EMERGENCY_PHONE}")
    else:
        print("[INIT] WhatsApp SOS disabled (no --whatsapp-apikey provided)")

    print(f"[INIT] Thresholds: DROWSY={args.closed_seconds}s  SOS={args.sos_seconds}s")
    print("[INIT] Drowsiness detector running. Press 'q' to quit.")
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

            faces = detector(gray, 0)
            driver_face = select_driver_face(faces, frame_w)

            # Draw faded boxes around non-driver faces
            for face in faces:
                if driver_face is not None and face == driver_face:
                    continue
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

            # ══════════════════════════════════════════════════════════════
            # Determine if driver is "asleep" this frame
            # (for the continuous 10s SOS tracker)
            # ══════════════════════════════════════════════════════════════
            driver_is_asleep = False

            if driver_face is not None:
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

                # Get 68 facial landmarks
                shape = predictor(gray, driver_face)
                shape = face_utils.shape_to_np(shape)

                left_eye  = shape[LEFT_EYE_IDX[0]:LEFT_EYE_IDX[1]]
                right_eye = shape[RIGHT_EYE_IDX[0]:RIGHT_EYE_IDX[1]]

                left_ear  = eye_aspect_ratio(left_eye)
                right_ear = eye_aspect_ratio(right_eye)
                avg_ear   = (left_ear + right_ear) / 2.0

                # Draw eye contours
                left_hull  = cv2.convexHull(left_eye)
                right_hull = cv2.convexHull(right_eye)
                cv2.drawContours(frame, [left_hull],  -1, GREEN, 1)
                cv2.drawContours(frame, [right_hull], -1, GREEN, 1)

                # ── Eyes closed detection (time-based) ──
                if avg_ear < args.ear_threshold:
                    if eyes_closed_since is None:
                        eyes_closed_since = now

                    closed_duration = now - eyes_closed_since
                    driver_is_asleep = True  # eyes are closed → asleep

                    # 3s threshold → DROWSY_ALERT (wake-up attempt)
                    if closed_duration >= args.closed_seconds:
                        if now - last_alert_time > args.cooldown:
                            bridge.send_alert()
                            last_alert_time = now
                            alert_active = True
                            alert_reason = "EYES CLOSED"

                        cv2.putText(
                            frame, f"*** DROWSY: EYES CLOSED ({closed_duration:.1f}s) ***",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, RED, 2
                        )
                    elif closed_duration > 1.0:
                        remaining = args.closed_seconds - closed_duration
                        cv2.putText(
                            frame, f"Eyes closing... ({remaining:.1f}s to alert)",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, ORANGE, 2
                        )
                else:
                    eyes_closed_since = None
                    if alert_reason == "EYES CLOSED":
                        alert_active = False
                        alert_reason = ""

                # EAR overlay
                colour = RED if avg_ear < args.ear_threshold else GREEN
                cv2.putText(
                    frame, f"EAR: {avg_ear:.3f}",
                    (480, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, colour, 2
                )

            else:
                # ── No driver face → possible head-down ──
                eyes_closed_since = None
                elapsed_missing = now - driver_last_seen_time

                if elapsed_missing >= args.head_down_timeout:
                    driver_is_asleep = True  # head is down → asleep

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
                        frame, f"*** DROWSY: HEAD DOWN ({elapsed_missing:.1f}s) ***",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, RED, 2
                    )

                elif elapsed_missing > 1.0:
                    remaining = args.head_down_timeout - elapsed_missing
                    cv2.putText(
                        frame, f"Driver face lost ({remaining:.1f}s to alert)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, ORANGE, 2
                    )

            # ══════════════════════════════════════════════════════════════
            # CONTINUOUS SLEEP TRACKER → SOS after 10 seconds
            # ══════════════════════════════════════════════════════════════
            if driver_is_asleep:
                if sleep_start_time is None:
                    sleep_start_time = now

                total_sleep = now - sleep_start_time

                # 10s continuous sleep → SOS EMERGENCY
                if total_sleep >= args.sos_seconds and not sos_triggered:
                    sos_triggered = True

                    print("=" * 50)
                    print("[SOS] DRIVER INCAPACITATED — 10s continuous sleep!")
                    print("[SOS] Triggering emergency beacon on ESP32...")
                    print("=" * 50)

                    # Tell ESP32 to go directly into SOS mode
                    bridge.send_sos_trigger()

                    # Send WhatsApp message
                    if args.whatsapp_apikey:
                        print("[SOS] Sending WhatsApp SOS message...")
                        send_whatsapp_sos(
                            phone=EMERGENCY_PHONE,
                            apikey=args.whatsapp_apikey,
                            lat=bridge.last_lat,
                            lon=bridge.last_lon,
                        )
                    else:
                        print("[SOS] WhatsApp not configured (--whatsapp-apikey not set)")

                # Show SOS countdown on screen
                if total_sleep > args.closed_seconds and not sos_triggered:
                    remaining_sos = args.sos_seconds - total_sleep
                    cv2.putText(
                        frame, f"SOS in {remaining_sos:.1f}s",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, ORANGE, 2
                    )
                elif sos_triggered:
                    cv2.putText(
                        frame, "!!! SOS BEACON ACTIVE !!!",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, RED, 2
                    )
            else:
                # Driver is awake — reset everything
                if sleep_start_time is not None:
                    sleep_start_time = None
                if sos_triggered:
                    sos_triggered = False
                    print("[SOS] Driver woke up — SOS state cleared on Python side.")

            # ── Status bar ──
            if sos_triggered:
                status_text = "!!! SOS ACTIVE !!!"
                status_colour = RED
            elif alert_active:
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

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("o"):
                bridge.send_driver_ok()
                # Also reset SOS state locally
                sos_triggered = False
                sleep_start_time = None
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
