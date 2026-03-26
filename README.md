# V2V-Communication

**Decentralised, Offline Vehicle-to-Vehicle Safety System**

Sub-85ms latency, peer-to-peer radio telemetry and emergency alerts — no cloud required.

---

## Hardware (Per Node)

| Component | Model | Interface |
|-----------|-------|-----------|
| MCU | ESP32 DevKit v1 | — |
| Radio | NRF24L01+ | SPI (VSPI) |
| GPS | NEO-6M | UART2 (9600 baud) |
| Alerts | Active Buzzer + RGB LEDs | GPIO |
| Dashcam Sim | Laptop + Webcam | USB Serial |

### Wiring

```
ESP32           NRF24L01+
─────           ─────────
GPIO 4  ───►    CE
GPIO 5  ───►    CSN
GPIO 18 ───►    SCK   (VSPI)
GPIO 23 ───►    MOSI  (VSPI)
GPIO 19 ◄───    MISO  (VSPI)
3.3V    ───►    VCC   (⚠️ NOT 5V!)
GND     ───►    GND

ESP32           NEO-6M GPS
─────           ──────────
GPIO 16 (RX) ◄──  TX
GPIO 17 (TX) ──►  RX
3.3V         ──►  VCC
GND          ──►  GND

ESP32           Alerts
─────           ──────
GPIO 25 ───►    Buzzer (+)
GPIO 26 ───►    Red LED (via 220Ω)
GPIO 27 ───►    Green LED (via 220Ω)
GPIO 14 ───►    Blue LED (via 220Ω)
```

---

## Build & Flash (ESP32 Firmware)

Requires [PlatformIO](https://platformio.org/).

```bash
cd firmware
pio run                  # compile only
pio run --target upload  # flash to ESP32
pio device monitor       # serial monitor @ 115200
```

---

## Drowsiness Detection (Laptop)

### Prerequisites

```bash
cd dashcam
pip install -r requirements.txt
```

Download the dlib shape predictor model:
```bash
wget http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
bunzip2 shape_predictor_68_face_landmarks.dat.bz2
```

### Run

```bash
# With serial to ESP32
python drowsiness_detector.py --port /dev/tty.usbserial-0001

# Dry-run (no serial, console only)
python drowsiness_detector.py --no-serial

# Custom thresholds
python drowsiness_detector.py --no-serial --ear-threshold 0.22 --consec-frames 25
```

Press **q** to quit.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         ESP32 Node                          │
│                                                             │
│  Core 0                          Core 1                     │
│  ┌──────────────────┐            ┌───────────────────┐      │
│  │ taskGPS          │            │ taskRFTransmit    │      │
│  │ (UART2, NeoGPS)  │            │ (1Hz normal,      │      │
│  └──────────────────┘            │  priority emerg.) │      │
│  ┌──────────────────┐            └───────────────────┘      │
│  │ taskSerialListen │            ┌───────────────────┐      │
│  │ (DROWSY_ALERT    │            │ taskRFReceive     │      │
│  │  from laptop)    │            │ (→ ring buffer)   │      │
│  └──────────────────┘            └───────────────────┘      │
│                                  ┌───────────────────┐      │
│                                  │ taskRiskAssess    │      │
│                                  │ (distance, TTC,   │      │
│                                  │  alerts, logging) │      │
│                                  └───────────────────┘      │
└─────────────────────────────────────────────────────────────┘
         ▲                              │
         │ USB Serial                   │ SPI
    ┌────┴────┐                    ┌────┴─────┐
    │ Laptop  │                    │ NRF24L01 │◄──► Peer Nodes
    │ (dlib)  │                    └──────────┘
    └─────────┘
```

---

## Packet Formats

| Packet | Byte | Field | Type | Notes |
|--------|------|-------|------|-------|
| **Normal** (11 B) | 0 | packetType | uint8 | `0` |
| | 1–4 | latitude | float | |
| | 5–8 | longitude | float | |
| | 9–10 | speed | uint16 | km/h × 10 |
| | 11 | turnIndicator | uint8 | 0/1/2 |
| **Emergency** (10 B) | 0 | packetType | uint8 | `1` |
| | 1 | eventCode | uint8 | 1=Collision, 2=Drowsy |
| | 2–5 | latitude | float | |
| | 6–9 | longitude | float | |

---

## Emergency Response (No-Cloud)

On collision/drowsiness detection, the system:
1. **Broadcasts** an `EmergencyPacket` over NRF24L01 to all nearby nodes.
2. **Logs** GPS coordinates + event type to USB Serial — the connected laptop captures this for offline handoff to emergency services.
3. **Fires** buzzer + red LEDs on all receiving nodes.

---

## License

MIT
