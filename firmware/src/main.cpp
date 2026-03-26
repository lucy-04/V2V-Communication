/*******************************************************************************
 * V2V Communication System — ESP32 Firmware
 * ===========================================================================
 * Decentralised, offline Vehicle-to-Vehicle safety system.
 * Hardware: ESP32 + NRF24L01 + NEO-6M GPS + Buzzer + LEDs
 *
 * Architecture
 *   Core 0 → GPS UART parsing, Serial listener (DROWSY_ALERT from laptop)
 *   Core 1 → RF TX/RX, risk-assessment math, GPIO alerts
 *
 * Coding rules
 *   • NO delay()  — millis()-based non-blocking timers only
 *   • Packed structs ≤ 32 bytes (NRF24L01 hard limit)
 *   • Circular ring buffer for inbound RF packets
 ******************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <NMEAGPS.h>          // NeoGPS — fast, memory-efficient NMEA parser

// ─────────────────────────── PIN DEFINITIONS ────────────────────────────────
// NRF24L01 (VSPI — default SPI bus on ESP32)
#define NRF_CE_PIN    4
#define NRF_CSN_PIN   5

// NEO-6M GPS (UART2)
#define GPS_RX_PIN   16       // ESP32 RX ← GPS TX
#define GPS_TX_PIN   17       // ESP32 TX → GPS RX (not used, but wired)

// Alert GPIOs
#define BUZZER_PIN   25
#define LED_RED_PIN  26
#define LED_GRN_PIN  27
#define LED_BLU_PIN  14

// ─────────────────────────── CONSTANTS ──────────────────────────────────────
static const uint64_t RF_PIPE_ADDR     = 0xF0F0F0F0E1LL;   // shared broadcast pipe
static const uint8_t  RF_CHANNEL       = 108;               // 2.508 GHz — low Wi-Fi overlap
static const uint32_t NORMAL_TX_INTERVAL_MS  = 1000;        // 1 Hz telemetry
static const float    ALERT_DISTANCE_M       = 50.0f;       // proximity threshold
static const float    ALERT_TTC_S            = 3.0f;        // time-to-collision threshold
static const uint32_t ALERT_DURATION_MS      = 2000;        // buzzer/LED on-time per trigger
static const uint32_t SERIAL_POLL_INTERVAL   = 50;          // ms between Serial checks
static const uint32_t DROWSY_ALERT_REARM_MS  = 5000;        // cooldown before another drowsy alert

// Earth radius in metres (for Haversine / equirectangular)
static const float EARTH_RADIUS_M = 6371000.0f;

// ─────────────────────────── PACKET STRUCTURES (packed, ≤32 B) ─────────────
// Normal telemetry packet — 11 bytes
struct __attribute__((packed)) NormalPacket {
    uint8_t  packetType;      // 0
    float    latitude;        // 4 B
    float    longitude;       // 4 B
    uint16_t speed;           // 2 B  (km/h × 10 for 0.1 resolution)
    uint8_t  turnIndicator;   // 0=straight, 1=left, 2=right
};
static_assert(sizeof(NormalPacket) <= 32, "NormalPacket exceeds 32-byte NRF limit");

// Emergency packet — 10 bytes
struct __attribute__((packed)) EmergencyPacket {
    uint8_t  packetType;      // 1
    uint8_t  eventCode;       // 1=Collision, 2=Drowsy
    float    latitude;        // 4 B
    float    longitude;       // 4 B
};
static_assert(sizeof(EmergencyPacket) <= 32, "EmergencyPacket exceeds 32-byte NRF limit");

// ─────────────────────── GENERIC RING BUFFER (lock-free SPSC) ──────────────
// Single-producer, single-consumer — safe across two FreeRTOS tasks without mutex.
template <typename T, uint8_t N>
class RingBuffer {
    static_assert((N & (N - 1)) == 0, "RingBuffer size must be a power of 2");
public:
    bool push(const T& item) {
        uint8_t nextHead = (head_ + 1) & (N - 1);
        if (nextHead == tail_) return false;   // full
        buf_[head_] = item;
        head_ = nextHead;
        return true;
    }
    bool pop(T& item) {
        if (head_ == tail_) return false;      // empty
        item = buf_[tail_];
        tail_ = (tail_ + 1) & (N - 1);
        return true;
    }
    uint8_t count() const {
        return (head_ - tail_) & (N - 1);
    }
private:
    volatile uint8_t head_ = 0;
    volatile uint8_t tail_ = 0;
    T buf_[N];
};

// ─────────────────────────── GLOBAL STATE ───────────────────────────────────
// RF
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// GPS (NeoGPS)
static NMEAGPS  gpsParser;                  // parser instance
static gps_fix  gpsFix;                     // latest fix
static HardwareSerial& gpsSerial = Serial2; // UART2

// Ring buffer: holds raw 32-byte blobs; we decode after pop
RingBuffer<uint8_t[32], 16> rxRingBuffer;

// Latest own-vehicle state (written on Core 0, read on Core 1)
static volatile float  ownLat       = 0.0f;
static volatile float  ownLon       = 0.0f;
static volatile uint16_t ownSpeed   = 0;     // km/h × 10
static volatile uint8_t  ownTurn    = 0;     // 0=straight

// Flags (atomic-safe on ESP32 — single-word writes)
static volatile bool emergencyDrowsy    = false;
static volatile bool emergencyCollision = false;

// Non-blocking alert timer
static volatile uint32_t alertActiveUntil = 0;

// Drowsy cooldown
static volatile uint32_t lastDrowsyTrigger = 0;

// ─────────────────── MATH: EQUIRECTANGULAR DISTANCE (fast) ─────────────────
// Good enough for distances < 1 km; ~10× faster than full Haversine.
static float approxDistanceM(float lat1, float lon1, float lat2, float lon2) {
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1) * cosf(radians((lat1 + lat2) * 0.5f));
    return EARTH_RADIUS_M * sqrtf(dLat * dLat + dLon * dLon);
}

// ─────────────────── MATH: TIME-TO-COLLISION ───────────────────────────────
// closingSpeed in m/s, distance in m.  Returns seconds.  <0 means diverging.
static float computeTTC(float distance, float closingSpeedMs) {
    if (closingSpeedMs <= 0.0f) return 999.0f;   // not closing
    return distance / closingSpeedMs;
}

// ──────────────────── ALERT SYSTEM (non-blocking) ──────────────────────────
static void fireAlert() {
    alertActiveUntil = millis() + ALERT_DURATION_MS;
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GRN_PIN, LOW);
    digitalWrite(LED_BLU_PIN, LOW);
}

static void updateAlerts() {
    if (millis() >= alertActiveUntil && alertActiveUntil != 0) {
        // Turn off
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_RED_PIN, LOW);
        alertActiveUntil = 0;
    }
}

// ────────────────── SERIAL LOGGING FOR EMERGENCY RESPONDERS ────────────────
// When a collision is detected, log GPS coordinates + timestamp to Serial
// so the connected laptop can capture them for emergency services (no cloud).
static void logCollisionToSerial(float lat, float lon, uint8_t eventCode) {
    Serial.println("========== EMERGENCY EVENT ==========");
    Serial.print("EVENT: ");
    Serial.println(eventCode == 1 ? "COLLISION" : "DROWSY_DRIVER");
    Serial.print("LAT: "); Serial.println(lat, 6);
    Serial.print("LON: "); Serial.println(lon, 6);
    Serial.print("UPTIME_MS: "); Serial.println(millis());
    Serial.println("ACTION_REQUIRED: NOTIFY EMERGENCY SERVICES");
    Serial.println("======================================");
}

/*============================================================================
 *  CORE 0 TASKS — GPS & Serial Listener
 *==========================================================================*/

// ── Task: GPS UART Parsing ────────────────────────────────────────────────
// Reads NMEA sentences from NEO-6M at 9600 baud.  NeoGPS handles incremental
// parse so we never block.
void taskGPS(void* pvParameters) {
    (void)pvParameters;
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    for (;;) {
        while (gpsParser.available(gpsSerial)) {
            gpsFix = gpsParser.read();
            if (gpsFix.valid.location) {
                ownLat = gpsFix.latitude();
                ownLon = gpsFix.longitude();
            }
            if (gpsFix.valid.speed) {
                // NeoGPS gives speed in km/h as float; store × 10 for 0.1 res
                ownSpeed = (uint16_t)(gpsFix.speed_kph() * 10.0f);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));   // yield — not a blocking delay
    }
}

// ── Task: Serial Listener (Laptop → ESP32 for DROWSY_ALERT) ──────────────
// Non-blocking: polls Serial.available() on a short interval.
// Reads accumulated chars into a small line buffer; when '\n' is seen,
// checks for the "DROWSY_ALERT" keyword.
void taskSerialListener(void* pvParameters) {
    (void)pvParameters;

    static char lineBuf[64];
    static uint8_t lineIdx = 0;
    uint32_t lastPoll = 0;

    for (;;) {
        uint32_t now = millis();
        if (now - lastPoll < SERIAL_POLL_INTERVAL) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        lastPoll = now;

        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                lineBuf[lineIdx] = '\0';
                // ── Check for DROWSY_ALERT ──
                if (lineIdx > 0 && strstr(lineBuf, "DROWSY_ALERT") != nullptr) {
                    if (now - lastDrowsyTrigger > DROWSY_ALERT_REARM_MS) {
                        emergencyDrowsy = true;
                        lastDrowsyTrigger = now;
                        Serial.println("[V2V] DROWSY_ALERT received — broadcasting emergency!");
                    }
                }
                lineIdx = 0;
            } else if (lineIdx < sizeof(lineBuf) - 1) {
                lineBuf[lineIdx++] = c;
            }
        }
    }
}

/*============================================================================
 *  CORE 1 TASKS — RF TX/RX & Risk Assessment
 *==========================================================================*/

// ── Task: RF Transmit ─────────────────────────────────────────────────────
// Broadcasts NormalPacket at 1 Hz.  Immediately pre-empts with EmergencyPacket
// when a drowsy or collision flag is raised.
void taskRFTransmit(void* pvParameters) {
    (void)pvParameters;
    uint32_t lastTx = 0;

    for (;;) {
        uint32_t now = millis();

        // ── Priority: Emergency packets ──
        if (emergencyDrowsy) {
            emergencyDrowsy = false;
            EmergencyPacket pkt;
            pkt.packetType = 1;
            pkt.eventCode  = 2;   // Drowsy
            pkt.latitude   = ownLat;
            pkt.longitude  = ownLon;

            radio.stopListening();
            radio.write(&pkt, sizeof(pkt));
            radio.startListening();

            logCollisionToSerial(pkt.latitude, pkt.longitude, pkt.eventCode);
            fireAlert();   // local alert too
        }

        if (emergencyCollision) {
            emergencyCollision = false;
            EmergencyPacket pkt;
            pkt.packetType = 1;
            pkt.eventCode  = 1;   // Collision
            pkt.latitude   = ownLat;
            pkt.longitude  = ownLon;

            radio.stopListening();
            radio.write(&pkt, sizeof(pkt));
            radio.startListening();

            logCollisionToSerial(pkt.latitude, pkt.longitude, pkt.eventCode);
            fireAlert();
        }

        // ── Normal 1 Hz telemetry ──
        if (now - lastTx >= NORMAL_TX_INTERVAL_MS) {
            lastTx = now;
            NormalPacket pkt;
            pkt.packetType    = 0;
            pkt.latitude      = ownLat;
            pkt.longitude     = ownLon;
            pkt.speed         = ownSpeed;
            pkt.turnIndicator = ownTurn;

            radio.stopListening();
            radio.write(&pkt, sizeof(pkt));
            radio.startListening();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ── Task: RF Receive ──────────────────────────────────────────────────────
// Non-blocking poll; pushes raw 32-byte payloads into ring buffer.
void taskRFReceive(void* pvParameters) {
    (void)pvParameters;

    for (;;) {
        if (radio.available()) {
            uint8_t payload[32] = {0};
            radio.read(&payload, sizeof(payload));
            rxRingBuffer.push(payload);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ── Task: Risk Assessment ─────────────────────────────────────────────────
// Pops packets from ring buffer, decodes, computes distance & TTC,
// fires alerts when thresholds are breached.
void taskRiskAssessment(void* pvParameters) {
    (void)pvParameters;

    // Store last-known positions for TTC (closing speed) calculation
    static float lastKnownDist = 999.0f;
    static uint32_t lastDistTime = 0;

    for (;;) {
        uint8_t raw[32];
        while (rxRingBuffer.pop(raw)) {
            uint8_t packetType = raw[0];

            if (packetType == 0) {
                // ── Normal Packet ──
                NormalPacket pkt;
                memcpy(&pkt, raw, sizeof(NormalPacket));

                float dist = approxDistanceM(ownLat, ownLon, pkt.latitude, pkt.longitude);

                // Closing speed estimate (Δdist / Δtime)
                uint32_t now = millis();
                float closingSpeedMs = 0.0f;
                if (lastDistTime > 0 && now > lastDistTime) {
                    float dt = (now - lastDistTime) / 1000.0f;
                    closingSpeedMs = (lastKnownDist - dist) / dt;  // +ve = closing
                }
                lastKnownDist = dist;
                lastDistTime  = now;

                float ttc = computeTTC(dist, closingSpeedMs);

                // ── Threshold check ──
                if (dist < ALERT_DISTANCE_M || ttc < ALERT_TTC_S) {
                    fireAlert();

                    Serial.print("[V2V] PROXIMITY ALERT! dist=");
                    Serial.print(dist, 1);
                    Serial.print("m  TTC=");
                    Serial.print(ttc, 1);
                    Serial.println("s");

                    // If TTC is critically low, treat as imminent collision
                    if (ttc < 1.5f && dist < 30.0f) {
                        emergencyCollision = true;
                        logCollisionToSerial(ownLat, ownLon, 1);
                    }
                }

            } else if (packetType == 1) {
                // ── Emergency Packet ──
                EmergencyPacket pkt;
                memcpy(&pkt, raw, sizeof(EmergencyPacket));

                fireAlert();

                Serial.print("[V2V] EMERGENCY from nearby vehicle! Event=");
                Serial.println(pkt.eventCode == 1 ? "COLLISION" : "DROWSY");
                Serial.print("  Location: ");
                Serial.print(pkt.latitude, 6);
                Serial.print(", ");
                Serial.println(pkt.longitude, 6);

                // Log for emergency responders on the connected laptop
                logCollisionToSerial(pkt.latitude, pkt.longitude, pkt.eventCode);
            }
        }

        // Update alert GPIOs (non-blocking off-timer)
        updateAlerts();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*============================================================================
 *  SETUP & LOOP
 *==========================================================================*/
void setup() {
    // ── Serial (USB — laptop comms + debug) ──
    Serial.begin(115200);
    while (!Serial) { ; }
    Serial.println("\n[V2V] ========================================");
    Serial.println("[V2V]  Vehicle-to-Vehicle Safety System v1.0");
    Serial.println("[V2V]  No-Cloud | Sub-85ms Latency Target");
    Serial.println("[V2V] ========================================");

    // ── GPIO ──
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GRN_PIN, OUTPUT);
    pinMode(LED_BLU_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GRN_PIN, LOW);
    digitalWrite(LED_BLU_PIN, LOW);

    // Quick LED self-test (non-blocking: just a fast blink)
    digitalWrite(LED_GRN_PIN, HIGH);
    // We allow a tiny startup delay here since setup() runs once before the scheduler
    // In production, even this could be removed.

    // ── NRF24L01 ──
    if (!radio.begin()) {
        Serial.println("[V2V] ERROR: NRF24L01 not detected! Check wiring.");
        // Blink red forever (non-blocking in the main loop would be better,
        // but we halt here because the system is non-functional without radio)
        while (true) {
            digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    radio.setChannel(RF_CHANNEL);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_2MBPS);
    radio.setPayloadSize(32);
    radio.setAutoAck(false);        // broadcast mode — no ACKs
    radio.openWritingPipe(RF_PIPE_ADDR);
    radio.openReadingPipe(1, RF_PIPE_ADDR);
    radio.startListening();
    Serial.println("[V2V] NRF24L01 initialised — channel " + String(RF_CHANNEL));

    // ── FreeRTOS Task Creation ──
    // Core 0: GPS + Serial Listener
    xTaskCreatePinnedToCore(
        taskGPS,              // function
        "GPS_Parse",          // name
        4096,                 // stack (bytes)
        NULL,                 // params
        2,                    // priority (higher = more urgent)
        NULL,                 // handle
        0                     // core 0
    );

    xTaskCreatePinnedToCore(
        taskSerialListener,
        "Serial_Listen",
        4096,
        NULL,
        1,                    // lower priority than GPS
        NULL,
        0
    );

    // Core 1: RF TX, RF RX, Risk Assessment
    xTaskCreatePinnedToCore(
        taskRFTransmit,
        "RF_TX",
        4096,
        NULL,
        3,                    // highest priority — emergency TX must pre-empt
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        taskRFReceive,
        "RF_RX",
        4096,
        NULL,
        3,                    // equal to TX
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        taskRiskAssessment,
        "Risk_Assess",
        4096,
        NULL,
        2,
        NULL,
        1
    );

    Serial.println("[V2V] All FreeRTOS tasks launched. System active.");
    digitalWrite(LED_GRN_PIN, LOW);  // end self-test blink
}

// Arduino loop() is essentially idle — all real work is in FreeRTOS tasks.
// We use it only for a heartbeat LED to confirm the system is alive.
void loop() {
    static uint32_t lastHeartbeat = 0;
    uint32_t now = millis();
    if (now - lastHeartbeat >= 2000) {
        lastHeartbeat = now;
        // Brief green blink = system alive
        digitalWrite(LED_GRN_PIN, HIGH);
    }
    if (now - lastHeartbeat >= 100 && digitalRead(LED_GRN_PIN)) {
        digitalWrite(LED_GRN_PIN, LOW);
    }
}
