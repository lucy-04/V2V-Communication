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
 *
 * Debug mode
 *   • All values printed to Serial for hardware validation
 *   • Prefix tags: [GPS] [TX] [RX] [CSMA] [RISK] [SOS] [DROWSY] [CMD] [SYS]
 ******************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <NMEAGPS.h> // NeoGPS — fast, memory-efficient NMEA parser

// ─────────────────────────── PIN DEFINITIONS ────────────────────────────────
// NRF24L01 (VSPI — default SPI bus on ESP32)
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 5

// NEO-6M GPS (UART2)
#define GPS_RX_PIN 16 // ESP32 RX ← GPS TX
#define GPS_TX_PIN 17 // ESP32 TX → GPS RX (not used, but wired)

// Alert GPIOs
#define BUZZER_PIN 25
#define LED_RED_PIN 26
#define LED_GRN_PIN 27
#define LED_BLU_PIN 14

// ─────────────────────────── CONSTANTS ──────────────────────────────────────
static const uint64_t RF_PIPE_ADDR = 0xF0F0F0F0E1LL; // shared broadcast pipe
static const uint8_t RF_CHANNEL = 108;               // 2.508 GHz — low Wi-Fi overlap
static const uint32_t NORMAL_TX_INTERVAL_MS = 1000;  // 1 Hz telemetry (base, before jitter)
static const float ALERT_DISTANCE_M = 50.0f;         // proximity threshold
static const float ALERT_TTC_S = 3.0f;               // time-to-collision threshold
static const uint32_t ALERT_DURATION_MS = 2000;      // buzzer/LED on-time per trigger
static const uint32_t SERIAL_POLL_INTERVAL = 50;     // ms between Serial checks
static const uint32_t DROWSY_ALERT_REARM_MS = 5000;  // cooldown before another drowsy alert

// ─── CSMA / Jitter / Channel Hopping ───────────────────────────────────────
static const uint32_t TX_JITTER_MAX_MS = 200;               // random 0-200 ms added to normal TX
static const uint8_t CSMA_CARRIER_SENSE_US = 130;           // µs to let testCarrier() settle
static const uint8_t CSMA_MAX_BACKOFF_TRIES = 5;            // max backoff attempts before skip
static const uint8_t EMERGENCY_RETRANSMITS = 3;             // send emergency on each channel
static const uint8_t EMERGENCY_CHANNELS[] = {90, 108, 120}; // channel-hop list
static const uint8_t NUM_EMERGENCY_CHANNELS = sizeof(EMERGENCY_CHANNELS);
static const uint32_t EMERGENCY_HOP_GAP_US = 500; // µs between channel hops (non-blocking)

// ─── Duplicate suppression ─────────────────────────────────────────────────
// Cache of recently-seen (vehicleID, packetID) pairs to ignore re-broadcasts.
static const uint8_t DEDUP_CACHE_SIZE = 32;

// Drowsy wake-up escalation — 3 stages, increasingly urgent
static const uint8_t WAKEUP_STAGES = 3;                         // number of escalating pulses
static const uint32_t WAKEUP_STAGE_ON_MS[3] = {300, 600, 1200}; // buzz on-time per stage
static const uint32_t WAKEUP_STAGE_OFF_MS[3] = {700, 400, 300}; // pause between stages
// Total wake-up window ≈ 300+700 + 600+400 + 1200+300 = 3500 ms
// After the last stage, we wait an extra WAKEUP_GRACE_MS for a cancel command.
static const uint32_t WAKEUP_GRACE_MS = 1000; // extra grace period after last pulse

// Incapacitation / SOS beacon
static const uint32_t INCAP_TIMEOUT_MS = 30000;      // 30 s without DRIVER_OK → SOS
static const uint32_t SOS_BEACON_INTERVAL_MS = 2000; // re-broadcast SOS every 2 s

// Earth radius in metres (for Haversine / equirectangular)
static const float EARTH_RADIUS_M = 6371000.0f;

// ─────────────────────────── PACKET STRUCTURES (packed, ≤32 B) ─────────────
// Normal telemetry packet — 14 bytes (was 11; added vehicleID + packetID)
struct __attribute__((packed)) NormalPacket
{
    uint8_t packetType;    // 0
    uint8_t vehicleID;     // unique per vehicle (set at boot from MAC)
    uint16_t packetID;     // incrementing sequence number
    float latitude;        // 4 B
    float longitude;       // 4 B
    uint16_t speed;        // 2 B  (km/h × 10 for 0.1 resolution)
    uint8_t turnIndicator; // 0=straight, 1=left, 2=right
};
static_assert(sizeof(NormalPacket) <= 32, "NormalPacket exceeds 32-byte NRF limit");

// Emergency packet — 13 bytes (was 10; added vehicleID + packetID)
struct __attribute__((packed)) EmergencyPacket
{
    uint8_t packetType; // 1
    uint8_t vehicleID;  // unique per vehicle
    uint16_t packetID;  // incrementing sequence number
    uint8_t eventCode;  // 1=Collision, 2=Drowsy, 3=SOS (incapacitated)
    float latitude;     // 4 B
    float longitude;    // 4 B
};
static_assert(sizeof(EmergencyPacket) <= 32, "EmergencyPacket exceeds 32-byte NRF limit");

// ─────────────────────── GENERIC RING BUFFER (lock-free SPSC) ──────────────
// Single-producer, single-consumer — safe across two FreeRTOS tasks without mutex.
template <typename T, uint8_t N>
class RingBuffer
{
    static_assert((N & (N - 1)) == 0, "RingBuffer size must be a power of 2");

public:
    bool push(const T &item)
    {
        uint8_t nextHead = (head_ + 1) & (N - 1);
        if (nextHead == tail_)
            return false; // full
        buf_[head_] = item;
        head_ = nextHead;
        return true;
    }
    bool pop(T &item)
    {
        if (head_ == tail_)
            return false; // empty
        item = buf_[tail_];
        tail_ = (tail_ + 1) & (N - 1);
        return true;
    }
    uint8_t count() const
    {
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
static NMEAGPS gpsParser;                   // parser instance
static gps_fix gpsFix;                      // latest fix
static HardwareSerial &gpsSerial = Serial2; // UART2

// Ring buffer: holds raw 32-byte blobs; we decode after pop
struct RawPacket
{
    uint8_t data[32];
};

RingBuffer<RawPacket, 16> rxRingBuffer;

// Latest own-vehicle state (written on Core 0, read on Core 1)
static volatile float ownLat = 0.0f;
static volatile float ownLon = 0.0f;
static volatile uint16_t ownSpeed = 0; // km/h × 10
static volatile uint8_t ownTurn = 0;   // 0=straight

// ─── Debug counters ────────────────────────────────────────────────────────
static volatile uint32_t dbgTxNormalCount = 0;    // total normal packets sent
static volatile uint32_t dbgTxEmergencyCount = 0; // total emergency packets sent
static volatile uint32_t dbgRxRawCount = 0;       // total raw packets received
static volatile uint32_t dbgRxDupCount = 0;       // duplicates suppressed
static volatile uint32_t dbgRxOwnCount = 0;       // own packets filtered
static volatile uint32_t dbgCsmaBusyCount = 0;    // CSMA channel-busy events
static volatile uint32_t dbgGpsFixCount = 0;      // GPS fix updates

// ─── Packet identification ─────────────────────────────────────────────────
static uint8_t myVehicleID = 0; // derived from ESP32 MAC at boot
static uint16_t txPacketID = 0; // auto-incrementing per TX

// ─── CSMA backoff state ────────────────────────────────────────────────────
static uint8_t csmaBackoffAttempt = 0; // current backoff attempt count
static uint32_t csmaBackoffUntil = 0;  // millis() until which we wait

// ─── Duplicate suppression cache ───────────────────────────────────────────
struct DedupEntry
{
    uint8_t vehicleID;
    uint16_t packetID;
    uint32_t timestamp; // millis() when seen — for expiry
};
static DedupEntry dedupCache[DEDUP_CACHE_SIZE];
static uint8_t dedupIdx = 0; // circular write index

// Flags (atomic-safe on ESP32 — single-word writes)
static volatile bool emergencyDrowsy = false;
static volatile bool emergencyCollision = false;

// Non-blocking alert timer
static volatile uint32_t alertActiveUntil = 0;

// Drowsy cooldown
static volatile uint32_t lastDrowsyTrigger = 0;

// ─── Drowsy wake-up state machine ──────────────────────────────────────────
// States: 0=IDLE, 1=WAKEUP_ON (buzzer/LED on for current stage),
//         2=WAKEUP_OFF (pause between stages), 3=GRACE (waiting for cancel),
//         4=BROADCAST (wake-up failed — send emergency)
static volatile uint8_t drowsyState = 0;       // 0=IDLE
static volatile uint8_t drowsyStage = 0;       // current escalation stage (0..2)
static volatile uint32_t drowsyStateStart = 0; // millis() when current state began
static volatile bool drowsyCancelFlag = false; // set by Serial listener

// ─── Incapacitation / SOS state ────────────────────────────────────────────
// After a collision or unresponsive-drowsy event, a 30 s countdown begins.
// If the driver does not send DRIVER_OK, we enter SOS mode and repeatedly
// broadcast the vehicle's GPS location for emergency responders.
static volatile bool incapTimerActive = false; // countdown running
static volatile uint32_t incapTimerStart = 0;  // millis() when countdown began
static volatile bool sosActive = false;        // SOS beacon mode
static volatile uint32_t lastSosBeacon = 0;    // millis() of last SOS broadcast
static volatile bool driverOkFlag = false;     // set by Serial when DRIVER_OK received

// ─────────────────── MATH: EQUIRECTANGULAR DISTANCE (fast) ─────────────────
// Good enough for distances < 1 km; ~10× faster than full Haversine.
static float approxDistanceM(float lat1, float lon1, float lat2, float lon2)
{
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1) * cosf(radians((lat1 + lat2) * 0.5f));
    return EARTH_RADIUS_M * sqrtf(dLat * dLat + dLon * dLon);
}

// ─────────────────── MATH: TIME-TO-COLLISION ───────────────────────────────
// closingSpeed in m/s, distance in m.  Returns seconds.  <0 means diverging.
static float computeTTC(float distance, float closingSpeedMs)
{
    if (closingSpeedMs <= 0.0f)
        return 999.0f; // not closing
    return distance / closingSpeedMs;
}

// ─────────────────── CSMA: CARRIER SENSE ───────────────────────────────────
// Returns true if the channel appears free.  Uses testCarrier() which needs
// the radio in RX mode + ~128 µs settle time.
static bool isChannelClear()
{
    radio.startListening();
    delayMicroseconds(CSMA_CARRIER_SENSE_US); // only µs — not blocking
    return !radio.testCarrier();
}

// ─────────────────── DUPLICATE SUPPRESSION ─────────────────────────────────
// Returns true if we have already seen this (vehicleID, packetID) recently.
static bool isDuplicate(uint8_t vid, uint16_t pid)
{
    uint32_t now = millis();
    for (uint8_t i = 0; i < DEDUP_CACHE_SIZE; i++)
    {
        if (dedupCache[i].vehicleID == vid &&
            dedupCache[i].packetID == pid &&
            (now - dedupCache[i].timestamp) < 5000) // expire after 5 s
            return true;
    }
    // Not found — add to cache
    dedupCache[dedupIdx].vehicleID = vid;
    dedupCache[dedupIdx].packetID = pid;
    dedupCache[dedupIdx].timestamp = now;
    dedupIdx = (dedupIdx + 1) % DEDUP_CACHE_SIZE;
    return false;
}

// ─────── EMERGENCY MULTI-CHANNEL TRANSMIT ──────────────────────────────────
// Sends an EmergencyPacket EMERGENCY_RETRANSMITS times, once per channel in
// EMERGENCY_CHANNELS[].  Bypasses CSMA — emergencies always pre-empt.
static void sendEmergencyMultiChannel(const EmergencyPacket &pkt)
{
    for (uint8_t ch = 0; ch < NUM_EMERGENCY_CHANNELS; ch++)
    {
        radio.stopListening();
        radio.setChannel(EMERGENCY_CHANNELS[ch]);
        bool ok = radio.write(&pkt, sizeof(pkt));
        if (ch < NUM_EMERGENCY_CHANNELS - 1)
            delayMicroseconds(EMERGENCY_HOP_GAP_US); // tiny gap between hops
    }
    dbgTxEmergencyCount++;
    // Return to normal channel + RX mode
    radio.setChannel(RF_CHANNEL);
    radio.startListening();
}

// ──────────────────── ALERT SYSTEM (non-blocking) ──────────────────────────
static void fireAlert()
{
    alertActiveUntil = millis() + ALERT_DURATION_MS;
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GRN_PIN, LOW);
    digitalWrite(LED_BLU_PIN, LOW);
}

static void updateAlerts()
{
    if (millis() >= alertActiveUntil && alertActiveUntil != 0)
    {
        // Turn off
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_RED_PIN, LOW);
        alertActiveUntil = 0;
    }
}

// ──────────── DROWSY WAKE-UP STATE MACHINE (non-blocking) ──────────────────
// Called every tick from the wake-up task.  Drives the escalating buzzer/LED
// sequence and, if the driver fails to cancel, sets emergencyDrowsy so the
// RF TX task broadcasts to nearby vehicles.
static void updateDrowsyWakeup()
{
    if (drowsyState == 0)
        return; // IDLE — nothing to do

    uint32_t now = millis();
    uint32_t elapsed = now - drowsyStateStart;

    // ── Check for driver cancel at any state ──
    if (drowsyCancelFlag)
    {
        drowsyCancelFlag = false;
        drowsyState = 0;
        // Silence everything
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_BLU_PIN, LOW);
        Serial.println("[V2V] Driver responded — drowsy alert CANCELLED. No broadcast sent.");
        return;
    }

    switch (drowsyState)
    {
    case 1: // WAKEUP_ON — buzzer + LED on for this stage's duration
        if (elapsed >= WAKEUP_STAGE_ON_MS[drowsyStage])
        {
            // Turn off buzzer/LED, move to pause
            digitalWrite(BUZZER_PIN, LOW);
            digitalWrite(LED_RED_PIN, LOW);
            digitalWrite(LED_BLU_PIN, LOW);
            drowsyState = 2; // WAKEUP_OFF
            drowsyStateStart = now;
        }
        break;

    case 2: // WAKEUP_OFF — pause between stages
        if (elapsed >= WAKEUP_STAGE_OFF_MS[drowsyStage])
        {
            drowsyStage++;
            if (drowsyStage >= WAKEUP_STAGES)
            {
                // All stages exhausted — enter grace period
                drowsyState = 3; // GRACE
                drowsyStateStart = now;
                Serial.println("[V2V] Wake-up pulses complete. Waiting for driver response...");
            }
            else
            {
                // Next escalation stage — turn on with more intensity
                drowsyState = 1; // WAKEUP_ON
                drowsyStateStart = now;
                digitalWrite(BUZZER_PIN, HIGH);
                digitalWrite(LED_RED_PIN, HIGH);
                // Stage 2+ adds blue LED for extra visual urgency
                if (drowsyStage >= 1)
                    digitalWrite(LED_BLU_PIN, HIGH);
                Serial.print("[V2V] Wake-up stage ");
                Serial.print(drowsyStage + 1);
                Serial.print("/");
                Serial.print(WAKEUP_STAGES);
                Serial.println(" — ESCALATING");
            }
        }
        break;

    case 3: // GRACE — final window for driver to send CANCEL_DROWSY
        if (elapsed >= WAKEUP_GRACE_MS)
        {
            // No cancel received — driver is unresponsive
            drowsyState = 4; // BROADCAST
            Serial.println("[V2V] Driver unresponsive! Broadcasting emergency to nearby vehicles.");
        }
        break;

    case 4: // BROADCAST — set the flag so RF TX task sends emergency packet
        emergencyDrowsy = true;
        drowsyState = 0; // back to IDLE
        break;
    }
}

// ─────── Start incapacitation countdown (called after collision / drowsy) ───
static void startIncapTimer()
{
    if (!incapTimerActive && !sosActive)
    {
        incapTimerActive = true;
        incapTimerStart = millis();
        Serial.println("[V2V] ⚠ Incapacitation timer STARTED — send DRIVER_OK within 30 s to cancel.");
    }
}

// ──────────── INCAPACITATION / SOS STATE MACHINE (non-blocking) ────────────
// Monitors the 30 s countdown and, once expired, repeatedly broadcasts SOS
// packets and logs to serial so the laptop can relay to emergency services.
static void updateIncapSos()
{
    uint32_t now = millis();

    // ── Driver confirmed OK — cancel everything ──
    if (driverOkFlag)
    {
        driverOkFlag = false;
        incapTimerActive = false;
        sosActive = false;
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_BLU_PIN, LOW);
        Serial.println("[V2V] DRIVER_OK received — SOS / incap timer CANCELLED.");
        return;
    }

    // ── Countdown phase ──
    if (incapTimerActive && !sosActive)
    {
        uint32_t elapsed = now - incapTimerStart;
        if (elapsed >= INCAP_TIMEOUT_MS)
        {
            // Driver did not respond in 30 s — enter SOS mode
            incapTimerActive = false;
            sosActive = true;
            lastSosBeacon = 0; // force immediate first beacon
            Serial.println("[V2V] ══════════════════════════════════════");
            Serial.println("[V2V]  DRIVER INCAPACITATED — SOS MODE ON");
            Serial.println("[V2V]  Broadcasting location to all nearby");
            Serial.println("[V2V]  vehicles and emergency responders.");
            Serial.println("[V2V] ══════════════════════════════════════");
        }
        else
        {
            // Periodic reminder every 10 s
            static uint32_t lastReminder = 0;
            if (now - lastReminder >= 10000)
            {
                lastReminder = now;
                Serial.print("[V2V] Incap countdown: ");
                Serial.print((INCAP_TIMEOUT_MS - elapsed) / 1000);
                Serial.println("s remaining. Send DRIVER_OK to cancel.");
            }
        }
    }

    // ── SOS beacon mode — broadcast every SOS_BEACON_INTERVAL_MS ──
    if (sosActive)
    {
        if (now - lastSosBeacon >= SOS_BEACON_INTERVAL_MS)
        {
            lastSosBeacon = now;

            // Build SOS emergency packet (eventCode 3)
            EmergencyPacket pkt;
            pkt.packetType = 1;
            pkt.vehicleID = myVehicleID;
            pkt.packetID = txPacketID++;
            pkt.eventCode = 3; // SOS — driver incapacitated
            pkt.latitude = ownLat;
            pkt.longitude = ownLon;

            // Multi-channel broadcast — bypass CSMA
            sendEmergencyMultiChannel(pkt);

            // Persistent alert — keep buzzer + LEDs on
            digitalWrite(BUZZER_PIN, HIGH);
            digitalWrite(LED_RED_PIN, HIGH);
            digitalWrite(LED_BLU_PIN, HIGH);

            // Log to serial for emergency-responder relay
            Serial.println("========== SOS BEACON ==========");
            Serial.println("EVENT: DRIVER_INCAPACITATED");
            Serial.print("LAT: ");
            Serial.println(ownLat, 6);
            Serial.print("LON: ");
            Serial.println(ownLon, 6);
            Serial.print("UPTIME_MS: ");
            Serial.println(now);
            Serial.println("ACTION_REQUIRED: DISPATCH EMERGENCY SERVICES");
            Serial.println("================================");
        }
    }
}

// ────────────────── SERIAL LOGGING FOR EMERGENCY RESPONDERS ────────────────
// When a collision is detected, log GPS coordinates + timestamp to Serial
// so the connected laptop can capture them for emergency services (no cloud).
static void logCollisionToSerial(float lat, float lon, uint8_t eventCode)
{
    Serial.println("========== EMERGENCY EVENT ==========");
    Serial.print("EVENT: ");
    switch (eventCode)
    {
    case 1:
        Serial.println("COLLISION");
        break;
    case 2:
        Serial.println("DROWSY_DRIVER");
        break;
    case 3:
        Serial.println("DRIVER_INCAPACITATED");
        break;
    default:
        Serial.println("UNKNOWN");
        break;
    }
    Serial.print("LAT: ");
    Serial.println(lat, 6);
    Serial.print("LON: ");
    Serial.println(lon, 6);
    Serial.print("UPTIME_MS: ");
    Serial.println(millis());
    Serial.println("ACTION_REQUIRED: NOTIFY EMERGENCY SERVICES");
    Serial.println("======================================");
}

/*============================================================================
 *  CORE 0 TASKS — GPS & Serial Listener
 *==========================================================================*/

// ── Task: GPS UART Parsing ────────────────────────────────────────────────
// Reads NMEA sentences from NEO-6M at 9600 baud.  NeoGPS handles incremental
// parse so we never block.
void taskGPS(void *pvParameters)
{
    (void)pvParameters;
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    static uint32_t lastGpsLog = 0;

    for (;;)
    {
        while (gpsParser.available(gpsSerial))
        {
            gpsFix = gpsParser.read();
            bool locUpdated = false;
            bool spdUpdated = false;

            if (gpsFix.valid.location)
            {
                ownLat = gpsFix.latitude();
                ownLon = gpsFix.longitude();
                locUpdated = true;
            }
            if (gpsFix.valid.speed)
            {
                // NeoGPS gives speed in km/h as float; store × 10 for 0.1 res
                ownSpeed = (uint16_t)(gpsFix.speed_kph() * 10.0f);
                spdUpdated = true;
            }

            // Update GPS fix count (no logging to serial)
            uint32_t now = millis();
            if (now - lastGpsLog >= 2000)
            {
                lastGpsLog = now;
                dbgGpsFixCount++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // yield — not a blocking delay
    }
}

// ── Task: Serial Listener (Laptop → ESP32 for DROWSY_ALERT) ──────────────
// Non-blocking: polls Serial.available() on a short interval.
// Reads accumulated chars into a small line buffer; when '\n' is seen,
// checks for the "DROWSY_ALERT" keyword.
void taskSerialListener(void *pvParameters)
{
    (void)pvParameters;

    static char lineBuf[64];
    static uint8_t lineIdx = 0;
    uint32_t lastPoll = 0;

    for (;;)
    {
        uint32_t now = millis();
        if (now - lastPoll < SERIAL_POLL_INTERVAL)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        lastPoll = now;

        while (Serial.available())
        {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r')
            {
                lineBuf[lineIdx] = '\0';

                // ── Check for DRIVER_OK (driver confirmed conscious) ──
                if (lineIdx > 0 && strstr(lineBuf, "DRIVER_OK") != nullptr)
                {
                    driverOkFlag = true;     // cancels incap timer / SOS
                    drowsyCancelFlag = true; // also cancels any drowsy wake-up
                }
                // ── Check for CANCEL_DROWSY (driver confirmed awake) ──
                else if (lineIdx > 0 && strstr(lineBuf, "CANCEL_DROWSY") != nullptr)
                {
                    drowsyCancelFlag = true;
                }
                // ── Check for DROWSY_ALERT ──
                else if (lineIdx > 0 && strstr(lineBuf, "DROWSY_ALERT") != nullptr)
                {
                    if (now - lastDrowsyTrigger > DROWSY_ALERT_REARM_MS)
                    {
                        lastDrowsyTrigger = now;
                        // Start wake-up escalation — do NOT broadcast yet
                        drowsyStage = 0;
                        drowsyState = 1; // WAKEUP_ON
                        drowsyStateStart = now;
                        drowsyCancelFlag = false;
                        // Turn on buzzer + red LED for first (gentlest) stage
                        digitalWrite(BUZZER_PIN, HIGH);
                        digitalWrite(LED_RED_PIN, HIGH);
                    }
                }
                lineIdx = 0;
            }
            else if (lineIdx < sizeof(lineBuf) - 1)
            {
                lineBuf[lineIdx++] = c;
            }
        }
    }
}

/*============================================================================
 *  CORE 1 TASKS — RF TX/RX & Risk Assessment
 *==========================================================================*/

// ── Task: RF Transmit ─────────────────────────────────────────────────────
// Broadcasts NormalPacket at ~1 Hz (+ random jitter).  Emergency packets
// pre-empt immediately, bypass CSMA, and hop across multiple channels.
void taskRFTransmit(void *pvParameters)
{
    (void)pvParameters;
    uint32_t lastTx = 0;
    uint32_t txJitter = random(0, TX_JITTER_MAX_MS); // initial jitter

    for (;;)
    {
        uint32_t now = millis();

        // ══════════════════════════════════════════════════════════════
        // PRIORITY 1: Emergency packets — bypass CSMA, multi-channel
        // ══════════════════════════════════════════════════════════════
        if (emergencyDrowsy)
        {
            emergencyDrowsy = false;
            EmergencyPacket pkt;
            pkt.packetType = 1;
            pkt.vehicleID = myVehicleID;
            pkt.packetID = txPacketID++;
            pkt.eventCode = 2; // Drowsy
            pkt.latitude = ownLat;
            pkt.longitude = ownLon;

            // Transmit across all emergency channels (no carrier check)
            sendEmergencyMultiChannel(pkt);

            logCollisionToSerial(pkt.latitude, pkt.longitude, pkt.eventCode);
            fireAlert(); // local alert too

            // Driver was unresponsive during wake-up → start incap countdown
            startIncapTimer();
        }

        if (emergencyCollision)
        {
            emergencyCollision = false;
            EmergencyPacket pkt;
            pkt.packetType = 1;
            pkt.vehicleID = myVehicleID;
            pkt.packetID = txPacketID++;
            pkt.eventCode = 1; // Collision
            pkt.latitude = ownLat;
            pkt.longitude = ownLon;

            // Transmit across all emergency channels (no carrier check)
            sendEmergencyMultiChannel(pkt);

            logCollisionToSerial(pkt.latitude, pkt.longitude, pkt.eventCode);
            fireAlert();

            // Collision detected → start incap countdown
            startIncapTimer();
        }

        // ══════════════════════════════════════════════════════════════
        // PRIORITY 2: Normal 1 Hz telemetry — with CSMA + jitter
        // ══════════════════════════════════════════════════════════════
        if (now - lastTx >= NORMAL_TX_INTERVAL_MS + txJitter)
        {
            // ── CSMA: check if in backoff ──
            if (now < csmaBackoffUntil)
            {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }

            // ── CSMA: carrier sense ──
            if (isChannelClear())
            {
                // Channel free — transmit
                lastTx = now;
                txJitter = random(0, TX_JITTER_MAX_MS); // new jitter for next cycle
                csmaBackoffAttempt = 0;

                NormalPacket pkt;
                pkt.packetType = 0;
                pkt.vehicleID = myVehicleID;
                pkt.packetID = txPacketID++;
                pkt.latitude = ownLat;
                pkt.longitude = ownLon;
                pkt.speed = ownSpeed;
                pkt.turnIndicator = ownTurn;

                radio.stopListening();
                bool ok = radio.write(&pkt, sizeof(pkt));
                radio.startListening();
                dbgTxNormalCount++;
            }
            else
            {
                // Channel busy — exponential backoff
                dbgCsmaBusyCount++;
                csmaBackoffAttempt++;

                if (csmaBackoffAttempt >= CSMA_MAX_BACKOFF_TRIES)
                {
                    // Give up — skip this TX cycle, reset for next
                    lastTx = now;
                    txJitter = random(0, TX_JITTER_MAX_MS);
                    csmaBackoffAttempt = 0;
                }
                else
                {
                    // backoff = random(1..10) * attempt  (in ms)
                    uint32_t bo = (uint32_t)random(1, 11) * csmaBackoffAttempt;
                    csmaBackoffUntil = now + bo;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ── Task: RF Receive ──────────────────────────────────────────────────────
// Non-blocking poll; pushes raw 32-byte payloads into ring buffer.
// Periodically hops to emergency channels to catch multi-channel broadcasts.
void taskRFReceive(void *pvParameters)
{
    (void)pvParameters;
    uint8_t rxChIdx = 0;                   // index into EMERGENCY_CHANNELS for scanning
    uint32_t lastChHop = 0;                // millis() of last channel hop
    const uint32_t RX_HOP_DWELL_MS = 50;   // dwell 50 ms on each emergency channel
    const uint32_t RX_HOME_DWELL_MS = 200; // stay on home channel longer
    bool onHomeChannel = true;

    for (;;)
    {
        // Read all available packets on current channel
        while (radio.available())
        {
            RawPacket pkt;
            radio.read(&pkt.data, sizeof(pkt.data));
            rxRingBuffer.push(pkt);
            dbgRxRawCount++;

            uint8_t pType = pkt.data[0];
            uint8_t curCh = onHomeChannel ? RF_CHANNEL : EMERGENCY_CHANNELS[rxChIdx];
            Serial.print("[RX] Raw packet on ch=");
            Serial.print(curCh);
            Serial.print(" type=");
            Serial.print(pType);
            Serial.print(" rxTotal=");
            Serial.print(dbgRxRawCount);
            Serial.print(" bufUsed=");
            Serial.println(rxRingBuffer.count());
        }

        // ── Channel hopping for emergency reception ──
        uint32_t now = millis();
        uint32_t dwellTime = onHomeChannel ? RX_HOME_DWELL_MS : RX_HOP_DWELL_MS;
        if (now - lastChHop >= dwellTime)
        {
            lastChHop = now;
            if (onHomeChannel)
            {
                // Hop to first emergency channel
                rxChIdx = 0;
                onHomeChannel = false;
                radio.stopListening();
                radio.setChannel(EMERGENCY_CHANNELS[rxChIdx]);
                radio.startListening();
            }
            else
            {
                rxChIdx++;
                if (rxChIdx >= NUM_EMERGENCY_CHANNELS)
                {
                    // Return to home channel
                    onHomeChannel = true;
                    radio.stopListening();
                    radio.setChannel(RF_CHANNEL);
                    radio.startListening();
                }
                else
                {
                    radio.stopListening();
                    radio.setChannel(EMERGENCY_CHANNELS[rxChIdx]);
                    radio.startListening();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ── Task: Risk Assessment ─────────────────────────────────────────────────
// Pops packets from ring buffer, decodes, deduplicates, computes distance
// & TTC, fires alerts when thresholds are breached.
void taskRiskAssessment(void *pvParameters)
{
    (void)pvParameters;

    // Store last-known positions for TTC (closing speed) calculation
    static float lastKnownDist = 999.0f;
    static uint32_t lastDistTime = 0;

    for (;;)
    {
        RawPacket pktRaw;
        while (rxRingBuffer.pop(pktRaw))
        {
            uint8_t *raw = pktRaw.data;
            uint8_t packetType = raw[0];

            if (packetType == 0)
            {
                // ── Normal Packet ──
                NormalPacket pkt;
                memcpy(&pkt, raw, sizeof(NormalPacket));

                // Ignore our own packets
                if (pkt.vehicleID == myVehicleID)
                {
                    dbgRxOwnCount++;
                    continue;
                }

                // Duplicate suppression
                if (isDuplicate(pkt.vehicleID, pkt.packetID))
                {
                    dbgRxDupCount++;
                    Serial.print("[RISK] Duplicate normal pkt suppressed: vID=");
                    Serial.print(pkt.vehicleID);
                    Serial.print(" pktID=");
                    Serial.println(pkt.packetID);
                    continue;
                }

                float dist = approxDistanceM(ownLat, ownLon, pkt.latitude, pkt.longitude);

                // Closing speed estimate (Δdist / Δtime)
                uint32_t now = millis();
                float closingSpeedMs = 0.0f;
                if (lastDistTime > 0 && now > lastDistTime)
                {
                    float dt = (now - lastDistTime) / 1000.0f;
                    closingSpeedMs = (lastKnownDist - dist) / dt; // +ve = closing
                }
                lastKnownDist = dist;
                lastDistTime = now;

                float ttc = computeTTC(dist, closingSpeedMs);

                // Log every received normal packet with full detail
                Serial.print("[RISK] Normal from vID=");
                Serial.print(pkt.vehicleID);
                Serial.print(" pktID=");
                Serial.print(pkt.packetID);
                Serial.print(" lat=");
                Serial.print(pkt.latitude, 6);
                Serial.print(" lon=");
                Serial.print(pkt.longitude, 6);
                Serial.print(" spd=");
                Serial.print(pkt.speed / 10.0f, 1);
                Serial.print("km/h turn=");
                Serial.print(pkt.turnIndicator);
                Serial.print(" | dist=");
                Serial.print(dist, 1);
                Serial.print("m closing=");
                Serial.print(closingSpeedMs, 2);
                Serial.print("m/s TTC=");
                Serial.print(ttc, 1);
                Serial.println("s");

                // ── Threshold check ──
                if (dist < ALERT_DISTANCE_M || ttc < ALERT_TTC_S)
                {
                    fireAlert();

                    Serial.print("[RISK] *** PROXIMITY ALERT! dist=");
                    Serial.print(dist, 1);
                    Serial.print("m  TTC=");
                    Serial.print(ttc, 1);
                    Serial.print("s  vID=");
                    Serial.print(pkt.vehicleID);
                    Serial.print("  pktID=");
                    Serial.println(pkt.packetID);

                    // If TTC is critically low, treat as imminent collision
                    if (ttc < 1.5f && dist < 30.0f)
                    {
                        Serial.println("[RISK] !!! IMMINENT COLLISION — triggering emergency");
                        emergencyCollision = true;
                        logCollisionToSerial(ownLat, ownLon, 1);
                    }
                }
            }
            else if (packetType == 1)
            {
                // ── Emergency Packet ──
                EmergencyPacket pkt;
                memcpy(&pkt, raw, sizeof(EmergencyPacket));

                // Ignore our own packets
                if (pkt.vehicleID == myVehicleID)
                {
                    dbgRxOwnCount++;
                    continue;
                }

                // Duplicate suppression — critical for multi-channel rebroadcasts
                if (isDuplicate(pkt.vehicleID, pkt.packetID))
                {
                    dbgRxDupCount++;
                    Serial.print("[RISK] Duplicate emergency pkt suppressed: vID=");
                    Serial.print(pkt.vehicleID);
                    Serial.print(" pktID=");
                    Serial.println(pkt.packetID);
                    continue;
                }

                fireAlert();

                const char *evtName;
                switch (pkt.eventCode)
                {
                case 1:
                    evtName = "COLLISION";
                    break;
                case 2:
                    evtName = "DROWSY";
                    break;
                case 3:
                    evtName = "SOS";
                    break;
                default:
                    evtName = "UNKNOWN";
                    break;
                }
                Serial.print("[RISK] *** EMERGENCY from vID=");
                Serial.print(pkt.vehicleID);
                Serial.print(" pktID=");
                Serial.print(pkt.packetID);
                Serial.print(" Event=");
                Serial.print(evtName);
                Serial.print(" lat=");
                Serial.print(pkt.latitude, 6);
                Serial.print(" lon=");
                Serial.println(pkt.longitude, 6);

                if (pkt.eventCode == 3)
                {
                    Serial.println("[RISK] ⚠ Nearby vehicle driver is INCAPACITATED — SOS beacon!");
                }

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
void setup()
{
    // ── Serial (USB — laptop comms + debug) ──
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    Serial.println("\n[SYS] ========================================");
    Serial.println("[SYS]  Vehicle-to-Vehicle Safety System v2.0");
    Serial.println("[SYS]  VERBOSE DEBUG MODE — all values printed");
    Serial.println("[SYS]  No-Cloud | CSMA/CA | Multi-Channel");
    Serial.println("[SYS] ========================================");
    Serial.print("[SYS] Free heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
    Serial.print("[SYS] CPU freq: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");
    Serial.print("[SYS] Chip model: ");
    Serial.println(ESP.getChipModel());

    // ── Vehicle ID from ESP32 MAC (unique per chip) ──
    // Use lowest byte of the base MAC — simple and unique enough for local RF
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    myVehicleID = mac[5];                   // last byte is the most unique
    randomSeed(mac[4] ^ (millis() & 0xFF)); // seed RNG with MAC + boot time
    Serial.print("[V2V] Vehicle ID: ");
    Serial.println(myVehicleID);

    // ── Initialise dedup cache ──
    memset(dedupCache, 0, sizeof(dedupCache));

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
    if (!radio.begin())
    {
        Serial.println("[V2V] ERROR: NRF24L01 not detected! Check wiring.");
        // Blink red forever (non-blocking in the main loop would be better,
        // but we halt here because the system is non-functional without radio)
        while (true)
        {
            digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    radio.setChannel(RF_CHANNEL);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_2MBPS);
    radio.setPayloadSize(32);
    radio.setAutoAck(false); // broadcast mode — no ACKs
    radio.openWritingPipe(RF_PIPE_ADDR);
    radio.openReadingPipe(1, RF_PIPE_ADDR);
    radio.startListening();
    Serial.println("[SYS] NRF24L01 initialised OK");
    Serial.print("[SYS]   Home channel: ");
    Serial.println(RF_CHANNEL);
    Serial.print("[SYS]   PA level: MAX | Data rate: 2Mbps | Payload: 32B");
    Serial.println();
    Serial.print("[SYS]   Emergency channels: ");
    for (uint8_t i = 0; i < NUM_EMERGENCY_CHANNELS; i++)
    {
        Serial.print(EMERGENCY_CHANNELS[i]);
        if (i < NUM_EMERGENCY_CHANNELS - 1)
            Serial.print(", ");
    }
    Serial.println();
    Serial.print("[SYS]   NormalPacket size: ");
    Serial.print(sizeof(NormalPacket));
    Serial.print("B  EmergencyPacket size: ");
    Serial.print(sizeof(EmergencyPacket));
    Serial.println("B");

    // ── FreeRTOS Task Creation ──
    // Core 0: GPS + Serial Listener
    xTaskCreatePinnedToCore(
        taskGPS,     // function
        "GPS_Parse", // name
        4096,        // stack (bytes)
        NULL,        // params
        2,           // priority (higher = more urgent)
        NULL,        // handle
        0            // core 0
    );

    xTaskCreatePinnedToCore(
        taskSerialListener,
        "Serial_Listen",
        4096,
        NULL,
        1, // lower priority than GPS
        NULL,
        0);

    // Core 1: RF TX, RF RX, Risk Assessment
    xTaskCreatePinnedToCore(
        taskRFTransmit,
        "RF_TX",
        4096,
        NULL,
        3, // highest priority — emergency TX must pre-empt
        NULL,
        1);

    xTaskCreatePinnedToCore(
        taskRFReceive,
        "RF_RX",
        4096,
        NULL,
        3, // equal to TX
        NULL,
        1);

    xTaskCreatePinnedToCore(
        taskRiskAssessment,
        "Risk_Assess",
        4096,
        NULL,
        2,
        NULL,
        1);

    // Drowsy wake-up state machine — runs on Core 1 alongside other RF tasks
    xTaskCreatePinnedToCore(
        [](void *)
        {
            for (;;)
            {
                updateDrowsyWakeup();
                vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz update — smooth escalation
            }
        },
        "Drowsy_Wake",
        2048,
        NULL,
        2, // same priority as risk assessment
        NULL,
        1 // core 1
    );

    // Incapacitation / SOS beacon task — monitors 30 s countdown + SOS broadcast
    xTaskCreatePinnedToCore(
        [](void *)
        {
            for (;;)
            {
                updateIncapSos();
                vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz — sufficient for 2 s beacon
            }
        },
        "Incap_SOS",
        2048,
        NULL,
        3, // high priority — SOS must not be starved
        NULL,
        1 // core 1
    );

    Serial.println("[SYS] All FreeRTOS tasks launched. System active.");
    Serial.println("[SYS] ── Debug tags: [GPS] [TX] [RX] [CSMA] [RISK] [SOS] [DROWSY] [CMD] [SYS] ──");
    digitalWrite(LED_GRN_PIN, LOW); // end self-test blink
}

// Arduino loop() is essentially idle — all real work is in FreeRTOS tasks.
// We use it for heartbeat LED + periodic debug status dump.
void loop()
{
    static uint32_t lastHeartbeat = 0;
    static uint32_t lastStatusDump = 0;
    uint32_t now = millis();

    // ── Heartbeat blink ──
    if (now - lastHeartbeat >= 2000)
    {
        lastHeartbeat = now;
        // Brief green blink = system alive
        digitalWrite(LED_GRN_PIN, HIGH);
    }
    if (now - lastHeartbeat >= 100 && digitalRead(LED_GRN_PIN))
    {
        digitalWrite(LED_GRN_PIN, LOW);
    }

    // ── Periodic status dump every 10 s ──
    if (now - lastStatusDump >= 10000)
    {
        lastStatusDump = now;
        Serial.println("────────────── [SYS] STATUS DUMP ──────────────");
        Serial.print("  Uptime: ");
        Serial.print(now / 1000);
        Serial.println("s");
        Serial.print("  Free heap: ");
        Serial.print(ESP.getFreeHeap());
        Serial.println(" bytes");
        Serial.print("  GPS: lat=");
        Serial.print(ownLat, 6);
        Serial.print(" lon=");
        Serial.print(ownLon, 6);
        Serial.print(" speed=");
        Serial.print(ownSpeed / 10.0f, 1);
        Serial.print("km/h fixes=");
        Serial.println(dbgGpsFixCount);
        Serial.print("  TX: normal=");
        Serial.print(dbgTxNormalCount);
        Serial.print(" emergency=");
        Serial.print(dbgTxEmergencyCount);
        Serial.print(" nextPktID=");
        Serial.println(txPacketID);
        Serial.print("  RX: raw=");
        Serial.print(dbgRxRawCount);
        Serial.print(" dup=");
        Serial.print(dbgRxDupCount);
        Serial.print(" own=");
        Serial.print(dbgRxOwnCount);
        Serial.print(" bufUsed=");
        Serial.println(rxRingBuffer.count());
        Serial.print("  CSMA: busyEvents=");
        Serial.println(dbgCsmaBusyCount);
        Serial.print("  State: drowsy=");
        Serial.print(drowsyState);
        Serial.print(" incapTimer=");
        Serial.print(incapTimerActive ? "ON" : "OFF");
        Serial.print(" SOS=");
        Serial.println(sosActive ? "ON" : "OFF");
        Serial.print("  GPIO: buzzer=");
        Serial.print(digitalRead(BUZZER_PIN) ? "ON" : "OFF");
        Serial.print(" red=");
        Serial.print(digitalRead(LED_RED_PIN) ? "ON" : "OFF");
        Serial.print(" grn=");
        Serial.print(digitalRead(LED_GRN_PIN) ? "ON" : "OFF");
        Serial.print(" blu=");
        Serial.println(digitalRead(LED_BLU_PIN) ? "ON" : "OFF");
        Serial.println("───────────────────────────────────────────────");
    }
}
