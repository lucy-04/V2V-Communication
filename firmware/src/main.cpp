/*******************************************************************************
 * V2V Communication System — ESP32 Firmware
 * ===========================================================================
 * Decentralised, offline Vehicle-to-Vehicle safety system.
 * Hardware: ESP32 + NRF24L01 + NEO-6M GPS + SSD1306 OLED + Buzzer + LEDs
 *
 * Architecture
 *   Core 0 → GPS UART parsing, Serial listener, OLED display
 *   Core 1 → RF TX/RX, risk-assessment math, GPIO alerts
 *
 * Display
 *   • 128×64 SSD1306 OLED (I2C) shows all system status
 *   • Serial is reserved for Python script commands + emergency logs
 *
 * Coding rules
 *   • NO delay()  — millis()-based non-blocking timers only
 *   • Packed structs ≤ 32 bytes (NRF24L01 hard limit)
 *   • Circular ring buffer for inbound RF packets
 ******************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <stdint.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RF24.h>
#include <NMEAGPS.h>

// === [ HARDWARE PIN DEFINITIONS ] ===
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 5

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 22
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_I2C_ADDR 0x3C

#define BUZZER_PIN 25
#define LED_RED_PIN 26
#define LED_GRN_PIN 27
#define LED_BLU_PIN 14

// === [ SYSTEM CONSTANTS & THRESHOLDS ] ===
static const uint64_t RF_PIPE_ADDR = 0xF0F0F0F0E1LL;
static const uint8_t RF_CHANNEL = 108;
static const uint32_t NORMAL_TX_INTERVAL_MS = 1000;
static const float ALERT_DISTANCE_M = 50.0f;
static const float ALERT_TTC_S = 3.0f;
static const uint32_t ALERT_DURATION_MS = 2000;
static const uint32_t SERIAL_POLL_INTERVAL = 50;
static const uint32_t DROWSY_ALERT_REARM_MS = 5000;

// -> CSMA-CA, Random Jitter & Multi-Channel Hopping Constants
static const uint32_t TX_JITTER_MAX_MS = 200;
static const uint8_t CSMA_CARRIER_SENSE_US = 130;
static const uint8_t CSMA_MAX_BACKOFF_TRIES = 5;
static const uint8_t EMERGENCY_CHANNELS[] = {90, 108, 120};
static const uint8_t NUM_EMERGENCY_CHANNELS = sizeof(EMERGENCY_CHANNELS);
static const uint32_t EMERGENCY_HOP_GAP_US = 500;

// Duplicate suppression
static const uint8_t DEDUP_CACHE_SIZE = 32;

// Drowsy wake-up escalation
static const uint8_t WAKEUP_STAGES = 3;
static const uint32_t WAKEUP_STAGE_ON_MS[3] = {300, 600, 1200};
static const uint32_t WAKEUP_STAGE_OFF_MS[3] = {700, 400, 300};
static const uint32_t WAKEUP_GRACE_MS = 1000;

// Incapacitation / SOS beacon
static const uint32_t INCAP_TIMEOUT_MS = 30000;
static const uint32_t SOS_BEACON_INTERVAL_MS = 2000;

// OLED display
static const uint32_t OLED_REFRESH_MS = 250;
static const uint32_t OLED_PAGE_CYCLE_MS = 3000;
static const uint32_t OLED_ALERT_DISPLAY_MS = 3000;

// Earth radius in metres
static const float EARTH_RADIUS_M = 6371000.0f;

// === [ NETWORK PACKET STRUCTURES (Max 32 Bytes) ] ===
struct __attribute__((packed)) NormalPacket
{
    uint8_t packetType;    // 0
    uint8_t vehicleID;
    uint16_t packetID;
    float latitude;
    float longitude;
    uint16_t speed;        // km/h × 10
    uint8_t turnIndicator; // 0=straight, 1=left, 2=right
};
static_assert(sizeof(NormalPacket) <= 32, "NormalPacket exceeds 32-byte NRF limit");

struct __attribute__((packed)) EmergencyPacket
{
    uint8_t packetType; // 1
    uint8_t vehicleID;
    uint16_t packetID;
    uint8_t eventCode;  // 1=Collision, 2=Drowsy, 3=SOS
    float latitude;
    float longitude;
};
static_assert(sizeof(EmergencyPacket) <= 32, "EmergencyPacket exceeds 32-byte NRF limit");

// === [ LOCK-FREE SPSC RING BUFFER ] ===
template <typename T, uint8_t N>
class RingBuffer
{
    static_assert((N & (N - 1)) == 0, "RingBuffer size must be a power of 2");

public:
    bool push(const T &item)
    {
        uint8_t nextHead = (head_ + 1) & (N - 1);
        if (nextHead == tail_)
            return false;
        buf_[head_] = item;
        head_ = nextHead;
        return true;
    }
    bool pop(T &item)
    {
        if (head_ == tail_)
            return false;
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

// === [ GLOBAL STATE & OBJECTS ] ===
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

static NMEAGPS gpsParser;
static gps_fix gpsFix;
static HardwareSerial &gpsSerial = Serial2;

struct RawPacket
{
    uint8_t data[32];
};
RingBuffer<RawPacket, 16> rxRingBuffer;

// Own-vehicle state (written Core 0, read Core 1)
static volatile float ownLat = 0.0f;
static volatile float ownLon = 0.0f;
static volatile uint16_t ownSpeed = 0;
static volatile uint8_t ownTurn = 0;

// Debug counters
static volatile uint32_t dbgTxNormalCount = 0;
static volatile uint32_t dbgTxEmergencyCount = 0;
static volatile uint32_t dbgRxRawCount = 0;
static volatile uint32_t dbgRxDupCount = 0;
static volatile uint32_t dbgRxOwnCount = 0;
static volatile uint32_t dbgCsmaBusyCount = 0;
static volatile uint32_t dbgGpsFixCount = 0;

// Packet identification
static uint8_t myVehicleID = 0;
static uint16_t txPacketID = 0;

// CSMA backoff state
static uint8_t csmaBackoffAttempt = 0;
static uint32_t csmaBackoffUntil = 0;

// Duplicate suppression cache
struct DedupEntry
{
    uint8_t vehicleID;
    uint16_t packetID;
    uint32_t timestamp;
};
static DedupEntry dedupCache[DEDUP_CACHE_SIZE];
static uint8_t dedupIdx = 0;

// Flags
static volatile bool emergencyDrowsy = false;
static volatile bool emergencyCollision = false;

// Non-blocking alert timer
static volatile uint32_t alertActiveUntil = 0;

// Drowsy cooldown
static volatile uint32_t lastDrowsyTrigger = 0;

// Drowsy wake-up state machine
static volatile uint8_t drowsyState = 0;
static volatile uint8_t drowsyStage = 0;
static volatile uint32_t drowsyStateStart = 0;
static volatile bool drowsyCancelFlag = false;

// Incapacitation / SOS state
static volatile bool incapTimerActive = false;
static volatile uint32_t incapTimerStart = 0;
static volatile bool sosActive = false;
static volatile uint32_t lastSosBeacon = 0;
static volatile bool driverOkFlag = false;

// OLED display state (cross-task)
static volatile char oledAlertText[22] = "";
static volatile uint32_t oledAlertStart = 0;
static volatile float oledLastDist = 0.0f;
static volatile float oledLastTTC = 999.0f;

// === [ MATHEMATICAL UTILITIES ] ===
static float approxDistanceM(float lat1, float lon1, float lat2, float lon2)
{
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1) * cosf(radians((lat1 + lat2) * 0.5f));
    return EARTH_RADIUS_M * sqrtf(dLat * dLat + dLon * dLon);
}

static float computeTTC(float distance, float closingSpeedMs)
{
    if (closingSpeedMs <= 0.0f)
        return 999.0f;
    return distance / closingSpeedMs;
}

// === [ CARRIER SENSE MULTIPLE ACCESS (CSMA) ] ===
static bool isChannelClear()
{
    radio.startListening();
    delayMicroseconds(CSMA_CARRIER_SENSE_US);
    return !radio.testCarrier();
}

// === [ PACKET DUPLICATE SUPPRESSION ] ===
static bool isDuplicate(uint8_t vid, uint16_t pid)
{
    uint32_t now = millis();
    for (uint8_t i = 0; i < DEDUP_CACHE_SIZE; i++)
    {
        if (dedupCache[i].vehicleID == vid &&
            dedupCache[i].packetID == pid &&
            (now - dedupCache[i].timestamp) < 5000)
            return true;
    }
    dedupCache[dedupIdx].vehicleID = vid;
    dedupCache[dedupIdx].packetID = pid;
    dedupCache[dedupIdx].timestamp = now;
    dedupIdx = (dedupIdx + 1) % DEDUP_CACHE_SIZE;
    return false;
}

// === [ EMERGENCY BROADCAST LOGIC ] ===
static void sendEmergencyMultiChannel(const EmergencyPacket &pkt)
{
    for (uint8_t ch = 0; ch < NUM_EMERGENCY_CHANNELS; ch++)
    {
        radio.stopListening();
        radio.setChannel(EMERGENCY_CHANNELS[ch]);
        radio.write(&pkt, sizeof(pkt));
        if (ch < NUM_EMERGENCY_CHANNELS - 1)
            delayMicroseconds(EMERGENCY_HOP_GAP_US);
    }
    dbgTxEmergencyCount++;
    radio.setChannel(RF_CHANNEL);
    radio.startListening();
}

// === [ HARDWARE ALERT CONTROLLER (Non-blocking) ] ===
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
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_RED_PIN, LOW);
        alertActiveUntil = 0;
    }
}

// -> Helper: Push text to the OLED alert overlay
static void setOledAlert(const char *text)
{
    strncpy((char *)oledAlertText, text, sizeof(oledAlertText) - 1);
    ((char *)oledAlertText)[sizeof(oledAlertText) - 1] = '\0';
    oledAlertStart = millis();
}

// === [ DROWSY WAKE-UP STATE MACHINE ] ===
static void updateDrowsyWakeup()
{
    if (drowsyState == 0)
        return;

    uint32_t now = millis();
    uint32_t elapsed = now - drowsyStateStart;

    if (drowsyCancelFlag)
    {
        drowsyCancelFlag = false;
        drowsyState = 0;
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_BLU_PIN, LOW);
        setOledAlert("DROWSY CANCELLED");
        return;
    }

    switch (drowsyState)
    {
    case 1: // WAKEUP_ON
        if (elapsed >= WAKEUP_STAGE_ON_MS[drowsyStage])
        {
            digitalWrite(BUZZER_PIN, LOW);
            digitalWrite(LED_RED_PIN, LOW);
            digitalWrite(LED_BLU_PIN, LOW);
            drowsyState = 2;
            drowsyStateStart = now;
        }
        break;

    case 2: // WAKEUP_OFF
        if (elapsed >= WAKEUP_STAGE_OFF_MS[drowsyStage])
        {
            drowsyStage++;
            if (drowsyStage >= WAKEUP_STAGES)
            {
                drowsyState = 3;
                drowsyStateStart = now;
            }
            else
            {
                drowsyState = 1;
                drowsyStateStart = now;
                digitalWrite(BUZZER_PIN, HIGH);
                digitalWrite(LED_RED_PIN, HIGH);
                if (drowsyStage >= 1)
                    digitalWrite(LED_BLU_PIN, HIGH);
            }
        }
        break;

    case 3: // GRACE
        if (elapsed >= WAKEUP_GRACE_MS)
        {
            drowsyState = 4;
            setOledAlert("DRIVER UNRESPONSIVE!");
        }
        break;

    case 4: // BROADCAST
        emergencyDrowsy = true;
        drowsyState = 0;
        break;
    }
}

// === [ INCAPACITATION COUNTDOWN TIMER ] ===
static void startIncapTimer()
{
    if (!incapTimerActive && !sosActive)
    {
        incapTimerActive = true;
        incapTimerStart = millis();
    }
}

// === [ SOS & INCAPACITATION BEACON SYSTEM ] ===
static void updateIncapSos()
{
    uint32_t now = millis();

    if (driverOkFlag)
    {
        driverOkFlag = false;
        incapTimerActive = false;
        sosActive = false;
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_BLU_PIN, LOW);
        setOledAlert("DRIVER OK");
        return;
    }

    if (incapTimerActive && !sosActive)
    {
        uint32_t elapsed = now - incapTimerStart;
        if (elapsed >= INCAP_TIMEOUT_MS)
        {
            incapTimerActive = false;
            sosActive = true;
            lastSosBeacon = 0;
            setOledAlert("!! SOS MODE ON !!");
        }
    }

    if (sosActive)
    {
        if (now - lastSosBeacon >= SOS_BEACON_INTERVAL_MS)
        {
            lastSosBeacon = now;

            EmergencyPacket pkt;
            pkt.packetType = 1;
            pkt.vehicleID = myVehicleID;
            pkt.packetID = txPacketID++;
            pkt.eventCode = 3;
            pkt.latitude = ownLat;
            pkt.longitude = ownLon;

            sendEmergencyMultiChannel(pkt);

            digitalWrite(BUZZER_PIN, HIGH);
            digitalWrite(LED_RED_PIN, HIGH);
            digitalWrite(LED_BLU_PIN, HIGH);

            // Emergency-responder log — kept on Serial for laptop relay
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

// === [ EMERGENCY RESPONDER TELEMETRY LOGS ] ===
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

// =========================================================================
//                           CORE 0 TASKS
//                   (GPS Parsing & Serial Listener)
// =========================================================================

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

            if (gpsFix.valid.location)
            {
                ownLat = gpsFix.latitude();
                ownLon = gpsFix.longitude();
            }
            if (gpsFix.valid.speed)
            {
                ownSpeed = (uint16_t)(gpsFix.speed_kph() * 10.0f);
            }

            uint32_t now = millis();
            if (now - lastGpsLog >= 2000)
            {
                lastGpsLog = now;
                dbgGpsFixCount++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

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

                if (lineIdx > 0 && strstr(lineBuf, "DRIVER_OK") != nullptr)
                {
                    driverOkFlag = true;
                    drowsyCancelFlag = true;
                }
                else if (lineIdx > 0 && strstr(lineBuf, "CANCEL_DROWSY") != nullptr)
                {
                    drowsyCancelFlag = true;
                }
                else if (lineIdx > 0 && strstr(lineBuf, "DROWSY_ALERT") != nullptr)
                {
                    if (now - lastDrowsyTrigger > DROWSY_ALERT_REARM_MS)
                    {
                        lastDrowsyTrigger = now;
                        drowsyStage = 0;
                        drowsyState = 1;
                        drowsyStateStart = now;
                        drowsyCancelFlag = false;
                        digitalWrite(BUZZER_PIN, HIGH);
                        digitalWrite(LED_RED_PIN, HIGH);
                        setOledAlert("!! DROWSY !!");
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

// =========================================================================
//                           CORE 1 TASKS
//                   (RF Transmit/Receive & Risk Assessment)
// =========================================================================

void taskRFTransmit(void *pvParameters)
{
    (void)pvParameters;
    uint32_t lastTx = 0;
    uint32_t txJitter = random(0, TX_JITTER_MAX_MS);

    for (;;)
    {
        uint32_t now = millis();

        // PRIORITY 1: Emergency packets — bypass CSMA, multi-channel
        if (emergencyDrowsy)
        {
            emergencyDrowsy = false;
            EmergencyPacket pkt;
            pkt.packetType = 1;
            pkt.vehicleID = myVehicleID;
            pkt.packetID = txPacketID++;
            pkt.eventCode = 2;
            pkt.latitude = ownLat;
            pkt.longitude = ownLon;

            sendEmergencyMultiChannel(pkt);
            logCollisionToSerial(pkt.latitude, pkt.longitude, pkt.eventCode);
            fireAlert();
            setOledAlert("TX DROWSY EMERG");
            startIncapTimer();
        }

        if (emergencyCollision)
        {
            emergencyCollision = false;
            EmergencyPacket pkt;
            pkt.packetType = 1;
            pkt.vehicleID = myVehicleID;
            pkt.packetID = txPacketID++;
            pkt.eventCode = 1;
            pkt.latitude = ownLat;
            pkt.longitude = ownLon;

            sendEmergencyMultiChannel(pkt);
            logCollisionToSerial(pkt.latitude, pkt.longitude, pkt.eventCode);
            fireAlert();
            setOledAlert("TX COLLISION!");
            startIncapTimer();
        }

        // PRIORITY 2: Normal 1 Hz telemetry — with CSMA + jitter
        if (now - lastTx >= NORMAL_TX_INTERVAL_MS + txJitter)
        {
            if (now < csmaBackoffUntil)
            {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }

            if (isChannelClear())
            {
                lastTx = now;
                txJitter = random(0, TX_JITTER_MAX_MS);
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
                radio.write(&pkt, sizeof(pkt));
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
                    lastTx = now;
                    txJitter = random(0, TX_JITTER_MAX_MS);
                    csmaBackoffAttempt = 0;
                }
                else
                {
                    uint32_t bo = (uint32_t)random(1, 11) * csmaBackoffAttempt;
                    csmaBackoffUntil = now + bo;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void taskRFReceive(void *pvParameters)
{
    (void)pvParameters;
    uint8_t rxChIdx = 0;
    uint32_t lastChHop = 0;
    const uint32_t RX_HOP_DWELL_MS = 50;
    const uint32_t RX_HOME_DWELL_MS = 200;
    bool onHomeChannel = true;

    for (;;)
    {
        while (radio.available())
        {
            RawPacket pkt;
            radio.read(&pkt.data, sizeof(pkt.data));
            rxRingBuffer.push(pkt);
            dbgRxRawCount++;
        }

        // Channel hopping for emergency reception
        uint32_t now = millis();
        uint32_t dwellTime = onHomeChannel ? RX_HOME_DWELL_MS : RX_HOP_DWELL_MS;
        if (now - lastChHop >= dwellTime)
        {
            lastChHop = now;
            if (onHomeChannel)
            {
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

void taskRiskAssessment(void *pvParameters)
{
    (void)pvParameters;

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
                NormalPacket pkt;
                memcpy(&pkt, raw, sizeof(NormalPacket));

                if (pkt.vehicleID == myVehicleID)
                {
                    dbgRxOwnCount++;
                    continue;
                }

                if (isDuplicate(pkt.vehicleID, pkt.packetID))
                    continue;

                float dist = approxDistanceM(ownLat, ownLon, pkt.latitude, pkt.longitude);

                uint32_t now = millis();
                float closingSpeedMs = 0.0f;
                if (lastDistTime > 0 && now > lastDistTime)
                {
                    float dt = (now - lastDistTime) / 1000.0f;
                    closingSpeedMs = (lastKnownDist - dist) / dt;
                }
                lastKnownDist = dist;
                lastDistTime = now;

                float ttc = computeTTC(dist, closingSpeedMs);

                // Update OLED display state
                oledLastDist = dist;
                oledLastTTC = ttc;

                if (dist < ALERT_DISTANCE_M || ttc < ALERT_TTC_S)
                {
                    fireAlert();
                    setOledAlert("PROXIMITY ALERT!");

                    if (ttc < 1.5f && dist < 30.0f)
                    {
                        emergencyCollision = true;
                        logCollisionToSerial(ownLat, ownLon, 1);
                        setOledAlert("!! COLLISION !!");
                    }
                }
            }
            else if (packetType == 1)
            {
                EmergencyPacket pkt;
                memcpy(&pkt, raw, sizeof(EmergencyPacket));

                if (pkt.vehicleID == myVehicleID)
                {
                    dbgRxOwnCount++;
                    continue;
                }

                if (isDuplicate(pkt.vehicleID, pkt.packetID))
                {
                    dbgRxDupCount++;
                    continue;
                }

                fireAlert();

                switch (pkt.eventCode)
                {
                case 1:
                    setOledAlert("RX: COLLISION!");
                    break;
                case 2:
                    setOledAlert("RX: DROWSY!");
                    break;
                case 3:
                    setOledAlert("RX: SOS BEACON!");
                    break;
                default:
                    setOledAlert("RX: EMERGENCY!");
                    break;
                }

                logCollisionToSerial(pkt.latitude, pkt.longitude, pkt.eventCode);
            }
        }

        updateAlerts();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =========================================================================
//                           OLED DISPLAY TASK
// =========================================================================

void taskOLED(void *pvParameters)
{
    (void)pvParameters;

    uint8_t currentPage = 0;
    const uint8_t NUM_PAGES = 3;
    uint32_t lastPageSwitch = millis();
    uint32_t lastRefresh = 0;

    for (;;)
    {
        uint32_t now = millis();

        if (now - lastRefresh < OLED_REFRESH_MS)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        lastRefresh = now;

        oled.clearDisplay();
        oled.setTextColor(SSD1306_WHITE);

        // Check for alert overlay
        bool alertActive = (oledAlertText[0] != '\0') &&
                           (now - oledAlertStart < OLED_ALERT_DISPLAY_MS);

        if (alertActive)
        {
            oled.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);
            oled.drawRect(2, 2, OLED_WIDTH - 4, OLED_HEIGHT - 4, SSD1306_WHITE);

            oled.setTextSize(1);
            oled.setCursor(30, 8);
            oled.print("!! ALERT !!");

            oled.setTextSize(1);
            int16_t textLen = strlen((const char *)oledAlertText);
            int16_t xPos = (OLED_WIDTH - textLen * 6) / 2;
            if (xPos < 4)
                xPos = 4;
            oled.setCursor(xPos, 28);
            oled.print((const char *)oledAlertText);

            if (oledLastDist < 500.0f)
            {
                char buf[22];
                snprintf(buf, sizeof(buf), "Dist:%.0fm TTC:%.1fs", (double)oledLastDist, (double)oledLastTTC);
                textLen = strlen(buf);
                xPos = (OLED_WIDTH - textLen * 6) / 2;
                if (xPos < 4)
                    xPos = 4;
                oled.setCursor(xPos, 44);
                oled.print(buf);
            }

            if (now - oledAlertStart >= OLED_ALERT_DISPLAY_MS)
            {
                ((char *)oledAlertText)[0] = '\0';
            }
        }
        else
        {
            // Auto-cycle pages
            if (now - lastPageSwitch >= OLED_PAGE_CYCLE_MS)
            {
                lastPageSwitch = now;
                currentPage = (currentPage + 1) % NUM_PAGES;
            }

            // Header bar
            oled.setTextSize(1);
            oled.setCursor(0, 0);
            oled.print("V2V ID:");
            oled.print(myVehicleID);

            oled.setCursor(80, 0);
            if (ownLat != 0.0f || ownLon != 0.0f)
                oled.print("GPS:OK");
            else
                oled.print("GPS:--");

            oled.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);

            switch (currentPage)
            {
            case 0: // Status Overview
            {
                oled.setCursor(0, 13);
                oled.print("Spd: ");
                oled.print(ownSpeed / 10.0f, 1);
                oled.print(" km/h");

                oled.setCursor(0, 23);
                oled.print("Lat: ");
                oled.print(ownLat, 5);

                oled.setCursor(0, 33);
                oled.print("Lon: ");
                oled.print(ownLon, 5);

                oled.setCursor(0, 43);
                oled.print("Up: ");
                oled.print(now / 1000);
                oled.print("s  Heap:");
                oled.print(ESP.getFreeHeap() / 1024);
                oled.print("k");

                oled.setCursor(0, 55);
                if (sosActive)
                    oled.print("!! SOS ACTIVE !!");
                else if (incapTimerActive)
                    oled.print("INCAP TIMER ON");
                else if (drowsyState > 0)
                    oled.print("DROWSY WAKEUP...");
                else
                    oled.print("System OK");

                oled.setCursor(110, 55);
                oled.print("1/3");
                break;
            }

            case 1: // RF Statistics
            {
                oled.setCursor(0, 13);
                oled.print("TX norm: ");
                oled.print(dbgTxNormalCount);

                oled.setCursor(0, 23);
                oled.print("TX emrg: ");
                oled.print(dbgTxEmergencyCount);

                oled.setCursor(0, 33);
                oled.print("RX raw:  ");
                oled.print(dbgRxRawCount);

                oled.setCursor(0, 43);
                oled.print("RX dup:");
                oled.print(dbgRxDupCount);
                oled.print(" own:");
                oled.print(dbgRxOwnCount);

                oled.setCursor(0, 53);
                oled.print("CSMA busy: ");
                oled.print(dbgCsmaBusyCount);

                oled.setCursor(110, 55);
                oled.print("2/3");
                break;
            }

            case 2: // Nearby Vehicles
            {
                oled.setCursor(0, 13);
                oled.print("Nearest dist:");

                oled.setCursor(0, 23);
                if (oledLastDist < 500.0f && oledLastDist > 0.01f)
                {
                    oled.setTextSize(2);
                    oled.print(oledLastDist, 0);
                    oled.setTextSize(1);
                    oled.print(" m");
                }
                else
                {
                    oled.print("-- no vehicle --");
                }

                oled.setCursor(0, 43);
                oled.print("TTC: ");
                if (oledLastTTC < 100.0f)
                {
                    oled.print(oledLastTTC, 1);
                    oled.print("s");
                }
                else
                {
                    oled.print("safe");
                }

                oled.setCursor(0, 53);
                oled.print("Buf:");
                oled.print(rxRingBuffer.count());
                oled.print(" GPS fix:");
                oled.print(dbgGpsFixCount);

                oled.setCursor(110, 55);
                oled.print("3/3");
                break;
            }
            }
        }

        oled.display();
    }
}

// =========================================================================
//                           SETUP & LOOP
// =========================================================================
void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    // Vehicle ID from ESP32 MAC
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    myVehicleID = mac[5];
    randomSeed(mac[4] ^ (millis() & 0xFF));

    // Initialise dedup cache
    memset(dedupCache, 0, sizeof(dedupCache));

    // GPIO
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GRN_PIN, OUTPUT);
    pinMode(LED_BLU_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GRN_PIN, LOW);
    digitalWrite(LED_BLU_PIN, LOW);

    // Quick LED self-test
    digitalWrite(LED_GRN_PIN, HIGH);

    // OLED Display
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR))
    {
        for (int i = 0; i < 6; i++)
        {
            digitalWrite(LED_BLU_PIN, !digitalRead(LED_BLU_PIN));
            delay(200);
        }
    }
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(10, 8);
    oled.println("V2V Safety System");
    oled.setCursor(10, 24);
    oled.print("ID: ");
    oled.println(myVehicleID);
    oled.setCursor(10, 40);
    oled.println("Booting...");
    oled.display();

    // NRF24L01
    if (!radio.begin())
    {
        oled.clearDisplay();
        oled.setCursor(0, 20);
        oled.setTextSize(1);
        oled.println("NRF24L01 ERROR!");
        oled.println("Check wiring.");
        oled.display();

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
    radio.setAutoAck(false);
    radio.openWritingPipe(RF_PIPE_ADDR);
    radio.openReadingPipe(1, RF_PIPE_ADDR);
    radio.startListening();

    // Update OLED boot screen
    oled.clearDisplay();
    oled.setCursor(10, 8);
    oled.println("V2V Safety System");
    oled.setCursor(10, 24);
    oled.print("ID: ");
    oled.print(myVehicleID);
    oled.print("  RF:OK");
    oled.setCursor(10, 40);
    oled.println("Launching tasks...");
    oled.display();

    // FreeRTOS Task Creation
    // Core 0: GPS + Serial Listener + OLED
    xTaskCreatePinnedToCore(taskGPS, "GPS_Parse", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(taskSerialListener, "Serial_Listen", 4096, NULL, 1, NULL, 0);

    // Core 1: RF TX, RF RX, Risk Assessment
    xTaskCreatePinnedToCore(taskRFTransmit, "RF_TX", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskRFReceive, "RF_RX", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskRiskAssessment, "Risk_Assess", 4096, NULL, 2, NULL, 1);

    // Drowsy wake-up state machine — Core 1
    xTaskCreatePinnedToCore(
        [](void *)
        {
            for (;;)
            {
                updateDrowsyWakeup();
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        },
        "Drowsy_Wake", 2048, NULL, 2, NULL, 1);

    // Incapacitation / SOS beacon — Core 1
    xTaskCreatePinnedToCore(
        [](void *)
        {
            for (;;)
            {
                updateIncapSos();
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        },
        "Incap_SOS", 2048, NULL, 3, NULL, 1);

    // OLED display task — Core 0
    xTaskCreatePinnedToCore(taskOLED, "OLED_Display", 4096, NULL, 1, NULL, 0);

    digitalWrite(LED_GRN_PIN, LOW);
}

void loop()
{
    static uint32_t lastHeartbeat = 0;
    uint32_t now = millis();

    if (now - lastHeartbeat >= 2000)
    {
        lastHeartbeat = now;
        digitalWrite(LED_GRN_PIN, HIGH);
    }
    if (now - lastHeartbeat >= 100 && digitalRead(LED_GRN_PIN))
    {
        digitalWrite(LED_GRN_PIN, LOW);
    }
}