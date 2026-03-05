// ============================================================
//  main.cpp  —  Test Pipeline K1→K2→K3→K4→K5
//  MotionSensor → DataNormalizer → SignalProcessor
//  → AccidentDetector → AlertManager
//
//  [TEST 1] Tat ca 5 khoi khoi tao thanh cong
//  [TEST 2] Khong drop frame trong pipeline
//  [TEST 3] State machine: UNKNOWN → NORMAL
//  [TEST 4] Khong false positive khi dung yen (state = NORMAL)
//  [TEST 5] Phat hien tai nan → buzzer bat (isAlerting=true)
//  [TEST 6] Auto-reset sau ACCIDENT_HOLD → buzzer tat
//  [TEST 7] HEARTBEAT_BEEP: beep ngan khong lam isAlerting=true
// ============================================================

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "DataTypes.h"
#include "IModule.h"
#include "QueueManager.h"
#include "MotionSensor.h"
#include "DataNormalizer.h"
#include "SignalProcessor.h"
#include "AccidentDetector.h"
#include "AlertManager.h"
#include "Buzzer_Active.h"

// ============================================================
//  Globals
// ============================================================
static MotionSensor*     g_motionSensor     = nullptr;
static DataNormalizer*   g_dataNormalizer   = nullptr;
static SignalProcessor*  g_signalProcessor  = nullptr;
static AccidentDetector* g_accidentDetector = nullptr;
static AlertManager*     g_alertManager     = nullptr;
static BuzzerActive*     g_buzzer           = nullptr;

static void monitorTask(void* pvParameters);
static void accidentBridgeTask(void* pvParameters);
static void printSeparator();
static const char* stateToStr(VehicleState s);

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    printSeparator();
    Serial.println("  TEST: Pipeline K1 -> K2 -> K3 -> K4 -> K5");
    printSeparator();

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_FREQUENCY);

    QueueManager& qm = QueueManager::getInstance();
    if (!qm.begin()) {
        Serial.println("[FATAL] QueueManager init failed!");
        while (true) { delay(1000); }
    }

    // --- K1: MotionSensor ---
    Serial.println("[SETUP] >> Giu yen sensor trong calibration! <<");
    g_motionSensor = new MotionSensor(Wire, qm.getRawDataQueue());
    if (!g_motionSensor->begin()) {
        Serial.println("[FATAL] MotionSensor failed"); while(true){}
    }

    // --- K2: DataNormalizer ---
    g_dataNormalizer = new DataNormalizer(
        qm.getRawDataQueue(),
        qm.getNormalizedQueue(),
        g_motionSensor->getGyroOffsetX(),
        g_motionSensor->getGyroOffsetY(),
        g_motionSensor->getGyroOffsetZ()
    );
    if (!g_dataNormalizer->begin()) {
        Serial.println("[FATAL] DataNormalizer failed"); while(true){}
    }

    // --- K3: SignalProcessor ---
    g_signalProcessor = new SignalProcessor(
        qm.getNormalizedQueue(),
        qm.getFeaturesQueue()
    );
    if (!g_signalProcessor->begin()) {
        Serial.println("[FATAL] SignalProcessor failed"); while(true){}
    }

    // --- K4: AccidentDetector ---
    g_accidentDetector = new AccidentDetector(
        qm.getFeaturesQueue(),
        qm.getAccidentQueue(),
        qm.getSystemEventGroup()
    );
    if (!g_accidentDetector->begin()) {
        Serial.println("[FATAL] AccidentDetector failed"); while(true){}
    }

    // --- K5: AlertManager ---
    g_buzzer      = new BuzzerActive(PIN_BUZZER);
    g_alertManager = new AlertManager(qm.getAlertQueue(), g_buzzer);
    if (!g_alertManager->begin()) {
        Serial.println("[FATAL] AlertManager failed"); while(true){}
    }

    // [TEST 1] Health check 5 khoi
    Serial.printf("[TEST 1] K1 MotionSensor     : %s\n",
        g_motionSensor->isHealthy()     ? "PASS" : "FAIL");
    Serial.printf("[TEST 1] K2 DataNormalizer   : %s\n",
        g_dataNormalizer->isHealthy()   ? "PASS" : "FAIL");
    Serial.printf("[TEST 1] K3 SignalProcessor  : %s\n",
        g_signalProcessor->isHealthy()  ? "PASS" : "FAIL");
    Serial.printf("[TEST 1] K4 AccidentDetector : %s\n",
        g_accidentDetector->isHealthy() ? "PASS" : "FAIL");
    Serial.printf("[TEST 1] K5 AlertManager     : %s\n",
        g_alertManager->isHealthy()     ? "PASS" : "FAIL");

    // Task cau noi: doc accidentQueue → gui AlertRequest vao alertQueue
    xTaskCreatePinnedToCore(
        accidentBridgeTask, "BridgeTask", 2048,
        nullptr, 2, nullptr, CORE_MANAGEMENT_TASKS
    );

    // Task monitor bao cao dinh ky
    xTaskCreatePinnedToCore(
        monitorTask, "MonitorTask", 4096,
        nullptr, 1, nullptr, CORE_MANAGEMENT_TASKS
    );

    printSeparator();
    Serial.println("[SETUP] Giu yen: kiem tra no false-positive (TEST 4)");
    Serial.println("[SETUP] Lac/nghieng de kich hoat buzzer   (TEST 5)");
    printSeparator();
}

void loop() { vTaskDelay(portMAX_DELAY); }

// ============================================================
//  accidentBridgeTask
//  Doc AccidentEvent tu accidentQueue
//  → neu isActive=true  : gui START_ALERT vao alertQueue
//  → neu isActive=false : log SUSPICIOUS (khong bep)
//  Phan tach K4 va K5 hoan toan qua queue
// ============================================================
static void accidentBridgeTask(void* pvParameters) {
    QueueManager&  qm          = QueueManager::getInstance();
    QueueHandle_t  accidentQ   = qm.getAccidentQueue();
    QueueHandle_t  alertQ      = qm.getAlertQueue();

    AccidentEvent event;

    while (true) {
        if (xQueueReceive(accidentQ, &event,
                          pdMS_TO_TICKS(50)) == pdTRUE)
        {
            if (event.isActive) {
                AlertRequest req;
                req.command      = AlertCommand::START_ALERT;
                req.accidentType = event.type;
                req.timestamp    = event.eventTimestamp;
                xQueueSend(alertQ, &req,
                           pdMS_TO_TICKS(QUEUE_SEND_TIMEOUT_MS));

            } else {
                // SUSPICIOUS: chi log, khong bat buzzer
                Serial.printf("[BRIDGE] SUSPICIOUS → %s (no beep)\n",
                    accidentTypeToString(event.type));
            }
        }
    }
}

// ============================================================
//  monitorTask — bao cao moi 2 giay
// ============================================================
static void monitorTask(void* pvParameters) {
    QueueManager& qm = QueueManager::getInstance();

    VehicleState  prevState      = VehicleState::UNKNOWN;
    uint32_t      lastReportMs   = millis();
    uint32_t      lastK3Snap     = 0;
    uint32_t      lastK4Snap     = 0;
    uint32_t      lastAlertSnap  = 0;

    while (true) {
        uint32_t now = millis();
        if (now - lastReportMs >= 2000) {
            VehicleState curState  = g_accidentDetector->getCurrentState();
            uint32_t     k3frames  = g_signalProcessor->getProcessedCount();
            uint32_t     k4frames  = g_accidentDetector->getProcessedCount();
            uint32_t     alerts    = g_alertManager->getAlertCount();

            Serial.println();
            printSeparator();
            Serial.printf("[MONITOR] Uptime: %lu ms\n", now);

            // [TEST 1] Health check 5 khoi
            Serial.printf("[TEST 1] K1:%s K2:%s K3:%s K4:%s K5:%s\n",
                g_motionSensor->isHealthy()     ? "OK" : "FAIL",
                g_dataNormalizer->isHealthy()   ? "OK" : "FAIL",
                g_signalProcessor->isHealthy()  ? "OK" : "FAIL",
                g_accidentDetector->isHealthy() ? "OK" : "FAIL",
                g_alertManager->isHealthy()     ? "OK" : "FAIL");

            // [TEST 2] Drop check
            Serial.printf("[TEST 2] Drops K1:%lu K2:%lu K3:%lu %s\n",
                g_motionSensor->getDroppedFrames(),
                g_dataNormalizer->getDroppedCount(),
                g_signalProcessor->getDroppedCount(),
                (g_motionSensor->getDroppedFrames() == 0 &&
                 g_dataNormalizer->getDroppedCount() == 0 &&
                 g_signalProcessor->getDroppedCount() == 0) ? "PASS" : "WARN");

            // Throughput
            Serial.printf("         K3:%lu (%lu/2s) K4:%lu (%lu/2s)\n",
                k3frames, k3frames - lastK3Snap,
                k4frames, k4frames - lastK4Snap);

            // [TEST 3] State init
            Serial.printf("[TEST 3] State init  : %s\n",
                curState != VehicleState::UNKNOWN ? "PASS" : "WAIT");

            // [TEST 4] No false positive
            Serial.printf("[TEST 4] No false-pos: %s  (state=%s)\n",
                curState == VehicleState::NORMAL ? "PASS" : "CHECK",
                stateToStr(curState));

            // [TEST 5] Buzzer status
            Serial.printf("[TEST 5] buzzerEnabled: %-5s | isAlerting: %-7s (alerts=%lu)\n",
                g_alertManager->isBuzzerEnabled() ? "ON"  : "OFF",
                g_alertManager->isAlerting()      ? "true" : "false",
                alerts);

            // State change log
            if (curState != prevState) {
                Serial.printf("  *** State: %s → %s ***\n",
                    stateToStr(prevState), stateToStr(curState));
                prevState = curState;
            }

            lastReportMs  = now;
            lastK3Snap    = k3frames;
            lastK4Snap    = k4frames;
            lastAlertSnap = alerts;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static const char* stateToStr(VehicleState s) {
    switch (s) {
        case VehicleState::UNKNOWN:    return "UNKNOWN";
        case VehicleState::NORMAL:     return "NORMAL";
        case VehicleState::SUSPICIOUS: return "SUSPICIOUS";
        case VehicleState::ACCIDENT:   return "ACCIDENT";
        default:                       return "?";
    }
}

static void printSeparator() {
    Serial.println("============================================");
}
