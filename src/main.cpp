// ============================================================
//  main.cpp  —  Full Pipeline K1→K2→K3→K4→K5→K6→K7
//
//  K6 SystemWatchdog:
//    - Giám sát heartbeat từ tất cả task
//    - Timeout > 500ms → set EVENT_TASK_TIMEOUT
//    - K7 nhận event → SystemState → ERROR
//
//  Mỗi module gọi QueueManager::sendHeartbeat() trong task loop
//  của mình. Main chỉ cần đăng ký task với Watchdog.
//
//  [TEST 1] Tất cả 7 khối khởi tạo thành công
//  [TEST 2] Không drop frame trong pipeline
//  [TEST 3] SystemState: BOOT→INIT→RUNNING
//  [TEST 4] Không false positive khi đứng yên
//  [TEST 5] Tai nạn → K7 nhận event → buzzer bật
//  [TEST 6] Auto-reset K4 → K7 gửi STOP_ALERT → buzzer tắt
//  [TEST 7] Tất cả task gửi heartbeat → không timeout
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
#include "SystemStateManager.h"
#include "SystemWatchdog.h"

// ============================================================
//  Globals
// ============================================================
static MotionSensor*        g_motionSensor      = nullptr;
static DataNormalizer*      g_dataNormalizer    = nullptr;
static SignalProcessor*     g_signalProcessor   = nullptr;
static AccidentDetector*    g_accidentDetector  = nullptr;
static AlertManager*        g_alertManager      = nullptr;
static BuzzerActive*        g_buzzer            = nullptr;
static SystemStateManager*  g_stateManager      = nullptr;
static SystemWatchdog*      g_watchdog          = nullptr;

static void monitorTask(void* pvParameters);
static void printSeparator();
static const char* stateToStr(VehicleState s);
static const char* sysStateToStr(SystemState s);

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    printSeparator();
    Serial.println("  Full Pipeline K1→K2→K3→K4→K5→K6→K7");
    printSeparator();

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_FREQUENCY);

    QueueManager& qm = QueueManager::getInstance();
    if (!qm.begin()) {
        Serial.println("[FATAL] QueueManager failed"); while(true){}
    }

    // --- K1 ---
    Serial.println("[SETUP] >> Giu yen sensor trong calibration! <<");
    g_motionSensor = new MotionSensor(Wire, qm.getRawDataQueue());
    if (!g_motionSensor->begin()) {
        Serial.println("[FATAL] K1 failed"); while(true){}
    }

    // --- K2 ---
    g_dataNormalizer = new DataNormalizer(
        qm.getRawDataQueue(), qm.getNormalizedQueue(),
        g_motionSensor->getGyroOffsetX(),
        g_motionSensor->getGyroOffsetY(),
        g_motionSensor->getGyroOffsetZ()
    );
    if (!g_dataNormalizer->begin()) {
        Serial.println("[FATAL] K2 failed"); while(true){}
    }

    // --- K3 ---
    g_signalProcessor = new SignalProcessor(
        qm.getNormalizedQueue(), qm.getFeaturesQueue()
    );
    if (!g_signalProcessor->begin()) {
        Serial.println("[FATAL] K3 failed"); while(true){}
    }

    // --- K4 ---
    g_accidentDetector = new AccidentDetector(
        qm.getFeaturesQueue(), qm.getAccidentQueue(),
        qm.getSystemEventGroup()
    );
    if (!g_accidentDetector->begin()) {
        Serial.println("[FATAL] K4 failed"); while(true){}
    }

    // --- K5 ---
    g_buzzer       = new BuzzerActive(PIN_BUZZER);
    g_alertManager = new AlertManager(qm.getAlertQueue(), g_buzzer);
    if (!g_alertManager->begin()) {
        Serial.println("[FATAL] K5 failed"); while(true){}
    }

    // --- K6: SystemWatchdog ---
    g_watchdog = new SystemWatchdog(
        qm.getWatchdogQueue(),
        qm.getSystemEventGroup()
    );
    if (!g_watchdog->begin()) {
        Serial.println("[FATAL] K6 failed"); while(true){}
    }

    // Đăng ký tất cả task với Watchdog
    // (seed lastSeen = now → tránh false timeout khi mới boot)
    g_watchdog->registerTask(TaskID::MOTION_SENSOR,     "MotionSensor");
    g_watchdog->registerTask(TaskID::DATA_NORMALIZER,   "DataNormalizer");
    g_watchdog->registerTask(TaskID::SIGNAL_PROCESSOR,  "SignalProcessor");
    g_watchdog->registerTask(TaskID::ACCIDENT_DETECTOR, "AccidentDetector");
    g_watchdog->registerTask(TaskID::ALERT_MANAGER,     "AlertManager");
    g_watchdog->registerTask(TaskID::STATE_MANAGER,     "StateManager");

    // --- K7 ---
    g_stateManager = new SystemStateManager(
        g_motionSensor, g_dataNormalizer, g_signalProcessor,
        g_accidentDetector, g_alertManager, qm
    );
    if (!g_stateManager->begin()) {
        Serial.println("[FATAL] K7 failed"); while(true){}
    }

    // [TEST 1] Health check
    Serial.printf("[TEST 1] K1:%s K2:%s K3:%s K4:%s K5:%s K6:%s K7:%s\n",
        g_motionSensor->isHealthy()     ? "PASS" : "FAIL",
        g_dataNormalizer->isHealthy()   ? "PASS" : "FAIL",
        g_signalProcessor->isHealthy()  ? "PASS" : "FAIL",
        g_accidentDetector->isHealthy() ? "PASS" : "FAIL",
        g_alertManager->isHealthy()     ? "PASS" : "FAIL",
        g_watchdog->isHealthy()         ? "PASS" : "FAIL",
        g_stateManager->isHealthy()     ? "PASS" : "FAIL");

    xTaskCreatePinnedToCore(
        monitorTask, "MonitorTask", 4096,
        nullptr, 1, nullptr, CORE_MANAGEMENT_TASKS
    );

    printSeparator();
    Serial.println("[SETUP] Giu yen: no false-positive  (TEST 4)");
    Serial.println("[SETUP] Lac/nghieng: bat buzzer      (TEST 5)");
    Serial.println("[SETUP] Cho 30s: tu tat buzzer       (TEST 6)");
    Serial.println("[SETUP] Quan sat: khong watchdog TO  (TEST 7)");
    printSeparator();
}

void loop() { vTaskDelay(portMAX_DELAY); }

// ============================================================
//  monitorTask
// ============================================================
static void monitorTask(void* pvParameters) {
    VehicleState prevVState  = VehicleState::UNKNOWN;
    SystemState  prevSState  = SystemState::BOOT;
    uint32_t     lastReportMs = millis();
    uint32_t     lastK3Snap  = 0;
    uint32_t     lastK4Snap  = 0;

    while (true) {
        // MonitorTask cũng gửi heartbeat
        QueueManager::getInstance().sendHeartbeat(TaskID::STATE_MANAGER);

        uint32_t now = millis();
        if (now - lastReportMs >= 2000) {
            VehicleState vState   = g_accidentDetector->getCurrentState();
            SystemState  sState   = g_stateManager->getCurrentState();
            uint32_t     k3frames = g_signalProcessor->getProcessedCount();
            uint32_t     k4frames = g_accidentDetector->getProcessedCount();

            Serial.println();
            printSeparator();
            Serial.printf("[MONITOR] Uptime: %lu ms\n", now);

            // [TEST 1] Health
            Serial.printf("[TEST 1] K1:%s K2:%s K3:%s K4:%s K5:%s K6:%s K7:%s\n",
                g_motionSensor->isHealthy()     ? "OK" : "FAIL",
                g_dataNormalizer->isHealthy()   ? "OK" : "FAIL",
                g_signalProcessor->isHealthy()  ? "OK" : "FAIL",
                g_accidentDetector->isHealthy() ? "OK" : "FAIL",
                g_alertManager->isHealthy()     ? "OK" : "FAIL",
                g_watchdog->isHealthy()         ? "OK" : "FAIL",
                g_stateManager->isHealthy()     ? "OK" : "FAIL");

            // [TEST 2] Drops + Throughput
            Serial.printf("[TEST 2] Drops K1:%lu K2:%lu K3:%lu  %s\n",
                g_motionSensor->getDroppedFrames(),
                g_dataNormalizer->getDroppedCount(),
                g_signalProcessor->getDroppedCount(),
                (g_motionSensor->getDroppedFrames()   == 0 &&
                 g_dataNormalizer->getDroppedCount()  == 0 &&
                 g_signalProcessor->getDroppedCount() == 0) ? "PASS" : "WARN");
            Serial.printf("         K3:%lu (%lu/2s) | K4:%lu (%lu/2s)\n",
                k3frames, k3frames - lastK3Snap,
                k4frames, k4frames - lastK4Snap);

            // [TEST 3] SystemState
            Serial.printf("[TEST 3] SystemState : %s\n",
                sysStateToStr(sState));

            // [TEST 4] No false positive
            Serial.printf("[TEST 4] VehicleState: %-10s  %s\n",
                stateToStr(vState),
                vState == VehicleState::NORMAL ? "PASS" : "CHECK");

            // [TEST 5/6] Buzzer
            Serial.printf("[TEST 5] buzzerEnabled: %-5s | accidents: %lu\n",
                g_alertManager->isBuzzerEnabled() ? "ON" : "OFF",
                g_accidentDetector->getAccidentCount());

            // [TEST 7] Watchdog
            Serial.printf("[TEST 7] Watchdog timeouts: %lu  %s\n",
                g_watchdog->getTimeoutCount(),
                g_watchdog->getTimeoutCount() == 0 ? "PASS" : "WARN");

            // State change logs
            if (vState != prevVState) {
                Serial.printf("  *** VehicleState: %s → %s ***\n",
                    stateToStr(prevVState), stateToStr(vState));
                prevVState = vState;
            }
            if (sState != prevSState) {
                Serial.printf("  *** SystemState : %s → %s ***\n",
                    sysStateToStr(prevSState), sysStateToStr(sState));
                prevSState = sState;
            }

            lastReportMs = now;
            lastK3Snap   = k3frames;
            lastK4Snap   = k4frames;
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

static const char* sysStateToStr(SystemState s) {
    switch (s) {
        case SystemState::BOOT:        return "BOOT";
        case SystemState::INIT:        return "INIT";
        case SystemState::RUNNING:     return "RUNNING";
        case SystemState::ERROR:       return "ERROR";
        case SystemState::RECOVERY:    return "RECOVERY";
        default:                       return "?";
    }
}

static void printSeparator() {
    Serial.println("============================================");
}
