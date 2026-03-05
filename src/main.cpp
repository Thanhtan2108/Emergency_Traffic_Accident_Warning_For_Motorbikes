// ============================================================
//  main.cpp  —  Test Pipeline K1→K2→K3→K4
//  MotionSensor→DataNormalizer→SignalProcessor→AccidentDetector
//
//  [TEST 1] Tất cả 4 khối khởi tạo thành công
//  [TEST 2] Không drop frame ở K1, K2 (K3 drop khi chưa có K4 đọc)
//  [TEST 3] State machine: UNKNOWN → NORMAL sau frame đầu
//  [TEST 4] Không false positive khi đứng yên (state = NORMAL)
//  [TEST 5] Phát hiện CRASH khi lắc mạnh  (totalAcc > 2.5g)
//  [TEST 6] Phát hiện FALL khi nghiêng    (angle > 60°)
//  [TEST 7] Debounce: cần 5 frame liên tiếp mới xác nhận ACCIDENT
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Config.h>
#include <DataTypes.h>
#include <IModule.h>
#include <QueueManager.h>
#include <MotionSensor.h>
#include <DataNormalizer.h>
#include <SignalProcessor.h>
#include <AccidentDetector.h>

static void monitorTask(void* pvParameters);
static void printSeparator();
static const char* stateToStr(VehicleState s);

static MotionSensor*     g_motionSensor     = nullptr;
static DataNormalizer*   g_dataNormalizer   = nullptr;
static SignalProcessor*  g_signalProcessor  = nullptr;
static AccidentDetector* g_accidentDetector = nullptr;

// ============================================================
//  setup()
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);

    printSeparator();
    Serial.println("  TEST: Pipeline K1 -> K2 -> K3 -> K4");
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

    // [TEST 1] Health check tất cả 4 khối
    Serial.printf("[TEST 1] K1 MotionSensor     : %s\n",
        g_motionSensor->isHealthy()     ? "PASS" : "FAIL");
    Serial.printf("[TEST 1] K2 DataNormalizer   : %s\n",
        g_dataNormalizer->isHealthy()   ? "PASS" : "FAIL");
    Serial.printf("[TEST 1] K3 SignalProcessor  : %s\n",
        g_signalProcessor->isHealthy()  ? "PASS" : "FAIL");
    Serial.printf("[TEST 1] K4 AccidentDetector : %s\n",
        g_accidentDetector->isHealthy() ? "PASS" : "FAIL");

    xTaskCreatePinnedToCore(
        monitorTask, "MonitorTask", 4096,
        nullptr, 1, nullptr, CORE_MANAGEMENT_TASKS
    );

    printSeparator();
    Serial.println("[SETUP] Giu yen: kiem tra no false-positive (TEST 4)");
    Serial.println("[SETUP] Lac manh / nghieng > 60 de kich hoat (TEST 5/6)");
    printSeparator();
}

void loop() { vTaskDelay(portMAX_DELAY); }

// ============================================================
//  monitorTask
// ============================================================
static void monitorTask(void* pvParameters) {
    QueueManager& qm        = QueueManager::getInstance();
    QueueHandle_t accidentQ = qm.getAccidentQueue();

    AccidentEvent event;
    VehicleState  prevState       = VehicleState::UNKNOWN;
    uint32_t      lastReportMs    = millis();
    uint32_t      lastK3Snap      = 0;
    uint32_t      lastK4Snap      = 0;

    while (true) {
        // --- Đọc accidentQueue ngay khi có event ---
        while (xQueueReceive(accidentQ, &event, 0) == pdTRUE) {
            Serial.println();
            if (event.isActive) {
                Serial.println("  ╔══════════════════════════════╗");
                Serial.println("  ║   *** ACCIDENT DETECTED ***  ║");
                Serial.println("  ╚══════════════════════════════╝");
            } else {
                Serial.println("  [ SUSPICIOUS detected ]");
            }
            Serial.printf("  Type     : %s\n",
                accidentTypeToString(event.type));
            Serial.printf("  totalAcc : %.3f g  (nguong: %.1fg)\n",
                event.totalAccAtEvent, ACC_CRASH_THRESHOLD);
            Serial.printf("  angleX/Y : %.1f / %.1f deg  (nguong: %.0fdeg)\n",
                event.angleXAtEvent, event.angleYAtEvent, TILT_ANGLE_THRESHOLD);
            Serial.printf("  jerk     : %.3f g/s  (nguong: %.0f g/s)\n",
                event.jerkAtEvent, JERK_THRESHOLD);
            Serial.printf("  time     : %lu ms\n", event.eventTimestamp);
        }

        // --- Báo cáo định kỳ mỗi 2 giây ---
        uint32_t now = millis();
        if (now - lastReportMs >= 2000) {
            VehicleState curState  = g_accidentDetector->getCurrentState();
            uint32_t     k3frames  = g_signalProcessor->getProcessedCount();
            uint32_t     k4frames  = g_accidentDetector->getProcessedCount();

            Serial.println();
            printSeparator();
            Serial.printf("[MONITOR] Uptime: %lu ms\n", now);

            // [TEST 1] Health check 4 khối
            Serial.printf("[TEST 1] K1 healthy: %s | K2: %s | K3: %s | K4: %s\n",
                g_motionSensor->isHealthy()     ? "PASS" : "FAIL",
                g_dataNormalizer->isHealthy()   ? "PASS" : "FAIL",
                g_signalProcessor->isHealthy()  ? "PASS" : "FAIL",
                g_accidentDetector->isHealthy() ? "PASS" : "FAIL");

            // [TEST 2] Drop check — K3 drop được chấp nhận khi test đơn lẻ
            uint32_t k1drop = g_motionSensor->getDroppedFrames();
            uint32_t k2drop = g_dataNormalizer->getDroppedCount();
            uint32_t k3drop = g_signalProcessor->getDroppedCount();
            Serial.printf("[TEST 2] Drops — K1: %lu %s | K2: %lu %s | K3: %lu\n",
                k1drop, k1drop == 0 ? "PASS" : "WARN",
                k2drop, k2drop == 0 ? "PASS" : "WARN",
                k3drop);
            // K3 drop = 0 vì K4 đang đọc featuresQueue liên tục
            Serial.printf("         K3 drops: %lu  %s\n",
                k3drop, k3drop == 0 ? "PASS" : "WARN (K4 xu ly khong kip)");

            // Throughput pipeline
            Serial.printf("         K3: %lu frames (%lu/2s) | K4: %lu frames (%lu/2s)\n",
                k3frames, k3frames - lastK3Snap,
                k4frames, k4frames - lastK4Snap);

            // [TEST 3] State đã qua UNKNOWN
            Serial.printf("[TEST 3] State init  : %s\n",
                curState != VehicleState::UNKNOWN ? "PASS" : "WAIT");

            // [TEST 4] No false positive
            Serial.printf("[TEST 4] No false-pos: %s  (state = %s)\n",
                curState == VehicleState::NORMAL ? "PASS" : "CHECK",
                stateToStr(curState));

            // Accident summary
            Serial.printf("         Accidents: %lu | Last type: %s\n",
                g_accidentDetector->getAccidentCount(),
                accidentTypeToString(g_accidentDetector->getLastAccidentType()));

            // State change
            if (curState != prevState) {
                Serial.printf("  *** State change: %s -> %s ***\n",
                    stateToStr(prevState), stateToStr(curState));
                prevState = curState;
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

static void printSeparator() {
    Serial.println("============================================");
}
