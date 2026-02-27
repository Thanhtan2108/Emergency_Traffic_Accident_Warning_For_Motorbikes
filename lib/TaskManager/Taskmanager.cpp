// ============================================================
//  TaskManager.cpp
//  Implement Tầng 4 – 4 FreeRTOS Task và shared resources.
//
//  Tài liệu: README.md – Section VIII, IX
// ============================================================

#include "TaskManager.h"


// ============================================================
//  ĐỊNH NGHĨA SHARED RESOURCES
//  (Khai báo extern trong TaskManager.h)
// ============================================================
QueueHandle_t      sensorQueue      = nullptr;
QueueHandle_t      alertQueue       = nullptr;
EventGroupHandle_t alertGroup       = nullptr;
SemaphoreHandle_t  stateMutex       = nullptr;
SystemHealth       health           = {};

TaskHandle_t sensorTaskHandle   = nullptr;
TaskHandle_t stateTaskHandle    = nullptr;
TaskHandle_t outputTaskHandle   = nullptr;
TaskHandle_t watchdogTaskHandle = nullptr;

SensorReader*     g_sensorReader     = nullptr;
SignalProcessor*  g_signalProcessor  = nullptr;
AccidentDetector* g_accidentDetector = nullptr;


// ============================================================
//  initTasks()
//  Tạo Queue, EventGroup, Mutex, rồi tạo 4 task.
//  Gọi trong setup() sau khi tất cả objects đã begin().
// ============================================================
void initTasks(SensorReader* sr, SignalProcessor* sp, AccidentDetector* ad) {
    // Lưu references để các task function truy cập
    g_sensorReader     = sr;
    g_signalProcessor  = sp;
    g_accidentDetector = ad;

    // --- Tạo Queue ---
    sensorQueue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(ProcessedData));
    alertQueue  = xQueueCreate(ALERT_QUEUE_SIZE,  sizeof(AlertInfo));

    // --- Tạo EventGroup ---
    alertGroup = xEventGroupCreate();

    // --- Tạo Mutex (có priority inheritance – tránh priority inversion) ---
    stateMutex = xSemaphoreCreateMutex();

    // --- Khởi tạo SystemHealth ---
    health.last_sensor_tick    = 0;
    health.sensor_task_alive   = false;
    health.sensor_error_count  = 0;
    health.queue_overflow_count = 0;
    health.last_state_tick     = 0;
    health.state_enter_time    = 0;
    health.current_state       = WARMUP;
    health.state_task_alive    = false;

    // --- Tạo 4 FreeRTOS Task ---
    // Core 1: SensorTask + StateTask (real-time)
    // Core 0: OutputTask + WatchdogTask (I/O + monitoring)
    xTaskCreatePinnedToCore(
        SensorTask, "SensorTask",
        SENSOR_TASK_STACK, nullptr,
        SENSOR_TASK_PRIORITY, &sensorTaskHandle,
        SENSOR_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        StateTask, "StateTask",
        STATE_TASK_STACK, nullptr,
        STATE_TASK_PRIORITY, &stateTaskHandle,
        STATE_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        WatchdogTask, "WatchdogTask",
        WATCHDOG_TASK_STACK, nullptr,
        WATCHDOG_TASK_PRIORITY, &watchdogTaskHandle,
        WATCHDOG_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        OutputTask, "OutputTask",
        OUTPUT_TASK_STACK, nullptr,
        OUTPUT_TASK_PRIORITY, &outputTaskHandle,
        OUTPUT_TASK_CORE
    );
}


// ============================================================
//  SENSOR TASK  (Core 1, Priority 3)
//
//  Đọc MPU6050 → SignalProcessor → đẩy ProcessedData vào Queue.
//  Dùng vTaskDelayUntil() để giữ chu kỳ chính xác 100Hz (10ms).
//
//  KHÔNG dùng vTaskDelay() vì chu kỳ sẽ bị trôi dần
//  (delay(10) + thời gian xử lý ≠ 10ms cố định).
//
//  Queue timeout = 0: SensorTask KHÔNG được block bởi Queue.
//  Thà bỏ 1 mẫu còn hơn làm lệch chu kỳ 100Hz.
// ============================================================
void SensorTask(void* param) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Block đến mốc thời gian tuyệt đối tiếp theo (10ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_PERIOD_MS));

        // Đọc sensor thô
        RawSensorData raw = g_sensorReader->read();

        // Cập nhật health (volatile – không cần mutex, single writer)
        health.last_sensor_tick  = millis();
        health.sensor_task_alive = true;

        // Xử lý lỗi đọc sensor
        if (!raw.valid) {
            health.sensor_error_count++;

            // Log WARNING một lần duy nhất khi vượt ngưỡng, không spam
            if (health.sensor_error_count == SENSOR_MAX_ERROR_COUNT + 1) {
                Serial.println("[WARNING] SensorTask: sensor errors exceeded threshold");
            }
            continue;  // Bỏ qua chu kỳ này, không đẩy vào queue
        }

        // Reset error counter khi nhận được mẫu tốt
        health.sensor_error_count = 0;

        // Xử lý tín hiệu qua pipeline 6 bước
        ProcessedData processed = g_signalProcessor->process(raw);

        // Đẩy vào queue – timeout = 0 (không block)
        BaseType_t result = xQueueSend(sensorQueue, &processed, 0);
        if (result == errQUEUE_FULL) {
            // Không dùng mutex – volatile counter, single writer
            health.queue_overflow_count++;
        }
    }
}


// ============================================================
//  STATE TASK  (Core 1, Priority 3)
//
//  Block chờ ProcessedData từ sensorQueue → chạy AccidentDetector
//  → đẩy alert vào alertQueue + set EventGroup bit khi có cảnh báo.
//
//  Cập nhật SystemHealth (StateTask fields) sau mỗi chu kỳ,
//  bảo vệ bằng stateMutex vì WatchdogTask đọc đồng thời.
//
//  Lưu ý về baseline control:
//    StateTask đồng bộ setBaselineUpdateEnabled() với state
//    hiện tại TRƯỚC khi gọi accidentDetector.process().
//    AccidentDetector cũng gọi setBaselineUpdateEnabled() nội bộ
//    khi cần lock/unlock ngay tại thời điểm chuyển state.
//    Hai lớp này không conflict vì cùng thread (Core 1).
// ============================================================
void StateTask(void* param) {
    ProcessedData data;

    for (;;) {
        // Block vô thời hạn chờ dữ liệu – không tốn CPU khi chờ
        xQueueReceive(sensorQueue, &data, portMAX_DELAY);

        // Đồng bộ baseline update theo state hiện tại
        // (AccidentDetector cũng có thể override trong process())
        DetectorState currentState = g_accidentDetector->getState();
        g_signalProcessor->setBaselineUpdateEnabled(
            currentState == NORMAL || currentState == WARMUP
        );

        // Chạy State Machine
        g_accidentDetector->process(data);

        // Kiểm tra và xử lý alert mới
        if (g_accidentDetector->hasNewAlert()) {
            AlertInfo alert = g_accidentDetector->getLastAlert();

            // Đẩy vào alertQueue – timeout = 0 (StateTask không block vì I/O)
            xQueueSend(alertQueue, &alert, 0);

            // Set EventGroup bit để OutputTask unblock ngay lập tức
            if (alert.type == ALERT_FALL) {
                xEventGroupSetBits(alertGroup, ALERT_FALL_BIT);
            } else if (alert.type == ALERT_IMPACT) {
                xEventGroupSetBits(alertGroup, ALERT_IMPACT_BIT);
            }
        }

        // Cập nhật SystemHealth – cần mutex vì WatchdogTask đọc đồng thời
        // Giữ mutex ngắn nhất có thể: chỉ copy giá trị, không làm gì thêm
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        health.last_state_tick   = millis();
        health.state_task_alive  = true;
        health.current_state     = g_accidentDetector->getState();
        health.state_enter_time  = g_accidentDetector->getStateEnterTime();
        xSemaphoreGive(stateMutex);
    }
}


// ============================================================
//  OUTPUT TASK  (Core 0, Priority 1)
//
//  Block chờ EventGroup bit (FALL hoặc IMPACT).
//  Sau khi unblock: lấy AlertInfo từ alertQueue, kích hoạt
//  buzzer/LED và in log ra Serial.
//
//  Chạy ở Core 0 để không ảnh hưởng real-time Core 1.
//  Priority 1 (thấp nhất) vì I/O không cần độ trễ thấp.
//
//  Lưu ý buzzer/LED: hiện tại chỉ log Serial.
//  Khi có phần cứng thực, thêm buzzer.xxx() và led.xxx() tại đây.
// ============================================================
void OutputTask(void* param) {
    AlertInfo alert;

    for (;;) {
        // Block vô thời hạn chờ bất kỳ alert bit nào (OR logic)
        // pdTRUE = tự clear bit sau khi nhận
        EventBits_t bits = xEventGroupWaitBits(
            alertGroup,
            ALERT_FALL_BIT | ALERT_IMPACT_BIT,
            pdTRUE,         // clear on exit
            pdFALSE,        // wait for ANY bit (OR)
            portMAX_DELAY
        );

        // Lấy AlertInfo từ queue (timeout 10ms – queue phải có vì EventGroup
        // và Queue được set cùng lúc trong StateTask)
        BaseType_t got = xQueueReceive(alertQueue, &alert, pdMS_TO_TICKS(10));

        if (bits & ALERT_FALL_BIT) {
            // --- TÉ NGÃ – cảnh báo nghiêm trọng ---
            // buzzer.alarmContinuous();
            // led.setRed();
            if (got == pdTRUE) {
                Serial.printf(
                    "[FALL]   t=%lu  peak_a=%.2fg  peak_w=%.1f°/s  "
                    "roll=%.1f°  pitch=%.1f°\n",
                    (unsigned long)alert.timestamp,
                    alert.peak_a, alert.peak_omega,
                    alert.roll,   alert.pitch
                );
            } else {
                Serial.println("[FALL]   (alert payload missing from queue)");
            }
        }

        if (bits & ALERT_IMPACT_BIT) {
            // --- VA CHẠM MẠNH – cảnh báo mức cao ---
            // buzzer.beep(3);
            // led.setYellow();
            if (got == pdTRUE) {
                Serial.printf(
                    "[IMPACT] t=%lu  peak_a=%.2fg  peak_w=%.1f°/s  "
                    "roll=%.1f°  pitch=%.1f°\n",
                    (unsigned long)alert.timestamp,
                    alert.peak_a, alert.peak_omega,
                    alert.roll,   alert.pitch
                );
            } else {
                Serial.println("[IMPACT] (alert payload missing from queue)");
            }
        }
    }
}


// ============================================================
//  WATCHDOG TASK  (Core 0, Priority 2)
//
//  Chạy mỗi 1000ms. Kiểm tra:
//    1. SensorTask còn sống (last_sensor_tick không quá cũ)
//    2. StateTask còn sống (last_state_tick không quá cũ)
//    3. State Machine không bị kẹt ở intermediate state > 5s
//    4. Stack watermark (DEBUG_STACK mode)
//    5. Queue overflow counter
//    6. Feed hardware watchdog (esp_task_wdt_reset)
//
//  Khi phát hiện state kẹt: gọi accidentDetector.forceReset()
//  dưới sự bảo vệ của stateMutex.
// ============================================================
void WatchdogTask(void* param) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- Snapshot StateTask fields (cần mutex) ---
        // Giữ mutex ngắn nhất có thể: chỉ copy, không xử lý bên trong
        uint32_t      snap_state_tick;
        uint32_t      snap_enter_time;
        DetectorState snap_state;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            snap_state_tick  = health.last_state_tick;
            snap_enter_time  = health.state_enter_time;
            snap_state       = health.current_state;
            xSemaphoreGive(stateMutex);
        } else {
            // Không lấy được mutex trong 100ms → StateTask có thể đang bị treo
            Serial.println("[ERROR] WatchdogTask: stateMutex timeout (StateTask hung?)");
            esp_task_wdt_reset();
            continue;
        }

        // --- Snapshot SensorTask fields (volatile – không cần mutex) ---
        uint32_t snap_sensor_tick   = health.last_sensor_tick;
        uint16_t snap_error_count   = health.sensor_error_count;
        uint16_t snap_q_overflow    = health.queue_overflow_count;

        uint32_t now = millis();

        // === Kiểm tra 1: SensorTask còn sống? ===
        if (snap_sensor_tick > 0 &&
            (now - snap_sensor_tick) > SENSOR_DEAD_TIMEOUT_MS)
        {
            Serial.printf(
                "[ERROR] SensorTask không phản hồi > %ums (last tick: %lu)\n",
                SENSOR_DEAD_TIMEOUT_MS, (unsigned long)snap_sensor_tick
            );
        }

        // === Kiểm tra 2: StateTask còn sống? ===
        if (snap_state_tick > 0 &&
            (now - snap_state_tick) > 1000)
        {
            Serial.printf(
                "[ERROR] StateTask không phản hồi > 1000ms (last tick: %lu)\n",
                (unsigned long)snap_state_tick
            );
        }

        // === Kiểm tra 3: State Machine kẹt? ===
        // Các "terminal/stable" states không tính là kẹt:
        //   NORMAL, WARMUP, STRONG_IMPACT, FALL_DETECTED, ROAD_SHOCK
        // Các "intermediate" states phải hoàn thành trong T_STATE_MAX_TIMEOUT:
        //   EVENT_DETECTED, CLASSIFYING, IMPACT_PENDING, FALL_PENDING, TIMEOUT_RESET
        bool is_intermediate = (snap_state != NORMAL       &&
                                snap_state != WARMUP        &&
                                snap_state != STRONG_IMPACT &&
                                snap_state != FALL_DETECTED &&
                                snap_state != ROAD_SHOCK);

        if (is_intermediate && snap_enter_time > 0) {
            uint32_t time_in_state = now - snap_enter_time;

            if (time_in_state > T_STATE_MAX_TIMEOUT) {
                Serial.printf(
                    "[ERROR] State %d kẹt %lums – force reset\n",
                    (int)snap_state, (unsigned long)time_in_state
                );

                // Force reset dưới mutex vì accidentDetector không thread-safe
                xSemaphoreTake(stateMutex, portMAX_DELAY);
                g_accidentDetector->forceReset();
                xSemaphoreGive(stateMutex);
            }
        }

        // === Kiểm tra 4: Stack watermark (chỉ khi DEBUG_STACK) ===
#ifdef DEBUG_STACK
        Serial.printf(
            "[STACK] free words – Sensor:%u  State:%u  Output:%u  Watchdog:%u\n",
            uxTaskGetStackHighWaterMark(sensorTaskHandle),
            uxTaskGetStackHighWaterMark(stateTaskHandle),
            uxTaskGetStackHighWaterMark(outputTaskHandle),
            uxTaskGetStackHighWaterMark(NULL)   // WatchdogTask itself
        );
#endif

        // === Kiểm tra 5: Queue overflow ===
        if (snap_q_overflow > 0) {
            Serial.printf(
                "[WARNING] sensorQueue overflow: %u lần\n",
                snap_q_overflow
            );
            // Reset (volatile, race condition nhỏ – chấp nhận được, không critical)
            health.queue_overflow_count = 0;
        }

        // === Feed hardware watchdog ===
        // Nếu WatchdogTask còn chạy đến đây → hệ thống chưa bị treo hoàn toàn
        esp_task_wdt_reset();
    }
}
