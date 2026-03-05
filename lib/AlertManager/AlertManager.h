#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "IModule.h"
#include "DataTypes.h"
#include "Config.h"
#include "Buzzer.h"

// ============================================================
//  AlertManager.h  —  Khối 5: Quản lý cảnh báo âm thanh
//
//  Logic cảnh báo:
//    - Bình thường         : buzzer TẮT hoàn toàn
//    - Phát hiện tai nạn   : buzzer KÊU LIÊN TỤC không tắt
//    - Chỉ dừng khi        : nhận STOP_ALERT (từ nút nhấn / StateManager)
//
//  Biến cờ _buzzerEnabled:
//    true  = buzzer được phép kêu (đang ở trạng thái cảnh báo)
//    false = buzzer bị tắt
//    → Sau này nút nhấn chỉ cần gọi toggle() để đổi trạng thái
//    → AlertManager tự xử lý bật/tắt dựa theo cờ này
//
//  Các lệnh (AlertCommand):
//    START_ALERT  → đặt _buzzerEnabled=true, bắt đầu beep liên tục
//    STOP_ALERT   → đặt _buzzerEnabled=false, tắt buzzer ngay
//    NONE         → bỏ qua
//
//  Pattern beep theo loại tai nạn (AccidentType):
//    CRASH       → beep nhanh      (ON=200ms, OFF=100ms)
//    FALL        → beep chậm       (ON=500ms, OFF=300ms)
//    SUDDEN_STOP → beep vừa        (ON=300ms, OFF=200ms)
//    COMBINED    → beep rất nhanh  (ON=100ms, OFF=100ms)
//
//  Nút nhấn (chưa phát triển):
//    Khi nhấn nút → gọi setBuzzerEnabled(!_buzzerEnabled)
//    hoặc gửi STOP_ALERT / START_ALERT vào alertQueue
// ============================================================

class AlertManager : public IModule {
public:
    AlertManager(QueueHandle_t alertQueue, Buzzer* buzzer);

    bool        begin()         override;
    const char* getModuleName() const override { return "AlertManager"; }
    bool        isHealthy()     const override;

    // --------------------------------------------------------
    // Getters
    // --------------------------------------------------------
    bool     isBuzzerEnabled()   const { return _buzzerEnabled; }
    bool     isAlerting()        const { return _isAlerting; }
    uint32_t getAlertCount()     const { return _alertCount; }
    uint32_t getProcessedCount() const { return _processedCount; }

    // --------------------------------------------------------
    // Dùng cho nút nhấn (sau này):
    // Cho phép set cờ từ bên ngoài mà không cần gửi qua queue
    // Gọi từ ISR hoặc button task khi nút được nhấn
    // --------------------------------------------------------
    void setBuzzerEnabled(bool enabled);

private:
    static void taskEntry(void* pvParameters);
    void        taskRun();

    void handleStartAlert(AccidentType type);
    void handleStopAlert();

    // Delay có thể bị ngắt nếu _buzzerEnabled bị đổi
    // Trả về true = tiếp tục beep, false = phải dừng
    bool interruptibleDelay(uint32_t durationMs);

    void getBeepPattern(AccidentType type,
                        uint32_t& outOnMs, uint32_t& outOffMs);

    // --------------------------------------------------------
    // Members
    // --------------------------------------------------------
    QueueHandle_t   _alertQueue;
    Buzzer*         _buzzer;
    TaskHandle_t    _taskHandle;

    volatile bool   _taskRunning;

    // Biến cờ chính — quản lý toàn bộ trạng thái buzzer
    // true  = buzzer ĐƯỢC PHÉP kêu
    // false = buzzer PHẢI TẮT
    volatile bool   _buzzerEnabled;

    // true = task đang trong vòng beep liên tục
    volatile bool   _isAlerting;

    uint32_t        _alertCount;
    uint32_t        _processedCount;
};
