#pragma once
// ============================================================
//  SignalProcessor.h
//  Tầng 2 – Xử lý tín hiệu từ RawSensorData → ProcessedData.
//
//  TRÁCH NHIỆM:
//    Chạy pipeline 6 bước theo thứ tự BẮT BUỘC:
//      Bước 0: Outlier Rejection
//      Bước 1: Tính |a_raw| và |ω_raw|
//      Bước 2: Dynamic Acceleration  ( |a_dynamic| = | |a_raw| - baseline | )
//      Bước 3: Moving Average Filter (N=5, circular buffer)
//      Bước 4: Tính roll_rate
//      Bước 5: Cập nhật Baseline EMA (chỉ khi được phép)
//
//  KHÔNG BIẾT GÌ VỀ: AccidentDetector, FreeRTOS, State Machine.
//
//  Tài liệu: README.md – Section VI
//  Thứ tự implement: file số 4 (sau SensorReader)
// ============================================================

#include <Arduino.h>
#include "DataTypes.h"
#include "config.h"


class SignalProcessor {
public:

    // --------------------------------------------------------
    //  begin()
    //  Khởi tạo toàn bộ trạng thái nội bộ. Gọi một lần trong setup().
    //
    //  Sau khi gọi:
    //    - Circular buffer fill = 0.0f
    //    - baseline             = BASELINE_INITIAL (1.0g)
    //    - warmup_sample_count  = 0
    //    - baseline_ready       = false
    //    - baseline_update_enabled = true  (warm-up chạy ngay)
    // --------------------------------------------------------
    void begin();

    // --------------------------------------------------------
    //  process()
    //  Chạy toàn bộ pipeline 6 bước và trả về ProcessedData.
    //
    //  Nếu input.valid == false (SensorReader báo lỗi):
    //    → trả ProcessedData rỗng với valid = false, bỏ qua pipeline.
    //
    //  THỨ TỰ CÁC BƯỚC KHÔNG ĐƯỢC ĐẢO:
    //    Bước 2 PHẢI trước Bước 3 – trừ baseline trên raw signal
    //    trước khi filter, tránh baseline bị smooth cùng spike.
    // --------------------------------------------------------
    ProcessedData process(const RawSensorData& raw);

    // --------------------------------------------------------
    //  setBaselineUpdateEnabled()
    //  Bật / tắt cập nhật baseline EMA ở Bước 5.
    //
    //  StateTask gọi hàm này để khóa baseline khi phát hiện sự kiện:
    //    - Tắt (false) khi vào EVENT_DETECTED
    //    - Bật (true)  khi về NORMAL hoặc TIMEOUT_RESET
    //
    //  Lưu ý: warm-up luôn chạy bất kể cờ này, vì warmup_sample_count
    //  được kiểm tra độc lập trong Bước 5.
    // --------------------------------------------------------
    void setBaselineUpdateEnabled(bool enabled);

    // --------------------------------------------------------
    //  isBaselineReady()
    //  Trả về true sau khi đủ BASELINE_WARMUP_SAMPLES mẫu.
    //  AccidentDetector dùng để quyết định thoát WARMUP state.
    // --------------------------------------------------------
    bool isBaselineReady() const;

    // --------------------------------------------------------
    //  getBaseline()
    //  Trả về giá trị baseline EMA hiện tại (g).
    //  Dùng để debug / log trên Serial.
    // --------------------------------------------------------
    float getBaseline() const;

    // --------------------------------------------------------
    //  resetFilter()
    //  Fill toàn bộ circular buffer về 0.0f.
    //  Gọi sau khi xử lý xong sự kiện (STRONG_IMPACT, FALL_DETECTED,
    //  TIMEOUT_RESET) để tránh "dư âm" spike cũ ảnh hưởng lần sau.
    //
    //  KHÔNG reset baseline – baseline là giá trị dài hạn.
    // --------------------------------------------------------
    void resetFilter();

private:
    // --- Circular buffer cho Moving Average Filter (Bước 3) ---
    float _a_buffer[MA_FILTER_SIZE];   // buffer |a_dynamic_raw|
    float _w_buffer[MA_FILTER_SIZE];   // buffer |ω_raw|
    int   _buffer_index;               // con trỏ vị trí ghi tiếp theo

    // --- Baseline EMA (Bước 5) ---
    float    _baseline;                // giá trị baseline hiện tại (g)
    bool     _baseline_update_enabled; // cờ bật/tắt từ StateTask
    uint16_t _warmup_sample_count;     // số mẫu đã xử lý trong warm-up
    bool     _baseline_ready;          // true sau BASELINE_WARMUP_SAMPLES mẫu

    // --- roll_rate (Bước 4) ---
    float    _roll_prev;               // roll ở chu kỳ trước (°)
    uint32_t _timestamp_prev;          // timestamp chu kỳ trước (ms)

    // --------------------------------------------------------
    //  Các hàm private thực hiện từng bước pipeline
    // --------------------------------------------------------

    // Bước 0: Trả về true nếu mẫu HỢP LỆ (không phải outlier)
    bool     _step0_outlierCheck(float a_mag, float w_mag) const;

    // Bước 1: Tính |a_raw| và |ω_raw| từ 3 trục
    float    _step1_magnitude(float x, float y, float z) const;

    // Bước 2: Tính |a_dynamic_raw| = | |a_raw| - baseline |
    float    _step2_dynamicAccel(float a_mag) const;

    // Bước 3: Đẩy giá trị mới vào circular buffer, trả về trung bình
    float    _step3_movingAverage(float* buffer, float new_value);

    // Bước 4: Tính roll_rate (°/s), cập nhật _roll_prev và _timestamp_prev
    float    _step4_rollRate(float roll_current, uint32_t timestamp_current);

    // Bước 5: Cập nhật baseline EMA nếu được phép
    void     _step5_updateBaseline(float a_raw_mag);
};
