# Test chức năng cho Motion Sensor

## Giải thích thiết kế file test

Tại sao không in thẳng 100Hz? `MonitorTask` không in từng frame mà drain queue liên tục rồi báo cáo gộp mỗi 500ms — tránh Serial bị flood, dễ đọc hơn nhiều.

7 test case kiểm tra gì:

| Test | Kiểm tra | Cách xác nhận | PASS |
| ------ | ---------- | --------------- | ------ |
| [1] | I2C kết nối + WHO_AM_I | begin() không halt | ✓ |
| [2] | Gyro calibration | Serial in offset X/Y/Z | ✓ |
| [3] | Task 100Hz chạy | sampleCount tăng mỗi kỳ | ✓ |
| [4] | Queue nhận data | receivedCount > 0 | ✓ |
| [5] | Data hợp lệ | Không all-zero, AccZ > 0.5g | ✓ |
| [6] | isHealthy() | Trả về true | ✓ |
| [7] | Sample rate & drop | Gần 100Hz, droppedFrames = 0 | ✓ |
