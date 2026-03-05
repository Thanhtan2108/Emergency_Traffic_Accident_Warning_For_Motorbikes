# Tạo folder dùng chung cho chức năng

## Bước 1 — Common Layer

Đây là nền móng toàn hệ thống, gồm 4 file trong `lib/Common/`:

## Những gì vừa được tạo

`Config.h` — Tất cả hằng số tập trung tại một chỗ:

- Pin GPIO (SDA, SCL, Buzzer, LED)

- Ngưỡng phát hiện tai nạn (`ACC_CRASH_THRESHOLD = 2.5g`, `TILT_ANGLE_THRESHOLD = 60°`, ...)

- FreeRTOS config (stack size, priority, core assignment cho từng task)

- Queue depths, timeouts, watchdog timeout

`DataTypes.h` — Toàn bộ struct truyền dữ liệu qua Queue:

- `RawSensorData` → `NormalizedData` → `MotionFeatures` → `AccidentEvent` → `AlertRequest`

- Các `enum class` rõ ràng: `VehicleState`, `AccidentType`, `SystemState`, `TaskID`

- Utility functions `toString()` cho từng enum để log

`IModule.h` — Interface chuẩn cho mọi khối:

- `IModule`: abstract base với `begin()`, `isHealthy()`, `getModuleName()`

- `IDataProducer<T>` / `IDataConsumer<T>`: template interface cho data pipeline

- `Logger`: utility class thread-safe với prefix tên module

`QueueManager.h` — Singleton quản lý tập trung tất cả FreeRTOS primitives:

- 5 data queues (pipeline từ raw → alert)

- 1 watchdog heartbeat queue

- 1 EventGroup (7 system event bits)

- 1 Serial mutex (tránh log bị xáo trộn giữa các task)

- Helper `sendHeartbeat()` cho các task gọi dễ dàng
