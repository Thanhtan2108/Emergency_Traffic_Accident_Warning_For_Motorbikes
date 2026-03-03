# Bước 4 — SignalProcessor

## Những gì được implement

### Pipeline xử lý tín hiệu theo 4 bước tuần tự

```text
NormalizedData → [LPF] → [Complementary Filter] → [Yaw Integration] → [Compute Features] → MotionFeatures
```

### Bước 1 — Low-Pass Filter (`alpha_acc=0.15`, `alpha_gyro=0.10`)

```text
y[n] = alpha × x[n] + (1 - alpha) × y[n-1]
```

Lọc nhiễu tần số cao (rung động cơ, điện từ) trước khi tính góc.

### Bước 2 — Complementary Filter (`gyro=98%`, `accel=2%`)

```text
angle = 0.98 × (angle + gyro_filtered × dt) + 0.02 × angleAcc
```

Kết hợp gyro (nhanh nhưng drift) với accel (ổn định nhưng nhiễu động).

### Bước 3 — Tích phân Yaw: `angleZ += gyroZ_filtered × dt` — accel không đo được yaw nên dùng gyro thuần

### Bước 4 — Compute Features: `totalAccMag`, `angularVelMag`, `jerk = Δ|acc|/dt`
