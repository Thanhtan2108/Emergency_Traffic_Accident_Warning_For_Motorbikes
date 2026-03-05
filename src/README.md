# Bước 5 — AccidentDetector

## State Machine

```text
UNKNOWN ──(frame đầu tiên)──► NORMAL
                                │
              phát hiện bất thường (1 frame)
                                │
                                ▼
                           SUSPICIOUS
                           /         \
        ≥5 frame bất thường           ≥10 frame bình thường
                /                               \
               ▼                                ▼
          ACCIDENT ──(sau 30 giây)──────► NORMAL
```

## Logic phân loại tai nạn

| Loại | Điều kiện |
| ---- | --------- |
| CRASH | totalAccMag > 2.5g |
| FALL | \|angleX\| hoặc \|angleY\| > 60° |
| SUDDEN_STOP | jerk > 10 g/s |
| COMBINED | ≥ 2 điều kiện cùng lúc |

## Cách test thực tế

TEST 3 — Giữ yên 10 giây: state=NORMAL liên tục → không false positive

TEST 4 — Lắc mạnh sensor: quan sát NORMAL→SUSPICIOUS→ACCIDENT với log snapshot

TEST 5 — Nghiêng > 60°: phát hiện FALL

TEST 7 — Debounce: lắc 1-4 frame rồi dừng → phải quay về NORMAL, không lên ACCIDENT
