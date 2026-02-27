# README2 — Tài liệu tổng hợp: Phân tích giá trị IMU cho Xe 2 Bánh (MPU6050, 1 cảm biến trên thân xe)

**Mục đích:** tổng hợp toàn bộ phân tích về các giá trị gia tốc (accelerometer) và vận tốc góc (gyroscope) trong các trạng thái thực tế của xe 2 bánh, kèm các thông tin cần thiết để đo đạc, xử lý và phân biệt giữa trạng thái *bình thường*, *va chạm mạnh* và *té ngã*. Tài liệu này là tài liệu tra cứu để thiết kế thí nghiệm, thu dữ liệu và xây thuật toán phân loại trạng thái.

---

## 1. Tổng quan nhanh (tóm tắt)

- **Cảm biến:** 1 MPU6050 gắn cứng lên thân xe, gần trọng tâm, trục X về trước, Y sang trái, Z lên trên.
- **Các đại lượng quan tâm:** ax, ay, az (đơn vị g hoặc m/s²); ωx, ωy, ωz (đơn vị °/s hoặc rad/s); các đại lượng tổng |a|, |ω|; góc nghiêng (roll, pitch).
- **Ý tưởng chính:** phát hiện sự kiện bất thường bằng các chỉ số tổng (|a|, |ω|) trong cửa sổ thời gian, xác nhận bằng tư thế cuối (tilt) để phân biệt *té* hay *va chạm*.

---

## 2. Hệ trục, đơn vị, và công thức quan trọng

- **Hệ trục thân xe (body frame):** X = phía trước; Y = sang trái; Z = lên trên.
- **Chuyển đổi & đơn vị:** đọc raw → chia theo scale tùy thang đo (ví dụ ±8g → 4096 LSB/g; ±500°/s → 65.5 LSB/°/s). Có thể nhân thêm 9.80665 để đổi g → m/s² hoặc đổi °/s → rad/s.
- **Các công thức cơ bản:**
  - Độ lớn gia tốc: `|a| = sqrt(ax² + ay² + az²)` (đơn vị g hoặc m/s²)
  - Độ lớn vận tốc góc: `|ω| = sqrt(ωx² + ωy² + ωz²)` (°/s hoặc rad/s)
  - Góc roll = `atan2(ay, az)`; pitch = `atan2(-ax, sqrt(ay²+az²))` (radians hoặc độ)

---

## 3. Phân tích các trạng thái — đặc trưng vật lý và mẫu IMU (khoảng tham khảo)

> Các con số là *khoảng tham khảo* dựa trên phân tích vật lý + kinh nghiệm; cần điều chỉnh theo dữ liệu thực tế.

### 3.1 Trạng thái bình thường (đứng yên, dựng thẳng)

- ax, ay ≈ 0g; az ≈ +1g (accel đo trọng lực)
- |a| ≈ 1.0 g ± ~0.05–0.3 g (tùy rung)
- |ω| ≈ 0–10 °/s (dao động nhỏ do rung + micro corrections của lái)
- Góc roll/pitch ≈ 0° (đứng thẳng)

### 3.2 Đứng yên — chống chân chống (xe nghiêng tĩnh)

- ax ≈ 0g; ay = g·sin(θ); az = g·cos(θ)
- |a| ≈ 1.0 g (vẫn ≈1g, nhưng thành phần trên trục Y tăng)
- |ω| ≈ 0 (không quay)
- Góc roll = θ (duy trì ổn định)

### 3.3 Chạy thẳng đều

- ax ≈ 0 (trong khung tịnh tiến), az ≈ 1g ± nhiễu, ay ≈ nhỏ
- |a| ≈ 1.0 g ±0.1–0.4
- |ω| nhỏ (dao động): < ~20 °/s

### 3.4 Tăng ga / Phanh gấp (tịnh tiến không đều)

- Tăng ga: ax > 0 (tùy mức, ví dụ 0.5–3 m/s² ≈ 0.05–0.3 g)
- Phanh: ax < 0 (biên độ có thể lớn hơn)
- |ω| có thể tăng nhẹ (ωy do pitching)

### 3.5 Vào cua (cua trái / phải)

- Gia tốc hướng tâm thể hiện ở a_y: a_y ≈ ± v²/R → có thể dao động 0.1–1.5 g tùy tốc độ & bán kính
- |ω|: ωz ≈ v/R (đổi dấu theo hướng cua) — giá trị 20–200 °/s phụ thuộc tốc độ và bán kính
- Xe nghiêng: a_z giảm vì thành phần trọng lực chia cho trục z

### 3.6 Đường ngoằn ngoèo (trái–phải liên tục)

- a_y dao động thay đổi dấu liên tục
- ωz thay đổi dấu liên tục; ωx xuất hiện khi nghiêng/dựng
- Biên độ trung bình lớn hơn chạy thẳng nhưng không có spike lớn giống va chạm

### 3.7 Đường gồ ghề / ổ gà

- a_z xuất hiện **xung ngắn** có biên độ 1.5–3 g (còn phụ thuộc phuộc, tốc độ)
- |ω| có xung ngắn ở ωx/ωy do body lắc
- Đặc trưng: xung ngắn, sau đó phục hồi về baseline; không có thay đổi tư thế dài

### 3.8 Va chạm mạnh (impact)

- Đặc trưng: **xung |a| rất lớn** (thường > 3–5 g; có thể lên tới > 8 g trong va mạnh) trong thời gian ngắn (ms → vài chục ms)
- |ω| có thể spike hoặc không, tùy góc va chạm
- Sau va chạm: nếu xe **không té**, |a| và |ω| quay về baseline; tư thế (tilt) không thay đổi nhiều

### 3.9 Té ngã (fall / tip-over)

- Quy trình thường gồm 3 pha:
  1. **Mất thăng bằng (pre-fall):** ωx (quay quanh trục ngang) tăng nhanh; a_y/ a_z biến đổi theo nghiêng
  2. **Impact với mặt đất:** xung |a| lớn (±2–8 g hoặc hơn); |ω| lớn, hỗn loạn
  3. **Post-fall (nằm yên):** |ω| ≈ 0; tilt (roll hoặc pitch) duy trì lớn (> ~45°–60°)
- Đặc trưng nhận biết: **kết hợp** impact + sự gia tăng |ω| + tư thế cuối nghiêng lâu

---

## 4. Các đại lượng, cửa sổ thời gian và ngưỡng đề xuất (để thử nghiệm)

- **Sampling rate:** 200–1000 Hz (khuyến nghị bắt đầu 500 Hz)
- **Window cho event detection:** 10–100 ms cho impact detection; 0.5–3 s cho xác nhận tư thế
- **Ngưỡng khởi điểm (tham khảo):**
  - Event trigger: `|a| > 2.5 g` OR `|ω| > 200 °/s`
  - Impact: `|a| > 4.0 g` trong cửa sổ 10–50 ms
  - Gyro spike: `|ω| > 300 °/s`
  - Fall confirmation: tilt > 45°–60° duy trì > 1.5–3 s

> Lưu ý: *tất cả ngưỡng cần tinh chỉnh theo dữ liệu thực tế*.

---

## 5. Quy trình đo đạc & mẫu dữ liệu cần thu

**Mẫu dữ liệu tối thiểu ghi lại cho mỗi sample:**

- timestamp (ms), raw_ax, raw_ay, raw_az, raw_wx, raw_wy, raw_wz, ax_g, ay_g, az_g, wx_dps, wy_dps, wz_dps, |a| (g), |ω| (°/s), roll (deg), pitch (deg)

**Kịch bản test (bench + field):**

1. Bench: đứng yên, dựng thẳng, dựng với chân chống (nhiều góc θ), lắc mạnh bằng tay, gõ mô phỏng va chạm, xoay nhanh.
2. Field (thực tế, tốc độ thấp ban đầu): qua ổ gà, phanh gấp, cua gắt, ngoằn ngoèo, mô phỏng va chạm nhẹ (gõ), thử nghiêng an toàn để nằm (nếu an toàn / có biện pháp bảo hộ).
3. Ghi log nhiều lần cho mỗi kịch bản, đặt nhãn (label) rõ ràng thời điểm event.

---

## 6. Xử lý tín hiệu & phân tích dữ liệu (hướng dẫn)

- **Lọc:** LPF (≤ 20 Hz) cho tilt/trọng lực; HPF để tách thành phần động; band-pass nếu cần tách rung máy.
- **RMS / SMA / năng lượng:** tính RMS trong cửa sổ để đánh giá năng lượng xung, tránh dùng mẫu đơn.
- **Smoothing:** dùng trung bình chạy (moving average) hoặc filter dạng IIR để giảm nhiễu.
- **Phát hiện peak:** xác định peaks trên |a| và |ω| với điều kiện thời gian tối thiểu (debounce) để loại nhiễu.
- **Tilt check:** sau event, chờ 0.5–3 s rồi tính roll/pitch trung bình để xác nhận tư thế cuối.
- **Saturation check:** kiểm tra raw đạt giá trị ±32767 → tăng thang đo nếu xảy ra.

---

## 7. Cấu hình thang đo đề xuất & lý do

- **Accelerometer:** ±8 g (điểm cân bằng giữa không bị kẹp khi va chạm mạnh và vẫn giữ độ nhạy cho rung/tilt).
- **Gyroscope:** ±500 °/s (đủ cho cua gắt và các sự kiện quay nhanh ở té; ±250 dễ kẹp khi té).

---

## 8. Ghi chú về gắn cảm biến và hiệu chỉnh

- Gắn **cứng** vào khung, tránh vị trí mềm (yên bọc, nhựa rung).
- Hướng trục: tuân thủ quy ước X/Y/Z để phân tích dễ dàng.
- **Calibration:** đo offset gyro (bias) khi đứng yên; đo bias accel (tham chiếu 1g trên Z khi dựng thẳng). Lưu bias để trừ trong phần mềm.

---

## 9. Cách phân biệt các hiện tượng dễ nhầm lẫn

- **ổ gà vs impact:** ổ gà → a_z spike ngắn + phục hồi nhanh, ω nhỏ; impact → a spike lớn ± kèm ω hỗn loạn; nếu sau đó tilt không thay đổi → không té.
- **cua gắt vs té:** cua → |ω| (ωz) lớn và a_y lớn, tilt có thể nhưng không có impact spike; té → có phase nghiêng trước, impact và tilt duy trì.

---

## 10. Kế hoạch thử nghiệm & tinh chỉnh ngưỡng

1. Thu data bench để có baseline nhiễu.
2. Thực hiện field tests nhiều kịch bản — gắn label chính xác.
3. Phân tích phân phối |a|, |ω| theo kịch bản; tìm overlapping region giữa classes.
4. Chọn ngưỡng ban đầu để tối ưu tradeoff false positive / false negative.
5. Lặp: điều chỉnh thang đo, filter, window, thresholds.

---

## 11. Tài liệu tham khảo & ghi chú cuối

- MPU6050 datasheet (để tra register setting, DLPF, thang đo)
- Tài liệu về xử lý tín hiệu IMU và detection patterns (tilt detection, peak detection)

> **Ghi chú:** tài liệu này là bản tóm tắt nghiên cứu/trao đổi để làm cơ sở cho thu thập dữ liệu thử nghiệm. Tất cả ngưỡng đề xuất **phải** được tinh chỉnh bằng dữ liệu thực tế trước khi sử dụng trong hệ báo động thực tế.

---

*End of README2 — lưu ý cập nhật sau khi có log thực tế.*
