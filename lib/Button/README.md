# CLASS PUSH BUTTON

Dùng cho nút nhấn `active LOW`

Nhiệm vụ chính của class này là khởi tạo nút nhấn và check xem nút nhấn đã được nhấn chưa

## Các phương thức (API) chính và chức năng

### `Constructor Button`

- Định nghĩa và khởi tạo các giá trị ban đầu cho đối tượng được tạo từ class

- Các giá trị này là giá trị ban đầu vốn có của đối tượng

### `begin`

- Đây là phương thức dùng để bật chức năng `INPUT` cho nút nhấn

- Có sử dụng điện trở nội kéo lên

- Nên kết hợp với mạch debouce nút nhấn phần cứng

### `wasPressed`

- Đây là phương thức dùng để xác định xem đã nhấn nút chưa

## Vấn đề quan trọng nhất trong việc phát triển là debouce cho nút nhấn trong phương thức `wasPressed`

Quy trình thực hiện debouce cho nút nhấn diễn ra như sau:

- Gán thời gian hiện tại MCU đo được `millis()` cho `currentDebouceTime`

- Đọc trạng thái hiện tại của nút nhấn `currentButtonState` bằng `digitalRead(button_pin)`

- Kiểm tra trạng thái hiện tại `currentButtonState` này có khác trạng thái nút nhấn trước đó `lastButtonState` không? Ban đầu `lastButtonState = HIGH`

  - Có:

    - Gán thời gian hiện tại `currentDebouceTime` cho thời gian trước đó `lastDebouceTime`. Ban đầu `lastDebouceTime = 0`

    - Gán trạng thái hiện tại `currentButtonState` cho trạng thái trước đó `lastButtonState`

    => Mục đích là nếu có nhấn nút thì nút nhấn nhận trạng thái hiện tại và dạng waveform chuyển mức logic.

    => Có sự `thay đổi` về `mức logic` -> `reset debouce time`.

  - Không:

    - Không làm gì cả

    - Xem như nút nhấn chưa được nhấn

- Kiểm tra thời gian debouce `currentDebouceTime - lastDebouceTime < debouceDelay` đã ổn định chưa

  - Có:
  
    - Thì xem như nút nhấn chưa debouce xong

    =>`return false` để báo nút nhấn chưa sẵn sàng.

  - Không:

    - Không làm gì cả

    - Xem như đã debouce nút nhấn xong

- Sau khi đã xác định có nhấn nút và debouce nút nhấn xong. Kiểm tra trạng thái nút hiện tại `currentButtonState` đã ổn định và đã thay đổi mức logic so với trạng thái nút nhấn `buttonState` chưa? Ban đầu trạng thái nút `buttonState = HIGH`

  - Có:

    - Gán trạng thái nút hiện tại `currentButtonState` cho trạng thái nút nhấn `buttonState`

    - Trả nút nhấn về trạng thái đã nhấn `return (buttonState = LOW)`. Nếu `buttonState = LOW` đúng thì `return về true`

  - Không:

    - Không làm gì cả

- `return false` để bình thường nút nhấn ở trạng thái `HIGH` thì luôn báo là không được nhấn
