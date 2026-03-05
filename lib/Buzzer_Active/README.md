# CLASS BUZZER

## Xây dựng 1 `class Buzzer` đóng vai trò là 1 `interface`

- `interface` là 1 loại giao diện chỉ chứa các phương thức công khai nhưng không implement phương thức đó

- Các class khác `BuzzerActive` kế thừa `interface` này sẽ có các phương thức sẵn có của interface

- Sử dụng từ khóa `virtual` để khi gọi hàm thông qua con trỏ hoặc tham chiếu kiểu lớp cơ sở, chương trình sẽ tự động chọn đúng phiên bản của hàm đã được override trong lớp dẫn xuất

## Xây dựng 1 `class BuzzerActive` kế thừa từ `class Buzzer`

### `class Buzzer` này có các phương thức chính

#### `Destructor Buzzer`

- Là hàm hủy để hủy đối tượng khi đối tượng bị hủy

#### `begin`

- Là phương thức dùng để bật chế độ `OUPUT` cho buzzer

#### `turnOn`

- Là phương thức để bật buzzer

#### `turnOff`

- Là phương thức dùng để tắt buzzer

#### Class Buzzer thể hiện tính OOP

- Thể hiện `tính đa hình` của OOP trong C++ ở chỗ trong `class Buzzer` chỉ định nghĩa các phương thức mà không implement phương thức đó

## `class BuzzerActive`

- Sử dụng loại buzzer chủ động `active HIGH`

### Các phương thức chính và chức năng

#### `Constructor BuzzerActive`

- Khởi tạo các giá trị ban đầu cho 1 đối tượng được tạo ra từ `class BuzzerActive`

#### `.begin`

- Là phương thức dùng để thiết lập chế độ `OUTPUT` cho nút nhấn

#### `.turnOn`

- Là phương thức dùng để bật buzzer chủ động lên mức `HIGH`

#### `.turnOff`

- Là phương thức dùng để tắt buzzer chủ động xuống mức `LOW`

### Class BuzzerActive thể hiện tính OOP

- Thể hiện `tính đóng gói` của OOP trong C++ ở chỗ có đóng gói các phương thức và thuộc tính trong 1 class duy nhất

- Thể hiện `tính kế thừa` của OOP trong C++ ở chỗ có kế thừa lại `class Buzzer` bằng `: public Buzzer`

- Thể hiện `tính trừu tượng hóa` trong OOP ở chỗ trong `class BuzzerActive` hoặc các class khác miễn kế thừa `class Buzzer`, các phương thức vẫn có tên giống nhau nhưng được implement nhiều cách khác nhau để thực hiện các nhiệm vụ khác nhau
