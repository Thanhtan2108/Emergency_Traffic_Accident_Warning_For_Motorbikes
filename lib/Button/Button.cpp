#include "Button.h"

Button::Button(uint8_t pinButton, unsigned long debounceDelay) {
    this->pinButton = pinButton;
    this->debounceDelay = debounceDelay;
    this->lastButtonState = HIGH;  // Giả sử active-low (không nhấn = HIGH)
    this->buttonState = HIGH;     // Thêm: Khởi tạo trạng thái ban đầu
    this->lastDebounceTime = 0;
}

void Button::begin() {
    pinMode(pinButton, INPUT_PULLUP);  // Kéo lên nội bộ cho active-low
}

bool Button::isPressed() {
    bool reading = digitalRead(pinButton);  // Đọc giá trị pin (dùng bool cho phù hợp)

    // Nếu giá trị đọc thay đổi so với lần trước → reset timer
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    // Nếu đủ thời gian debounce
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // Chỉ cập nhật trạng thái nếu reading khác buttonState hiện tại
        if (reading != buttonState) {
            buttonState = reading;
        }
    }

    // Cập nhật lastButtonState cho lần sau (quan trọng!)
    lastButtonState = reading;

    // Trả về true nếu đang nhấn (LOW)
    return (buttonState == LOW);
}
