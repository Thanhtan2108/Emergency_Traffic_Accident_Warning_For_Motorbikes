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

bool Button::wasPressed() {
    unsigned long currentDebouceTime = millis();

    bool currentButtonState = digitalRead(pinButton);

    // Nếu có thay đổi → reset debounce timer
    if (currentButtonState != lastButtonState) {
        lastDebounceTime = currentDebouceTime;
        lastButtonState = currentButtonState;
    }

    // Chưa ổn định đủ lâu → bỏ qua
    if ((currentDebouceTime - lastDebounceTime) < debounceDelay) {
        return false;
    }

    // Trạng thái ổn định thay đổi
    if (currentButtonState != buttonState) {
        buttonState = currentButtonState;

        // Cạnh nhấn: HIGH → LOW
        return (buttonState == LOW);
    }

    return false;
}
