#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class Button {
private:
    uint8_t pinButton;
    bool buttonState;      // Thêm: Trạng thái nút hiện tại (sau debounce)
    bool lastButtonState;  // Trạng thái đọc trước đó
    unsigned long lastDebounceTime;
    unsigned long debounceDelay;

public:
    Button(uint8_t pinButton, unsigned long debounceDelay = 50);
    void begin();
    bool wasPressed();
};

#endif
