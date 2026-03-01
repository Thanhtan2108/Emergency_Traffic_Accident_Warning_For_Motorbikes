#pragma once

#include <Arduino.h>
#include "DataTypes.h"

// ============================================================
//  IModule.h
//  Interface chuẩn (abstract base class) cho tất cả các khối
//  chức năng trong hệ thống cảm biến.
//
//  Nguyên tắc áp dụng:
//  - Interface Segregation: chỉ expose method thực sự cần thiết
//  - Dependency Inversion: code phụ thuộc vào abstraction này,
//    không phụ thuộc vào concrete class
//  - Open/Closed: thêm khối mới chỉ cần implement interface,
//    không sửa code hiện có
// ============================================================

class IModule {
public:
    virtual ~IModule() = default;

    // --------------------------------------------------------
    // begin() - Khởi tạo khối (hardware init, tạo task, v.v.)
    // Trả về: true nếu khởi tạo thành công, false nếu thất bại
    // --------------------------------------------------------
    virtual bool begin() = 0;

    // --------------------------------------------------------
    // getModuleName() - Tên khối để log và debug
    // --------------------------------------------------------
    virtual const char* getModuleName() const = 0;

    // --------------------------------------------------------
    // isHealthy() - Kiểm tra khối có đang hoạt động bình thường
    // Dùng bởi SystemWatchdog để giám sát sức khỏe hệ thống
    // --------------------------------------------------------
    virtual bool isHealthy() const = 0;

    // Xóa copy constructor và copy assignment để tránh nhân đôi
    // module (mỗi module là singleton trong hệ thống này)
    IModule(const IModule&)            = delete;
    IModule& operator=(const IModule&) = delete;

protected:
    // Constructor chỉ cho phép subclass gọi
    IModule() = default;
};


// ============================================================
//  IDataProducer<T> - Interface cho module tạo ra dữ liệu
//  Template pattern: T là kiểu dữ liệu output
// ============================================================
template<typename T>
class IDataProducer {
public:
    virtual ~IDataProducer() = default;

    // Lấy dữ liệu mới nhất (non-blocking)
    // Trả về true nếu có dữ liệu mới, false nếu không có
    virtual bool getData(T& outData) = 0;
};


// ============================================================
//  IDataConsumer<T> - Interface cho module nhận dữ liệu
//  Template pattern: T là kiểu dữ liệu input
// ============================================================
template<typename T>
class IDataConsumer {
public:
    virtual ~IDataConsumer() = default;

    // Nhận và xử lý dữ liệu mới
    virtual void onDataReceived(const T& data) = 0;
};


// ============================================================
//  Logger - Utility class dùng chung để log có prefix
//  Không phải module, nhưng dùng toàn hệ thống
// ============================================================
class Logger {
public:
    // Log với prefix tên module
    static void info(const char* moduleName, const char* msg) {
        if (!flag()) return;
        Serial.print("[INFO][");
        Serial.print(moduleName);
        Serial.print("] ");
        Serial.println(msg);
    }

    static void warn(const char* moduleName, const char* msg) {
        if (!flag()) return;
        Serial.print("[WARN][");
        Serial.print(moduleName);
        Serial.print("] ");
        Serial.println(msg);
    }

    static void error(const char* moduleName, const char* msg) {
        // Error luôn in ra bất kể flag()
        Serial.print("[ERR ][");
        Serial.print(moduleName);
        Serial.print("] ");
        Serial.println(msg);
    }

    // Log kèm giá trị số (tránh phải format string thủ công)
    static void infoValue(const char* moduleName, const char* label, float value) {
        if (!flag()) return;
        Serial.print("[INFO][");
        Serial.print(moduleName);
        Serial.print("] ");
        Serial.print(label);
        Serial.println(value, 4);
    }

    static void infoValue(const char* moduleName, const char* label, int32_t value) {
        if (!flag()) return;
        Serial.print("[INFO][");
        Serial.print(moduleName);
        Serial.print("] ");
        Serial.print(label);
        Serial.println(value);
    }

    static void setEnabled(bool enabled) { flag() = enabled; }
    static bool isEnabled()              { return flag(); }

private:
    // Local static — header-only, không cần .cpp, C++11 compatible
    static bool& flag() {
        static bool _enabled = true;
        return _enabled;
    }
};
