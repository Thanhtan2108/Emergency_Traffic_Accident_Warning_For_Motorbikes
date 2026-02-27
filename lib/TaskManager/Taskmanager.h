#pragma once
// ============================================================
//  TaskManager.h
//  Tầng 4 – FreeRTOS Task definitions và shared resources.
//
//  TRÁCH NHIỆM:
//    Khai báo 4 task functions, tất cả shared resources
//    (Queue, EventGroup, Mutex, SystemHealth), và hàm
//    initTasks() để main.ino gọi trong setup().
//
//  SHARED RESOURCES (tài liệu Section IX.1):
//    sensorQueue  – ProcessedData,  5 phần tử,  SensorTask → StateTask
//    alertQueue   – AlertInfo,       3 phần tử,  StateTask  → OutputTask
//    alertGroup   – EventGroup 2bit, StateTask  → OutputTask
//    stateMutex   – Mutex,           StateTask  ↔ WatchdogTask
//    health       – SystemHealth,    nhiều writer, quy tắc volatile/mutex
//
//  Tài liệu: README.md – Section VIII, IX
//  Thứ tự implement: file số 6 (sau AccidentDetector)
// ============================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>

#include "DataTypes.h"
#include "config.h"
#include "SensorReader.h"
#include "SignalProcessor.h"
#include "AccidentDetector.h"


// ============================================================
//  SHARED RESOURCES – extern declarations
//  Định nghĩa thực sự nằm trong TaskManager.cpp.
//  main.ino và tất cả task đều include file này để truy cập.
// ============================================================
extern QueueHandle_t     sensorQueue;   // ProcessedData: SensorTask → StateTask
extern QueueHandle_t     alertQueue;    // AlertInfo:     StateTask  → OutputTask
extern EventGroupHandle_t alertGroup;   // 2 bits:        StateTask  → OutputTask
extern SemaphoreHandle_t stateMutex;    // Bảo vệ StateTask fields của SystemHealth
extern SystemHealth      health;        // Dữ liệu sức khỏe hệ thống

// Task handles – dùng bởi WatchdogTask để đọc stack watermark
extern TaskHandle_t sensorTaskHandle;
extern TaskHandle_t stateTaskHandle;
extern TaskHandle_t outputTaskHandle;
extern TaskHandle_t watchdogTaskHandle;

// Object references – được set bởi initTasks() trước khi tạo task
extern SensorReader*      g_sensorReader;
extern SignalProcessor*   g_signalProcessor;
extern AccidentDetector*  g_accidentDetector;


// ============================================================
//  initTasks()
//  Khởi tạo tất cả shared resources và tạo 4 FreeRTOS task.
//  Gọi trong setup() SAU KHI đã gọi begin() trên tất cả objects.
//
//  Tham số:
//    sr  – con trỏ đến SensorReader đã được begin()
//    sp  – con trỏ đến SignalProcessor đã được begin()
//    ad  – con trỏ đến AccidentDetector đã được begin()
// ============================================================
void initTasks(SensorReader* sr, SignalProcessor* sp, AccidentDetector* ad);


// ============================================================
//  4 TASK FUNCTIONS
//  Khai báo theo đúng signature FreeRTOS: void fn(void* param)
// ============================================================
void SensorTask  (void* param);
void StateTask   (void* param);
void OutputTask  (void* param);
void WatchdogTask(void* param);
