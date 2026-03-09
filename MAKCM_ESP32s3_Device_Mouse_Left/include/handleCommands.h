#pragma once

#include "InitSettings.h"
#include "binary_protocol.h"
#include "ClonedHIDMouse.h"   // 包含 USBHIDMouse.h 和 ClonedHIDMouse 定义
#include <Arduino.h>
#include <USB.h>
#include "USBSetup.h"
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <cstring>
#include <atomic>

// ============================================================
// 外部变量
// ============================================================
extern ClonedHIDReportDescriptor clonedDescriptor;  // 定义在 USBSetup.cpp
extern ClonedHIDMouse Mouse;                         // 定义在 USBSetup.cpp
extern TaskHandle_t mouseMoveTaskHandle;
extern TaskHandle_t ledFlashTaskHandle;
extern TaskHandle_t btnSubTaskHandle;
extern const char *commandQueue[];
extern int currentCommandIndex;
extern bool usbReady;

extern int16_t mouseX;
extern int16_t mouseY;

// ============================================================
// 常量
// ============================================================
#define MAX_SERIAL0_COMMAND_LENGTH 100
#define MAX_SERIAL1_COMMAND_LENGTH 620

// ============================================================
// 原子变量（定义在 handleCommands.cpp）
// ============================================================
extern std::atomic<uint8_t> realMouseButtons;
extern std::atomic<bool>    isLeftButtonPressed;
extern std::atomic<bool>    isRightButtonPressed;
extern std::atomic<bool>    isMiddleButtonPressed;
extern std::atomic<bool>    isForwardButtonPressed;
extern std::atomic<bool>    isBackwardButtonPressed;
extern std::atomic<bool>    serial0Locked;
extern std::atomic<int>     btnSubIntervalMs;
extern std::atomic<bool>    kmMoveCom;

// ============================================================
// 函数声明
// ============================================================

// 二进制包处理
void handleBinaryMousePacket(const uint8_t* pkt);
void handleBinaryControlPacket(const uint8_t* pkt);

// 鼠标控制
void handleMove(int x, int y);
void handleMoveto(int x, int y);
void handleMouseButton(uint8_t button, bool press);
void handleMouseWheel(int wheelMovement);
void handleGetPos();
void handleGetButtons();

// 串口 RX 处理
void serial1RX();
void serial0RX();
void notifyLedFlashTask();

// USB 生命周期
void handleUsbHello();
void handleUsbGoodbye();
void handleNoDevice();
void sendNextCommand();

// RTOS 任务
void mouseMoveTask(void *pvParameters);
void ledFlashTask(void *parameter);
void btnSubscriptionTask(void *parameter);

// 命令处理
void processSerial0Command(const char *command);
void processSerial1TextCommand(const char *command);

// 调试/配置
void handleDebug(const char *command);
void handleSerial0Speed(const char *command);
void handleEspLog(const char *command);

// JSON 描述符接收函数（定义在 InitSettings.cpp）
extern void receiveDeviceInfo(const char *jsonString);
extern void receiveDescriptorDevice(const char *jsonString);
extern void receiveEndpointDescriptors(const char *jsonString);
extern void receiveInterfaceDescriptors(const char *jsonString);
extern void receiveHidDescriptors(const char *jsonString);
extern void receiveIADescriptors(const char *jsonString);
extern void receiveEndpointData(const char *jsonString);
extern void receiveUnknownDescriptors(const char *jsonString);
extern void receivedescriptorConfiguration(const char *jsonString);

// 命令表结构体
struct CommandEntry {
    const char *command;
    void (*handler)(const char *);
};
