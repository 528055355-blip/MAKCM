#pragma once

#include "InitSettings.h"
#include "binary_protocol.h"
#include <Arduino.h>
#include <USB.h>
#include <USBHIDMouse.h>
#include "USBSetup.h"
#include <esp_intr_alloc.h>
#include <cstring>
#include <atomic>

// Extern variables
extern USBHIDMouse Mouse;
extern TaskHandle_t mouseMoveTaskHandle;
extern TaskHandle_t ledFlashTaskHandle;
extern TaskHandle_t btnSubTaskHandle;
extern const char *commandQueue[];
extern int currentCommandIndex;
extern bool usbReady;

// Mouse position tracking
extern int16_t mouseX;
extern int16_t mouseY;

// Buffer lengths
#define MAX_SERIAL0_COMMAND_LENGTH 100
#define MAX_SERIAL1_COMMAND_LENGTH 620

// Atomic flags for button states (from real mouse)
extern std::atomic<uint8_t> realMouseButtons;

// Atomic flags for injected button states (from serial0)
extern std::atomic<bool> isLeftButtonPressed;
extern std::atomic<bool> isRightButtonPressed;
extern std::atomic<bool> isMiddleButtonPressed;
extern std::atomic<bool> isForwardButtonPressed;
extern std::atomic<bool> isBackwardButtonPressed;
extern std::atomic<bool> serial0Locked;

// Button subscription
extern std::atomic<int> btnSubIntervalMs;  // 0 = disabled

// Mutex for inject move
extern std::atomic<bool> kmMoveCom;

// Function declarations - binary packet handlers
void handleBinaryMousePacket(const uint8_t* pkt);
void handleBinaryControlPacket(const uint8_t* pkt);

// Function declarations - mouse control
void handleMove(int x, int y);
void handleMoveto(int x, int y);
void handleMouseButton(uint8_t button, bool press);
void handleMouseWheel(int wheelMovement);
void handleGetPos();
void handleGetButtons();

// Function declarations - serial handlers
void serial1RX();
void serial0RX();
void notifyLedFlashTask();

// Function declarations - USB
void handleUsbHello();
void handleUsbGoodbye();
void handleNoDevice();
void sendNextCommand();

// Function declarations - tasks
void mouseMoveTask(void *pvParameters);
void ledFlashTask(void *parameter);
void btnSubscriptionTask(void *parameter);

// Function declarations - serial0 command processing
void processSerial0Command(const char *command);

// Function declarations - serial1 text command processing (descriptor exchange)
void processSerial1TextCommand(const char *command);

// Debug
void handleDebug(const char *command);
void handleSerial0Speed(const char *command);
void handleEspLog(const char *command);

// Extern functions for JSON data handling (in InitSettings.cpp)
extern void receiveDeviceInfo(const char *jsonString);
extern void receiveDescriptorDevice(const char *jsonString);
extern void receiveEndpointDescriptors(const char *jsonString);
extern void receiveInterfaceDescriptors(const char *jsonString);
extern void receiveHidDescriptors(const char *jsonString);
extern void receiveIADescriptors(const char *jsonString);
extern void receiveEndpointData(const char *jsonString);
extern void receiveUnknownDescriptors(const char *jsonString);
extern void receivedescriptorConfiguration(const char *jsonString);

// Command table structure
struct CommandEntry {
    const char *command;
    void (*handler)(const char *);
};
