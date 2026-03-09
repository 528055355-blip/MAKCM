#include "handleCommands.h"   // 已包含 ClonedHIDMouse.h、USBSetup.h 等
#include "InitSettings.h"
#include "binary_protocol.h"
#include "tasks.h"
#include <Arduino.h>
#include <USB.h>
#include <esp_intr_alloc.h>
#include <cstring>
#include <atomic>
#include <mutex>
#include <RingBuf.h>

// ============================================================
// State Variables
// ============================================================

// Atomic variables for mouse movement injection
std::atomic<int> moveX(0);
std::atomic<int> moveY(0);
std::atomic<bool> kmMoveCom(false);

// Button states from real mouse (via binary packets)
std::atomic<uint8_t> realMouseButtons(0);

// Button states from serial injection
std::atomic<bool> isLeftButtonPressed(false);
std::atomic<bool> isRightButtonPressed(false);
std::atomic<bool> isMiddleButtonPressed(false);
std::atomic<bool> isForwardButtonPressed(false);
std::atomic<bool> isBackwardButtonPressed(false);
std::atomic<bool> serial0Locked(true);

// Button subscription interval (0 = disabled)
std::atomic<int> btnSubIntervalMs(0);

// Task handles
extern TaskHandle_t mouseMoveTaskHandle;
extern TaskHandle_t ledFlashTaskHandle;
extern TaskHandle_t btnSubTaskHandle;

std::mutex commandMutex;

volatile bool deviceConnected = false;
bool usbReady = false;
bool processingUsbCommands = false;

// Ring buffer for text lines on UART1 (descriptor exchange)
RingBuf<char, 620> serial1RingBuffer;
// Ring buffer for Serial0 (PC commands)
RingBuf<char, 620> serial0RingBuffer;

int currentCommandIndex = 0;
int16_t mouseX = 0;
int16_t mouseY = 0;

const unsigned long ledFlashTime = 25;

const char *commandQueue[] = {
    "sendDeviceInfo",
    "sendDescriptorDevice",
    "sendEndpointDescriptors",
    "sendInterfaceDescriptors",
    "sendHidDescriptors",
    "sendIADescriptors",
    "sendEndpointData",
    "sendUnknownDescriptors",
    "sendDescriptorconfig"
};

// Previous button state for edge detection on real mouse
static uint8_t prevRealButtons = 0;

// ============================================================
// Utility
// ============================================================

void trimCommand(char* command) {
    int len = strlen(command);
    while (len > 0 && (command[len - 1] == ' ' || command[len - 1] == '\n' || command[len - 1] == '\r')) {
        command[len - 1] = '\0';
        len--;
    }
}

// ============================================================
// Binary Packet Handlers (from Right MCU via UART1)
// ============================================================

void handleBinaryMousePacket(const uint8_t* pkt) {
    uint8_t buttons;
    int16_t x, y;
    int8_t wheel;

    if (!pkt_parse_mouse(pkt, &buttons, &x, &y, &wheel)) {
        return; // Bad checksum
    }

    // Store real mouse button state
    realMouseButtons.store(buttons);

    // Apply button changes (edge detection)
    uint8_t prev = prevRealButtons;
    uint8_t changed = buttons ^ prev;

    if (changed & BTN_LEFT) {
        handleMouseButton(MOUSE_BUTTON_LEFT, (buttons & BTN_LEFT) != 0);
    }
    if (changed & BTN_RIGHT) {
        handleMouseButton(MOUSE_BUTTON_RIGHT, (buttons & BTN_RIGHT) != 0);
    }
    if (changed & BTN_MIDDLE) {
        handleMouseButton(MOUSE_BUTTON_MIDDLE, (buttons & BTN_MIDDLE) != 0);
    }
    if (changed & BTN_FORWARD) {
        handleMouseButton(MOUSE_BUTTON_FORWARD, (buttons & BTN_FORWARD) != 0);
    }
    if (changed & BTN_BACKWARD) {
        handleMouseButton(MOUSE_BUTTON_BACKWARD, (buttons & BTN_BACKWARD) != 0);
    }
    prevRealButtons = buttons;

    // Apply movement
    if (x != 0 || y != 0) {
        // If serial0 is injecting movement, don't override
        if (!kmMoveCom) {
            handleMove(x, y);
        }
    }

    // Apply wheel
    if (wheel != 0) {
        handleMouseWheel(wheel);
    }

    notifyLedFlashTask();
}

void handleBinaryControlPacket(const uint8_t* pkt) {
    uint8_t type;
    if (!pkt_parse_ctrl(pkt, &type)) {
        return; // Bad checksum
    }

    switch (type) {
        case PKT_USB_HELLO:
            handleUsbHello();
            break;
        case PKT_USB_GOODBYE:
            handleUsbGoodbye();
            break;
        case PKT_USB_ISNULL:
            handleNoDevice();
            break;
        case PKT_USB_ISDEBUG:
            Serial0.println("RIGHT MCU is in debug mode");
            break;
        default:
            break;
    }
}

// ============================================================
// USB Lifecycle
// ============================================================

void handleUsbHello() {
    deviceConnected = true;
    usbReady = true;
    processingUsbCommands = true;
    currentCommandIndex = 0;
    sendNextCommand();
}

void handleUsbGoodbye() {
    Serial0.println("USB Device disconnected. Restarting!");
    handleMove(0, 0);
    handleMouseButton(MOUSE_BUTTON_LEFT, false);
    handleMouseButton(MOUSE_BUTTON_RIGHT, false);
    handleMouseButton(MOUSE_BUTTON_MIDDLE, false);
    handleMouseButton(MOUSE_BUTTON_FORWARD, false);
    handleMouseButton(MOUSE_BUTTON_BACKWARD, false);
    handleMouseWheel(0);
    vTaskDelay(100);
    ESP.restart();
}

void handleNoDevice() {
    deviceConnected = false;
}

void sendNextCommand() {
    if (!processingUsbCommands || currentCommandIndex >= (int)(sizeof(commandQueue) / sizeof(commandQueue[0]))) {
        return;
    }
    const char *command = commandQueue[currentCommandIndex];
    Serial1.println(command);
    currentCommandIndex++;
    if (currentCommandIndex >= (int)(sizeof(commandQueue) / sizeof(commandQueue[0]))) {
        usbReady = false;
        processingUsbCommands = false;

        // 描述符交换完成，此时 clonedDescriptor 应已就绪
        // （HID描述符包在枚举阶段发送，早于 PKT_USB_HELLO）
        if (clonedDescriptor.ready) {
            Serial0.print("HID desc cloned, len=");
            Serial0.println(clonedDescriptor.length);
        } else {
            Serial0.println("WARN: HID desc not ready, using default");
        }

        InitUSB();
        vTaskDelay(700);
        serial0Locked = false;
        Serial1.println("USB_INIT");
    }
}

// ============================================================
// Serial1 Text Command Processing (Descriptor Exchange Only)
// ============================================================

// Descriptor exchange commands from Right MCU (text/JSON, still newline-terminated)
struct TextCommandEntry {
    const char *prefix;
    void (*handler)(const char *);
};

static TextCommandEntry serial1TextCommands[] = {
    {"USB_sendDeviceInfo:", receiveDeviceInfo},
    {"USB_sendDescriptorDevice:", receiveDescriptorDevice},
    {"USB_sendEndpointDescriptors:", receiveEndpointDescriptors},
    {"USB_sendInterfaceDescriptors:", receiveInterfaceDescriptors},
    {"USB_sendHidDescriptors:", receiveHidDescriptors},
    {"USB_sendIADescriptors:", receiveIADescriptors},
    {"USB_sendEndpointData:", receiveEndpointData},
    {"USB_sendUnknownDescriptors:", receiveUnknownDescriptors},
    {"USB_sendDescriptorconfig:", receivedescriptorConfiguration},
    {"ESPLOG_", handleEspLog},
};

void processSerial1TextCommand(const char *command) {
    for (const auto &entry : serial1TextCommands) {
        if (strncmp(command, entry.prefix, strlen(entry.prefix)) == 0) {
            entry.handler(command);
            sendNextCommand();
            return;
        }
    }
    // Unknown text from right MCU - print to debug
    Serial0.println(command);
}

// ============================================================
// Serial0 Command Processing (PC Commands - Simplified)
// ============================================================
//
// Protocol:
//   m x,y      → relative move (inject, overrides real mouse)
//   M x,y      → move to absolute position
//   L1 / L0    → left button press/release
//   R1 / R0    → right button press/release
//   K1 / K0    → middle button press/release
//   F1 / F0    → forward button press/release
//   B1 / B0    → backward button press/release
//   W n        → wheel scroll
//   P          → get position → reply: P x,y
//   G          → get all button states → reply: G l,r,m,f,b
//   S ms       → subscribe: auto-send button states every ms milliseconds
//   X          → stop subscription
//   C speed    → change serial0 baud rate (115200-5000000)
//   D 1 / D 0  → debug on/off (forwarded to right MCU)

void processSerial0Command(const char *command) {
    if (serial0Locked && command[0] != 'D') {
        return; // Only debug commands allowed before USB init
    }

    int x, y, n;

    switch (command[0]) {
        case 'm': // m x,y → relative move
            if (sscanf(command + 1, " %d,%d", &x, &y) == 2) {
                if (!kmMoveCom) {
                    kmMoveCom = true;
                    {
                        std::lock_guard<std::mutex> lock(commandMutex);
                        moveX = x;
                        moveY = y;
                    }
                    if (mouseMoveTaskHandle != NULL) {
                        xTaskNotifyGive(mouseMoveTaskHandle);
                    }
                    kmMoveCom = false;
                }
            }
            break;

        case 'M': // M x,y → move to absolute
            if (sscanf(command + 1, " %d,%d", &x, &y) == 2) {
                handleMoveto(x, y);
            }
            break;

        case 'L': // L1/L0 → left button
            if (command[1] == '1') {
                if (!isLeftButtonPressed.exchange(true))
                    handleMouseButton(MOUSE_BUTTON_LEFT, true);
            } else if (command[1] == '0') {
                if (isLeftButtonPressed.exchange(false))
                    handleMouseButton(MOUSE_BUTTON_LEFT, false);
            }
            break;

        case 'R': // R1/R0 → right button
            if (command[1] == '1') {
                if (!isRightButtonPressed.exchange(true))
                    handleMouseButton(MOUSE_BUTTON_RIGHT, true);
            } else if (command[1] == '0') {
                if (isRightButtonPressed.exchange(false))
                    handleMouseButton(MOUSE_BUTTON_RIGHT, false);
            }
            break;

        case 'K': // K1/K0 → middle button
            if (command[1] == '1') {
                if (!isMiddleButtonPressed.exchange(true))
                    handleMouseButton(MOUSE_BUTTON_MIDDLE, true);
            } else if (command[1] == '0') {
                if (isMiddleButtonPressed.exchange(false))
                    handleMouseButton(MOUSE_BUTTON_MIDDLE, false);
            }
            break;

        case 'F': // F1/F0 → forward button
            if (command[1] == '1') {
                if (!isForwardButtonPressed.exchange(true))
                    handleMouseButton(MOUSE_BUTTON_FORWARD, true);
            } else if (command[1] == '0') {
                if (isForwardButtonPressed.exchange(false))
                    handleMouseButton(MOUSE_BUTTON_FORWARD, false);
            }
            break;

        case 'B': // B1/B0 → backward button
            if (command[1] == '1') {
                if (!isBackwardButtonPressed.exchange(true))
                    handleMouseButton(MOUSE_BUTTON_BACKWARD, true);
            } else if (command[1] == '0') {
                if (isBackwardButtonPressed.exchange(false))
                    handleMouseButton(MOUSE_BUTTON_BACKWARD, false);
            }
            break;

        case 'W': // W n → wheel
            if (sscanf(command + 1, " %d", &n) == 1) {
                handleMouseWheel(n);
            }
            break;

        case 'P': // P → get position
            handleGetPos();
            break;

        case 'G': // G → get button states
            handleGetButtons();
            break;

        case 'S': // S ms → subscribe button states
            if (sscanf(command + 1, " %d", &n) == 1 && n >= 1 && n <= 10000) {
                btnSubIntervalMs.store(n);
                if (btnSubTaskHandle != NULL) {
                    xTaskNotifyGive(btnSubTaskHandle);
                }
                Serial0.print("S ");
                Serial0.println(n);
            }
            break;

        case 'X': // X → stop subscription
            btnSubIntervalMs.store(0);
            Serial0.println("X");
            break;

        case 'C': // C speed → set serial0 baud rate
            handleSerial0Speed(command);
            break;

        case 'D': // D 1/D 0 → debug
            handleDebug(command);
            break;

        default:
            Serial0.print("? ");
            Serial0.println(command);
            break;
    }
}

// ============================================================
// Serial1 RX - Mixed Binary/Text Handler
// ============================================================

void serial1RX() {
    while (Serial1.available() > 0) {
        int peek = Serial1.peek();

        if (peek == PKT_SYNC) {
            // Binary packet mode
            if (Serial1.available() < 2) return; // Need at least sync + type

            // Peek at type byte without consuming
            uint8_t header[2];
            // Read first 2 bytes
            Serial1.readBytes(header, 2);
            uint8_t pktType = header[1];

            if (pktType == PKT_MOUSE_DATA) {
                // Mouse data: 9 bytes total, already read 2, need 7 more
                int remaining = PKT_MOUSE_LEN - 2;
                unsigned long t0 = micros();
                while (Serial1.available() < remaining && (micros() - t0) < 500) {
                    // Spin wait, max 500μs at 5Mbps
                }
                if (Serial1.available() >= remaining) {
                    uint8_t pkt[PKT_MOUSE_LEN];
                    pkt[0] = header[0];
                    pkt[1] = header[1];
                    Serial1.readBytes(pkt + 2, remaining);
                    handleBinaryMousePacket(pkt);
                }

            } else if (pktType == PKT_HID_REPORT_DESC) {
                // HID descriptor: variable length packet
                // Header format: [SYNC][TYPE][LEN_LO][LEN_HI]
                // Need 2 more bytes for length
                unsigned long t0 = micros();
                while (Serial1.available() < 2 && (micros() - t0) < 2000) {
                    // Wait for 2 length bytes
                }
                if (Serial1.available() < 2) return; // Timeout

                uint8_t lenBytes[2];
                Serial1.readBytes(lenBytes, 2);
                uint16_t descLen = lenBytes[0] | (lenBytes[1] << 8);

                if (descLen == 0 || descLen > MAX_HID_REPORT_DESC_SIZE) {
                    ESP_LOGE("serial1RX", "Invalid HID desc length: %d", descLen);
                    // Send NACK
                    uint8_t nack[PKT_CTRL_LEN];
                    pkt_build_ctrl(nack, PKT_NACK);
                    Serial1.write(nack, PKT_CTRL_LEN);
                    return;
                }

                // Wait for all descriptor bytes + checksum
                uint32_t waitStart = millis();
                while (Serial1.available() < (int)(descLen + 1) && (millis() - waitStart) < 300) {
                    vTaskDelay(1);
                }
                if (Serial1.available() < (int)(descLen + 1)) {
                    ESP_LOGE("serial1RX", "HID desc timeout, got %d need %d", Serial1.available(), descLen + 1);
                    uint8_t nack[PKT_CTRL_LEN];
                    pkt_build_ctrl(nack, PKT_NACK);
                    Serial1.write(nack, PKT_CTRL_LEN);
                    return;
                }

                // Read data and checksum
                Serial1.readBytes(clonedDescriptor.data, descLen);
                uint8_t rxChecksum = Serial1.read();

                // Verify checksum (covers: TYPE + LEN_LO + LEN_HI + all DATA)
                uint8_t calcChecksum = header[1] ^ lenBytes[0] ^ lenBytes[1];
                for (int i = 0; i < descLen; i++) {
                    calcChecksum ^= clonedDescriptor.data[i];
                }

                if (calcChecksum != rxChecksum) {
                    ESP_LOGE("serial1RX", "HID desc checksum fail: calc=0x%02X got=0x%02X", calcChecksum, rxChecksum);
                    uint8_t nack[PKT_CTRL_LEN];
                    pkt_build_ctrl(nack, PKT_NACK);
                    Serial1.write(nack, PKT_CTRL_LEN);
                    clonedDescriptor.ready = false;
                    return;
                }

                clonedDescriptor.length = descLen;
                clonedDescriptor.checksum = rxChecksum;
                clonedDescriptor.ready = true;
                ESP_LOGI("serial1RX", "HID Report Descriptor received OK, len=%d", descLen);

                // Send ACK
                uint8_t ack[PKT_CTRL_LEN];
                pkt_build_ctrl(ack, PKT_ACK);
                Serial1.write(ack, PKT_CTRL_LEN);

            } else {
                // Control packet: 3 bytes total, already read 2, need 1 more
                int remaining = PKT_CTRL_LEN - 2;
                unsigned long t0 = micros();
                while (Serial1.available() < remaining && (micros() - t0) < 500) {
                    // Spin wait
                }
                if (Serial1.available() >= remaining) {
                    uint8_t pkt[PKT_CTRL_LEN];
                    pkt[0] = header[0];
                    pkt[1] = header[1];
                    Serial1.readBytes(pkt + 2, remaining);
                    handleBinaryControlPacket(pkt);
                }
            }
        } else {
            // Text mode - read into ring buffer until newline
            char ch = Serial1.read();
            if (ch == '\r') continue;

            if (!serial1RingBuffer.isFull()) {
                serial1RingBuffer.push(ch);
            } else {
                Serial0.println("S1 overflow");
            }

            if (ch == '\n') {
                char commandBuffer[620];
                int idx = 0;
                while (!serial1RingBuffer.isEmpty() && idx < (int)sizeof(commandBuffer) - 1) {
                    serial1RingBuffer.pop(commandBuffer[idx++]);
                }
                commandBuffer[idx] = '\0';
                trimCommand(commandBuffer);
                if (idx > 0) {
                    processSerial1TextCommand(commandBuffer);
                }
            }
        }
    }
}

// ============================================================
// Serial0 RX - PC Commands
// ============================================================

void serial0RX() {
    while (Serial0.available() > 0) {
        char ch = Serial0.read();
        if (ch == '\r') continue;

        if (!serial0RingBuffer.isFull()) {
            serial0RingBuffer.push(ch);
        } else {
            Serial0.println("S0 overflow");
        }

        if (ch == '\n') {
            char commandBuffer[MAX_SERIAL0_COMMAND_LENGTH];
            int idx = 0;
            while (!serial0RingBuffer.isEmpty() && idx < (int)sizeof(commandBuffer) - 1) {
                serial0RingBuffer.pop(commandBuffer[idx++]);
            }
            commandBuffer[idx] = '\0';
            trimCommand(commandBuffer);
            if (idx > 0) {
                processSerial0Command(commandBuffer);
            }
        }
    }
}

// ============================================================
// Mouse Control Functions
// ============================================================

void handleMove(int x, int y) {
    Mouse.move(x, y);
    mouseX += x;
    mouseY += y;
}

void handleMoveto(int x, int y) {
    Mouse.move(x - mouseX, y - mouseY);
    mouseX = x;
    mouseY = y;
}

void handleMouseButton(uint8_t button, bool press) {
    if (press)
        Mouse.press(button);
    else
        Mouse.release(button);
}

void handleMouseWheel(int wheelMovement) {
    Mouse.move(0, 0, wheelMovement);
}

void handleGetPos() {
    Serial0.print("P ");
    Serial0.print(mouseX);
    Serial0.print(",");
    Serial0.println(mouseY);
}

void handleGetButtons() {
    uint8_t btns = realMouseButtons.load();
    Serial0.print("G ");
    Serial0.print((btns & BTN_LEFT) ? 1 : 0);
    Serial0.print(",");
    Serial0.print((btns & BTN_RIGHT) ? 1 : 0);
    Serial0.print(",");
    Serial0.print((btns & BTN_MIDDLE) ? 1 : 0);
    Serial0.print(",");
    Serial0.print((btns & BTN_FORWARD) ? 1 : 0);
    Serial0.print(",");
    Serial0.println((btns & BTN_BACKWARD) ? 1 : 0);
}

// ============================================================
// RTOS Tasks
// ============================================================

void mouseMoveTask(void *pvParameters) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int x, y;
        {
            std::lock_guard<std::mutex> lock(commandMutex);
            x = moveX;
            y = moveY;
        }
        handleMove(x, y);
    }
}

void ledFlashTask(void *parameter) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        digitalWrite(9, HIGH);
        vTaskDelay(ledFlashTime / portTICK_PERIOD_MS);
        digitalWrite(9, LOW);
        vTaskDelay(ledFlashTime / portTICK_PERIOD_MS);
    }
}

void notifyLedFlashTask() {
    xTaskNotifyGive(ledFlashTaskHandle);
}

void btnSubscriptionTask(void *parameter) {
    while (true) {
        // Wait for subscription to be activated
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (true) {
            int interval = btnSubIntervalMs.load();
            if (interval <= 0) break; // Subscription stopped

            handleGetButtons();
            vTaskDelay(interval / portTICK_PERIOD_MS);
        }
    }
}

// ============================================================
// Debug / Config Commands
// ============================================================

void handleEspLog(const char *command) {
    const char *message = command + strlen("ESPLOG_");
    if (strlen(message) > 0) {
        Serial0.println(message);
    }
}

void handleSerial0Speed(const char *command) {
    int speed;
    // Accept both "C speed" and "C speed\n" formats
    if (sscanf(command + 1, " %d", &speed) == 1) {
        if (speed >= 115200 && speed <= 5000000) {
            Serial0.print("C ");
            Serial0.println(speed);
            Serial0.end();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            Serial0.begin(speed);
            Serial0.onReceive(serial0ISR);
            Serial0.println("OK");
        } else {
            Serial0.println("E range");
        }
    } else {
        Serial0.println("E syntax");
    }
}

void handleDebug(const char *command) {
    if (command[1] == ' ' && command[2] == '1') {
        Serial1.println("DEBUG_ON");
        Serial0.println("D 1");
    } else if (command[1] == ' ' && command[2] == '0') {
        Serial1.println("DEBUG_OFF");
        Serial0.println("D 0");
    } else {
        Serial0.println("E debug");
    }
}

// requestUSBDescriptors is in USBSetup.cpp - not duplicated here
