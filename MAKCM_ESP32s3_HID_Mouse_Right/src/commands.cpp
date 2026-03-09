#include "EspUsbHost.h"
#include "binary_protocol.h"
#include "esp_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <sstream>


bool enableYield = false;

void EspUsbHost::handleIncomingCommands(const String &command)
{
    ESP_LOGI("EspUsbHost", "Received command: %s", command.c_str());

    if (command == "DEBUG_ON")
    {
        debugModeActive = true;
        serial1Send("Debug mode activated.\n");
        uint8_t pkt[PKT_CTRL_LEN];
        pkt_build_ctrl(pkt, PKT_USB_ISDEBUG);
        Serial1.write(pkt, PKT_CTRL_LEN);
        ESP_LOGI("EspUsbHost", "Debug mode activated.");
    }
    else if (command == "DEBUG_OFF")
    {
        if (USB_IS_DEBUG)
        {
            return;
        }
        else
        {
            debugModeActive = false;
            ESP_LOGI("EspUsbHost", "Debug mode deactivated. System will restart.");
            uint8_t pkt[PKT_CTRL_LEN];
            pkt_build_ctrl(pkt, PKT_USB_GOODBYE);
            Serial1.write(pkt, PKT_CTRL_LEN);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
    }
    else if (command == "READY")
    {
        uint8_t pkt[PKT_CTRL_LEN];
        if (debugModeActive)
        {
            pkt_build_ctrl(pkt, PKT_USB_ISDEBUG);
            Serial1.write(pkt, PKT_CTRL_LEN);
            ESP_LOGI("EspUsbHost", "Debug mode is active. Sent PKT_USB_ISDEBUG.");
        }
        else
        {
            if (EspUsbHost::deviceConnected)
            {
                // 阶段2：在发 HELLO 之前，先（重）发一次 HID 描述符包
                // 保证左侧 clonedDescriptor 在 JSON 交换期间就绪，
                // 避免 USB transfer 异步完成导致的竞态
                if (hidReportDescriptor.isValid) {
                    transmitHIDReportDescriptor();
                    ESP_LOGI("EspUsbHost", "HID descriptor re-sent before HELLO");
                }
                pkt_build_ctrl(pkt, PKT_USB_HELLO);
                Serial1.write(pkt, PKT_CTRL_LEN);
                ESP_LOGI("EspUsbHost", "Device is connected.");
            }
            else
            {
                pkt_build_ctrl(pkt, PKT_USB_ISNULL);
                Serial1.write(pkt, PKT_CTRL_LEN);
                ESP_LOGW("EspUsbHost", "No device is connected.");
            }
        }
    }
    else if (command == "USB_INIT")
    {
        deviceMouseReady = true;
        serial1Send("USB Initialized. Mouse ready.\n");
        ESP_LOGI("EspUsbHost", "USB initialized. Mouse ready.");
    }
    else if (command == "sendDeviceInfo")
    {
        sendDeviceInfo();
        serial1Send("Device information sent.\n");
        ESP_LOGI("EspUsbHost", "Sending device information.");
    }
    else if (command == "sendDescriptorDevice")
    {
        sendDescriptorDevice();
        serial1Send("Device descriptor sent.\n");
        ESP_LOGI("EspUsbHost", "Sending device descriptor.");
    }
    else if (command == "sendEndpointDescriptors")
    {
        sendEndpointDescriptors();
        serial1Send("Endpoint descriptors sent.\n");
        ESP_LOGI("EspUsbHost", "Sending endpoint descriptors.");
    }
    else if (command == "sendInterfaceDescriptors")
    {
        sendInterfaceDescriptors();
        serial1Send("Interface descriptors sent.\n");
        ESP_LOGI("EspUsbHost", "Sending interface descriptors.");
    }
    else if (command == "sendHidDescriptors")
    {
        sendHidDescriptors();
        serial1Send("HID descriptors sent.\n");
        ESP_LOGI("EspUsbHost", "Sending HID descriptors.");
    }
    else if (command == "sendIADescriptors")
    {
        sendIADescriptors();
        serial1Send("Interface Association Descriptors sent.\n");
        ESP_LOGI("EspUsbHost", "Sending Interface Association Descriptors.");
    }
    else if (command == "sendEndpointData")
    {
        sendEndpointData();
        serial1Send("Endpoint data sent.\n");
        ESP_LOGI("EspUsbHost", "Sending endpoint data.");
    }
    else if (command == "sendUnknownDescriptors")
    {
        sendUnknownDescriptors();
        serial1Send("Unknown descriptors sent.\n");
        ESP_LOGI("EspUsbHost", "Sending unknown descriptors.");
    }
    else if (command == "sendDescriptorconfig")
    {
        sendDescriptorconfig();
        serial1Send("Configuration descriptor sent.\n");
        ESP_LOGI("EspUsbHost", "Sending configuration descriptor.");
    }
    else if (command == "YIELD")
    {
        enableYield = true;
        serial1Send("Yield enabled.\n");
        ESP_LOGI("EspUsbHost", "Yield enabled.");
    }
    else if (command == "YIELD_END")
    {
        enableYield = false;
        serial1Send("Yield disabled.\n");
        ESP_LOGI("EspUsbHost", "Yield disabled.");
    }
    else
    {
        serial1Send("Unknown command received: %s\n", command.c_str());
        ESP_LOGW("EspUsbHost", "Unknown command received: %s", command.c_str());
    }
}
