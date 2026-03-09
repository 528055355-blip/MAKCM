#pragma once

#include <Arduino.h>
#include <USB.h>
#include "ClonedHIDMouse.h"   // 包含 USBHIDMouse + ClonedHIDMouse 定义
#include "InitSettings.h"

// ============================================================
// 全局鼠标对象（ClonedHIDMouse，支持描述符克隆）
// ============================================================
extern ClonedHIDMouse Mouse;

// 所有描述符相关外部变量（定义在 InitSettings.cpp）
extern DeviceInfo device_info;
extern DescriptorDevice descriptor_device;
extern usb_endpoint_descriptor_t endpoint_descriptors[MAX_ENDPOINT_DESCRIPTORS];
extern uint8_t endpointCounter;
extern usb_interface_descriptor_t interface_descriptors[MAX_INTERFACE_DESCRIPTORS];
extern uint8_t interfaceCounter;
extern usb_hid_descriptor_t hid_descriptors[MAX_HID_DESCRIPTORS];
extern uint8_t hidDescriptorCounter;
extern usb_iad_desc_t descriptor_interface_association;
extern endpoint_data_t endpoint_data_list[17];
extern usb_unknown_descriptor_t unknown_descriptors[MAX_UNKNOWN_DESCRIPTORS];
extern uint8_t unknownDescriptorCounter;
extern DescriptorConfiguration configuration_descriptor;

// ============================================================
// 函数声明
// ============================================================
void requestUSBDescriptors();
void InitUSB();
// processSerialData 已废弃（逻辑移至 serial1RX），保留声明兼容
void processSerialData();
