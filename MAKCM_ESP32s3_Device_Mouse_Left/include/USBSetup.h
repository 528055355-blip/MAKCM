#pragma once

#include <Arduino.h>
#include <USB.h>
#include <USBHIDMouse.h>
#include "InitSettings.h"


extern USBHIDMouse Mouse;

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


void requestUSBDescriptors();
void InitUSB();

// ============================================================
// 阶段2新增：HID Report Descriptor 接收和处理函数
// ============================================================
bool receiveHIDReportDescriptor();
void processSerialData();

// TinyUSB 回调函数声明
extern "C" uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance);
extern "C" uint16_t tud_hid_get_report_desc_cb(uint8_t instance);
