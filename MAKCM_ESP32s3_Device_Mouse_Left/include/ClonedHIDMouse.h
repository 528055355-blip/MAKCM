#pragma once
// ============================================================
// ClonedHIDMouse.h  —  阶段2核心：HID Report Descriptor 克隆注入
// ============================================================
//
// 技术背景：
//   Arduino ESP32 USB 库（espressif32 @6.x）中，
//   tud_hid_descriptor_report_cb() 定义在 USBHID.cpp。
//   它遍历所有向 USBHID 注册的 USBHIDDevice 对象，
//   对每个设备调用虚函数 _onGetDescriptor(uint8_t* buffer)，
//   把返回的描述符拼接成 TinyUSB 所需的完整 HID 报告描述符。
//
//   USBHIDMouse 继承自 USBHIDDevice，其 _onGetDescriptor
//   返回一个固定的标准5键鼠标描述符（约52字节）。
//
// 本类做法：
//   继承 USBHIDMouse，重写 _onGetDescriptor：
//     - clonedDescriptor.ready == true  → 使用真实鼠标的克隆描述符
//     - clonedDescriptor.ready == false → 回退父类标准描述符
//
//   这样完全不需要覆盖任何 TinyUSB 全局回调，
//   不会与 Arduino 库产生 multiple definition 冲突。
//
// 时序保证：
//   HID 描述符包在右侧 MCU 枚举阶段发送（_onReceiveControl），
//   早于 PKT_USB_HELLO 到达左侧 MCU。
//   sendNextCommand() 最后才调用 InitUSB()，此时
//   clonedDescriptor.ready 已经为 true。
// ============================================================

#include <USBHIDMouse.h>
#include "InitSettings.h"   // ClonedHIDReportDescriptor 定义 + extern clonedDescriptor

class ClonedHIDMouse : public USBHIDMouse {
public:
    ClonedHIDMouse() : USBHIDMouse() {}

    // 重写：返回克隆的 HID 报告描述符
    // buffer: TinyUSB 目标缓冲区，写入数据
    // 返回: 写入字节数
    uint16_t _onGetDescriptor(uint8_t* buffer) override {
        if (clonedDescriptor.ready && clonedDescriptor.length > 0) {
            memcpy(buffer, clonedDescriptor.data, clonedDescriptor.length);
            return clonedDescriptor.length;
        }
        // 回退到父类标准 52 字节描述符
        return USBHIDMouse::_onGetDescriptor(buffer);
    }
};
