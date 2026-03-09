#include "USBSetup.h"
#include <USB.h>
#include "tusb.h"
#include "binary_protocol.h"
#include "esp_log.h"

// ============================================================
// 外部变量（定义在 InitSettings.cpp 和 handleCommands.cpp）
// ============================================================
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
extern volatile bool deviceConnected;

// ============================================================
// 全局对象定义
// ============================================================

// ClonedHIDMouse 实例：其 _onGetDescriptor() 在 USB 枚举时
// 返回克隆的真实鼠标描述符（如已就绪），否则回退标准描述符
ClonedHIDMouse Mouse;

extern ESPUSB USB;

// 克隆的 HID Report Descriptor（由 serial1RX 填充）
ClonedHIDReportDescriptor clonedDescriptor = {
    .data     = {0},
    .length   = 0,
    .ready    = false,
    .checksum = 0
};

// ============================================================
// TinyUSB 字符串描述符回调（克隆真实鼠标字符串）
// ============================================================
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    static uint16_t desc[32];
    const char* str = nullptr;

    if (index == 0) {
        // Language ID: English 0x0409
        desc[1] = 0x0409;
        desc[0] = (TUSB_DESC_STRING << 8) | (2 + 2);
        return desc;
    }

    switch (index) {
        case 1: str = device_info.str_desc_manufacturer; break;
        case 2: str = device_info.str_desc_product;      break;
        case 3:
            // 若真实鼠标无序列号，返回 NULL 保持一致
            if (device_info.str_desc_serial_num[0] == '\0') return NULL;
            str = device_info.str_desc_serial_num;
            break;
        default:
            return NULL;
    }

    if (str == nullptr) return NULL;

    uint8_t chr_count = (uint8_t)strlen(str);
    if (chr_count > 31) chr_count = 31;  // 防止 desc[32] 溢出

    for (uint8_t i = 0; i < chr_count; i++) {
        desc[1 + i] = (uint16_t)str[i];
    }
    desc[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return desc;
}

// ============================================================
// InitUSB — 用真实鼠标参数初始化 USB Device
// ============================================================
void InitUSB() {
    // 将克隆的 Device Descriptor 参数写入 Arduino USB 对象
    USB.usbVersion(descriptor_device.bcdUSB);
    USB.firmwareVersion(descriptor_device.bcdDevice);
    USB.usbPower(configuration_descriptor.bMaxPower);
    USB.usbAttributes(configuration_descriptor.bmAttributes);
    USB.usbClass(descriptor_device.bDeviceClass);
    USB.usbSubClass(descriptor_device.bDeviceSubClass);
    USB.usbProtocol(descriptor_device.bDeviceProtocol);

    // Mouse.begin() → 向 TinyUSB 注册 HID 设备
    // 因为 Mouse 是 ClonedHIDMouse，其 _onGetDescriptor()
    // 会在 USB 枚举时返回 clonedDescriptor（如就绪）
    Mouse.begin();
    USB.begin();

    if (clonedDescriptor.ready) {
        Serial0.print("HID desc cloned OK, len=");
        Serial0.println(clonedDescriptor.length);
        ESP_LOGI("USBSetup", "USB init with CLONED HID descriptor, len=%d", clonedDescriptor.length);
    } else {
        Serial0.println("WARN: HID desc not ready, using default");
        ESP_LOGW("USBSetup", "USB init with DEFAULT HID descriptor");
    }
}

// ============================================================
// requestUSBDescriptors — 限速向右侧发 READY（每秒最多1次）
// ============================================================
void requestUSBDescriptors() {
    static uint32_t lastReadyTime = 0;
    if (deviceConnected) return;

    uint32_t now = millis();
    if (now - lastReadyTime >= 1000) {
        lastReadyTime = now;
        Serial1.println("READY");
    }
}

// ============================================================
// processSerialData — 已废弃，保留兼容性（不执行任何操作）
// HID 描述符接收逻辑已移至 handleCommands.cpp 的 serial1RX()
// ============================================================
void processSerialData() {
    // no-op: handled in serial1RX() PKT_HID_REPORT_DESC branch
}
