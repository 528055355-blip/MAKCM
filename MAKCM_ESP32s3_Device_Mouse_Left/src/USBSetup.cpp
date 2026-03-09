#include "USBSetup.h"
#include <USBHIDMouse.h>
#include <USB.h>
#include "tusb.h"
#include "binary_protocol.h"

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
extern bool usbIsDebug;

USBHIDMouse Mouse;
extern ESPUSB USB;

// ============================================================
// 阶段2新增：克隆的 HID Report Descriptor 全局变量
// ============================================================
ClonedHIDReportDescriptor clonedDescriptor = {
    .data = {0},
    .length = 0,
    .ready = false,
    .checksum = 0
};

// 默认 HID Report Descriptor（降级策略）
static const uint8_t defaultHIDReportDescriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (1)
    0x29, 0x05,        //     Usage Maximum (5)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x05,        //     Report Count (5)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data, Variable, Absolute)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x03,        //     Report Size (3)
    0x81, 0x01,        //     Input (Constant) - Padding
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data, Variable, Relative)
    0xC0,              //   End Collection
    0xC0               // End Collection
};

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    static uint16_t desc[32];
    const char* str;
    uint8_t chr_count;

    if (index == 0) {
        // Return language ID string descriptor (English) for now!
        desc[1] = 0x0409;
        desc[0] = (TUSB_DESC_STRING << 8) | (2 + 2);
        return desc;
    }

    switch (index) {
        case 1:
            str = device_info.str_desc_manufacturer;
            break;
        case 2:
            str = device_info.str_desc_product;
            break;
        case 3:
            str = device_info.str_desc_serial_num;
            break;
        default:
            return NULL; // TO DO, parse further strings
    }

    chr_count = strlen(str);
    for (uint8_t i = 0; i < chr_count; i++) {
        desc[1 + i] = str[i];
    }

    desc[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return desc;
}

void requestUSBDescriptors() {
    static uint32_t lastReadyTime = 0;
    if (deviceConnected) {
        return;
    }
    uint32_t now = millis();
    if (now - lastReadyTime >= 1000) {  // Send READY at most once per second
        lastReadyTime = now;
        Serial1.println("READY");
    }
}

void InitUSB() {
    USB.usbVersion(descriptor_device.bcdUSB);
    USB.firmwareVersion(descriptor_device.bcdDevice);
    USB.usbPower(configuration_descriptor.bMaxPower);
    USB.usbAttributes(configuration_descriptor.bmAttributes);
    USB.usbClass(descriptor_device.bDeviceClass);
    USB.usbSubClass(descriptor_device.bDeviceSubClass);
    USB.usbProtocol(descriptor_device.bDeviceProtocol);

    Mouse.begin();
    USB.begin();
}

// ============================================================
// 阶段2新增：接收 HID Report Descriptor 函数
// ============================================================
bool receiveHIDReportDescriptor()
{
    // 检查是否有足够的数据（至少需要包头）
    if (Serial1.available() < 4) {
        return false;
    }

    // 读取包头
    uint8_t header[4];
    Serial1.readBytes(header, 4);

    // 验证同步字节和类型
    if (header[0] != PKT_SYNC || header[1] != PKT_HID_REPORT_DESC) {
        ESP_LOGE("USBSetup", "Invalid packet header: sync=0x%02X, type=0x%02X", header[0], header[1]);
        
        // 发送 NACK
        uint8_t nack[PKT_CTRL_LEN];
        pkt_build_ctrl(nack, PKT_NACK);
        Serial1.write(nack, PKT_CTRL_LEN);
        
        return false;
    }

    // 解析长度
    uint16_t length = header[2] | (header[3] << 8);

    // 验证长度
    if (length <= 0 || length > MAX_HID_REPORT_DESC_SIZE) {
        ESP_LOGE("USBSetup", "Invalid descriptor length: %d", length);

        // 发送 NACK
        uint8_t nack[PKT_CTRL_LEN];
        pkt_build_ctrl(nack, PKT_NACK);
        Serial1.write(nack, PKT_CTRL_LEN);

        return false;
    }

    // 等待完整数据（数据 + 校验和）
    uint32_t startTime = millis();
    while (Serial1.available() < (int)(length + 1)) {
        if (millis() - startTime > 200) {  // 200ms 超时
            ESP_LOGE("USBSetup", "Timeout waiting for descriptor data");
            
            // 发送 NACK
            uint8_t nack[PKT_CTRL_LEN];
            pkt_build_ctrl(nack, PKT_NACK);
            Serial1.write(nack, PKT_CTRL_LEN);
            
            return false;
        }
        delay(1);
    }

    // 读取数据
    Serial1.readBytes(clonedDescriptor.data, length);
    uint8_t receivedChecksum = Serial1.read();

    // 计算并验证校验和
    uint8_t calculatedChecksum = header[1] ^ header[2] ^ header[3];
    for (int i = 0; i < length; i++) {
        calculatedChecksum ^= clonedDescriptor.data[i];
    }

    if (calculatedChecksum != receivedChecksum) {
        ESP_LOGE("USBSetup", "Checksum error: expected 0x%02X, got 0x%02X",
                 calculatedChecksum, receivedChecksum);

        // 发送 NACK
        uint8_t nack[PKT_CTRL_LEN];
        pkt_build_ctrl(nack, PKT_NACK);
        Serial1.write(nack, PKT_CTRL_LEN);

        return false;
    }

    // 存储描述符
    clonedDescriptor.length = length;
    clonedDescriptor.checksum = receivedChecksum;
    clonedDescriptor.ready = true;

    ESP_LOGI("USBSetup", "HID Report Descriptor received successfully, length: %d", length);

    // 发送 ACK
    uint8_t ack[PKT_CTRL_LEN];
    pkt_build_ctrl(ack, PKT_ACK);
    Serial1.write(ack, PKT_CTRL_LEN);

    return true;
}

// ============================================================
// 阶段2新增：处理串口数据函数
// ============================================================
void processSerialData()
{
    // 只处理 HID 描述符数据包（如果还没有接收到）
    if (!clonedDescriptor.ready && Serial1.available() >= 4) {
        receiveHIDReportDescriptor();
    }
    
    // 注意：鼠标数据包的处理在 tasks.cpp 的 serial1Task 中
}

// ============================================================
// 阶段2新增：TinyUSB 回调函数实现
// ============================================================
extern "C" uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance)
{
    if (instance != 0) {
        return nullptr;
    }

    if (clonedDescriptor.ready) {
        ESP_LOGD("USBSetup", "Using cloned HID Report Descriptor, length: %d", 
                 clonedDescriptor.length);
        return clonedDescriptor.data;
    } else {
        ESP_LOGW("USBSetup", "Cloned descriptor not ready, using default descriptor");
        return defaultHIDReportDescriptor;
    }
}

extern "C" uint16_t tud_hid_get_report_desc_cb(uint8_t instance)
{
    if (instance != 0) {
        return 0;
    }

    if (clonedDescriptor.ready) {
        return clonedDescriptor.length;
    } else {
        return sizeof(defaultHIDReportDescriptor);
    }
}
