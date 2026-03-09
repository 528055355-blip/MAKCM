#include "main.h"
#include "efuse.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#if USB_IS_DEBUG
    #warning "DEBUG MODE ENABLED: USB host will not work! For logging purposes only."
#endif

// Ensure the FIRMWARE_VERSION is defined
#ifndef FIRMWARE_VERSION
    #error "FIRMWARE_VERSION is not defined! Please set FIRMWARE_VERSION in the build flags."
#endif

const char* firmware = TOSTRING(FIRMWARE_VERSION);

void setup() {
    delay(1100);
    Serial0.begin(115200);
    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
    Serial1.begin(5000000, SERIAL_8N1, 1, 2);
    
    if (USB_IS_DEBUG) {
        Serial0.println("WARNING: Debug mode is enabled!");
        Serial0.println("USB Host will not work!");
        Serial0.println("For logging purposes only!!!\n");
    }

    Serial0.print("MAKCM version ");
    Serial0.print(firmware);  
    Serial0.println("\n");

    Serial1.onReceive(serial1ISR);
    Serial0.onReceive(serial0ISR);
    burn_usb_phy_sel_efuse();

    // 启动所有任务（包括 serial1Task 用于接收 HID 描述符）
    tasks();

    // HID 描述符将在 serial1Task 运行时由 serial1RX() 异步接收
    // TinyUSB 的 tud_hid_descriptor_report_cb 会在描述符就绪后使用克隆版本
    // 如果描述符尚未到达则使用默认描述符，等待鼠标重新连接即可自动更新
    ESP_LOGI("main", "Tasks started, HID descriptor will be received asynchronously");
}

void loop() {
    if (!USB_IS_DEBUG) {
        requestUSBDescriptors();
    }
}
