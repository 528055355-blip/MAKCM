#ifndef BINARY_PROTOCOL_H
#define BINARY_PROTOCOL_H

#include <stdint.h>

// ============================================================
// Binary Protocol for Right MCU → Left MCU (UART1, 5Mbps)
// ============================================================
// Mouse data is sent as compact binary packets.
// Descriptor exchange remains text/JSON (one-time, not perf critical).
// ============================================================

// Sync byte - marks start of binary packet
#define PKT_SYNC 0xAA

// --- Packet types (Right → Left) ---
#define PKT_MOUSE_DATA    0x01  // Full mouse state: buttons + x + y + wheel
#define PKT_USB_HELLO     0x02  // USB device connected
#define PKT_USB_GOODBYE   0x03  // USB device disconnected
#define PKT_USB_ISNULL    0x04  // No device present
#define PKT_USB_ISDEBUG   0x05  // Debug mode active on right MCU

// --- Mouse data packet (9 bytes total) ---
// [SYNC] [TYPE] [BUTTONS] [X_LO] [X_HI] [Y_LO] [Y_HI] [WHEEL] [CHK]
//  0xAA   0x01    uint8   ---- int16 ---- ---- int16 ---  int8   XOR
#define PKT_MOUSE_LEN 9

// --- Control packet (3 bytes total) ---
// [SYNC] [TYPE] [CHK]
//  0xAA   type   XOR
#define PKT_CTRL_LEN 3

// Button bit masks (matches USB HID / TinyUSB button bits)
#define BTN_LEFT     0x01
#define BTN_RIGHT    0x02
#define BTN_MIDDLE   0x04
#define BTN_BACKWARD 0x08  // HID bit 3 = Button 4 = Backward
#define BTN_FORWARD  0x10  // HID bit 4 = Button 5 = Forward

// Calculate XOR checksum over a range of bytes
static inline uint8_t pkt_checksum(const uint8_t* buf, uint8_t from, uint8_t to) {
    uint8_t cs = 0;
    for (uint8_t i = from; i <= to; i++) cs ^= buf[i];
    return cs;
}

// Build a mouse binary packet into buf[9]. Returns 9.
static inline uint8_t pkt_build_mouse(uint8_t* buf, uint8_t buttons, int16_t x, int16_t y, int8_t wheel) {
    buf[0] = PKT_SYNC;
    buf[1] = PKT_MOUSE_DATA;
    buf[2] = buttons;
    buf[3] = (uint8_t)(x & 0xFF);
    buf[4] = (uint8_t)((x >> 8) & 0xFF);
    buf[5] = (uint8_t)(y & 0xFF);
    buf[6] = (uint8_t)((y >> 8) & 0xFF);
    buf[7] = (uint8_t)wheel;
    buf[8] = pkt_checksum(buf, 1, 7);
    return PKT_MOUSE_LEN;
}

// Build a control binary packet into buf[3]. Returns 3.
static inline uint8_t pkt_build_ctrl(uint8_t* buf, uint8_t type) {
    buf[0] = PKT_SYNC;
    buf[1] = type;
    buf[2] = type; // checksum of single byte = itself
    return PKT_CTRL_LEN;
}

// Parse mouse packet from buf[9]. Returns true if checksum valid.
static inline bool pkt_parse_mouse(const uint8_t* buf, uint8_t* buttons, int16_t* x, int16_t* y, int8_t* wheel) {
    if (buf[0] != PKT_SYNC || buf[1] != PKT_MOUSE_DATA) return false;
    if (pkt_checksum(buf, 1, 7) != buf[8]) return false;
    *buttons = buf[2];
    *x = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
    *y = (int16_t)((uint16_t)buf[5] | ((uint16_t)buf[6] << 8));
    *wheel = (int8_t)buf[7];
    return true;
}

// Parse control packet from buf[3]. Returns true if checksum valid.
static inline bool pkt_parse_ctrl(const uint8_t* buf, uint8_t* type) {
    if (buf[0] != PKT_SYNC) return false;
    if (buf[1] != buf[2]) return false; // checksum
    *type = buf[1];
    return true;
}

#endif // BINARY_PROTOCOL_H
