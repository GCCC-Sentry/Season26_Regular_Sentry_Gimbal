#ifndef NAV_DECODER_H
#define NAV_DECODER_H

#include <stdint.h>
#include <stdbool.h>

// ROS2 -> STM32 Packet Structure
// Header: 0xAA (1 byte)
// X Speed: float (4 bytes)
// Y Speed: float (4 bytes)
// Rotation: float (4 bytes)
// Yaw Speed: float (4 bytes)
// Running State: uint8_t (1 byte)
// CRC16: uint16_t (2 bytes)
// Total: 20 bytes

#define NAV_PACKET_LEN 20
#define NAV_HEADER_BYTE 0xAA

#pragma pack(1)
typedef struct {
    uint8_t header;
    float x_speed;
    float y_speed;
    float rotation;
    float yaw_speed;
    uint8_t running_state;
} Nav_Wire_Data_t; // 18 bytes

typedef struct {
    Nav_Wire_Data_t data;
    uint16_t crc;
} Nav_Packet_t; // 20 bytes
#pragma pack()

typedef struct {
    float x_speed;
    float y_speed;
    float rotation;
    float yaw_speed;
    uint8_t running_state;
} Nav_Decoded_t;

/**
 * @brief Decodes a navigation packet from a raw buffer.
 * @param buffer Pointer to the 20-byte raw buffer received via UART.
 * @param length Length of the buffer (should be 20).
 * @param out_data Pointer to the structure to fill with decoded data.
 * @return true if packet is valid (Header & CRC match), false otherwise.
 */
bool Nav_Decode(const uint8_t* buffer, uint16_t length, Nav_Decoded_t* out_data);

#endif // NAV_DECODER_H
