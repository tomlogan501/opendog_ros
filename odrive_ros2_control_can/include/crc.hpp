#ifndef CRC_HPP
#define CRC_HPP

#include <stdint.h>
#include <limits.h>

namespace odrive_can {

constexpr uint8_t CANONICAL_CRC8_POLYNOMIAL = 0x37;
constexpr uint8_t CANONICAL_CRC8_INIT = 0x42;

// Default CRC-16 Polynomial: 0x9eb2 x^16 + x^13 + x^12 + x^11 + x^10 + x^8 + x^6 + x^5 + x^2 + 1
// Also known as CRC-16-DNP
constexpr uint16_t CANONICAL_CRC16_POLYNOMIAL = 0x3d65;
constexpr uint16_t CANONICAL_CRC16_INIT = 0x1337;

constexpr uint8_t CANONICAL_PREFIX = 0xAA;
constexpr uint16_t PROTOCOL_VERSION = 1;

// Calculates an arbitrary CRC for one byte.
template<typename T, unsigned POLYNOMIAL>
static T calc_crc(T remainder, uint8_t value) {
    constexpr T BIT_WIDTH = (CHAR_BIT * sizeof(T));
    constexpr T TOPBIT = ((T)1 << (BIT_WIDTH - 1));
    
    // Bring the next byte into the remainder.
    remainder ^= (value << (BIT_WIDTH - 8));

    // Perform modulo-2 division, a bit at a time.
    for (uint8_t bit = 8; bit; --bit) {
        if (remainder & TOPBIT) {
            remainder = (remainder << 1) ^ POLYNOMIAL;
        } else {
            remainder = (remainder << 1);
        }
    }

    return remainder;
}

template<typename T, unsigned POLYNOMIAL>
static T calc_crc(T remainder, const uint8_t* buffer, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        remainder = calc_crc<T, POLYNOMIAL>(remainder, buffer[i]);
    }
    return remainder;
}

// CRC8 functions
static uint8_t calc_crc8(uint8_t remainder, uint8_t value) {
    return calc_crc<uint8_t, CANONICAL_CRC8_POLYNOMIAL>(remainder, value);
}

static uint8_t calc_crc8(uint8_t remainder, const uint8_t* buffer, size_t length) {
    return calc_crc<uint8_t, CANONICAL_CRC8_POLYNOMIAL>(remainder, buffer, length);
}

// CRC16 functions
static uint16_t calc_crc16(uint16_t remainder, uint8_t value) {
    return calc_crc<uint16_t, CANONICAL_CRC16_POLYNOMIAL>(remainder, value);
}

static uint16_t calc_crc16(uint16_t remainder, const uint8_t* buffer, size_t length) {
    return calc_crc<uint16_t, CANONICAL_CRC16_POLYNOMIAL>(remainder, buffer, length);
}

} // namespace odrive_can

#endif // CRC_HPP