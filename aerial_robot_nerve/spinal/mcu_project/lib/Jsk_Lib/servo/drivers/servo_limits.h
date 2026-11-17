#pragma once
// Shared capacity for the direct-UART arrays and worst-case stuffed packets.
namespace spinal_servo {
constexpr unsigned MAX_SERVOS = 14;
constexpr unsigned PERSISTED_SERVOS = 10; // Preserve the existing flash layout.
constexpr unsigned PACKET_SIZE = 256;
inline bool instructionFits(unsigned length) {
  // Protocol length includes instruction and CRC (3 bytes). Conservative
  // 2x space conservatively bounds the shared encoder's byte stuffing.
  return length >= 3 && length-3 <= (PACKET_SIZE-10)/2;
}
}
