/**
 * @file packet.hpp
 * @brief Includes the Packet struct.
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <cstdint>
#include <vector>

namespace atum {
/**
 * @brief Provides a simple interface to convert between a more complex datatype
 * and the individual bytes that make it up.
 *
 * @tparam T
 */
template <typename T>
union PiecedData {
  T value;
  uint8_t bytes[sizeof(T)];
};

/**
 * @brief Provides the layout for the messages sent between serial devices and
 * the brain.
 *
 * Additionally, provides simple error checking in the form of a generated
 * parity byte for the given ID and data of a packet.
 *
 */
struct Packet {
  /**
   * @brief Constructs a new Packet object.
   *
   */
  Packet() = default;

  /**
   * @brief Constructs a new Packet object.
   *
   * ID referring to the type of message.
   *
   * @param iID
   */
  Packet(const uint8_t iID);

  /**
   * @brief Constructs a new Packet object.
   *
   * ID referring to the type of message. The value for raw is whatever data
   * will be passed along with the ID in the message.
   *
   * @tparam T
   * @param iID
   * @param raw
   */
  template <typename T>
  Packet(const uint8_t iID, const T &raw) {
    PiecedData<T> pieced{raw};
    Packet(
        iID,
        std::vector<uint8_t>(std::begin(pieced.bytes), std::end(pieced.bytes)));
  }

  /**
   * @brief Returns true if the parity byte is correct given the ID
   * and data of the packet.
   *
   * @return true
   * @return false
   */
  bool correct() const;

  uint8_t id;
  uint8_t checksum;
  std::vector<uint8_t> data;
};
} // namespace atum