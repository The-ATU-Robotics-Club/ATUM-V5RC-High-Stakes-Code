#include "packet.hpp"

namespace atum {
Packet::Packet(const uint8_t iID) : id{iID}, checksum{iID} {}

template <>
Packet::Packet(const uint8_t iID, const std::vector<uint8_t> &iData) :
    id{iID},
    data{iData} {
  checksum = id;
  for(uint8_t byte : data) {
    checksum ^= byte;
  }
}

bool Packet::correct() const {
  uint8_t parityByte{id};
  parityByte ^= checksum;
  for(uint8_t byte : data) {
    parityByte ^= byte;
  }
  return !parityByte;
}
} // namespace atum