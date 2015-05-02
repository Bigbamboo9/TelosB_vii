#include "CC2420TimeSyncMessage.h"

module SyncTimePacketP {
  provides interface Packet;
  uses interface Packet as SubPacket;
  uses interface PacketTimeSyncOffset;
} implementation {
  command void Packet.clear(message_t* msg) {
    call PacketTimeSyncOffset.cancel(msg);
    call SubPacket.clear(msg);
  }

  command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
    call SubPacket.setPayloadLength(msg, len + sizeof(timesync_footer_t));
  }

  command uint8_t Packet.payloadLength(message_t* msg) {
    return call SubPacket.payloadLength(msg) - sizeof(timesync_footer_t);
  }

  command uint8_t Packet.maxPayloadLength() {
    return call SubPacket.maxPayloadLength() - sizeof(timesync_footer_t);
  }

  command void* Packet.getPayload(message_t* msg, uint8_t len) {
    return call SubPacket.getPayload(msg, len + sizeof(timesync_footer_t));
  }
}
