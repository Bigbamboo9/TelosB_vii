#include "cc2420_x_packet.h"

module CC2420xPacketP {
  provides LplxPacket;
} implementation {
  command void LplxPacket.setPacketCI(message_t* m, uint8_t hop) {
    set_packet_ci(m, hop);
  }

  command void LplxPacket.setPacketDest(message_t* m, am_addr_t dest) {
    set_packet_dest(m, dest);
  }
}
