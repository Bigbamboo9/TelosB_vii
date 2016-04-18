#include "Timer.h"

configuration ActiveMessageC {
  provides {
    interface StdControl;

    interface AMSend[am_id_t id];
    interface Receive[am_id_t id];
    interface Receive as Snoop[am_id_t id];

    interface Packet;
    interface AMPacket;
    interface LplxPacket;
    interface PacketAcknowledgements;

    interface CC2420xControl;
    interface CC2420xProfile;
  }
}
implementation {
  components CC2420ActiveMessageC as AM;

  StdControl = AM;

  AMSend       = AM;
  Receive      = AM.Receive;
  Snoop        = AM.Snoop;
  Packet       = AM;
  AMPacket     = AM;
  LplxPacket   = AM;
  PacketAcknowledgements = AM;
  CC2420xControl = AM;
  CC2420xProfile = AM;
}
