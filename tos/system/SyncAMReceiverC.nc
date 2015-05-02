#include "AM.h"

generic configuration SyncAMReceiverC(am_id_t amId) {
  provides {
    interface Receive;
    interface Packet;
    interface AMPacket;
  }
}

implementation {
  components ActiveMessageC;
  components new SyncAMReceiverP();

  Receive = SyncAMReceiverP.Receive;
  Packet = ActiveMessageC;
  AMPacket = ActiveMessageC;

  SyncAMReceiverP.SubReceive -> ActiveMessageC.Receive[amId];
}

