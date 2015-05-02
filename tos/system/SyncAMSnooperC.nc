#include "AM.h"

generic configuration SyncAMSnooperC(am_id_t amId) {
  provides {
    interface Receive as Snoop;
    interface Packet;
    interface AMPacket;
  }
}

implementation {
  components ActiveMessageC;
  components new SyncAMSnooperP();

  Snoop = SyncAMSnooperP.Snoop;
  Packet = ActiveMessageC;
  AMPacket = ActiveMessageC;

  SyncAMSnooperP.SubSnoop -> ActiveMessageC.Snoop[amId];
}

