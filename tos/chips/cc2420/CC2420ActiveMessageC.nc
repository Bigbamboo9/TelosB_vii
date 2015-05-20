#include "CC2420.h"
#include "AM.h"
#include "Ieee154.h"

#ifdef IEEE154FRAMES_ENABLED
#error "CC2420 AM layer cannot work when IEEE 802.15.4 frames only are used"
#endif

configuration CC2420ActiveMessageC {
  provides {
    interface StdControl as RadioControl;
    interface LplxPacket;
    interface AMPacket;
    interface Packet;
    interface PacketAcknowledgements;

    interface OppoRouting;
    
    interface AMSend[am_id_t id];
    interface Receive[am_id_t id];
    interface Receive as Snoop[am_id_t id];
    interface RadioBackoff[am_id_t amId];
    interface SendNotifier[am_id_t amId];
  }
}
implementation {
  components CC2420ActiveMessageP as AM;
  components ActiveMessageAddressC;
  components CC2420xLplC;
  components CC2420xPacketP;

  RadioControl = CC2420xLplC;
  OppoRouting = CC2420xLplC;
  
  RadioBackoff = AM;
  AMSend = AM;
  SendNotifier = AM;
  Receive = AM.Receive;
  Snoop = AM.Snoop;

  LplxPacket = CC2420xPacketP;
  AMPacket = CC2420xPacketP;
  Packet = CC2420xPacketP;
  PacketAcknowledgements = CC2420xPacketP;

  AM.SubSend -> CC2420xLplC;
  AM.SubReceive -> CC2420xLplC.Receive;
  AM.SubSnoop -> CC2420xLplC.Snoop;
  AM.BulkSend -> CC2420xLplC;
  AM.LplxPacket -> CC2420xPacketP;
  AM.Packet -> CC2420xPacketP;
  AM.AMPacket -> CC2420xPacketP;
  
  components LedsC;
  AM.Leds -> LedsC;
}
