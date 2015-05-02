#include "AM.h"

generic configuration SyncAMSenderMicroC(am_id_t AMId) {
  provides {
    interface TimeSyncAMSend<TMicro, uint32_t>;
    interface Packet;
    interface AMPacket;
    interface TimeSyncPacket<TMicro, uint32_t>;
    interface PacketAcknowledgements as Acks;
  }
}

implementation {

#if defined(LOW_POWER_LISTENING)
  components new LplAMSenderC(AMId) as SenderC;
#else
  components new DirectAMSenderC(AMId) as SenderC;
#endif

  components new SyncAMSenderMicroP() as SyncSender;
  components SyncTimePacketP;
  components CC2420PacketC;

  TimeSyncAMSend = SyncSender.TimeSyncAMSendMicro;
  TimeSyncPacket = SyncSender.TimeSyncPacketMicro;
  Packet = SyncTimePacketP;
  AMPacket = SenderC;
  Acks = SenderC;

  SyncSender.Packet -> SyncTimePacketP;
  SyncSender.SubSend -> SenderC;
  SyncSender.PacketTimeStampMicro -> CC2420PacketC;
  SyncSender.PacketTimeSyncOffset -> CC2420PacketC;

  SyncTimePacketP.SubPacket -> SenderC;
  SyncTimePacketP.PacketTimeSyncOffset -> CC2420PacketC;
}

