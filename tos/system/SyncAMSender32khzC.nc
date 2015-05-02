#include "AM.h"

generic configuration SyncAMSender32khzC(am_id_t AMId) {
  provides {
    interface TimeSyncAMSend<T32khz, uint32_t>;
    interface Packet;
    interface AMPacket;
    interface TimeSyncPacket<T32khz, uint32_t>;
    interface PacketAcknowledgements as Acks;
  }
}

implementation {

#if defined(LOW_POWER_LISTENING)
  components new LplAMSenderC(AMId) as SenderC;
#else
  components new DirectAMSenderC(AMId) as SenderC;
#endif

  components new SyncAMSender32khzP() as SyncSender;
  components SyncTimePacketP;
  components CC2420PacketC;

  TimeSyncAMSend = SyncSender.TimeSyncAMSend32khz;
  TimeSyncPacket = SyncSender.TimeSyncPacket32khz;
  Packet = SyncTimePacketP;
  AMPacket = SenderC;
  Acks = SenderC;

  SyncSender.Packet -> SyncTimePacketP;
  SyncSender.SubSend -> SenderC;
  SyncSender.PacketTimeStamp32khz -> CC2420PacketC;
  SyncSender.PacketTimeOffset -> CC2420PacketC;

  SyncTimePacketP.SubPacket -> SenderC;
  SyncTimePacketP.PacketTimeSyncOffset -> CC2420PacketC;
}

