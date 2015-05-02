#include "AM.h"
#include "CC2420TimeSyncMessage.h"

generic module SyncAMSenderMicroP() {
  provides {
    interface TimeSyncAMSend<TMicro, uint32_t> as TimeSyncAMSendMicro;
    interface TimeSyncPacket<TMicro, uint32_t> as TimeSyncPacketMicro;
  }
  uses {
    interface Packet;
    interface AMSend as SubSend;
    interface PacketTimeStamp<TMicro, uint32_t> as PacketTimeStampMicro;
    interface PacketTimeSyncOffset;
  }
} implementation {

  inline timesync_footer_t* getFooter(message_t* msg) {
    return (timesync_footer_t*)(msg->data + call Packet.payloadLength(msg));
  }
/*----------------- TimeSyncAMSendMicro -----------------*/
  command error_t TimeSyncAMSendMicro.send(am_addr_t addr, message_t* msg, uint8_t len, uint32_t event_time) {
    error_t err;
    timesync_footer_t* footer = (timesync_footer_t*)(msg->data + len);
    footer->type = AM_TIMESYNCMSG;
    footer->timestamp = event_time;

    err = call SubSend.send(addr, msg, len + sizeof(timesync_footer_t));
    call PacketTimeSyncOffset.set(msg);
    return err;
  }

  command error_t TimeSyncAMSendMicro.cancel(message_t* msg) {
    call PacketTimeSyncOffset.cancel(msg);
    return call SubSend.cancel(msg);
  }

  command uint8_t TimeSyncAMSendMicro.maxPayloadLength() {
    return call SubSend.maxPayloadLength() - sizeof(timesync_footer_t);
  }

  command void* TimeSyncAMSendMicro.getPayload(message_t* msg, uint8_t len) {
    return call SubSend.getPayload(msg, len + sizeof(timesync_footer_t));
  }
/*----------------- TimeSyncPacketMicro -----------------*/
  command bool TimeSyncPacketMicro.isValid(message_t* msg) {
    return call PacketTimeStampMicro.isValid(msg) && getFooter(msg)->timestamp != CC2420_INVALID_TIMESTAMP;
  }

  command uint32_t TimeSyncPacketMicro.eventTime(message_t* msg) {
    return (uint32_t)(getFooter(msg)->timestamp) + call PacketTimeStampMicro.timestamp(msg);
  }
/*----------------- SubSend -----------------------------*/
  event void SubSend.sendDone(message_t* msg, error_t error) {
    signal TimeSyncAMSendMicro.sendDone(msg, error);
  }
}

