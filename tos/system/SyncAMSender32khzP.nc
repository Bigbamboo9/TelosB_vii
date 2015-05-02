#include "AM.h"
#include "CC2420TimeSyncMessage.h"

generic module SyncAMSender32khzP() {
  provides {
    interface TimeSyncAMSend<T32khz, uint32_t> as TimeSyncAMSend32khz;
    interface TimeSyncPacket<T32khz, uint32_t> as TimeSyncPacket32khz;
  }
  uses {
    interface Packet;
    interface AMSend as SubSend;
    interface PacketTimeStamp<T32khz, uint32_t> as PacketTimeStamp32khz;
    interface PacketTimeSyncOffset;
  }
} implementation {

  inline timesync_footer_t* getFooter(message_t* msg) {
    return (timesync_footer_t*)(msg->data + call Packet.payloadLength(msg));
  }
/*----------------- TimeSyncAMSend32khz -----------------*/
  command error_t TimeSyncAMSend32khz.send(am_addr_t addr, message_t* msg, uint8_t len, uint32_t event_time) {
    error_t err;
    timesync_footer_t* footer = (timesync_footer_t*)(msg->data + len);
    footer->type = AM_TIMESYNCMSG;
    footer->timestamp = event_time;

    err = call SubSend.send(addr, msg, len + sizeof(timesync_footer_t));
    call PacketTimeSyncOffset.set(msg);
    return err;
  }

  command error_t TimeSyncAMSend32khz.cancel(message_t* msg) {
    call PacketTimeSyncOffset.cancel(msg);
    return call SubSend.cancel(msg);
  }

  command uint8_t TimeSyncAMSend32khz.maxPayloadLength() {
    return call SubSend.maxPayloadLength() - sizeof(timesync_footer_t);
  }

  command void* TimeSyncAMSend32khz.getPayload(message_t* msg, uint8_t len) {
    return call SubSend.getPayload(msg, len + sizeof(timesync_footer_t));
  }
/*----------------- TimeSyncPacket32khz -----------------*/
  command bool TimeSyncPacket32khz.isValid(message_t* msg) {
    return call PacketTimeStamp32khz.isValid(msg) && getFooter(msg)->timestamp != CC2420_INVALID_TIMESTAMP;
  }

  command uint32_t TimeSyncPacket32khz.eventTime(message_t* msg) {
    return (uint32_t)(getFooter(msg)->timestamp) + call PacketTimeStamp32khz.timestamp(msg);
/*----------------- SubSend -----------------------------*/
  event void SubSend.sendDone(message_t* msg, error_t error) {
    signal TimeSyncAMSend32khz.sendDone(msg, error);
  }
}
