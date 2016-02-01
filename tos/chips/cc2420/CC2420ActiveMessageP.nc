#include <Ieee154.h> 
#include "CC2420.h"
#include "serial_fast_print.h"

module CC2420ActiveMessageP @safe() {
  provides {
    interface AMSend[am_id_t id];
    interface Receive[am_id_t id];
    interface Receive as Snoop[am_id_t id];
    interface SendNotifier[am_id_t id];
    interface RadioBackoff[am_id_t id];
  }
  
  uses {
    interface Send as SubSend;
    interface BulkSend;
    interface Receive as SubReceive;
    interface Receive as SubSnoop;
    interface LplxPacket;
    interface AMPacket;
    interface Packet;
    interface Leds;
  }
}
implementation {
  /** AMSend Commands **/
  command error_t AMSend.send[am_id_t id](am_addr_t addr, message_t* msg, uint8_t len) {
    uint8_t size = call LplxPacket.getPacketBulk(msg);
    
    if (len > call Packet.maxPayloadLength()) {
      return ESIZE;
    }
    
    call AMPacket.setType(msg, id);
    call AMPacket.setDestination(msg, addr);
    
    if (size > 1)
      return call BulkSend.send(msg, len);
    return call SubSend.send(msg, len);
  }

  command error_t AMSend.cancel[am_id_t id](message_t* msg) {
    uint8_t size = call LplxPacket.getPacketBulk(msg);
    if (size > 1)
      return call BulkSend.cancel(msg);
    return call SubSend.cancel(msg);
  }

  command uint8_t AMSend.maxPayloadLength[am_id_t id]() {
    return call Packet.maxPayloadLength();
  }

  command void* AMSend.getPayload[am_id_t id](message_t* m, uint8_t len) {
    return call Packet.getPayload(m, len);
  }

  
  /** SubSend Events **/
  event void SubSend.sendDone(message_t* msg, error_t result) {
    signal AMSend.sendDone[call AMPacket.type(msg)](msg, result);
  }

  /** BulkSend Events **/
  event void BulkSend.sendDone(message_t* msg, error_t result) {
    signal AMSend.sendDone[call AMPacket.type(msg)](msg, result);
  }
  
  /** SubReceive Events **/
  event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
    if (call AMPacket.isForMe(msg)) {
      // uint8_t type = call AMPacket.type(msg);
      // printf_u8(1, &type);
      return signal Receive.receive[call AMPacket.type(msg)](msg, payload, len);
    } else {
      return signal Snoop.receive[call AMPacket.type(msg)](msg, payload, len);
    }
  }

  /** SubSnoop Events **/
  event message_t* SubSnoop.receive(message_t* msg, void* payload, uint8_t len) {
    return signal Snoop.receive[call AMPacket.type(msg)](msg, payload, len);
  }
  
  /** RadioBackoff **/
  async command void RadioBackoff.setInitialBackoff[am_id_t amId](uint16_t backoffTime) { }
  async command void RadioBackoff.setCongestionBackoff[am_id_t amId](uint16_t backoffTime) { }
  async command void RadioBackoff.setCca[am_id_t amId](bool useCca) { }
  
  /** Default Event Handlers **/
  default event message_t* Receive.receive[am_id_t id](message_t* msg, void* payload, uint8_t len) {
    return msg;
  }
  default event message_t* Snoop.receive[am_id_t id](message_t* msg, void* payload, uint8_t len) {
    return msg;
  }
  default event void AMSend.sendDone[uint8_t id](message_t* msg, error_t err) { }
  default event void SendNotifier.aboutToSend[am_id_t amId](am_addr_t addr, message_t *msg) { }
  default async event void RadioBackoff.requestInitialBackoff[am_id_t id](message_t *msg) { }
  default async event void RadioBackoff.requestCongestionBackoff[am_id_t id](message_t *msg) { }
  default async event void RadioBackoff.requestCca[am_id_t id](message_t *msg) { }
}
