#include "cc2420_x_packet.h"

module CC2420xPacketP {
  provides interface LplxPacket;
  provides interface AMPacket;
  provides interface Packet;
  provides interface PacketAcknowledgements as Acks;
} implementation {
  /** Interface LplxPacket **/
  command void LplxPacket.setPacketCI(message_t* m, uint8_t hop) {
    set_packet_ci(m, hop);
  }

  command void LplxPacket.setOppoRouting(message_t* m, uint16_t metric, uint16_t progress) {
    set_packet_opportunistic(m, metric, progress);
  }

  command void LplxPacket.setPacketBulk(message_t* m, uint8_t size) {
    set_packet_bulk(m, size);
  }

  command uint8_t LplxPacket.getPacketBulk(message_t* m) {
    return get_packet_bulk(m);
  }

  command void LplxPacket.clearSettings(message_t* m) {
    clear_packet_settings(m);
  }

  /** Interface PacketAcknowledgements **/
  async command error_t Acks.requestAck( message_t* p_msg ) {
    ((cc2420_header_t*)get_packet_header(p_msg))->fcf |= 1 << IEEE154_FCF_ACK_REQ;
    ((rtx_setting_t*)get_packet_setting(p_msg))->ack = TRUE;
    return SUCCESS;
  }

  async command error_t Acks.noAck( message_t* p_msg ) {
    ((cc2420_header_t*)get_packet_header(p_msg))->fcf &= ~(1 << IEEE154_FCF_ACK_REQ);
    ((rtx_setting_t*)get_packet_setting(p_msg))->ack = FALSE;
    return SUCCESS;
  }

  async command bool Acks.wasAcked( message_t* p_msg ) {
    return ((cc2420_metadata_t*)get_packet_metadata(p_msg))->ack;
  }

  /** Interface AMpacket **/
  command am_addr_t AMPacket.address() {
    // return TOS_AM_ADDRESS;
    return TOS_NODE_ID;
  }

  command am_addr_t AMPacket.destination(message_t* amsg) {
    cc2420_header_t* header = (cc2420_header_t*)get_packet_header(amsg);
    return header->dest;
  }

  command am_addr_t AMPacket.source(message_t * amsg) {
    cc2420_header_t* header = (cc2420_header_t*)get_packet_header(amsg);
    return header->src;
  }

  command void AMPacket.setDestination(message_t* amsg, am_addr_t addr) {
    set_packet_dest(amsg, addr);
  }

  command void AMPacket.setSource(message_t* amsg, am_addr_t addr) {
    cc2420_header_t* header = (cc2420_header_t*)get_packet_header(amsg);
    header->src = addr;
  }

  command bool AMPacket.isForMe(message_t* amsg) {
    rtx_setting_t* p_ts = (rtx_setting_t*)get_packet_setting(amsg);
    
    return (call AMPacket.destination(amsg) == call AMPacket.address()
	     || call AMPacket.destination(amsg) == AM_BROADCAST_ADDR
	     || p_ts->addr == call AMPacket.address()
	     || p_ts->addr == AM_BROADCAST_ADDR
	     || p_ts->addr == OPPORTUNISTIC_ROUTING_ADDR);
  }

  command am_id_t AMPacket.type(message_t* amsg) {
    cc2420_header_t* header = (cc2420_header_t*)get_packet_header(amsg);
    return header->type;
  }

  command void AMPacket.setType(message_t* amsg, am_id_t type) {
    cc2420_header_t* header = (cc2420_header_t*)get_packet_header(amsg);
    header->type = type;
  }
  
  command am_group_t AMPacket.group(message_t* amsg) {
    return ((cc2420_header_t*)get_packet_header(amsg))->destpan;
  }

  command void AMPacket.setGroup(message_t* amsg, am_group_t grp) {
    ((cc2420_header_t*)get_packet_header(amsg))->destpan = grp;
  }

  command am_group_t AMPacket.localGroup() {
    return TOS_AM_GROUP;
  }
  
  /** Interface Packet **/
  command void Packet.clear(message_t* msg) {
    memset(get_packet_header(msg), 0x0, sizeof(cc2420_header_t));
    memset(get_packet_metadata(msg), 0x0, sizeof(cc2420_metadata_t));
  }
  
  command uint8_t Packet.payloadLength(message_t* msg) {
    return get_packet_payloadLen(msg);
  }
  
  command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
    set_payload_length(msg, len);
  }
  
  command uint8_t Packet.maxPayloadLength() {
    return get_packet_maxPayloadLen();
  }
 
  command void* Packet.getPayload(message_t* msg, uint8_t len) {
    if (len < get_packet_maxPayloadLen())
      return (void*)get_packet_payload(msg);
    return NULL;
  }
}
