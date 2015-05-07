#include "CC2420.h"
#include "message.h"
#include "cc2420_x_rtx.h"

static inline uint8_t* get_packet_header(message_t* m) {
  return (uint8_t*)((uint8_t*)m + offsetof(message_t, data) - sizeof(cc2420_header_t));
}

static inline uint8_t* get_packet_setting(message_t* m) {
  return (uint8_t*)((uint8_t*)m + offsetof(messaget_t, data) + 1);
}

static inline uint8_t* get_packet_payload(message_t* m) {
  return (uint8_t*)((uint8_t*)m + offsetof(messaget_t, data) + 1 + sizeof(rtx_setting_t));
}

static inline uint8_t* get_packet_preamble_dsn(message_t* m) {
  return (get_packet_payload(m)-1);
}

static inline uint8_t* get_packet_metadata(message_t* m) {
  return (uint8_t*)((uint8_t*)m + offsetof(message_t, metadata));
}

static inline uint8_t* get_packet_payloadLen(message_t* m) {
  return *(get_packet_setting(m)-1);
}

static inline uint8_t get_packet_maxPayloadLen() {
  return (CC2420_X_PACKET_SIZE-1-sizeof(rtx_setting_t));
}

static inline uint8_t get_packet_bulk(message_t* m) {
  rtx_setting_t* p_ts = get_packet_setting(m);
  if (p_ts->batched)
    return p_ts->size;
  return 0;
}

static inline void set_packet_header(message_t* m, uint8_t dsn) {
  cc2420_header_t* p_header = (cc2420_header_t*)get_packet_header(m);
  rtx_setting_t* p_ts = (rtx_setting_t*)get_packet_setting(m);

  p_header->fcf |= ( 1 << IEEE154_FCF_INTRAPAN ) 
                 | ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE )
                 | ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) ;
  p_header->dsn = m_dsn;
  p_header->destpan = 0;
  p_header->src = TOS_NODE_ID;
  if (p_header->dest != AM_BROADCAST_ADDR) {
    p_ts->ack = TRUE;
    // cc2420 acknowledgement request
    p_header->fcf |= (1 << IEEE154_FCF_ACK_REQ);
  }
  p_header->length = CC2420_X_PACKET_SIZE + CC2420_SIZE;
  p_header->dsn = dsn;
}

static inline void set_packet_dest(message_t* m, am_addr_t dest) {
  rtx_setting_t* p_ts = get_packet_setting(m);
  cc2420_header_t* p_header = (cc2420_header_t*)get_packet_header(m);
  p_header->dest = dest;
  p_ts->addr = dest;
}

static inline void set_packet_ci(message_t* m, uint8_t hop) {
  rtx_setting_t* p_ts = get_packet_setting(m);
  p_ts->ci = TRUE;
  p_ts->hop = hop;
}

static inline void set_packet_bulk(message_t* m, uint8_t size) {
  rtx_setting_t* p_ts = get_packet_setting(m);
  p_ts->batched = TRUE;
  p_ts->size = size;
}

static inline void set_packet_opportunistic(message_t* m, uint16_t metric, uint16_t progress) {
  rtx_setting_t* p_ts = get_packet_setting(m);
  cc2420_header_t* p_header = (cc2420_header_t*)get_packet_header(m);
  p_header->dest = OPPORTUNISTIC_ROUTING_ADDR;
  p_ts->addr = OPPORTUNISTIC_ROUTING_ADDR;
  p_ts->metric = metric;
  p_ts->progress = progress;
}

static inline void set_payload_length(message_t* m, uint8_t len) {
  *(get_packet_setting(m)-1) = len;
}

static inline void set_tx_setting(message_t* m, rtx_setting_t* ts) {
  memcpy(ts, get_packet_setting(m), sizeof(rtx_setting_t));
}