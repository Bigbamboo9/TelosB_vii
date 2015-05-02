generic module SyncAMReceiverP() {
  provides interface Receive;

  uses interface Receive as SubReceive;
}
implementation {
/*----------------- SubReceive -------------------*/
  event message_t* SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
    return signal Receive.receive(msg, payload, len - sizeof(timesync_footer_t));
  }

  default event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) { return msg; }
}
