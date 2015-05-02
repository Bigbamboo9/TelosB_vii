generic module SyncAMSnooperP() {
  provides interface Receive as Snoop;

  uses interface Receive as SubSnoop;
}
implementation {
/*----------------- SubReceive -------------------*/
  event message_t* SubSnoop.receive(message_t* msg, void* payload, uint8_t len) {
    return signal Snoop.receive(msg, payload, len - sizeof(timesync_footer_t));
  }

  default event message_t* Snoop.receive(message_t* msg, void* payload, uint8_t len) { return msg; }
}

