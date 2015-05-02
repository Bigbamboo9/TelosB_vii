interface LplSend {
  // up-layer deliver the messages and selected transmit options down
  command error_t send(message_t* msg, rtx_setting_t* ts);
  // signal the up-layer the messages that have been sent
  event void sendDone(message_t* msg, rtx_setting_t* ts, error_t error);
}