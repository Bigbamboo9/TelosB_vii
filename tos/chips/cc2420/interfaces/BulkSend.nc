interface BulkSend {
  command error_t send(message_t* msg, uint8_t size, uint8_t len);
  command error_t cancel(message_t* msg);
  event void sendDone(message_t* msg, error_t err);
}
