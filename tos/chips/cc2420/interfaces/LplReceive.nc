interface LplReceive {
  // up-layer informs that the rx has been turned on.
  command void rxOn();
  // up-layer initialize the rx status when the tx/rx start
  command error_t rxInit();
  // up-layer informs that the rx message has been handled
  command void rxBuffSet();
  // signal up-layer the received messages, msg is the point head of the message pool, size is the number of total messages.
  event void receive(message_t* msg, uint8_t size);
}