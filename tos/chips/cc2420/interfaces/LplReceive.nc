interface LplReceive {
  // up-layer restart the radio from idle to rx mode
  async command void rxEnable();
  // up-layer starts the signal detection
  async command void txDetect();
  // up-layer informs that the rx has been turned on.
  async command error_t rxOn();
  // up-layer initialize the rx status when the tx/rx start
  async command error_t rxInit();
  // up-layer informs that the rx message has been handled
  async command void rxBuffSet();
  // signal up-layer the received messages, msg is the point head of the message pool, size is the number of total messages.
  async event void receive(message_t* msg, uint8_t size);
}
