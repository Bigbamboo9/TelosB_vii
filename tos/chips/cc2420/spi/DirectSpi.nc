interface DirectSpi {
  // set the usart to SPI mode
  async command void setMode();

  async command void clearRxIntr();

  // read len bytes data from RX buffer
  async command void read(uint8_t* buf, uint8_t len);

  async command void continueRead(uint8_t* buf, uint8_t len);

  async command void readRssiAndCrc(uint8_t* buf);

  // reading finished
  async event void readDone(uint8_t* buf, uint8_t len, error_t err);

  async event void rssiAndCrcReadDone(uint8_t* buf);

  // write len bytes data to TX buffer
  async command void write(uint8_t* buff, uint8_t len);

  // writing finished
  async event void writeDone(uint8_t* buf, uint8_t len, error_t err);

  // set the power of transmission
  async command void TXCTRL(uint16_t data);

  // strobe the STXON strobe
  async command cc2420_status_t STXON();

  // flush TX buffer
  async command void SFLUSHTX();

  // flush RX buffer
  async command void SFLUSHRX();
}
