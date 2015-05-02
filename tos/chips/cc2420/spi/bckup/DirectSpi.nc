interface DirectSpi {
  async command void setMode();
  async command void read(uint8_t* buf, uint8_t len);
  async event void readDone(uint8_t* buf, uint8_t len, error_t err);
  async command void write(uint8_t* buff, uint8_t len);
  async event void writeDone(uint8_t* buf, uint8_t len, error_t err);
  async command void writeReg(uint16_t data);
  async command void writeSTXON();
  async command void writeSFLUSHTX();
  async command void writeSFLUSHRX();
}
