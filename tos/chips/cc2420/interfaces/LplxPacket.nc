interface LplxPacket {
  command void setPacketCI(message_t* m, uint8_t hop);
  command void setPacketDest(message_t* m, am_addr_t dest);
  command void setPacketBulk(message_t* m, uint8_t size);
  command uint8_t* getPacketPayload(message_t* m);
}
