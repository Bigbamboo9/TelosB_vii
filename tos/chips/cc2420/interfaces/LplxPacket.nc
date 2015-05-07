interface LplxPacket {
  command void setPacketCI(message_t* m, uint8_t hop);
  command void setOppoRouting(message_t* m, uint16_t metric, uint16_t progress);
  command void setPacketBulk(message_t* m, uint8_t size);
  command uint8_t getPacketBulk(message_t* m);
}
