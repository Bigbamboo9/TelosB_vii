configuration GlossyRTxC {
  provides {
    interface StdControl;
    interface CC2420Transmit;
    interface Receive;
  }
} implementation {
  components GlossyRTxP;
  StdControl = GlossyRTxP;
  CC2420Transmit = GlossyRTxP;
  Receive = GlossyRTxP;

  components MainC;
  MainC.SoftwareInit -> GlossyRTxP;

  components HplCC2420PinsC as Pins;
  // GlossyRTxP.CCA -> Pins.CCA;
  // GlossyRTxP.CSN -> Pins.CSN;
  GlossyRTxP.SFD -> Pins.SFD;
  GlossyRTxP.FIFO -> Pins.FIFO;
  GlossyRTxP.FIFOP -> Pins.FIFOP;

  components HplCC2420InterruptsC as Interrupts;
  GlossyRTxP.CaptureSFD -> Interrupts.CaptureSFD;
  GlossyRTxP.InterruptFIFOP -> Interrupts.InterruptFIFOP;

  // components new CC2420SpiC() as Spi;
  // GlossyRTxP.SpiResource -> Spi;
  // GlossyRTxP.STXON -> Spi.STXON;
  // GlossyRTxP.SRXON -> Spi.SRXON;
  // GlossyRTxP.SFLUSHTX -> Spi.SFLUSHTX;
  // GlossyRTxP.SFLUSHRX -> Spi.SFLUSHRX;
  // GlossyRTxP.TXCTRL -> Spi.TXCTRL;
  // GlossyRTxP.TXFIFO -> Spi.TXFIFO;
  // GlossyRTxP.RXFIFO -> Spi.RXFIFO;
  // GlossyRTxP.TXFIFO_RAM -> Spi.TXFIFO_RAM;

  components CC2420DirectSpiC;
  GlossyRTxP.DirectSpi -> CC2420DirectSpiC;

  components CC2420PacketC;
  GlossyRTxP.CC2420PacketBody -> CC2420PacketC;

  components new AlarmMicro32C();
  GlossyRTxP.TimeoutTimer -> AlarmMicro32C;

  components LedsC;
  GlossyRTxP.Leds -> LedsC;
}
