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
  GlossyRTxP.SFD -> Pins.SFD;
  GlossyRTxP.FIFO -> Pins.FIFO;
  GlossyRTxP.FIFOP -> Pins.FIFOP;

  components HplCC2420InterruptsC as Interrupts;
  GlossyRTxP.CaptureSFD -> Interrupts.CaptureSFD;
  GlossyRTxP.InterruptFIFOP -> Interrupts.InterruptFIFOP;

  components HplMsp430Usart0C;
  GlossyRTxP.CSN -> Pins.CSN;
  GlossyRTxP.Usart -> HplMsp430Usart0C;

  components CC2420PacketC;
  GlossyRTxP.CC2420PacketBody -> CC2420PacketC;

  components new AlarmMicro32C();
  GlossyRTxP.TimeoutTimer -> AlarmMicro32C;

  components LedsC;
  GlossyRTxP.Leds -> LedsC;
}
