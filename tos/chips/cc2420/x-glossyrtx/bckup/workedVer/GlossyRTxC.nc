configuration GlossyRTxC {
  provides {
    interface StdControl;
    interface Receive;
    interface Msp430TimerEvent as VectorTimerB1;
  }
} implementation {
  components GlossyRTxP;
  StdControl = GlossyRTxP;
  Receive = GlossyRTxP;
  VectorTimerB1 = GlossyRTxP;

  components MainC;
  MainC.SoftwareInit -> GlossyRTxP;

  components HplMsp430Usart0C;
  GlossyRTxP.Usart -> HplMsp430Usart0C;

  components LedsC;
  GlossyRTxP.Leds -> LedsC;
}
