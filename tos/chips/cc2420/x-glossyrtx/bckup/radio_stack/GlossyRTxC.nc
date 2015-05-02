configuration GlossyRTxC {
  provides {
    interface StdControl;
    interface Receive;
    interface CC2420Transmit;
    interface RadioBackoff;
    interface Msp430TimerEvent as VectorTimerB1;
  }
} implementation {
  components GlossyRTxP;
  StdControl = GlossyRTxP;
  Receive = GlossyRTxP;
  CC2420Transmit = GlossyRTxP;
  RadioBackoff = GlossyRTxP;
  VectorTimerB1 = GlossyRTxP;

  components MainC;
  MainC.SoftwareInit -> GlossyRTxP;

  components AlarmMultiplexC;
  GlossyRTxP.BackoffTimer -> AlarmMultiplexC;

  components LedsC;
  GlossyRTxP.Leds -> LedsC;
}
