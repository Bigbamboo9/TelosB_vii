configuration TestGlossyAppC {
  
} implementation {
  components MainC;
  components TestGlossyC;
  components CC2420ControlC;
  components GlossyRTxC;
  components LedsC;

  components new TimerMicroC();
  components RandomC;

  TestGlossyC.Boot -> MainC;
  TestGlossyC.Resource -> CC2420ControlC;
  TestGlossyC.CC2420Power -> CC2420ControlC;
  TestGlossyC.Leds -> LedsC;
  TestGlossyC.Receive -> GlossyRTxC;
  TestGlossyC.SubControl -> GlossyRTxC;
  TestGlossyC.Send -> GlossyRTxC;
  TestGlossyC.RadioBackoff -> GlossyRTxC;
  TestGlossyC.Random -> RandomC;
  TestGlossyC.Timer -> TimerMicroC;

  components PrintfC;
  components SerialStartC;
}
