configuration TestTimerAppC {
} implementation {
  components TestTimerC;
  components HilTimerMicroC;
  // components new TimerMilliC();
  components new TimerMicroC();
  components MainC;
  components LedsC;

  TestTimerC.Boot -> MainC;
  TestTimerC.MicroTime -> HilTimerMicroC;
  // TestTimerC.MilTimer -> TimerMilliC;
  TestTimerC.MicTimer -> TimerMicroC;
  TestTimerC.Leds -> LedsC;

  components PrintfC;
  components SerialStartC;
}
