configuration TestSpiAppC{
}
implementation {
  components MainC, TestSpiC, LedsC;

  TestSpiC.Boot -> MainC;
  TestSpiC.Leds -> LedsC;
}

