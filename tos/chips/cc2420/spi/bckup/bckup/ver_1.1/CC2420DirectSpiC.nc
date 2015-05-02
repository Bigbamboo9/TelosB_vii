configuration CC2420DirectSpiC {
  provides interface DirectSpi;
} implementation {
  components CC2420DirectSpiP;
  components HplMsp430Usart0C;
  components LedsC;

  DirectSpi = CC2420DirectSpiP.Spi;
  CC2420DirectSpiP.Usart -> HplMsp430Usart0C;
  // CC2420DirectSpiP.UsartInterrupts -> HplMsp430Usart0C;
  CC2420DirectSpiP.Leds -> LedsC;

  components new Msp430SpiNoDmaP();
  CC2420DirectSpiP.SpiByte -> Msp430SpiNoDmaP;
  CC2420DirectSpiP.SpiPacket -> Msp430SpiNoDmaP.SpiPacket[0];

  Msp430SpiNoDmaP.Usart -> HplMsp430Usart0C;
  Msp430SpiNoDmaP.UsartInterrupts -> HplMsp430Usart0C;
  Msp430SpiNoDmaP.Leds -> LedsC;
}
