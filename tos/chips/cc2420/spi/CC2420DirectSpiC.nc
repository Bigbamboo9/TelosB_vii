configuration CC2420DirectSpiC {
  provides interface DirectSpi;
} implementation {
  components CC2420DirectSpiP;
  components new AlarmMicro16C();
  components HplMsp430Usart0C;
  components HplCC2420PinsC as Pins;
  components new Msp430SpiNoDmaP();
  components LedsC;

  enum {
    CLIENT_ID = unique( MSP430_SPIO_BUS ),
  };

  DirectSpi = CC2420DirectSpiP.Spi;

  CC2420DirectSpiP.FastSpiByte -> Msp430SpiNoDmaP.FastSpiByte;
  CC2420DirectSpiP.Usart -> HplMsp430Usart0C;
  CC2420DirectSpiP.CSN -> Pins.CSN;
  CC2420DirectSpiP.FIFO -> Pins.FIFO;
  CC2420DirectSpiP.Time -> AlarmMicro16C;
  CC2420DirectSpiP.Leds -> LedsC;

  Msp430SpiNoDmaP.Usart -> HplMsp430Usart0C;
  Msp430SpiNoDmaP.UsartInterrupts -> HplMsp430Usart0C;
  Msp430SpiNoDmaP.Leds -> LedsC;
}
