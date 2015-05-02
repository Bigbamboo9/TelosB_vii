#include "msp430usart.h"
#include "CC2420.h"

module CC2420DirectSpiP {
  provides interface DirectSpi as Spi;
  uses interface FastSpiByte;
  uses interface GeneralIO as CSN;
  uses interface GeneralIO as FIFO;
  uses interface HplMsp430Usart as Usart;
  uses interface Alarm<TMicro, uint16_t> as Time;
  uses interface Leds;
} implementation {
  uint8_t m_state;

  uint8_t* m_rx_buf;
  uint8_t* m_tx_buf;

  inline uint8_t waitForRxFifo() {
    uint16_t timeout = call Time.getNow() + 4 * 16;
    while (call FIFO.get() && (timeout - call Time.getNow() < 0x7fff));
    return call FIFO.get();
  }

  async event void Time.fired() { }

  async command void Spi.setMode() {
    call CSN.makeOutput();
    call Usart.setModeSpi(&msp430_spi_default_config);
  }

  async command void Spi.clearRxIntr() {
    call Usart.clrRxIntr();
    // call CSN.set();
  }

  async command void Spi.read(uint8_t* buf, uint8_t len) {
    uint8_t idx;

    /** set CSN as low **/
    call CSN.set();    // set CSN, just in case it's not set
    call CSN.clr();    // clear CSN

    /** write the address **/
    call FastSpiByte.splitWrite( CC2420_RXFIFO | 0x40 );
    call FastSpiByte.splitRead();
    call FastSpiByte.splitWrite(0);
 
    /** read the rest bytes **/
    for (idx = 0; idx < len; idx++) {
       buf[idx] = call FastSpiByte.splitRead();
       // waitForRxFifo();
       call FastSpiByte.splitWrite(0);
    }

    // call CSN.set();

    signal Spi.readDone(buf, len, SUCCESS);
  }

  async command void Spi.continueRead(uint8_t* buf, uint8_t len) {
    uint8_t idx;

    for (idx = 0; idx < len; idx++) {
      buf[idx] = call FastSpiByte.splitRead();
      call FastSpiByte.splitWrite(0);
    }

    signal Spi.readDone(buf, len, SUCCESS);
  }

  async command void Spi.readRssiAndCrc(uint8_t* buf) {
    // read the Rssi
    *buf = call FastSpiByte.splitRead();
    call FastSpiByte.splitWrite(0);
    // read the CRC and LQI
    buf[1] = call FastSpiByte.splitRead();

    call CSN.set();

    signal Spi.rssiAndCrcReadDone(buf);
  }

  async command void Spi.write(uint8_t* buf, uint8_t len) {
    uint8_t idx;
    cc2420_status_t status;

    /** set CSN as low **/
    call CSN.set();
    call CSN.clr();

    /** write the address **/
    call FastSpiByte.splitWrite( CC2420_TXFIFO );

    /** write the rest bytes **/
    for (idx = 0; idx < len; idx++) {
      status = call FastSpiByte.splitReadWrite( buf[idx] );
      if (status & CC2420_STATUS_TX_UNDERFLOW) {
        call Leds.led2On();
      }
    }

    call FastSpiByte.splitRead();

    signal Spi.writeDone(m_tx_buf, len, SUCCESS);
  }

  async command void Spi.TXCTRL(uint16_t data) {
    call CSN.set();
    call CSN.clr();

    call FastSpiByte.splitWrite( CC2420_TXCTRL );

    call FastSpiByte.splitReadWrite( data >> 8 );
    call FastSpiByte.splitReadWrite( data & 0xff );

    call FastSpiByte.splitRead();

    call CSN.set();
  }

  async command cc2420_status_t Spi.STXON() {
    cc2420_status_t value;

    call CSN.set();
    call CSN.clr();

    call FastSpiByte.splitWrite( CC2420_STXON );
    value = call FastSpiByte.splitRead();

    call CSN.set();
    return value;
  }

  async command void Spi.SFLUSHTX() {
    call CSN.set();
    call CSN.clr();

    call FastSpiByte.splitWrite( CC2420_SFLUSHTX );
    call FastSpiByte.splitRead();

    call CSN.set();
  }

  async command void Spi.SFLUSHRX() {
    call CSN.set();
    call CSN.clr();

    call FastSpiByte.splitWrite( CC2420_SFLUSHRX );
    call FastSpiByte.splitRead();

    call CSN.set();
  }
}
