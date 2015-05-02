#include "msp430usart.h"

module CC2420DirectSpiP {
  provides interface DirectSpi as Spi;
  uses interface SpiByte;
  uses interface SpiPacket;
  uses interface HplMsp430Usart as Usart;
  // uses interface HplMsp430UsartInterrupts as UsartInterrupts;
  uses interface Leds;
} implementation {
  uint8_t m_pos;
  uint8_t m_len;
  uint8_t m_state;

  uint8_t* m_rx_buf;
  uint8_t* m_tx_buf;

  enum {
    SPI_STATE_RX,
    SPI_STATE_TX,
    SPI_ATOMIC_SIZE = 2,
  };

  inline void continueRx();
  inline void continueTx();

  async command void Spi.setMode() {
    call Usart.setModeSpi(&msp430_spi_default_config);
  }

  async command void Spi.read(uint8_t* buf, uint8_t len) {
    // m_pos = 0;
    // m_len = len;
    // m_state = SPI_STATE_RX;
    // m_rx_buf = buf;

    /** write the address first  **/
    
    call Usart.tx( CC2420_RXFIFO | 0x40 );
    while ( !call Usart.isRxIntrPending() );

    // call SpiByte.write( CC2420_RXFIFO | 0x40 );
    
    /** read the rest bytes **/
    /* 
    if (len) {
      call Usart.enableRxIntr();
      continueRx();
    } else {
      signal Spi.readDone(buf, len, SUCCESS);
    }
    */

    call SpiPacket.send(NULL, buf, len);

    /** signal readdone **/
    // signal Spi.readDone(buf, len, SUCCESS);
  }

  async event void SpiPacket.sendDone(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len, error_t err) {
    signal Spi.readDone(m_rx_buf, len, err);
  }

  async command void Spi.write(uint8_t* buf, uint8_t len) {
    // m_pos = 0;
    // m_len = len;
    // m_state = SPI_STATE_TX;
    // m_tx_buf = buf;

    call Usart.tx( CC2420_TXFIFO );
    while ( !call Usart.isRxIntrPending() );

    /*
    if (len) {
      call Usart.enableRxIntr();
      continueTx();
    } else {
      signal Spi.writeDone(buf, len, SUCCESS);
    }
    */

    call SpiPacket.send(buf, NULL, len);

    // signal Spi.readDone(buf, len, SUCCESS);
  }

  async command void Spi.writeReg(uint16_t data) {
    /*
    call Usart.tx( CC2420_TXCTRL );
    while ( !call Usart.isRxIntrPending() );
    call Usart.tx( data >> 8 );
    while ( !call Usart.isRxIntrPending() )
    call Usart.tx( data & 0xff );
    while ( !call Usart.isRxIntrPending() );
    */
    call SpiByte.write( CC2420_TXCTRL );
    call SpiByte.write( data >> 8);
    call SpiByte.write( data & 0xff );
  }

  async command void Spi.writeSTXON() {
    call Usart.tx( CC2420_STXON );
    while ( !call Usart.isRxIntrPending() );
  }

  async command void Spi.writeSFLUSHTX() {
    call Usart.tx( CC2420_SFLUSHTX );
    while ( !call Usart.isRxIntrPending() );
  }

  async command void Spi.writeSFLUSHRX() {
    call Usart.tx( CC2420_SFLUSHRX );
    while ( !call Usart.isRxIntrPending() );
  }
/*
  async event void UsartInterrupts.rxDone( uint8_t data ) {
    if ( m_rx_buf ) {
      m_rx_buf[m_pos] = data;
      m_pos++;
    }

    if ( m_pos < m_len ) {
      if (m_state == SPI_STATE_RX) {
        continueRx();
      } else if (m_state == SPI_STATE_TX) {
        continueTx();
      }
    } else {
      call Usart.disableRxIntr();
      if (m_state == SPI_STATE_RX) {
        signal Spi.readDone( m_rx_buf, m_len, SUCCESS );
      } else if (m_state == SPI_STATE_TX) {
        signal Spi.writeDone( m_tx_buf, m_len, SUCCESS );
      }
    }
  }

  async event void UsartInterrupts.txDone() { }
*/
  inline void continueRx() {
    uint8_t end;

    end = m_pos + SPI_ATOMIC_SIZE;

    if ( end > m_len ) {
      end = m_len;
    }

    while ( m_pos < end ) {
      if ( !call Usart.isRxIntrPending() ) { call Leds.led1Toggle(); }
      while ( !call Usart.isRxIntrPending() );
      m_rx_buf[m_pos] = call Usart.rx();
      m_pos++;
    }
    call Leds.led1Toggle();
  }

  inline void continueTx() {
    uint8_t end;

    end = m_pos + SPI_ATOMIC_SIZE;
    if ( end > m_len ) {
      end = m_len;
    }
    while ( m_pos < end ) {
      while ( !call Usart.isRxIntrPending() );
      call Usart.tx(m_tx_buf[m_pos]);
      m_pos++;
    }
  }
}
