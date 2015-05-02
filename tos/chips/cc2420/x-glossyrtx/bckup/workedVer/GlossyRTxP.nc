/**
 * Pending the Constructive Interference transmission, which require more strick
 * timing control, as part of the rx/tx radio stack. We provide two interface:
 * a. As Glossy does, forward the received packet immediately. We could define 
 *    ENABLE_GLOSSY to enable this.
 * b. Based on high resolution synchronization, get the packet from flash and then
 *    forward it.
 *
 * In this version, we access the rx\tx FIFO directly instead of an exatra abstraction.
 **/

#include "AM.h"
#include "CC2420.h"
#include "message.h"
#include "IEEE802154.h"
#include "CC2420TimeSyncMessage.h"
#include "msp430usart.h"
#include "Msp430Timer.h"
#include "pr.h"

#define rtimer_arch_now_dco() (TBR)

module GlossyRTxP {
  provides {
    interface Init;
    interface StdControl;
    interface Receive;
    interface Msp430TimerEvent as VectorTimerB1;
  }
  uses {
    interface HplMsp430Usart as Usart;
    interface Leds;
  }
} implementation {
  /** Define the states of transmission/receiving.
      We set the separate state for Tx, Rx and GlossyRTx.
      The state machines of Tx, Rx and GlossyRTx are main-
      tained separately! We assume there is not security
      issue here. **/
  typedef enum {
    S_STOPPED,
    S_WAITING,

    /** Normal Tx States 
        In constuctive interference TX, there is no backoff and ACK! **/
    S_TX_LOAD,
    S_TX_SAMPLECCA,
    S_TX_BEGIN_TRANSMIT,
    S_TX_SFD,
    S_TX_EFD,
    S_TX_ACK_WAITING,

    /** Normal Rx States **/
    S_RX_LENGTH,
    S_RX_DOWNLOAD,

#ifdef ENABLE_GLOSSY
    /** Glossy **/
    S_GLOSSY_WAITING,
    S_GLOSSY_RECEIVING,
    S_GLOSSY_RECEIVED,
    S_GLOSSY_TRANSMITTING,
    S_GLOSSY_TRANSMITED,
#endif

    S_CANCEL
  } cc2420_glossyrtx_state_t;

//  enum {
    /**This define the maximum jiffies between TXACTIVE and the rising edge
       of SFD is captured!**/
//    CC2420_ABORT_PERIOD = 320,
//    RXFIFO_SIZE = 128,
//    SACK_HEADER_LENGTH = 7
//  };

  /**Variables of Tx**/
  norace cc2420_glossyrtx_state_t m_tx_state = S_STOPPED;
  norace message_t* ONE_NOK m_p_tx_buf;
  message_t m_tx_buf;
  /**Variables of Rx**/
  norace cc2420_glossyrtx_state_t m_rx_state = S_STOPPED;
  norace uint8_t m_rx_len;
  norace uint8_t m_rx_readbytes;
  norace message_t* ONE_NOK m_p_rx_buf;
  message_t m_rx_buf;
  /**Variables of GlossyRTx**/
#ifdef ENABLE_GLOSSY
  norace cc2420_glossyrtx_state_t m_ci_state = S_STOPPED;
  norace uint8_t packet_len;
  norace uint16_t t_rx_timeout;
  norace uint16_t print_var;
  // interrupt register for usart
  norace uint16_t ie1;
  norace uint16_t ie2;
  // interrupt register for pins
  norace uint16_t p1ie;
  norace uint16_t p2ie;
  // interrupt vector for TimerB
  norace uint16_t tbiv;
#endif
  norace bool debug_state;

  task void signal_receive();
  task void print_task();
  task void print_packet();

  static inline uint8_t* get_packet_header(message_t* m);

  inline void packet_pr(uint8_t len) {
  #ifdef ENABLE_PR
    uint8_t idx;
    uint8_t* p_header = get_packet_header( m_p_rx_buf );
    for (idx = 0; idx < len; idx++) {
      pr("%d ", p_header[idx]);
    }
    pr("\n");
  #endif
  }

  static inline bool abort_timer_fired();
  static inline void radio_abort_rx();
  static inline void radio_abort_tx();
  static inline void radio_flush_rx();
  static inline void radio_flush_tx();
  static inline void radio_write_tx();
  static inline void radio_start_tx();

  static inline void fast_read_one(uint8_t* buf);
  static inline void fast_continue_read_one(uint8_t* buf);
  static inline void fast_read_any(uint8_t* buf, uint8_t len);
  static inline void fast_write_any(uint8_t* buf, uint8_t len);

  static inline void strobe(uint8_t reg);
  static inline void writeRegister(uint8_t reg, uint16_t val);
  static inline void split_write(uint8_t val);
  static inline uint8_t split_read();
  static inline uint8_t split_read_write(uint8_t val);

  static inline void glossy_disable_other_interrupts();
  static inline void glossy_enable_other_interrupts();

  static inline void get_time_now() {
    print_var = rtimer_arch_now_dco();
  }

  static inline void post_time_diff() {
    print_var = (rtimer_arch_now_dco() - print_var) % 0xFFFF;
    post print_task();
  }

#ifdef ENABLE_GLOSSY
  static inline void glossy_sfd_interrupt( uint16_t time );
  static inline void glossy_begin_rx();
  static inline void glossy_end_rx();
#endif

  command error_t Init.init() {

    m_p_tx_buf = &m_tx_buf;
    m_p_rx_buf = &m_rx_buf;

    packet_len = DATA_LENGTH + CC2420_SIZE;
    debug_state = FALSE;
    return SUCCESS;
  }

  command error_t StdControl.start() {
    uint16_t val =  ( 2 << CC2420_TXCTRL_TXMIXBUF_CUR ) |      \
                    ( 3 << CC2420_TXCTRL_PA_CURRENT )   |      \
                    ( 1 << CC2420_TXCTRL_RESERVED )     |      \
                    ( (CC2420_DEF_RFPOWER & 0x1F) << CC2420_TXCTRL_PA_LEVEL );

    atomic {
      P4DIR |= (1 << 2);    // CSN Port42
      P4DIR &= ~(1 << 1);   // FSD Port41
      P1DIR &= ~(1 << 3);   // FIFO Port13
      call Usart.setModeSpi(&msp430_spi_default_config);
      call Usart.clrRxIntr();
      writeRegister( CC2420_TXCTRL, val);
      m_ci_state = S_WAITING;
    }
    glossy_disable_other_interrupts();
    return SUCCESS;
  }

  command error_t StdControl.stop() {
    atomic {
      m_ci_state = S_STOPPED;
    }
    glossy_enable_other_interrupts();
    return SUCCESS;
  }

  TOSH_SIGNAL(TIMERB1_VECTOR) {
#ifdef ENABLE_GLOSSY
    uint16_t T_irq = ( (rtimer_arch_now_dco() - TBCCR1) - 24 ) << 1;

    if ( m_ci_state == S_GLOSSY_RECEIVING && ( !(P4IN & (1 << 1)) ) ) {

      print_var = T_irq;
      post print_task();

      if (T_irq <= 8) {
        asm volatile("add %[d], r0" : : [d] "m" (T_irq));
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");
        asm volatile("nop");

        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");

        tbiv = TBIV;

        radio_start_tx();
        glossy_end_rx();
      } else {
        tbiv = TBIV;
        radio_abort_rx();
      }

    } else if ( m_ci_state == S_WAITING && ( (P4IN & (1 << 1)) ) ) {

      tbiv = TBIV;
    /**According to the Glossy, Rx timeout : packet duration + 200 us
       packet duration : 32 us * packet_length, 1 DCO tick ~ 1 us**/
      t_rx_timeout = TBCCR1 + ( packet_len * 35 + 200 ) * 4;

      glossy_begin_rx();

    } else if ( m_ci_state == S_GLOSSY_RECEIVED && ( (P4IN & (1 << 1)) ) ) {

      tbiv = TBIV;
      m_ci_state = S_GLOSSY_TRANSMITTING;
      post signal_receive(); 

    } else if ( m_ci_state == S_GLOSSY_TRANSMITTING && ( !(P4IN & (1 << 1)) ) ) {

      tbiv = TBIV;
      m_ci_state = S_GLOSSY_TRANSMITED;
      radio_flush_tx();

    } else if ( (TBIV >> 1) == 7 )  {
      // TimerB overflow
      tbiv = TBIV;
    } else {
      signal VectorTimerB1.fired();
    }

  #endif
  }

  static inline void glossy_begin_rx() {
    uint8_t* p_header = get_packet_header( m_p_rx_buf );
    m_ci_state = S_GLOSSY_RECEIVING;

    while (((P1IN & (1 << 3)) == 0)) {
      if (abort_timer_fired()) {
        radio_abort_rx();
        return;
      }
    }

    fast_read_one( p_header );

    if (p_header[0] != packet_len) {
      radio_abort_rx();
      return;
    }

    call Leds.led0Toggle();

    m_rx_readbytes = 1;

    while (m_rx_readbytes <= packet_len - 8) {

      while (((P1IN & (1 << 3)) == 0)) {
        if (abort_timer_fired()) {
          call Leds.led2Toggle();
          radio_abort_rx();
          return;
        }
      }

      fast_continue_read_one( p_header + m_rx_readbytes );

      m_rx_readbytes++;
    }
  }

  static inline void glossy_end_rx() {
    uint8_t* p_buf = get_packet_header(m_p_rx_buf);

    fast_read_any( p_buf+m_rx_readbytes, packet_len-m_rx_readbytes+1 );
    m_rx_readbytes = packet_len + 1;

    if (p_buf[packet_len] >> 7) {
      m_ci_state = S_GLOSSY_RECEIVED;
      radio_write_tx();
    } else {
      radio_flush_rx();
    }
  }

  static inline bool abort_timer_fired() {
    return (((signed short)(rtimer_arch_now_dco() - t_rx_timeout)) > 0 );
  }

  static inline void radio_abort_rx() {
    m_ci_state = S_CANCEL;
    radio_flush_rx();
  }

  static inline void radio_write_tx() {
    fast_write_any( get_packet_header(m_p_rx_buf), packet_len );
  }

  static inline void radio_start_tx() {
    strobe( CC2420_STXON );
  }

  static inline void radio_flush_rx() {
    strobe( CC2420_SFLUSHRX );
    strobe( CC2420_SFLUSHRX );

    m_ci_state = S_WAITING;
  }

  static inline void radio_flush_tx() {
    strobe( CC2420_SFLUSHTX );

    radio_flush_rx();
  }

  static inline void fast_read_one(uint8_t* buf) {
    uint8_t tmp;

    P4OUT |= (1 << 2);
    P4OUT &= ~(1 << 2);

    U0TXBUF = CC2420_RXFIFO | 0x40;
    while ( !(IFG1 & URXIFG0) );
    tmp = U0RXBUF;

    U0TXBUF = 0;
    while ( !(IFG1 & URXIFG0) );
    buf[0] = U0RXBUF;
  }

  static inline void fast_continue_read_one(uint8_t* buf) {
    U0TXBUF = 0;
    while ( !(IFG1 & URXIFG0) );
    buf[0] = U0RXBUF;
  }

  static inline void fast_read_any(uint8_t* buf, uint8_t len) {
    uint8_t idx;
    uint8_t tmp;

    P4OUT |= (1 << 2);
    P4OUT &= ~(1 << 2);

    U0TXBUF = CC2420_RXFIFO | 0x40;
    while ( !(IFG1 & URXIFG0) );
    tmp = U0RXBUF;

    for (idx = 0; idx < len; idx++) {
      U0TXBUF = 0;
      while ( !(IFG1 & URXIFG0) );
      buf[idx] = U0RXBUF;
    }
  }

  static inline void fast_write_any(uint8_t* buf, uint8_t len) {
    uint8_t idx;
    uint8_t tmp;

    P4OUT |= (1 << 2);
    P4OUT &= ~(1 << 2);

    U0TXBUF = CC2420_TXFIFO;

    for (idx = 0; idx < len; idx++) {
      while ( !(IFG1 & URXIFG0) );
      tmp = U0RXBUF;
      while ( !(IFG1 & UTXIFG0) );
      U0TXBUF = buf[idx];
    }

    while ( !(IFG1 & URXIFG0) );
    tmp = U0RXBUF;

    P4OUT |= (1 << 2);
  }

  static inline void strobe(uint8_t reg) {
    uint8_t tmp;

    P4OUT |= (1 << 2);
    P4OUT &= ~(1 << 2);

    U0TXBUF = reg;
    while ( !(IFG1 & URXIFG0) );
    tmp = U0RXBUF;

    P4OUT |= (1 << 2);
  }

  static inline void writeRegister(uint8_t reg, uint16_t val) {
    P4OUT |= (1 << 2);
    P4OUT &= ~(1 << 2);

    split_write( reg );

    split_read_write( val >> 8 );
    split_read_write( val & 0xff );

    split_read();

    P4OUT |= (1 << 2);
  }

  static inline void split_write(uint8_t val) {
    call Usart.tx( val );
  }

  static inline uint8_t split_read() {
    while ( !call Usart.isRxIntrPending() );
    return call Usart.rx();
  }

  static inline uint8_t split_read_write(uint8_t val) {
    uint8_t tmp;

    while ( !call Usart.isRxIntrPending() );
    tmp = call Usart.rx();

    while ( !call Usart.isTxIntrPending() );
    call Usart.tx(val);

    return tmp;
  }

  static inline void glossy_disable_other_interrupts() {
    atomic {
      ie1 = IE1;
      ie2 = IE2;
      p1ie = P1IE;
      p2ie = P2IE;
      IE1 = 0;
      // IE2 = 0;
      P1IE = 0;
      P2IE = 0;
      CACTL1 &= ~CAIE;
      DMA0CTL &= ~DMAIE;
      DMA1CTL &= ~DMAIE;
      DMA2CTL &= ~DMAIE;
      TACCTL1 &= ~CCIE;
      TBCCTL0 = 0;
      // disable FIFOP interrupt
      P1IE &= ~(1 << 0);
      // clear FIFOP interrupt
      P1IFG &= ~(1 << 0);
      // set SFD pin
      P4SEL |= (1 << 1);
      // enable both edge interrupt
      TBCCTL1 = CM_3 | CAP | SCS;
      TBCCTL1 |= CCIE;
    }
  }

  static inline void glossy_enable_other_interrupts() {
    atomic {
      IE1 = ie1;
      IE2 = ie2;
      P1IE = p1ie;
      P2IE = p2ie;
      TACCTL1 |= CCIE;
      P1IES &= ~(1 << 0);
      P1IFG &= ~(1 << 0);
      P1IE |= (1 << 0);
    }
  }

  static inline uint8_t* get_packet_header(message_t* m) {
    return (uint8_t*)((uint8_t*)m + offsetof(message_t, data) - sizeof(cc2420_header_t));
  }

  task void signal_receive() {
    memcpy(m_p_tx_buf, m_p_rx_buf, sizeof(message_t));
    signal Receive.receive( m_p_tx_buf, m_p_tx_buf->data, packet_len - CC2420_SIZE );
  }

  task void print_task() {
    pr("%d\n", print_var);
  }

  task void print_packet() {
    packet_pr(m_rx_readbytes);
  }
}
