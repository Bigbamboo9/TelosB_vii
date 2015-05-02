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
    interface CC2420Transmit as Send;
    interface RadioBackoff;
    interface Receive;
    interface Msp430TimerEvent as VectorTimerB1;
  }
  uses {
    interface Alarm<TMicro, uint32_t> as BackoffTimer;
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

    /** Normal Tx States 
        In constuctive interference TX, there is no backoff and ACK! **/
    S_TX_STARTED,
    S_TX_LOAD,
    S_TX_SAMPLE_CCA,
    S_TX_SFD,
    S_TX_ESFD,
    S_TX_EFD,
    S_TX_ACK_WAITING,

    /** Normal Rx States **/
    S_RX_STARTED,
    S_RX_LENGTH,
    S_RX_DOWNLOAD,

#ifdef ENABLE_GLOSSY
    /** Glossy **/
    S_GLOSSY_WAITING,
    S_GLOSSY_RECEIVING,
    S_GLOSSY_ACK_RECEIVING,
    S_GLOSSY_RECEIVED,
    S_GLOSSY_TRANSMITTING,
    S_GLOSSY_TRANSMITED,
#endif

    S_CANCEL
  } cc2420_glossyrtx_state_t;

  enum {
  /**This define the maximum jiffies between TXACTIVE and the rising edge
     of SFD is captured!**/
    CC2420_ABORT_PERIOD = 10240L,
    ACK_WAITING_PERIOD = 8192L,
    RXFIFO_SIZE = 128,
    SACK_HEADER_LENGTH = 7
  };

  /**Variables of Tx**/
  norace cc2420_glossyrtx_state_t m_tx_state = S_STOPPED;
  norace message_t* ONE_NOK m_p_tx_buf;
  message_t m_tx_buf;
  uint32_t m_ini_bckoff;
  uint32_t m_congest_bckoff;
  /**Variables of Rx**/
  norace cc2420_glossyrtx_state_t m_rx_state = S_STOPPED;
  norace uint8_t m_rx_len;
  norace uint8_t m_rx_readbytes;
  norace message_t* ONE_NOK m_p_rx_buf;
  message_t m_rx_buf;
  /**Variables of GlossyRTx**/
#ifdef ENABLE_GLOSSY
  norace cc2420_glossyrtx_state_t m_ci_state = S_STOPPED;
  norace bool quick_forward = FALSE;
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
  static inline void signal_send_done(error_t err);
  static inline void radio_abort_rx();
  static inline void radio_abort_tx();
  static inline void radio_flush_rx();
  static inline void radio_flush_tx();
  static inline void radio_write_tx();
  static inline void radio_start_tx();
  static inline void radio_congest_tx();

  static inline void set_mode_spi(msp430_spi_union_config_t* config);

  static inline void fast_read_one(uint8_t* buf);
  static inline void fast_continue_read_one(uint8_t* buf);
  static inline void fast_read_any(uint8_t* buf, uint8_t len);
  static inline void fast_write_any(uint8_t* buf, uint8_t len);

  static inline uint8_t strobe(uint8_t reg);
  static inline void writeRegister(uint8_t reg, uint16_t val);
  static inline void writeRam(uint8_t addr, uint8_t offset, uint8_t* data, uint8_t len);
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

  error_t send(message_t* ONE p_msg, bool useCca);
  error_t resend(bool useCca);

#ifdef ENABLE_GLOSSY
  static inline void glossy_sfd_interrupt( uint16_t time );
  static inline void glossy_begin_rx();
  static inline void glossy_end_rx();
  static inline void glossy_ack_rx();
#endif

  command error_t Init.init() {

    m_p_tx_buf = &m_tx_buf;
    m_p_rx_buf = &m_rx_buf;

    packet_len = DATA_LENGTH + CC2420_SIZE;
    debug_state = FALSE;
    quick_forward = FALSE;
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
      set_mode_spi(&msp430_spi_default_config);
      writeRegister( CC2420_TXCTRL, val);
      m_ci_state = S_GLOSSY_WAITING;
      m_rx_state = S_RX_STARTED;
      m_tx_state = S_TX_STARTED;
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

  async command error_t Send.send(message_t* ONE p_msg, bool useCca) {
    return send( p_msg, useCca );
  }

  async command error_t Send.resend(bool useCca) {
    return resend( useCca );
  }

  async command error_t Send.cancel() {
    switch (m_tx_state) {
      case S_TX_LOAD:
      case S_TX_SAMPLE_CCA:
        m_tx_state = S_CANCEL;
        radio_flush_tx();
        break;
      default:
        return FAIL;
    }
    return SUCCESS;
  }

  async command error_t Send.modify(uint8_t offset, uint8_t* buf, uint8_t len) {
    writeRam(CC2420_RAM_TXFIFO, offset, buf, len);
  }

  async command void RadioBackoff.setInitialBackoff(uint16_t backoff_time) {
    m_ini_bckoff = backoff_time;
  }

  async command void RadioBackoff.setCongestionBackoff(uint16_t backoff_time) {
    m_congest_bckoff = backoff_time;
  }

  async command void RadioBackoff.setCca(bool use_cca) {
  }

  TOSH_SIGNAL(TIMERB1_VECTOR) {
#ifdef ENABLE_GLOSSY
    uint16_t T_irq = ( (rtimer_arch_now_dco() - TBCCR1) - 24 ) << 1;

    if ( m_ci_state == S_GLOSSY_RECEIVING && ( !(P4IN & (1 << 1)) ) ) {

      print_var = T_irq;
      post print_task();

      if (T_irq <= 8 && quick_forward) {
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
        TBCCTL1 = CM_1 | CAP | SCS | CCIE;

        radio_start_tx();
        glossy_end_rx();
      } else {
        tbiv = TBIV;
        TBCCTL1 = CM_1 | CAP | SCS | CCIE;

        // radio_abort_rx();
        quick_forward = FALSE;
        glossy_end_rx();
      }

    } else if ( m_ci_state == S_GLOSSY_WAITING && ( (P4IN & (1 << 1)) ) ) {

      tbiv = TBIV;
      TBCCTL1 = CM_2 | CAP | SCS | CCIE;
    /**According to the Glossy, Rx timeout : packet duration + 200 us
       packet duration : 32 us * packet_length, 1 DCO tick ~ 1 us**/
      t_rx_timeout = TBCCR1 + ( packet_len * 35 + 200 ) * 4;
      quick_forward = TRUE;

      glossy_begin_rx();
      // TODO: synchronization time stamp

    } else if ( m_ci_state == S_GLOSSY_ACK_RECEIVING && ( !(P4IN & (1 << 1)) ) ) {

      tbiv = TBIV;
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;

      glossy_ack_rx();

    } else if ( m_ci_state == S_GLOSSY_RECEIVED && ( (P4IN & (1 << 1)) ) ) {

      tbiv = TBIV;
      TBCCTL1 = CM_2 | CAP | SCS | CCIE;

      m_ci_state = S_GLOSSY_TRANSMITTING;
      post signal_receive(); 

    } else if ( m_ci_state == S_GLOSSY_TRANSMITTING && ( !(P4IN & (1 << 1)) ) ) {

      tbiv = TBIV;
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;

      m_ci_state = S_GLOSSY_TRANSMITED;
      radio_flush_tx();

    } else if ( m_tx_state == S_TX_SFD ) {

      tbiv = TBIV;
      TBCCTL1 = CM_2 | CAP | SCS | CCIE;

      m_tx_state = S_TX_ESFD;
      // TODO : time synchronization 

    } else if ( m_tx_state == S_TX_ESFD ) {

      tbiv = TBIV;
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;

      m_tx_state = S_TX_ACK_WAITING;
      call BackoffTimer.start(ACK_WAITING_PERIOD);

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
      if (p_header[0] == SACK_HEADER_LENGTH) {
        m_ci_state = S_GLOSSY_ACK_RECEIVING; 
      } else {
        radio_abort_rx();
      }
      return;
    }

    // call Leds.led1Toggle();

    m_rx_readbytes = 1;

    while (m_rx_readbytes <= packet_len - 8) {

      while (((P1IN & (1 << 3)) == 0)) {
        if (abort_timer_fired()) {
          // call Leds.led2Toggle();
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
      if (quick_forward) {
        m_ci_state = S_GLOSSY_RECEIVED;
        radio_write_tx();
      } else {
        // TODO: add rssi and lqi info to the received packet
        m_ci_state = S_GLOSSY_WAITING;
        post signal_receive();
      }
    } else {
      if (quick_forward) {
        radio_flush_tx();
      } else {
        radio_flush_rx();
      }
    }
  }

  static inline void glossy_ack_rx() {
    uint8_t type = 0xff;
    cc2420_header_t* m_rx_header = (cc2420_header_t*)get_packet_header(m_p_rx_buf);
    cc2420_header_t* m_tx_header = (cc2420_header_t*)get_packet_header(m_p_tx_buf);

    fast_read_any( ((uint8_t*)m_rx_header)+1, SACK_HEADER_LENGTH );
    type = ( m_rx_header->fcf >> IEEE154_FCF_FRAME_TYPE ) & 7;

    if ( m_tx_state == S_TX_ACK_WAITING
      && type == IEEE154_TYPE_ACK
      && m_rx_header->dsn == m_tx_header->dsn ) {
      call BackoffTimer.stop();
      ((cc2420_metadata_t*)(m_p_tx_buf->metadata))->ack = TRUE;
      signal_send_done(SUCCESS);
    }
  }

  static inline bool abort_timer_fired() {
    return (((signed short)(rtimer_arch_now_dco() - t_rx_timeout)) > 0 );
  }

  static inline void radio_abort_rx() {
    m_ci_state = S_CANCEL;
    TBCCTL1 = CM_1 | CAP | SCS | CCIE;
    radio_flush_rx();
  }

  static inline void radio_write_tx() {
    fast_write_any( get_packet_header(m_p_rx_buf), packet_len );
  }

  static inline void radio_start_tx() {
    strobe( CC2420_STXON );
  }

  static inline void radio_congest_tx() {
    bool m_congest = TRUE;
    uint8_t status = strobe( CC2420_STXONCCA );
    if ( !(status & CC2420_STATUS_TX_ACTIVE) ) {
      status = strobe( CC2420_SNOP );
      if ( status & CC2420_STATUS_TX_ACTIVE ) {
        m_congest = FALSE;
      }
    }
    m_tx_state = m_congest ? S_TX_SAMPLE_CCA : S_TX_SFD;
    if (m_congest) {
      signal RadioBackoff.requestCongestionBackoff(m_p_tx_buf);
      call BackoffTimer.start(m_congest_bckoff);
    } else {
      call BackoffTimer.start(CC2420_ABORT_PERIOD);
    }
  }

  static inline void radio_flush_rx() {
    strobe( CC2420_SFLUSHRX );
    strobe( CC2420_SFLUSHRX );

    m_ci_state = S_GLOSSY_WAITING;
  }

  static inline void radio_flush_tx() {
    strobe( CC2420_SFLUSHTX );

    radio_flush_rx();
  }

  static inline void set_mode_spi(msp430_spi_union_config_t* config) {
    atomic {
      U0CTL = SWRST;                     // reset USART
      ME1 &= ~(UTXE0 | URXE0);           // disable USRT Module
      P3SEL &= ~(0x01 << 4);             // UTXD
      P3SEL &= ~(0x01 << 5);             // URXD
      // configure the SPI parameters
      U0CTL = (config->spiRegisters.uctl) | SYNC | SWRST;
      U0TCTL = config->spiRegisters.utctl;
      U0BR0 = (config->spiRegisters.ubr) & 0x00FF;
      U0BR1 = ((config->spiRegisters.ubr) >> 8) & 0x00FF;
      U0MCTL = 0x00;
      // select module function
      P3SEL |= (0x01 << 1);
      P3SEL |= (0x01 << 2);
      P3SEL |= (0x01 << 3);
      ME1 |= USPIE0;
      U0CTL &= ~SWRST;
      IFG1 &= ~(UTXIFG0 | URXIFG0);
      IE1 &= ~(UTXIE0 | URXIE0);
    }
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

  static inline uint8_t strobe(uint8_t reg) {
    uint8_t tmp;

    P4OUT |= (1 << 2);
    P4OUT &= ~(1 << 2);

    U0TXBUF = reg;
    while ( !(IFG1 & URXIFG0) );
    tmp = U0RXBUF;

    P4OUT |= (1 << 2);

    return tmp;
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

  static inline void writeRam(uint8_t addr, uint8_t offset, uint8_t* data, uint8_t len) {
    uint8_t i = 0;

    P4OUT |= (1 << 2);
    P4OUT &= ~(1 << 2);

    addr += offset;

    split_write( addr | 0x80 );
    split_read_write( (addr >> 1) & 0xc0 );

    for ( ; i<len; i++) {
      split_read_write( data[i] );
    }

    split_read();

    P4OUT |= (1 << 2);
  }

  static inline void split_write(uint8_t val) {
    U0TXBUF = val;
  }

  static inline uint8_t split_read() {
    while ( !(IFG1 & URXIFG0) );
    return U0RXBUF;
  }

  static inline uint8_t split_read_write(uint8_t val) {
    uint8_t tmp;

    while ( !(IFG1 & URXIFG0) );
    tmp = U0RXBUF;

    while ( !(IFG1 & UTXIFG0) );
    U0TXBUF = val;

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
      DMA0CTL &= ~DMAIE;
      DMA1CTL &= ~DMAIE;
      DMA2CTL &= ~DMAIE;
      // disable FIFOP interrupt
      P1IE &= ~(1 << 0);
      // clear FIFOP interrupt
      P1IFG &= ~(1 << 0);
      // set SFD pin
      P4SEL |= (1 << 1);
      // TACCTL1 is used to low precision timer configuration.
      CACTL1 &= ~CAIE;
      TACCTL0 &= ~CCIE;
      TBCCTL0 &= ~CCIE;
      // enable rising edge interrupt
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;
      TBCCTL1 &= ~CCIFG;
    }
  }

  static inline void glossy_enable_other_interrupts() {
    atomic {
      IE1 = ie1;
      IE2 = ie2;
      P1IE = p1ie;
      P2IE = p2ie;
      P1IES &= ~(1 << 0);
      P1IFG &= ~(1 << 0);
      P1IE |= (1 << 0);
    }
  }

  static inline uint8_t* get_packet_header(message_t* m) {
    return (uint8_t*)((uint8_t*)m + offsetof(message_t, data) - sizeof(cc2420_header_t));
  }

  error_t send(message_t* ONE p_msg, bool useCca) {
    atomic {
      if (m_tx_state == S_CANCEL) {
        return ECANCEL;
      }
      if ( (m_ci_state != S_GLOSSY_WAITING) && (m_ci_state != S_STOPPED) ) {
        return EBUSY;
      }

      m_tx_state = S_TX_LOAD;
      m_p_tx_buf = p_msg;
    }
 
    fast_write_any( get_packet_header(m_p_tx_buf), packet_len );

    if (useCca) {
      atomic {
        m_tx_state = S_TX_SAMPLE_CCA;
      }
      signal RadioBackoff.requestInitialBackoff(p_msg);
      call BackoffTimer.start( m_ini_bckoff ); 
    } else {
      radio_start_tx();
      m_tx_state = S_TX_SFD;
      call BackoffTimer.start( CC2420_ABORT_PERIOD );
    }

    return SUCCESS;
  }

  error_t resend(bool useCca) { 
    if (m_tx_state == S_CANCEL) {
      return ECANCEL;
    }

    if (m_rx_state != S_RX_STARTED || ((m_ci_state != S_GLOSSY_WAITING) && (m_ci_state != S_STOPPED)) ) {
      return EBUSY;
    }
 
    if (useCca) {
      atomic {
        m_tx_state = S_TX_SAMPLE_CCA;
      }
      signal RadioBackoff.requestInitialBackoff(m_p_tx_buf);
      call BackoffTimer.start( m_ini_bckoff ); 
    } else {
      radio_start_tx();
      m_tx_state = S_TX_SFD;
      call BackoffTimer.start( CC2420_ABORT_PERIOD );
    }

    return SUCCESS;
  }

  void signal_send_done(error_t err) {
    atomic m_tx_state = S_TX_STARTED;
    signal Send.sendDone( m_p_tx_buf, err );
  }

  async event void BackoffTimer.fired() {
    atomic {
      switch (m_tx_state) {
        case S_TX_SAMPLE_CCA:
          radio_congest_tx(); 
          break;
        case S_TX_ACK_WAITING:
          signal_send_done( SUCCESS ); 
          break;
        case S_TX_SFD:
          strobe( CC2420_SFLUSHTX);
          TBCCTL1 = CM_1 | CAP | SCS | CCIE;
          signal_send_done( ERETRY );
          break;
        default:
          break;
      }
    }
  }

  task void signal_receive() {
    signal Receive.receive( m_p_rx_buf, m_p_rx_buf->data, packet_len - CC2420_SIZE );
  }

  task void print_task() {
    pr("%d\n", print_var);
  }

  task void print_packet() {
    packet_pr(m_rx_readbytes);
  }
}
