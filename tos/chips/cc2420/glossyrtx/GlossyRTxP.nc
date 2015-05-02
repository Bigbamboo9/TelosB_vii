/**
 * Pending the Constructive Interference transmission, which require more strick
 * timing control, as part of the rx/tx radio stack. We provide two interface:
 * a. As Glossy does, forward the received packet immediately. We could define 
 *    ENABLE_GLOSSY to enable this.
 * b. Based on high resolution synchronization, get the packet from flash and then
 *    forward it.
 **/

#include "AM.h"
#include "CC2420.h"
#include "message.h"
#include "IEEE802154.h"
#include "CC2420TimeSyncMessage.h"
#include "pr.h"

module GlossyRTxP {
  provides {
    interface Init;
    interface StdControl;

    interface CC2420Transmit as Send;
    // interface GlossySend;

    interface Receive;
  }
  uses {
    interface Alarm<TMicro, uint32_t> as TimeoutTimer;

    interface CC2420PacketBody;

    interface GeneralIO as SFD;
    interface GeneralIO as FIFO;
    interface GeneralIO as FIFOP;

    interface GpioCapture as CaptureSFD;
    interface GpioInterrupt as InterruptFIFOP;

    interface DirectSpi;

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
    S_GLOSSY_LENGTH,
    S_GLOSSY_DOWNLOAD,
    S_GLOSSY_DOWNLOAD_REST,
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
  norace uint32_t t_rx_timeout;
  norace uint8_t t_min_jitter;
  norace uint32_t print_var;
#endif
  norace uint8_t debug_counter;

  task void signal_receive();
  task void print_task();
  task void print_packet();

  inline void packet_pr(uint8_t len) {
    uint8_t idx;
    uint8_t* p_header = (uint8_t*)call CC2420PacketBody.getHeader( m_p_rx_buf );
    for (idx = 0; idx < len; idx++) {
      pr("%d ", p_header[idx]);
    }
    pr("\n");
  }

  static inline bool abort_timer_fired();
  static inline void radio_abort_rx();
  static inline void radio_abort_tx();
  static inline void radio_flush_rx();
  static inline void radio_flush_tx();
  static inline void radio_write_tx();
  static inline void radio_start_tx();

  static inline void get_time_now() {
    print_var = call TimeoutTimer.getNow();
  }

  static inline void post_time_diff() {
    print_var = call TimeoutTimer.getNow() - print_var;
    post print_task();
  }

#ifdef ENABLE_GLOSSY
  static inline void glossy_sfd_interrupt( uint16_t time );
  static inline void glossy_begin_rx();
  static inline void glossy_end_rx();
  static inline void glossy_start_load();
#endif

  command error_t Init.init() {
    call SFD.makeInput();

    m_p_tx_buf = &m_tx_buf;
    m_p_rx_buf = &m_rx_buf;

    packet_len = DATA_LENGTH + CC2420_SIZE;
    t_min_jitter = 0xFF;
    return SUCCESS;
  }

  command error_t StdControl.start() {
    atomic {
      call CaptureSFD.captureRisingEdge();
      /** Use SFD interrupt to start download data **/
      call InterruptFIFOP.disable();
      call DirectSpi.setMode();
      m_ci_state = S_WAITING;
    }
    return SUCCESS;
  }

  command error_t StdControl.stop() {
    atomic {
      call CaptureSFD.disable();
      call InterruptFIFOP.disable();

      m_ci_state = S_STOPPED;
    }
    return SUCCESS;
  }

  async command error_t Send.send( message_t* ONE msg, bool cca ) {
    return SUCCESS;
  }

  async command error_t Send.resend( bool cca ) {
    return SUCCESS;
  }

  async command error_t Send.cancel() {
    return SUCCESS;
  }

  async command error_t Send.modify( uint8_t offset, uint8_t* buf, uint8_t len ) {
    return SUCCESS;
  }

  async event void InterruptFIFOP.fired() {
  }

  async event void CaptureSFD.captured( uint16_t time ) {
  #ifdef ENABLE_GLOSSY
    glossy_sfd_interrupt( time );
  #endif
  }

  async event void DirectSpi.readDone( uint8_t* rx_buf, uint8_t rx_len, error_t err) {
  #ifdef ENABLE_GLOSSY
    uint8_t* p_header = (uint8_t*)call CC2420PacketBody.getHeader( m_p_rx_buf );

    if (m_ci_state == S_GLOSSY_LENGTH) {
      m_rx_len = p_header[0];
      if (m_rx_len != packet_len) {
        radio_abort_rx();
        return;
      } else {
        call Leds.led0Toggle();

        m_ci_state = S_GLOSSY_DOWNLOAD;
        m_rx_readbytes = 1;
 
        while (!call FIFO.get()) {
          if (abort_timer_fired()) {
            radio_abort_rx();
            return;
          }
        }

        call DirectSpi.continueRead( p_header + m_rx_readbytes, 1 );
      }
    } else if (m_ci_state == S_GLOSSY_DOWNLOAD) {
      m_rx_readbytes = m_rx_readbytes + 1;

      if (m_rx_readbytes <= packet_len - 40) {

        while (!call FIFO.get()) {
          if (abort_timer_fired()) {
            radio_abort_rx();
            return;
          }
        } 
 
        call DirectSpi.continueRead( p_header + m_rx_readbytes, 1 );
      } else {
        if (abort_timer_fired()) {
          call Leds.led2Toggle();
        }
        post_time_diff();
      }
    } else if (m_ci_state == S_GLOSSY_DOWNLOAD_REST) {
      m_rx_readbytes = packet_len - 1;
      call DirectSpi.readRssiAndCrc( p_header + m_rx_readbytes );
    }
  #endif  
  }

  async event void DirectSpi.rssiAndCrcReadDone(uint8_t* buf) {
    m_ci_state = S_GLOSSY_RECEIVED;
    m_rx_readbytes = packet_len + 1;

    // call DirectSpi.clearRxIntr();
    // radio_start_tx();

    call DirectSpi.clearRxIntr();
    glossy_start_load();
  }

  async event void DirectSpi.writeDone(uint8_t* buf, uint8_t len, error_t err) {
    call DirectSpi.clearRxIntr();
    radio_start_tx();
  }

  async event void TimeoutTimer.fired() {
  }

#ifdef ENABLE_GLOSSY
  static inline void glossy_sfd_interrupt( uint16_t time ) {
    if ( m_ci_state == S_WAITING && call SFD.get() ) {
    /**According to the Glossy, Rx timeout : packet duration + 200 us
       packet duration : 32 us * packet_length, 1 DCO tick ~ 1 us**/
      t_rx_timeout = call TimeoutTimer.getNow() + packet_len * 35 + 200;

      call CaptureSFD.captureFallingEdge();

      call DirectSpi.clearRxIntr();

      glossy_begin_rx();

    } else if ( m_ci_state == S_GLOSSY_DOWNLOAD && !call SFD.get() ) {

      uint8_t T_irq = call TimeoutTimer.getNow() - time;

      t_min_jitter = (T_irq > t_min_jitter) ? t_min_jitter : T_irq;
      T_irq = T_irq - t_min_jitter;

      // print_var = T_irq;
      // post print_task();

      if (T_irq <= 8) {
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        asm volatile ("nop");
        switch (T_irq) {
          case 0:
          case 1:
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            break;
          case 2:
          case 3:
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            break;
          case 4:
          case 5:
            asm volatile("nop");
            asm volatile("nop");
            break;
          case 6:
          case 7:
            asm volatile("nop");
            break;
          default:
        }

        call CaptureSFD.captureRisingEdge();

        call DirectSpi.clearRxIntr();
        // radio_start_tx();
        glossy_end_rx();
      } else {
        call Leds.led1Toggle();
        radio_abort_rx();
      }

    } else if (m_ci_state == S_GLOSSY_RECEIVED && call SFD.get()) {

      m_ci_state = S_GLOSSY_TRANSMITTING;
      call CaptureSFD.captureFallingEdge();

      post signal_receive(); 

    } else if (m_ci_state == S_GLOSSY_TRANSMITTING && !call SFD.get()) {
    
      m_ci_state = S_GLOSSY_TRANSMITED;
      radio_flush_tx();

    }
  }
#endif

  static inline void glossy_begin_rx() {
    m_ci_state = S_GLOSSY_LENGTH;

    while (!call FIFO.get()) {
      if (abort_timer_fired()) {
        radio_abort_rx();
        return;
      }
    }

    get_time_now();

    call DirectSpi.read( (uint8_t*)call CC2420PacketBody.getHeader(m_p_rx_buf), 1);
  }

  static inline void glossy_end_rx() {
    uint8_t* p_buf = (uint8_t*)call CC2420PacketBody.getHeader(m_p_rx_buf);

    if (m_ci_state == S_GLOSSY_DOWNLOAD) {
      m_ci_state = S_GLOSSY_DOWNLOAD_REST;

      call DirectSpi.read( p_buf+m_rx_readbytes, packet_len-m_rx_readbytes-1 );
    }
  }

  static inline void glossy_start_load() {
    if (m_ci_state == S_GLOSSY_RECEIVED) {
      uint8_t* p_buf = (uint8_t*)call CC2420PacketBody.getHeader(m_p_rx_buf);

      // Check the CRC of the receiving packet
      if (p_buf[packet_len] >> 7) {
        radio_write_tx();
      } else {
        radio_flush_tx();
      }
    }
  }

  static inline bool abort_timer_fired() {
    return (call TimeoutTimer.getNow() > t_rx_timeout);
  }

  static inline void radio_abort_rx() {
    m_ci_state = S_CANCEL;
    call DirectSpi.clearRxIntr();
    radio_flush_rx();
  }

  static inline void radio_write_tx() {

    call DirectSpi.TXCTRL( ( 2 << CC2420_TXCTRL_TXMIXBUF_CUR ) | \
                       ( 3 << CC2420_TXCTRL_PA_CURRENT ) |      \
                       ( 1 << CC2420_TXCTRL_RESERVED ) |        \
                       ( (CC2420_DEF_RFPOWER & 0x1F) << CC2420_TXCTRL_PA_LEVEL ));

    call DirectSpi.write( (uint8_t*)call CC2420PacketBody.getHeader(m_p_rx_buf), packet_len );
  }

  static inline void radio_start_tx() {
    cc2420_status_t status;
    status = call DirectSpi.STXON();
  }

  static inline void radio_flush_rx() {
    call DirectSpi.SFLUSHRX();
    call DirectSpi.SFLUSHRX();

    call CaptureSFD.captureRisingEdge();
    m_ci_state = S_WAITING;
  }

  static inline void radio_flush_tx() {
    call DirectSpi.SFLUSHTX();

    radio_flush_rx();
  }

  task void signal_receive() {
    memcpy(m_p_tx_buf, m_p_rx_buf, sizeof(message_t));
    signal Receive.receive( m_p_tx_buf, m_p_tx_buf->data, packet_len - CC2420_SIZE );
  }

  task void print_task() {
    pr("%ld\n", print_var);
  }

  task void print_packet() {
    packet_pr(m_rx_readbytes);
  }
}