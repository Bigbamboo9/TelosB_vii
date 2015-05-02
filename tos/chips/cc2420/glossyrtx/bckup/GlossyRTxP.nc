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
    // interface RadioBackoff;
/**
    interface ReceiveIndicator as EnergyIndicator;
    interface ReceiveIndicator as ByteIndicator;
    interface ReceiveIndicator as PacketIndicator;
 **/
  }
  uses {
    // interface Alarm<T32khz, uint32_t> as BackoffTimer;
    interface Alarm<TMicro, uint32_t> as TimeoutTimer;

    // interface CC2420Config;
    // interface CC2420Packet;
    interface CC2420PacketBody;
    // interface PacketTimeStamp<TMicro, uint32_t>;
    // interface PacketTimeSyncOffset;

    interface GeneralIO as CSN;
    // interface GeneralIO as CCA;
    interface GeneralIO as SFD;
    interface GeneralIO as FIFO;
    interface GeneralIO as FIFOP;

    interface GpioCapture as CaptureSFD;
    interface GpioInterrupt as InterruptFIFOP;

    // interface Resource as SpiResource;
    // interface ChipSpiResource;
    // interface CC2420Fifo as TXFIFO;
    // interface CC2420Fifo as RXFIFO;
    interface DirectSpi;

    // interface CC2420Strobe as SNOP;
    // interface CC2420Strobe as STXON;
    // interface CC2420Strobe as SRXON;
    // interface CC2420Strobe as SACK;
    // interface CC2420Strobe as STXONCCA;
    // interface CC2420Strobe as SFLUSHTX;
    // interface CC2420Strobe as SFLUSHRX;

    // interface CC2420Ram as TXFIFO_RAM;
    // interface CC2420Ram as RXFIFO_RAM;

    // interface CC2420Register as TXCTRL;

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

  task void signal_receive();
  task void print_task();

  static inline bool abort_timer_fired();
  static inline void radio_abort_rx();
  static inline void radio_abort_tx();
  static inline void radio_flush_rx();
  static inline void radio_flush_tx();
  static inline void radio_write_tx();
  static inline void radio_start_tx(); 

#ifdef ENABLE_GLOSSY
  static inline void glossy_sfd_interrupt( uint16_t time );
  static inline void glossy_begin_rx();
  static inline void glossy_end_rx();
#endif

  command error_t Init.init() {
    // call CCA.makeInput();
    call CSN.makeOutput();
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
      // call InterruptFIFOP.enableFallingEdge();
      call InterruptFIFOP.disable();
      // call CSN.clr();
      call DirectSpi.setMode();
      // call CSN.set();
      // m_tx_state = S_WAITING;
      // m_rx_state = S_WAITING;
      m_ci_state = S_WAITING;
    }
    return SUCCESS;
  }

  command error_t StdControl.stop() {
    atomic {
      call CaptureSFD.disable();
      call InterruptFIFOP.disable();
      // call BackoffTimer.stop();
      // call SpiResource.release();
      call CSN.set();

      // m_tx_state = S_STOPPED;
      // m_rx_state = S_STOPPED;
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
/*
    call CSN.clr();
    call TXFIFO_RAM.write( offset, buf, len );
    call CSN.set();
*/
    return SUCCESS;
  }
/**
  command bool EnergyIndicator.isReceiving() {
    return !(call CCA.get());
  }

  command bool ByteIndicator.isReceiving() {
    return FALSE;
  }

  command bool PacketIndicator.isReceiving() {
    return FALSE;
  }

  async command void RadioBackoff.setInitialBackoff( uint16_t time ) {
  }

  async command void RadioBackoff.setCongestionBackoff( uint16_t time ) {
  }

  async command void RadioBackoff.setCca( bool cca ) {
  }
**/
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
        m_ci_state = S_GLOSSY_DOWNLOAD;
        m_rx_readbytes = 1;

        while (!call FIFO.get()) {
          if (abort_timer_fired()) {
            radio_abort_rx();
            return;
          }
        }

        // call RXFIFO.continueRead( p_header + m_rx_readbytes, 1 ); 
        call DirectSpi.read( p_header + m_rx_readbytes, 1 );
      }
    }

    if (m_ci_state == S_GLOSSY_DOWNLOAD) {
      m_rx_readbytes = m_rx_readbytes + 1;

      if (m_rx_readbytes <= packet_len - 8) {
        while (!call FIFO.get()) {
          if (abort_timer_fired()) {
            radio_abort_rx();
            return;
          }
        }

        // call RXFIFO.continueRead( p_header + m_rx_readbytes, 1 );
        call DirectSpi.read( p_header + m_rx_readbytes, 1);
      } else {
        call Leds.led0Toggle();
      }
    }

    if (m_ci_state == S_GLOSSY_RECEIVED) {
      glossy_end_rx();
    }
  #endif  
  }

  // async event void RXFIFO.readDone( uint8_t* rx_buf, uint8_t rx_len, error_t err) { }

  // async event void RXFIFO.writeDone(uint8_t* buf, uint8_t len, error_t err) {  }

  // async event void TXFIFO.readDone(uint8_t* buf, uint8_t len, error_t err) { }

  // async event void TXFIFO.writeDone(uint8_t* buf, uint8_t len, error_t err) {

  async event void DirectSpi.writeDone(uint8_t* buf, uint8_t len, error_t err) {
    call CSN.set();
    radio_start_tx();
  }
/*
  async event void BackoffTimer.fired() {
  }
*/
  async event void TimeoutTimer.fired() {
  }
/*
  event void SpiResource.granted() {
    if (abort_timer_fired()) {
      radio_abort_rx();
    } else {
      glossy_begin_rx();
    }
  }
*/
#ifdef ENABLE_GLOSSY
  static inline void glossy_sfd_interrupt( uint16_t time ) {
    if ( m_ci_state == S_WAITING && call SFD.get() ) {
    /**According to the Glossy, Rx timeout : packet duration + 200 us
       packet duration : 32 us * packet_length, 1 DCO tick ~ 0.23 us**/
      t_rx_timeout = call TimeoutTimer.getNow() + ( packet_len * 35 + 200 ) * 4;

      call CaptureSFD.captureFallingEdge();
    /*
      if (call SpiResource.isOwner()) {
        glossy_begin_rx();
      } else if (call SpiResource.immediateRequest() == SUCCESS) {
        glossy_begin_rx();
      } else {
        call SpiResource.request();
      }
    */
      glossy_begin_rx();
    }

    if ( m_ci_state == S_GLOSSY_DOWNLOAD && !call SFD.get() ) {
      uint8_t T_irq = call TimeoutTimer.getNow() - time;
      t_min_jitter = (T_irq > t_min_jitter) ? t_min_jitter : T_irq;
      T_irq = T_irq - t_min_jitter;
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

        if (m_rx_readbytes == packet_len - 8) {
          call Leds.led2Toggle();
        }

        // radio_start_tx();
        glossy_end_rx();
        // post print_task();
        call CaptureSFD.captureRisingEdge();
      } else {
        radio_flush_rx();
      }
    }

    if (m_ci_state == S_GLOSSY_RECEIVED && call SFD.get()) {
      m_ci_state = S_GLOSSY_TRANSMITTING;
      // call SpiResource.release();
      post signal_receive(); 
      call CaptureSFD.captureFallingEdge();
    }

    if (m_ci_state == S_GLOSSY_TRANSMITTING && !call SFD.get()) {
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

    call CSN.set();
    call CSN.clr();
    // call RXFIFO.beginRead( (uint8_t*)(call CC2420PacketBody.getHeader(m_p_rx_buf)), 1 );
    call DirectSpi.read( (uint8_t*)call CC2420PacketBody.getHeader(m_p_rx_buf), 1);
  }

  static inline void glossy_end_rx() {
    uint8_t* p_buf = (uint8_t*)call CC2420PacketBody.getHeader(m_p_rx_buf);

    if (m_ci_state == S_GLOSSY_DOWNLOAD) {
      // call RXFIFO.continueRead( p_buf+m_rx_readbytes, packet_len-m_rx_readbytes+1 );
      call DirectSpi.read( p_buf+m_rx_readbytes, packet_len-m_rx_readbytes+1 );
      m_rx_readbytes = packet_len + 1;
      m_ci_state = S_GLOSSY_RECEIVED;
    }

    if (m_ci_state == S_GLOSSY_RECEIVED) {
      // Check the CRC
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
    radio_flush_rx();
  }

  static inline void radio_write_tx() {
    call CSN.clr();
/*
    call TXCTRL.write( ( 2 << CC2420_TXCTRL_TXMIXBUF_CUR ) |
                       ( 3 << CC2420_TXCTRL_PA_CURRENT ) |
                       ( 1 << CC2420_TXCTRL_RESERVED ) |
                       ( (CC2420_DEF_RFPOWER & 0x1F) << CC2420_TXCTRL_PA_LEVEL ));
*/
    call DirectSpi.writeReg( ( 2 << CC2420_TXCTRL_TXMIXBUF_CUR ) | \
                       ( 3 << CC2420_TXCTRL_PA_CURRENT ) |      \
                       ( 1 << CC2420_TXCTRL_RESERVED ) |        \
                       ( (CC2420_DEF_RFPOWER & 0x1F) << CC2420_TXCTRL_PA_LEVEL ));
    // call TXFIFO.write( (uint8_t*)call CC2420PacketBody.getHeader(m_p_rx_buf), packet_len-1 );
    call DirectSpi.write( (uint8_t*)call CC2420PacketBody.getHeader(m_p_rx_buf), packet_len-1 );
  }

  static inline void radio_start_tx() {
    call CSN.clr();
    // call STXON.strobe();
    call DirectSpi.writeSTXON();
    call CSN.set();
  }

  static inline void radio_flush_rx() {
    call CSN.clr();
    // call SFLUSHRX.strobe();
    // call SFLUSHRX.strobe();
    call DirectSpi.writeSFLUSHRX();
    call DirectSpi.writeSFLUSHRX();
    call CSN.set();
    // call SpiResource.release();
    call CaptureSFD.captureRisingEdge();

    m_ci_state = S_WAITING;
  }

  static inline void radio_flush_tx() {
    call CSN.clr();
    // call SFLUSHTX.strobe();
    call DirectSpi.writeSFLUSHTX();
    // call SRXON.strobe();
    call CSN.set();

    radio_flush_rx();
  }

  task void signal_receive() {
    memcpy(m_p_tx_buf, m_p_rx_buf, sizeof(message_t));
    signal Receive.receive( m_p_tx_buf, m_p_tx_buf->data, packet_len - CC2420_SIZE );
  }

  task void print_task() {
    // pr("%d %d\n", packet_len, m_rx_len);
    // pr("%d %d\n", t_min_jitter, print_var);
    pr("%ld\n", print_var);
    // pr("%d\n", m_rx_readbytes);
  }
}
