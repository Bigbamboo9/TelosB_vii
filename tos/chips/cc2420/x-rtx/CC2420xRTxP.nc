#include "cc2420_x_rtx.h"
#include "cc2420_x_control.h"
#include "cc2420_x_packet.h"
#include "serial_fast_print.h"

module CC2420xRTxP {
  provides interface Init;
  provides interface LplSend;
  provides interface LplReceive;
  provides interface Receive as Snoop;
  provides interface LplTime;
  provides interface OppoRouting;
  provides interface Msp430TimerEvent as VectorTimerB1;

  uses interface Leds;
} implementation {
  /** status variable **/
  cc2420_rtx_state_t rtx_status;
  uint8_t pkt_length;
  uint16_t tbiv;

  /** timer system variable **/
  // measure the rx duration of individual packet reception from the rising SFD to the falling SFD
  uint16_t rx_duration;
  // measure the tx duration of individual packet transmission from the rising SFD to the falling SFD
  uint16_t tx_duration;
  // measure the ack duration of individual ack transmssion from the rising SFD to the falling SFD
  uint16_t ack_duration;
  // measure the turn-around period between the strobe and SFD interrupt
  uint16_t turn_around;
  // channel access duration measurement
  uint16_t detect_duration;
  // abortion duration record
  uint16_t abortion_duration;
  rtx_time_compensation_t rtx_time;
  // buffer the value of TBCCR1
  uint16_t tb1_buffer;

  /** signal detection variable **/
  int noise_floor;

  /** force to close the radio after receive **/
  bool force_sleep;
  uint8_t continuous_duplicate;

  /** receiving and transmitting variable **/
  /*---- rtx strategy ----*/
  /**
   * 1. if receive data targeted this node, reception is the highest priority
   * 2. if receive data not targeted this node, transmit data with CI
   * 3. if set tx priority bit is set, transmission has the highest priority
   **/
  message_t* p_tx_buf;
  // tx swap buffer for ci tranmission
  message_t* swap_tx_buf;
  // the number of preamble packets have been transmitted
  uint8_t tx_counter;
  // the tx setting of current transmission process
  rtx_setting_t* tx_setting;
  // message buffer for packets reception
  message_t rx_buf[RX_BUFFER_SIZE];
  rx_buffer_t m_rx_buf;
  // the number of bytes read from RxFIFO to RAM
  uint8_t rx_readbytes;
  // the rx setting of current receive process
  rtx_setting_t rx_setting;
  // local opportunistic routing metric
  uint16_t local_metric;
  // whether ack the received packet
  bool ack_mark;

  uint8_t rssi_print;

  /** Detect the potential ongoing packet transmission by being awake and checking SFD for two adjacent SFD period **/
  static inline void cc2420_signal_detect(uint16_t time);
  /** Start to read data from RxFIFO after the SFD up edge is detected during receiving process **/
  static inline void cc2420_begin_rx();
  /** Finish the lefted data transfer, send ack back and determine the further settings after the SFD down edge is detected during receiving process **/
  static inline void cc2420_end_rx();
  /** Strobe the software ACK transmission **/
  static inline void cc2420_ack_strobe_rx();
  /** Finish the ack sending after the SFD down edge is detected during the receiving process **/
  static inline void cc2420_ack_end_rx();
  /** Load the data from memory to TxFIFO after the Tx strbe **/
  static inline void cc2420_load_tx();
  /** Strobe the TXON to initialize the transmission process **/
  static inline void cc2420_strobe_tx();
  /** ACK waiting after the falling edge SFD is detected during transmitting process **/
  static inline void cc2420_ack_wait_tx();
  /** falling edge SFD is detected, ACK has been received during transmission **/
  static inline void cc2420_ack_rx();
  /** dealing with the timing exception after ack receiving **/
  static inline void cc2420_ack_rx_except();

  command error_t Init.init() {
    rtx_status = S_RTX_IDLE;
    pkt_length = CC2420_X_PACKET_SIZE + CC2420_SIZE;
    noise_floor = -77;
    p_tx_buf = NULL;
    swap_tx_buf = NULL;
    tx_counter = 0;
    rx_readbytes = 0;
    m_rx_buf.received = FALSE;
    m_rx_buf.max_size = RX_BUFFER_SIZE;
    m_rx_buf.occ_size = 0;
    m_rx_buf.pos_buf = 0;
    m_rx_buf.p_rx_buf = &rx_buf[0];
    timer_initialization();
    rtx_time.calibration_factor = 128;
    rtx_time.pkt_rtx_time = 10335;
    rtx_time.ack_time = 672;
    rtx_time.turnaround_time = 537;
    rtx_time.turnaround_total_time = 0;
    rtx_time.rtx_total_time = 0;
    rtx_time.ack_total_time = 0;
    rtx_time.radio_on_time = 0;
    rtx_time.tail_total_time = 0;
    force_sleep = FALSE;
    continuous_duplicate = 0;
    return SUCCESS;
  }

  command error_t LplSend.send(message_t* msg, rtx_setting_t* ts) {
    if (rtx_status != S_RTX_IDLE)
      return EBUSY;
    if (msg == NULL)
      return FAIL;
    atomic {
      tx_setting = ts;
      p_tx_buf = msg;
      tx_counter = 0;
      force_sleep = FALSE;
      continuous_duplicate = 0;
      rtx_time.pkt_recv = 0;
      rtx_time.pkt_send = 0;
      rtx_time.pkt_ack = 0;
      rtx_time.pkt_turnaround = 0;
      rtx_time.channel_detection = 0;
      rtx_status = S_TX_DETECT;
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;
    }
    cc2420_signal_detect(rtimer_arch_now_dco());
    return SUCCESS;
  }

  command void LplReceive.rxOn() {
    if ( (rtx_status != S_RTX_IDLE) |(m_rx_buf.occ_size == m_rx_buf.max_size) )
      return;
    atomic {
      m_rx_buf.p_rx_buf = &rx_buf[(m_rx_buf.pos_buf + m_rx_buf.occ_size) % m_rx_buf.max_size];
      force_sleep = FALSE;
      continuous_duplicate = 0;
      rtx_time.pkt_recv = 0;
      rtx_time.pkt_send = 0;
      rtx_time.pkt_ack = 0;
      rtx_time.pkt_turnaround = 0;
      rtx_time.channel_detection = 0;
      rtx_status = S_RX_DETECT;
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;
    }
    cc2420_signal_detect(rtimer_arch_now_dco());
  }

  command error_t LplReceive.rxInit() {
    atomic {
      m_rx_buf.received = FALSE;
    }
    if (m_rx_buf.occ_size == m_rx_buf.max_size)
      return EBUSY;
    return SUCCESS;
  }

  command void LplReceive.rxBuffSet() {
    atomic {
      m_rx_buf.pos_buf = (m_rx_buf.pos_buf + m_rx_buf.occ_size) % m_rx_buf.max_size;
      m_rx_buf.occ_size = 0;
    }
  }

  default event void LplSend.sendDone(message_t* msg, rtx_setting_t* ts, error_t error) { }
  default event void LplReceive.receive(message_t* msg, uint8_t size) { }

  TOSH_SIGNAL(TIMERB1_VECTOR) {
    // the delta ticks for TB1 campature interrupt
    uint16_t TB1_irq = ( (rtimer_arch_now_dco() - TBCCR1) - 39 ) << 1;
    // the delta ticks for TB2 (ACK) compare interrupt
    // uint16_t TB2_irq = ( (rtimer_arch_now_dco() - TBCCR2) - 61 ) << 1; // w printf
    uint16_t TB2_irq = ( (rtimer_arch_now_dco() - TBCCR2) - 53 ) << 1; // w/o printf
    uint8_t debug;
    tbiv = TBIV;
    tb1_buffer = TBCCR1;

    // printf_u16(1, &tbiv);

    if (tbiv == 2) {
      // printf_u16(1, &TB1_irq);
      // compensate the if-else jump for ACK waiting timer
      if ( (rtx_status == S_TX_ACK || rtx_status == S_CI_ACK) && ( !(P4IN & (1 << 1)) ) ) {
        debug = 11;
        printf_u8(1, &debug);
        /** ack has been received **/
        ack_duration = TBCCR1 - ack_duration;
        detect_duration = TBCCR1;
        // compensate the if-else jump for RX_ACK
        if ( rtx_status == S_RX_ACK && (!(P4IN & (1 << 1))));
        // turn off the SFD abortion timer
        TBCCTL3 &= ~(CCIE | CCIFG);
        // set interrupt edge
        TBCCTL1 = CM_1 | CAP | SCS | CCIE;

        if (TB1_irq <= 8) {
          // compensate the interrupt handling delay
          asm volatile ("add %[d], r0" : : [d] "m" (TB1_irq));
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          // compensate the unsynchronous clock frequency between MCU and Radio
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          
          cc2420_strobe_tx();

          cc2420_ack_rx();
        } else {
          cc2420_ack_rx_except();
        }

        rtx_time.pkt_ack++;
        ack_time_update(&rtx_time, ack_duration);
      } else if ( rtx_status == S_RX_ACK && (!(P4IN & (1 << 1)))) {
        debug = 22;
        printf_u8(1, &debug);
        /** ack has been sent **/
        ack_duration = TBCCR1 - ack_duration;
        detect_duration = TBCCR1;
        
        // turn off the SFD abortion timer
        TBCCTL3 &= ~(CCIE | CCIFG);
        // set interrupt edge
        TBCCTL1 = CM_1 | CAP | SCS | CCIE;

        if (TB1_irq <= 8) {
          // compensate the interrupt handling delay
          asm volatile ("add %[d], r0" : : [d] "m" (TB1_irq));
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          // compensate the unsynchronous clock frequency between MCU and Radio
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");

          cc2420_strobe_tx();

          if (p_tx_buf != NULL && tx_setting->priority) {
            // current transmission hold the high priority, transmit with ci
            rtx_status = S_TX_SFD;
            cc2420_load_tx();
          } else if ((m_rx_buf.occ_size != 0) && (m_rx_buf.occ_size != m_rx_buf.max_size)) {
            // exactly received data packet
            if (rx_setting.ci && rx_setting.hop < CI_HOP_THRESHOLD) {
              // transmit the received data with ci
              rtx_status = S_CI_SFD;
              if (p_tx_buf == NULL) {
                p_tx_buf = m_rx_buf.p_rx_buf;
              } else {
                // save the p_tx_buf first
                swap_tx_buf = p_tx_buf;
                p_tx_buf = m_rx_buf.p_rx_buf;
              }
              tx_counter = 0;
              cc2420_load_tx();
            } else if (rx_setting.batched) {
              // some packets will be comming soon
              radio_flush_tx();
              rtx_status = S_RX_DETECT;
              rtx_status = S_RX_DETECT;
              TBCCR5 = TBCCR1 + LISTENING_TAIL;
              detect_duration = TBCCR1;
              TBCCTL5 = CCIE;
            } else {
              radio_flush_tx();
              rtx_status = S_RX_DETECT;
              cc2420_signal_detect(TBCCR1);
            }
          } else if (p_tx_buf != NULL) {
            // nothing will be received further, start to transmit
            rtx_status = S_TX_SFD;
            cc2420_load_tx();
          } else if (m_rx_buf.occ_size == m_rx_buf.max_size) {
            // no buffer left, turn off the radio process
            radio_flush_tx();
            TBCCTL1 = CAP;
            rtx_time.pkt_ack++;
            ack_time_update(&rtx_time, ack_duration);
            signal LplTime.timeRadio(&rtx_time);
            if (m_rx_buf.occ_size > 0) {
              signal LplReceive.receive(&rx_buf[m_rx_buf.pos_buf], m_rx_buf.occ_size);
            }
            if (p_tx_buf != NULL) {
              signal LplSend.sendDone(p_tx_buf, tx_setting, SUCCESS);
              p_tx_buf = NULL;
            }
            signal LplTime.timeCompensated(rtimer_arch_now_dco() - TBCCR1, &rtx_time);
            return;
          } else {
            //nothing will be received and transmited, start channel checking
            radio_flush_tx();
            rtx_status = S_RX_DETECT;
            TBCCR5 = TBCCR1 + LISTENING_TAIL;
            detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
          }
        } else {
          // TODO: exception handler
          // ignore the CI
          if (p_tx_buf != NULL && tx_setting->priority) {
            rtx_status = S_TX_SFD;
            cc2420_signal_detect(TBCCR1);
          } else if ((m_rx_buf.occ_size != 0) && (m_rx_buf.occ_size != m_rx_buf.max_size)) {
            if (rx_setting.batched) {
              rtx_status = S_RX_DETECT;
              TBCCR5 = TBCCR1 + LISTENING_TAIL;
              detect_duration = TBCCR1;
              TBCCTL5 = CCIE;
            } else {
              radio_flush_tx();
              rtx_status = S_RX_DETECT;
              cc2420_signal_detect(TBCCR1);
            }
          } else if (p_tx_buf != NULL) {
            rtx_status = S_TX_SFD;
            cc2420_signal_detect(TBCCR1);
          } else if (m_rx_buf.occ_size == m_rx_buf.max_size) {
            TBCCTL1 = CAP;
            rtx_time.pkt_ack++;
            ack_time_update(&rtx_time, ack_duration);
            signal LplTime.timeRadio(&rtx_time);
            if (m_rx_buf.received) {
              signal LplReceive.receive(&rx_buf[m_rx_buf.pos_buf], m_rx_buf.occ_size);
            }
            if (p_tx_buf != NULL) {
              signal LplSend.sendDone(p_tx_buf, tx_setting, SUCCESS);
              p_tx_buf = NULL;
            }
            signal LplTime.timeCompensated(rtimer_arch_now_dco() - TBCCR1, &rtx_time);
            return;
          } else {
            rtx_status = S_RX_DETECT;
            TBCCR5 = TBCCR1 + LISTENING_TAIL;
            detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
          }
        }

        rtx_time.pkt_ack++;
        ack_time_update(&rtx_time, ack_duration);
      } else if ( (rtx_status == S_RX_RECEIVE) && ( !(P4IN & (1 << 1)) ) ) {
        debug = 33;
        printf_u8(1, &debug);
        /** data packet has been received, wether transmit with CI and ack the packet **/
        // turn off receive execption timer
        rx_duration = TBCCR1 - rx_duration;
        TBCCTL3 &= ~(CCIE | CCIFG);
        detect_duration = TBCCR1;
        if ( (rtx_status== S_TX_SFD) && (!(P4IN & (1 << 1))) );
        // set interrupt edge
        TBCCTL1 = CM_1 | CAP | SCS | CCIE;
        
        if (TB1_irq <= 8) {
          // compensate the interrupt handling delay
          asm volatile ("add %[d], r0" : : [d] "m" (TB1_irq));
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          // compensate the unsynchronous clock frequency between MCU and Radio
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");

          cc2420_ack_strobe_rx();
          // cc2420_ack_wait_tx();
        }
        
        cc2420_end_rx();
        rtx_time.pkt_recv++;
        rx_time_update(&rtx_time, rx_duration);
      } else if ( (rtx_status== S_TX_SFD) && (!(P4IN & (1 << 1))) ) {
        debug = 44;
        printf_u8(1, &debug);
        /** packet transmission has finished **/
        tx_duration = TBCCR1 - tx_duration;
        detect_duration = TBCCR1;
        // turn off SFD abortion timer
        TBCCTL3 &= ~(CCIE | CCIFG);
        // set interrupt edge
        TBCCTL1 = CM_1 | CAP | SCS | CCIE;

        // time compensation need? I think the compensation is needed!
        if (TB1_irq <= 8) {
          // compensate the interrupt handling delay
          asm volatile ("add %[d], r0" : : [d] "m" (TB1_irq));
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          // compensate the unsynchronous clock frequency between MCU and Radio
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");

          cc2420_ack_wait_tx();
          rtx_status = S_TX_ACK;
        } else {
          // TODO: detect again
          radio_flush_tx();
          rtx_status = S_TX_DETECT;
          cc2420_signal_detect(TBCCR1);
        }

        rtx_time.pkt_send++;
        tx_time_update(&rtx_time, tx_duration);
      } else if ( rtx_status == S_CI_SFD && ( !(P4IN & (1 << 1)) ) ) {
        debug = 55;
        printf_u8(1, &debug);
        /** CI transmission end **/
        detect_duration = TBCCR1;
        // turn off reception exception timer
        TBCCTL3 &= ~(CCIE | CCIFG);

        TBCCTL1 = CM_1 | CAP | SCS | CCIE;

        // time compensation need? I think the compensation is needed!
        if (TB1_irq <= 8) {
          // compensate the interrupt handling delay
          asm volatile ("add %[d], r0" : : [d] "m" (TB1_irq));
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          // compensate the unsynchronous clock frequency between MCU and Radio
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");
          asm volatile ("nop");

          cc2420_ack_wait_tx();
          rtx_status = S_CI_ACK;
        } else {
          radio_flush_tx();
          rtx_status = S_RTX_IDLE;
          TBCCR5 = TBCCR1 + LISTENING_TAIL;
          detect_duration = TBCCR1;
          TBCCTL5 = CCIE;
        }

        rtx_time.pkt_send++;
      } else if ( (rtx_status == S_RX_DETECT || rtx_status == S_TX_DETECT) && (P4IN & (1 << 1)) ) {
        debug = 66;
        printf_u8(1, &debug);
        /** data packet prepare to receive and start reception exception timer **/
        // turn off the detection waiting timer
        detect_duration = TBCCR1 - detect_duration;
        rx_duration = TBCCR1;
        
        if (!force_sleep)
          TBCCTL5 &= ~(CCIE | CCIFG);
      
        TBCCTL1 = CM_2 | CAP | SCS | CCIE;
        /**According to the Glossy, Rx timeout : packet duration + 200 us packet duration : 32 us * packet_length, 1 DCO tick ~ 1 us**/
        TBCCR3 = TBCCR1 + ( pkt_length * 32 + 200 ) * 4;
        abortion_duration = TBCCR1;
        TBCCTL3 = CCIE;

        m_rx_buf.received = TRUE;
        cc2420_begin_rx();

        rtx_time.channel_detection += detect_duration;
        // turnaround time is already included in detection time!
        // rtx_time.pkt_turnaround++;
        // TODO: synchronization time stamp
      } else if ( (rtx_status == S_RX_ACK || rtx_status == S_TX_ACK || rtx_status == S_CI_ACK) && (P4IN & (1 << 1)) ) {
        debug = 77;
        printf_u8(1, &debug);
        /** transmissin of ack starts**/
        // turn off ack waiting timer
        turn_around = TBCCR1 - turn_around;
        ack_duration = TBCCR1;
        
        TBCCTL2 &= ~(CCIE | CCIFG);
      
        // turn on SFD abortion timer
        TBCCR3 = TBCCR1 + ( ACK_LENGTH * 32 + 200 ) * 4;
        abortion_duration = TBCCR1;
        TBCCTL3 = CCIE;

        TBCCTL1 = CM_2 | CAP | SCS | CCIE;

        turnaround_time_update(&rtx_time, turn_around);
        rtx_time.pkt_turnaround++;
      } else if ( rtx_status == S_CI_SFD && (P4IN & (1 << 1))) {
        debug = 88;
        printf_u8(1, &debug);
        /** CI transmission start **/
        TBCCTL1 = CM_2 | CAP | SCS | CCIE;

        TBCCR3 = TBCCR1 +  ( pkt_length * 32 + 200 ) * 4;
        abortion_duration = TBCCR1;
        TBCCTL3 = CCIE;

        rtx_time.pkt_turnaround++;
      } else if ( (rtx_status == S_TX_SFD) && (P4IN & (1 << 1))) {
        debug = 99;
        printf_u8(1, &debug);
        /** packet transmission start **/
        tx_duration = TBCCR1;
        
        TBCCTL1 = CM_2 | CAP | SCS | CCIE;

        TBCCR3 = TBCCR1 +  ( pkt_length * 32 + 200 ) * 4;
        abortion_duration = TBCCR1;
        TBCCTL3 = CCIE;
 
        rtx_time.pkt_turnaround++;
      }
    } else {
      switch(tbiv) {
        case 4:
          detect_duration = TBCCR2;
          TBCCTL2 &= ~(CCIE | CCIFG);
          // ACK waiting period fired
          if (TB2_irq <= 8) {
            // compensate the interrupt handling delay
            asm volatile("add %[d], r0" : : [d] "m" (TB2_irq));
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            // compensate the unsynchronous clock frequency between MCU and Radio
            asm volatile ("nop");
            asm volatile ("nop");
            asm volatile ("nop");
            asm volatile ("nop");
            asm volatile ("nop");
            asm volatile ("nop");
            asm volatile ("nop");
            asm volatile ("nop");

            cc2420_strobe_tx();

            if (rtx_status == S_RX_ACK) {
            // does not successful or need to ack the received data
              if (p_tx_buf != NULL) {
                rtx_status = S_TX_SFD;
                cc2420_load_tx();
              } else {
                radio_flush_tx();
                rtx_status = S_RX_DETECT;
                TBCCR5 = TBCCR1 + LISTENING_TAIL;
                detect_duration = TBCCR2;
                TBCCTL5 = CCIE;
              }
            } else if (rtx_status == S_TX_ACK) {
              rtx_status = S_TX_SFD;
              tx_counter++;
              // printf_u8(1, &tx_counter);
              write_ram(CC2420_RAM_TXFIFO, sizeof(cc2420_header_t)+sizeof(rtx_setting_t), &tx_counter, 1);
              if (tx_counter == PREAMBLE_PACKET_LENGTH) {
                tx_setting->size--;
                if (tx_setting->size != 0) {
                  rtx_status = S_TX_SFD;
                  p_tx_buf = (message_t*)((uint8_t*)p_tx_buf + sizeof(message_t));
                  cc2420_load_tx();
                  tx_counter = 0;
                } else {
                  radio_flush_tx();
                  rtx_status = S_RX_DETECT;
                  TBCCR5 = TBCCR1 + LISTENING_TAIL;
                  detect_duration = TBCCR2;
                  TBCCTL5 = CCIE;
                }
              }
            } else if (rtx_status == S_CI_ACK) {
              rtx_status = S_CI_SFD;
              tx_counter++;
              write_ram(CC2420_RAM_TXFIFO, sizeof(cc2420_header_t)+sizeof(rtx_setting_t), &tx_counter, 1);
              if (tx_counter == PREAMBLE_PACKET_LENGTH) {
                if (swap_tx_buf != NULL) {
                  rtx_status = S_TX_SFD;
                  p_tx_buf = swap_tx_buf;
                  cc2420_load_tx();
                  swap_tx_buf = NULL;
                  tx_counter = 0;
                } else {
                  radio_flush_tx();
                  rtx_status = S_RX_DETECT;
                  TBCCR5 = TBCCR1 + LISTENING_TAIL;
                  detect_duration = TBCCR2;
                  TBCCTL5 = CCIE;
                }
              }
            }
          } else {
            // printf_u16(1, &TB2_irq);
            if (p_tx_buf != NULL) {
              tx_counter++;
              if (tx_counter == PREAMBLE_PACKET_LENGTH) {
                tx_setting->size--;
                if (tx_setting->size != 0) {
                  rtx_status = S_TX_SFD;
                  p_tx_buf = (message_t*)((uint8_t*)p_tx_buf + sizeof(message_t));
                  cc2420_load_tx();
                  tx_counter = 0;
                } else {
                  radio_flush_tx();
                  rtx_status = S_RX_DETECT;
                  TBCCR5 = TBCCR1 + LISTENING_TAIL;
                  detect_duration = TBCCR2;
                  TBCCTL5 = CCIE;
                }
              } else {
                rtx_status = S_TX_DETECT;
                cc2420_signal_detect(TBCCR2);
              }
            } else {
              radio_flush_tx();
              rtx_status = S_RX_DETECT;
              TBCCR5 = TBCCR1 + LISTENING_TAIL;
              detect_duration = TBCCR2;
              TBCCTL5 = CCIE;
            }
          }

          rtx_time.pkt_ack++;
          rtx_time.pkt_turnaround++;
          break;
        case 6:
          TBCCTL3 &= ~(CCIE | CCIFG);
        // SFD abortion or reception exception
          radio_flush_tx();
          rtx_status = S_RX_DETECT;
          abortion_duration = TBCCR3 - abortion_duration;
          rtx_time.channel_detection += abortion_duration;
          TBCCTL1 = CM_1 | CAP | SCS | CCIE;
          TBCCR5 = TBCCR1 + LISTENING_TAIL;
          detect_duration = TBCCR1;
          TBCCTL5 = CCIE;
          break;
        case 10:
          TBCCTL5 &= ~(CCIE | CCIFG);
          radio_flush_rx();
        // no packet in channel is detected
          detect_duration = TBCCR5 - detect_duration;
          rtx_time.channel_detection += detect_duration;
          rtx_status = S_RTX_IDLE;
          TBCCTL1 = CAP;
          signal LplTime.timeRadio(&rtx_time);
          if (m_rx_buf.occ_size > 0) {
            signal LplReceive.receive(&rx_buf[m_rx_buf.pos_buf], m_rx_buf.occ_size);
          }
          if (p_tx_buf != NULL) {
            signal LplSend.sendDone(p_tx_buf, tx_setting, SUCCESS);
            p_tx_buf = NULL;
          }
          signal LplTime.timeCompensated(rtimer_arch_now_dco() - TBCCR5, &rtx_time);
          break;
        default:
          signal VectorTimerB1.fired();
      }
    }
  }

  static inline void cc2420_signal_detect(uint16_t time) {
    uint8_t i;
    uint8_t pos_number = 0;
    uint8_t noise_number = 0;
    int rssi_max = noise_floor;
    int noise_floor_sum = 0;
    int pos_sum = 0;
    int rssi_list[SIGNAL_DETECT_PERIOD];
    // statistics for detection time
    detect_duration = time;
    // waiting RSSI is valid
    while ((strobe(CC2420_SNOP) & CC2420_STATUS_RSSI_VALID) == 0);
    // sampling signal strength
    for (i = 0; (i < SIGNAL_DETECT_PERIOD) && (!m_rx_buf.received); i++) {
      atomic { rssi_list[i] = cc2420_get_rssi(); }
      if (rssi_list[i] > (noise_floor + 3)) {
        pos_number++;
        pos_sum += rssi_list[i];
        if (rssi_list[i] > rssi_max) {
          rssi_max = rssi_list[i];
        }
      } else {
        noise_number++;
        noise_floor_sum += rssi_list[i];
      }
    }
    // some signal with large energy is detected, but no packet is received yet
    if ((pos_number > RSSI_UP_THRESHOLD) && (!m_rx_buf.received)) {
      atomic {
        int pos_avr = pos_sum / pos_number;
        // PAPR is small so it is probably ZigBee. Start the tail timer.
        if (rssi_max - pos_avr < RSSI_PAPR_THRESHOLD) {
          //TBCCR5 = rtimer_arch_now_dco() + (rtx_time.pkt_rtx_time == 0) ? (packet_len * 32 + 200) * 4 : rtx_time.pkt_rtx_time;
          TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
          TBCCTL5 = CCIE;
          return;
        }
        if ( rtx_status == S_TX_DETECT ) {
          rtx_status = S_TX_SFD;
          cc2420_strobe_tx();
          cc2420_load_tx();
          return;
        }
        if ( rtx_status == S_RX_DETECT ) {
          //TODO: signal LPL layer to sleep
          detect_duration = rtimer_arch_now_dco() - detect_duration;
          rtx_time.channel_detection += detect_duration;
          rtx_status = S_RTX_IDLE;
          TBCCTL1 = CAP;
          signal LplTime.timeRadio(&rtx_time);
          if (m_rx_buf.occ_size > 0) {
            signal LplReceive.receive(&rx_buf[m_rx_buf.pos_buf], m_rx_buf.occ_size);
          }
          if (p_tx_buf != NULL) {
            signal LplSend.sendDone(p_tx_buf, tx_setting, SUCCESS);
            p_tx_buf = NULL;
          }
          signal LplTime.timeCompensated(rtimer_arch_now_dco() - TBCCR1, &rtx_time);
          return;
        }
      }
    }
    // update the noise floor
    if (!m_rx_buf.received) {
      atomic {
        noise_floor_sum = noise_floor_sum / noise_number;
        noise_floor = (8 * noise_floor + 2 * noise_floor_sum) / 10;
        if ( rtx_status == S_TX_DETECT ) {
          rtx_status = S_TX_SFD;
          cc2420_strobe_tx();
          cc2420_load_tx();
          return;
        }
        if ( rtx_status == S_RX_DETECT) {
          //TODO: signal LPL layer to sleep
          detect_duration = rtimer_arch_now_dco() - detect_duration;
          rtx_time.channel_detection += detect_duration;
          rtx_status = S_RTX_IDLE;
          TBCCTL1 = CAP;
          signal LplTime.timeRadio(&rtx_time);
          if (m_rx_buf.occ_size > 0) {
            signal LplReceive.receive(&rx_buf[m_rx_buf.pos_buf], m_rx_buf.occ_size);
          }
          if (p_tx_buf != NULL) {
            signal LplSend.sendDone(p_tx_buf, tx_setting, SUCCESS);
            p_tx_buf = NULL;
          }
          signal LplTime.timeCompensated(rtimer_arch_now_dco() - TBCCR1, &rtx_time);
        }
      }
    }
  }

  static inline void cc2420_begin_rx() {
    uint8_t* p_header = get_packet_header( m_rx_buf.p_rx_buf );
    rtx_status = S_RX_RECEIVE;

    while (((P1IN & (1 << 3)) == 0)) {
      if (TBCCTL3 & CCIFG) {
        radio_flush_rx();
        rtx_status = S_RTX_IDLE;
        TBCCTL1 = CM_1 | CAP | SCS | CCIE;
        return;
      }
    }

    fast_read_one( p_header );

    if (p_header[0] != pkt_length) {
      radio_flush_rx();
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;
      return;
    }

    rx_readbytes = 1;

    while (rx_readbytes <= pkt_length - 2) {

      while (((P1IN & (1 << 3)) == 0)) {
        if (TBCCTL3 & CCIFG) {
          radio_flush_rx();
          TBCCTL1 = CM_1 | CAP | SCS | CCIE;
          return;
        }
      }

      fast_continue_read_one( p_header + rx_readbytes );

      rx_readbytes++;
      /** deal with the packet header here **/
      if (rx_readbytes == CC2420_X_HEADER_LENGTH) {
        rtx_setting_t* m_ptr_setting = (rtx_setting_t*)get_packet_setting(m_rx_buf.p_rx_buf);
        // receive the data, acknowledge the packet, else abort the ongoing ack transmission
        if ((!m_ptr_setting->ack) 
         || ( (m_ptr_setting->addr != TOS_NODE_ID)
           && ( (m_ptr_setting->addr != OPPORTUNISTIC_ROUTING_ADDR)
             || (m_ptr_setting->metric - local_metric < m_ptr_setting->progress)))
         || (!(m_ptr_setting->ci && (m_ptr_setting->hop != 0)))) {
          ack_mark = FALSE;
        } else {
          ack_mark = TRUE;
        }
      }
    }
  }

  static inline void cc2420_end_rx() {
    uint8_t i;
    bool duplicate = FALSE;
    uint8_t* p_buf = get_packet_header(m_rx_buf.p_rx_buf);

    rtx_status = S_RX_ACK;

    fast_read_any( p_buf+rx_readbytes, pkt_length-rx_readbytes+1 );
    rx_readbytes = pkt_length + 1;

    if (p_buf[pkt_length] >> 7) {
      rtx_setting_t* m_ptr_setting = (rtx_setting_t*)get_packet_setting(m_rx_buf.p_rx_buf);
      
      if (!ack_mark) {
        radio_flush_tx();
        if (m_ptr_setting->addr != AM_BROADCAST_ADDR) {
          // overheard packet
          signal Snoop.receive(m_rx_buf.p_rx_buf, (void*)get_packet_payload(m_rx_buf.p_rx_buf), pkt_length);
          radio_flush_rx();
          return;
        }
        // the ack waiting timer is running, deal with this after ACK is transmitted
      }
      /** Too long to execute here, no left time to terminate the ACK transmission
      // receive the data, acknowledge the packet, else abort the ongoing ack transmission
      if ((!m_ptr_setting->ack) 
       || ( (m_ptr_setting->addr != TOS_NODE_ID)
         && ( (m_ptr_setting->addr != OPPORTUNISTIC_ROUTING_ADDR)
           || (m_ptr_setting->metric - local_metric < m_ptr_setting->progress)))
       || (!(m_ptr_setting->ci && (m_ptr_setting->hop != 0)))) {
        printf_u16(1, &(m_ptr_setting->addr));
        radio_flush_tx();
        if (m_ptr_setting->addr != AM_BROADCAST_ADDR) {
          // overheard packet
          signal Snoop.receive(m_rx_buf.p_rx_buf, (void*)get_packet_payload(m_rx_buf.p_rx_buf), pkt_length);
          radio_flush_rx();
          return;
        }
        // the ack waiting timer is running, deal with this after ACK is transmitted
      } **/
      // duplicate check
      for (i = 0; i < m_rx_buf.occ_size; i++) {
        cc2420_header_t* p_header = (cc2420_header_t*)get_packet_header(&rx_buf[(m_rx_buf.pos_buf + i) % RX_BUFFER_SIZE]);
        if ((((cc2420_header_t*)p_buf)->dsn == p_header->dsn) && (((cc2420_header_t*)p_buf)->src == p_header->src)) {
          duplicate = TRUE;
          break;
        }
      }
      if (!duplicate) {
        // set the rx setting
        m_ptr_setting->hop--;
        memcpy(&rx_setting, m_ptr_setting, sizeof(rtx_setting_t));
        // buffer swapping
        m_rx_buf.occ_size++;
        if (m_rx_buf.occ_size != m_rx_buf.max_size) {
          m_rx_buf.p_rx_buf = &rx_buf[(m_rx_buf.pos_buf + m_rx_buf.occ_size) % RX_BUFFER_SIZE];
          /** deal with this after ACK is transmitted **/
        } else {
          // no extra buffer for further data packet receiving, deal with this after ACK is transmitted
        }
      } else {
        continuous_duplicate++;
        if (continuous_duplicate > DUPLICATE_COUNT)
          force_sleep = TRUE;
        // received duplicate
        signal Snoop.receive(m_rx_buf.p_rx_buf, (void*)get_packet_payload(m_rx_buf.p_rx_buf), pkt_length);
      }
    } else {
      radio_flush_tx();
      radio_flush_rx();
      // the ack waiting timer is running, deal with this after ACK is transmitted
    }
  }
  
  static inline void cc2420_ack_strobe_rx() {
    radio_flush_rx();
    TBCCTL1 &= ~CCIFG;
    detect_duration = rtimer_arch_now_dco() - detect_duration;
    turn_around = rtimer_arch_now_dco();
    strobe( CC2420_SACK );
    TBCCR2 = turn_around + rtx_time.turnaround_time + rtx_time.ack_time;
    TBCCTL2 = CCIE;
    rtx_time.channel_detection += detect_duration;
  }

  static inline void cc2420_load_tx() {
    fast_write_any( get_packet_header(p_tx_buf), pkt_length );
  }

  static inline void cc2420_strobe_tx() {
    radio_flush_rx();
    TBCCTL1 &= ~CCIFG;
    detect_duration = rtimer_arch_now_dco() - detect_duration;
    // printf_u16(1, &detect_duration);
    strobe( CC2420_STXON );
    rtx_time.channel_detection += detect_duration;
  }

  static inline void cc2420_ack_wait_tx() {
    radio_flush_rx();
    TBCCTL1 &= ~CCIFG;
    detect_duration = rtimer_arch_now_dco() - detect_duration;
    // printf_u16(1, &detect_duration);
    // TBCCR2 = TBCCR1 + ACK_WAITING_PERIOD;
    TBCCR2 = rtimer_arch_now_dco() + rtx_time.turnaround_time + rtx_time.ack_time;
    TBCCTL2 = CCIE;
    rtx_time.channel_detection += detect_duration;
  }

  static inline void cc2420_ack_rx() {
    uint8_t type = 0xff;
    cc2420_header_t* m_rx_header = (cc2420_header_t*)get_packet_header(m_rx_buf.p_rx_buf);
    cc2420_header_t* m_tx_header = (cc2420_header_t*)get_packet_header(p_tx_buf);

    fast_read_any( ((uint8_t*)m_rx_header), ACK_LENGTH );

    type = ( m_rx_header->fcf >> IEEE154_FCF_FRAME_TYPE ) & 7;

    if (rtx_status == S_TX_ACK) {
      if ( type == IEEE154_TYPE_ACK
        && m_rx_header->dsn == m_tx_header->dsn ) {
        ((cc2420_metadata_t*)(p_tx_buf->metadata))->ack = TRUE;
        tx_setting->size--;
        if (tx_setting->size != 0) {
          rtx_status = S_TX_SFD;
          tx_counter = 0;
          p_tx_buf = (message_t*)((uint8_t*)p_tx_buf + sizeof(message_t));
          cc2420_load_tx();
        } else if (tx_setting->ci && tx_counter != PREAMBLE_PACKET_LENGTH) {
          rtx_status = S_TX_SFD;
          tx_counter++;
        } else {
          radio_flush_tx();
          rtx_status = S_RX_DETECT;
          TBCCR5 = TBCCR1 + LISTENING_TAIL;
          detect_duration = TBCCR1;
          TBCCTL5 = CCIE;
        }
      } else {
        tx_counter++;        
        if (tx_counter == PREAMBLE_PACKET_LENGTH) {
          tx_setting->size--;
          if (tx_setting->size != 0) {
            rtx_status = S_TX_SFD;
            tx_counter = 0;
            p_tx_buf = (message_t*)((uint8_t*)p_tx_buf + sizeof(message_t));
            cc2420_signal_detect(TBCCR1);
          } else {
            radio_flush_tx();
            rtx_status = S_RX_DETECT;
            TBCCR5 = TBCCR1 + LISTENING_TAIL;
            detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
          }
        } else {
          rtx_status = S_TX_SFD;
          //TODO:modify the preamble dsn
          write_ram(CC2420_RAM_TXFIFO, sizeof(cc2420_header_t)+sizeof(rtx_setting_t), &tx_counter, 1);
        }
      }
    }

    if (rtx_status == S_CI_ACK) {
      // TODO: stop transmission or continously transmission till rereach the maximum counter
      if (tx_counter == PREAMBLE_PACKET_LENGTH) {
        radio_flush_tx();
        rtx_status = S_RX_DETECT;
        TBCCR5 = TBCCR1 + LISTENING_TAIL;
        detect_duration = TBCCR1;
        TBCCTL5 = CCIE;
      } else {
        rtx_status = S_CI_SFD;
        tx_counter++;
        write_ram(CC2420_RAM_TXFIFO, sizeof(cc2420_header_t)+sizeof(rtx_setting_t), &tx_counter, 1);
      }
    }
  }

  static inline void cc2420_ack_rx_except() {
    uint8_t type = 0xff;
    cc2420_header_t* m_rx_header = (cc2420_header_t*)get_packet_header(m_rx_buf.p_rx_buf);
    cc2420_header_t* m_tx_header = (cc2420_header_t*)get_packet_header(p_tx_buf);

    fast_read_any( ((uint8_t*)m_rx_header), ACK_LENGTH );

    type = ( m_rx_header->fcf >> IEEE154_FCF_FRAME_TYPE ) & 7;

    if (rtx_status == S_TX_ACK) {
      if ( type == IEEE154_TYPE_ACK
        && m_rx_header->dsn == m_tx_header->dsn ) {
        ((cc2420_metadata_t*)(p_tx_buf->metadata))->ack = TRUE;
        tx_setting->size--;
        if (tx_setting->size != 0) {
          rtx_status = S_TX_DETECT;
          tx_counter = 0;
          p_tx_buf = (message_t*)((uint8_t*)p_tx_buf + sizeof(message_t));
          cc2420_signal_detect(TBCCR1);          
        } else {
          radio_flush_tx();
          rtx_status = S_RX_DETECT;
          TBCCR5 = TBCCR1 + LISTENING_TAIL;
          detect_duration = TBCCR1;
          TBCCTL5 = CCIE;
        }
      } else {
        tx_counter++;
        write_ram(CC2420_RAM_TXFIFO, sizeof(cc2420_header_t)+sizeof(rtx_setting_t), &tx_counter, 1);
        if (tx_counter == PREAMBLE_PACKET_LENGTH) {
          if (tx_setting->size != 0) {
            rtx_status = S_TX_DETECT;
            tx_counter = 0;
            p_tx_buf = (message_t*)((uint8_t*)p_tx_buf + sizeof(message_t));
            cc2420_signal_detect(TBCCR1);
          } else {
            radio_flush_tx();
            rtx_status = S_RX_DETECT;
            TBCCR5 = TBCCR1 + LISTENING_TAIL;
            detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
          }
        } else {
          rtx_status = S_TX_DETECT;
          cc2420_signal_detect(TBCCR1);
        }
      }
    }

    if (rtx_status == S_CI_ACK) {
      radio_flush_tx();
      rtx_status = S_RX_DETECT;
      TBCCR5 = TBCCR1 + LISTENING_TAIL;
      detect_duration = TBCCR1;
      TBCCTL5 = CCIE;
    }
  }

  command void OppoRouting.setLocalMetric(uint16_t metric) {
    atomic local_metric = metric;
  }

  default event void LplTime.timeRadio(rtx_time_compensation_t* p_rtx_time) {}
  default event void LplTime.timeCompensated(uint16_t time, rtx_time_compensation_t* p_rtx_time) {}
}
