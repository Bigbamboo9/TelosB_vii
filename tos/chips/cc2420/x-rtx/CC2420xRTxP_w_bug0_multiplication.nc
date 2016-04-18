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
  // the number of bytes read from RxFIFO to RAM
  uint8_t rx_readbytes;
  // the rx setting of current receive process
  rtx_setting_t* rx_setting;
  // local opportunistic routing metric
  uint16_t local_metric;
  // whether ack the received packet
  bool ack_mark;
  // buffer control struct
  rx_buffer_t m_rx_buf;
  // message buffer for packets reception
  message_t rx_buf[RX_BUFFER_SIZE];

  // record the dco drift to estimate the rtx sfd delta and make-up waiting time
  // dco_cap is DCO ticks corresponding to 8 32KHz ticks
  uint32_t dco_cap;
  // waiting ticks or temp variable
  uint32_t w_ticks;

  // rssi debug
  uint8_t rssi_print;
  // time interval measurement debug
  uint16_t time_interval;
  uint16_t debug_interval;

  /** Detect the potential ongoing packet transmission by being awake and checking SFD for two adjacent SFD period **/
  /** execute atomicly to avoid the excute tail **/
  static inline void cc2420_signal_detect(uint16_t time);
  /** Start to read data from RxFIFO after the SFD up edge is detected during receiving process **/
  static inline void cc2420_begin_rx();
  /** Finish the lefted data transfer, send ack back and determine the further settings after the SFD down edge is detected during receiving process **/
  static inline void cc2420_end_rx();
  /** Fnish the rest data transfer, since miss the timming, reset the status to receive the following packet **/
  static inline void cc2420_expired_end_rx();
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
  /** calculate the header crc byte by byte **/
  // static inline uint16_t cc2420_header_crc(uint8_t byte, uint16_t crc);
  /** update dco_cap **/
  static inline void dco_drift_update();

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
    rtx_time.ack_time = 807;
    rtx_time.turnaround_time = 1522;
    rtx_time.turnaround_total_time = 0;
    rtx_time.rtx_total_time = 0;
    rtx_time.ack_total_time = 0;
    rtx_time.radio_on_time = 0;
    rtx_time.tail_total_time = 0;
    continuous_duplicate = 0;
    local_metric = 0;
    return SUCCESS;
  }

  async command error_t LplSend.send(message_t* msg, rtx_setting_t* ts) {
    if (rtx_status != S_RTX_IDLE)
      return EBUSY;
    if (msg == NULL)
      return FAIL;
    atomic {
      tx_setting = ts;
      p_tx_buf = msg;
      tx_counter = 0;
      continuous_duplicate = 0;
      rtx_time.pkt_recv = 0;
      rtx_time.pkt_send = 0;
      rtx_time.pkt_ack = 0;
      rtx_time.pkt_turnaround = 0;
      rtx_time.channel_detection = 0;
      rtx_status = S_TX_DETECT;
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;
      // for those early comming rx SFD
      detect_duration = rtimer_arch_now_dco();
      cc2420_rx_start();
      if ( !(TBCCTL1 & CCIFG) )
        // when the radio is in the process of Tx, the Rx is only enable befor channel sampling and disable after the Tx begin
        // cc2420_rx_start();
        cc2420_signal_detect(rtimer_arch_now_dco());
    }
    return SUCCESS;
  }

  async command void LplReceive.rxEnable() {
    // for those early comming rx SFD
    detect_duration = rtimer_arch_now_dco();
    cc2420_rx_start();
  }

  async command void LplReceive.txDetect() {
    if ( !(TBCCTL1 & CCIFG) ) {
      // cc2420_rx_start();
      cc2420_signal_detect(rtimer_arch_now_dco());
    }
  }

  async command error_t LplReceive.rxOn() {
    // executed in atomic
    if ( (rtx_status != S_RTX_IDLE) | (m_rx_buf.occ_size == m_rx_buf.max_size) ) {
      // something bad happened, force to reset and remove all buffered message
      rtx_status = S_RTX_IDLE;
      m_rx_buf.occ_size = 0;
      m_rx_buf.pos_buf = 0;
      return FAIL;
    }
    continuous_duplicate = 0;
    rtx_time.pkt_recv = 0;
    rtx_time.pkt_send = 0;
    rtx_time.pkt_ack = 0;
    rtx_time.pkt_turnaround = 0;
    rtx_time.channel_detection = 0;
    rtx_status = S_RX_DETECT;
    TBCCTL1 = CM_1 | CAP | SCS | CCIE;
    return SUCCESS;
  }

  async command error_t LplReceive.rxInit() {
    // executed in atomic
    m_rx_buf.received = FALSE;
    if (m_rx_buf.occ_size == m_rx_buf.max_size)
      return EBUSY;
    return SUCCESS;
  }

  async command void LplReceive.rxBuffSet() {
    atomic {
      m_rx_buf.pos_buf = (m_rx_buf.pos_buf + m_rx_buf.occ_size) % m_rx_buf.max_size;
      //printf_u8(1,&(m_rx_buf.pos_buf));
      m_rx_buf.p_rx_buf = &rx_buf[m_rx_buf.pos_buf];
      m_rx_buf.occ_size = 0;
    }
  }

  default async event void LplSend.sendDone(message_t* msg, rtx_setting_t* ts, error_t error) { }
  default async event void LplReceive.receive(message_t* msg, uint8_t size) { }

  TOSH_SIGNAL(TIMERB1_VECTOR) {
    // the delta ticks for TB1 campature interrupt
    uint16_t TB1_irq = ( (rtimer_arch_now_dco() - TBCCR1) - 38 ) << 1;
    // the delta ticks for TB2 (ACK) compare interrupt
    uint16_t TB2_irq = ( (rtimer_arch_now_dco() - TBCCR2) - 57 ) << 1;
    // uint16_t TB2_irq = ( (rtimer_arch_now_dco() - TBCCR2) - 53 ) << 1;
    uint8_t debug;
    tbiv = TBIV;
    TBCCTL1 = CAP;
    // tb1_buffer = TBCCR1;
#ifdef RADIO_X_DEBUG
    // printf_u16(1, &tbiv);
#endif
    if (tbiv == 2) {
      // printf_u16(1, &TB1_irq);
#ifdef RADIO_X_DEBUG
      // printf_u16(1, &TB1_irq);
      // debug = 2;
      // printf_u8(1, &debug);
#endif
      // compensate the if-else jump for ACK waiting timer
      if ( ( (rtx_status == S_TX_ACK) || (rtx_status == S_CI_ACK) ) && ( !(P4IN & (1 << 1)) ) ) {
#ifdef RADIO_X_DEBUG
        debug = 11;
        printf_u8(1, &debug);
#endif
        /** ack has been received **/
        ack_duration = TBCCR1 - ack_duration;
        // printf_u16(1, &ack_duration);
        detect_duration = TBCCR1;
        // compensate the if-else jump for RX_ACK
        if ( rtx_status == S_RX_ACK && (!(P4IN & (1 << 1))));
        // turn off the SFD abortion timer
        TBCCTL3 &= ~(CCIE | CCIFG);
        // set interrupt edge, but the new setting of interrupt edge should follow the specific event to avoid the interrupt mistatke
        // need clear the capture edge setting instead of only clear the CCIFG
        // TBCCTL1 = CM_1 | CAP | SCS | CCIE;
        // TBCCTL1 &= ~(CCIE | CCIFG);
        TBCCTL1 = CAP;

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

          // if ( !dco_drift_compensate(t_cap, EXTRA_EXECUTE_TICK_ACK) ) {

          // }

          cc2420_strobe_tx();

          cc2420_ack_rx();
        } else {
          cc2420_ack_rx_except();
        }

        rtx_time.pkt_ack++;
        ack_time_update(&rtx_time, ack_duration);
      } else if ( (rtx_status == S_RX_ACK) && (!(P4IN & (1 << 1)))) {
#ifdef RADIO_X_DEBUG
        debug = 22;
        printf_u8(1, &debug);
#endif
        
        /** ack has been sent **/
        ack_duration = TBCCR1 - ack_duration;
        // printf_u16(1, &ack_duration);
        detect_duration = TBCCR1;

        // turn off the SFD abortion timer
        TBCCTL3 &= ~(CCIE | CCIFG);
        // set interrupt edge
        // TBCCTL1 = CM_1 | CAP | SCS | CCIE;
        // TBCCTL1 &= ~(CCIE | CCIFG);
        TBCCTL1 = CAP;

        // time_interval = rtimer_arch_now_dco() - time_interval;
        // printf_u16(1, &time_interval);

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

          // ack transmition need extra sfd delta compensate
          sfd_delta_compensate();

          // if ( !dco_drift_compensate(t_cap, EXTRA_EXECUTE_TICK_ACK) ) {

          // }

          cc2420_strobe_tx();

          if ( (tx_setting->size != 0) && (p_tx_buf != NULL) && tx_setting->priority) {
            // current transmission hold the high priority, transmit with ci
            rtx_status = S_TX_SFD;
            cc2420_load_tx();
          } else if ((m_rx_buf.received) && (m_rx_buf.occ_size != m_rx_buf.max_size)) {
            // exactly received data packet
            if (rx_setting->ci && rx_setting->hop < CI_HOP_THRESHOLD) {
              // transmit the received data with ci
              rtx_status = S_CI_SFD;
              if (p_tx_buf == NULL) {
                p_tx_buf = m_rx_buf.p_rx_buf;
              } else if (tx_setting->size != 0) {
                // save the p_tx_buf first
                swap_tx_buf = p_tx_buf;
                p_tx_buf = m_rx_buf.p_rx_buf;
              }
              tx_counter = 0;
              cc2420_load_tx();
            } else if (rx_setting->batched) {
              // some packets will be comming soon
              radio_flush_tx();
              while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
              radio_flush_tx();
              rtx_status = S_RX_DETECT;
              // TBCCR5 = TBCCR1 + LISTENING_TAIL;
              TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
              detect_duration = TBCCR1;
              TBCCTL5 = CCIE;
            } else {
              radio_flush_tx();
              while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
              radio_flush_tx();
              rtx_status = S_RX_DETECT;
              atomic {
                // cc2420_rx_start();
                cc2420_signal_detect(TBCCR1);
              }
            }
          } else if ( (tx_setting->size != 0) && (p_tx_buf != NULL) ) {
            // nothing will be received further, start to transmit
            rtx_status = S_TX_SFD;
            cc2420_load_tx();
          } else if (m_rx_buf.occ_size == m_rx_buf.max_size) {
            // no buffer left, turn off the radio process
            radio_flush_tx();
            while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
            radio_flush_tx();
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
            //nothing will be received and transmited, start channel checking
            radio_flush_tx();
            while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
            radio_flush_tx();
            // enable rising edge interrupt capture to receive
            TBCCTL1 = CM_1 | CAP | SCS | CCIE;
            rtx_status = S_RX_DETECT;
            // TBCCR5 = TBCCR1 + LISTENING_TAIL;
            TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
            detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
          }
        } else {
          // TODO: exception handler
          // ignore the CI
          if ( (tx_setting->size != 0) && (p_tx_buf != NULL) && tx_setting->priority) {
            // enable rising edge interrupt capture to receive
            TBCCTL1 = CM_1 | CAP | SCS | CCIE;
            rtx_status = S_TX_DETECT;
            atomic {
              // cc2420_rx_start();
              cc2420_signal_detect(TBCCR1);
            }
          } else if ((m_rx_buf.received) && (m_rx_buf.occ_size != m_rx_buf.max_size)) {
            if (rx_setting->batched) {
              // enable rising edge interrupt capture to receive
              TBCCTL1 = CM_1 | CAP | SCS | CCIE;
              rtx_status = S_RX_DETECT;
              // TBCCR5 = TBCCR1 + LISTENING_TAIL;
              TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
              detect_duration = TBCCR1;
              TBCCTL5 = CCIE;
            } else {
              radio_flush_tx();
              while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
              radio_flush_tx();
              // enable rising edge interrupt capture to receive
              TBCCTL1 = CM_1 | CAP | SCS | CCIE;
              rtx_status = S_RX_DETECT;
              atomic { cc2420_signal_detect(TBCCR1); }
            }
          } else if ( (tx_setting->size != 0) && (p_tx_buf != NULL) ) {
            // enable rising edge interrupt capture to receive
            TBCCTL1 = CM_1 | CAP | SCS | CCIE;
            rtx_status = S_TX_DETECT;
            atomic {
              // cc2420_rx_start();
              cc2420_signal_detect(TBCCR1);
            }
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
            // enable rising edge interrupt capture to receive
            TBCCTL1 = CM_1 | CAP | SCS | CCIE;
            rtx_status = S_RX_DETECT;
            // TBCCR5 = TBCCR1 + LISTENING_TAIL;
            TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
            detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
          }
        }

        rtx_time.pkt_ack++;
#ifdef RADIO_X_DEBUG
        // printf_u16(1, &ack_duration);
#endif
        ack_time_update(&rtx_time, ack_duration);
      } else if ( (rtx_status == S_RX_RECEIVE) && ( !(P4IN & (1 << 1)) ) ) {
#ifdef RADIO_X_DEBUG
        debug = 33;
        printf_u8(1, &debug);
#endif
        /** data packet has been received, wether transmit with CI and ack the packet **/
        // turn off receive execption timer
        rx_duration = TBCCR1 - rx_duration;
        // printf_u16(1, &rx_duration);
        detect_duration = TBCCR1;
        TBCCTL3 &= ~(CCIE | CCIFG);
        // detect_duration = TBCCR1;
        if ( (rtx_status== S_TX_SFD) && (!(P4IN & (1 << 1))) );
        // set interrupt edge
        // TBCCTL1 &= ~(CCIE | CCIFG);
        TBCCTL1 = CAP;
        // TBCCTL1 = CM_1 | CAP | SCS | CCIE;
        // TBCCTL1 = CM_2 | CAP | SCS | CCIE;

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

          // time_interval = rtimer_arch_now_dco();
          cc2420_end_rx();
          // time_interval = rtimer_arch_now_dco() - time_interval;
          // printf_u16(1, &time_interval);

        } else {
          cc2420_expired_end_rx();

          // TBCCR5 = TBCCR1 + LISTENING_TAIL;
          // enable rising edge interrupt to receive
          TBCCTL1 = CM_1 | CAP | SCS | CCIE;
          TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
          detect_duration = TBCCR1;
          TBCCTL5 = CCIE;
        }

        // cc2420_end_rx();
        rtx_time.pkt_recv++;
        rx_time_update(&rtx_time, rx_duration);
        /** debug **
        {
          uint16_t debug_delay = rtimer_arch_now_dco();
          debug_delay = debug_delay - turn_around;
          printf_u16(1, &debug_delay);
        }
        ** debug **/
      } else if ( (rtx_status== S_TX_SFD) && (!(P4IN & (1 << 1))) ) {
#ifdef RADIO_X_DEBUG
        debug = 44;
        printf_u8(1, &debug);
#endif
        /** packet transmission has finished **/
        tx_duration = TBCCR1 - tx_duration;
        // printf_u16(1, &tx_duration);
        detect_duration = TBCCR1;
        // turn off SFD abortion timer
        TBCCTL3 &= ~(CCIE | CCIFG);
        // set interrupt edge
        // TBCCTL1 = CM_1 | CAP | SCS | CCIE;
        // TBCCTL1 &= ~(CCIE | CCIFG);
        TBCCTL1 = CAP;

        // TODO: add compensation of the difference of Tx/Rx SFD interrupt

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

          // cc2420_ack_wait_tx();
          // manually inline
          // if ( !(dco_drift_compensate(t_cap, EXTRA_EXECUTE_TICK_RTX)) ) {
            /*
            rtx_status = S_TX_DETECT;
            TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
            detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
            return;
            */
          // }
  
          w_ticks = EXTRA_EXECUTE_TICK_RTX;
          w_ticks = ( w_ticks - ( (dco_cap * w_ticks) >> 10 ) + 7 ) << 1;

  {
    debug_interval = w_ticks;
    printf_u16(1, &debug_interval);
    debug_interval = w_ticks >> 16;
    printf_u16(1, &debug_interval);
    printf_u8(1, &tx_counter);
  }
    {
          // uint32_t drift_factor = 1020;
          // dco_drift_test(dco_cap, EXTRA_EXECUTE_TICK_RTX);
    }

          w_ticks = (RTX_SFD_DRIFT + FIXED_TURN_AROUND_TICK + FIXED_ACK_TICK) * 1024 / dco_cap;

          detect_duration = rtimer_arch_now_dco() - detect_duration;
          turn_around = rtimer_arch_now_dco();
          // turn_around = TBCCR1;
          // printf_u16(1, &detect_duration);
          
          TBCCTL1 = CM_1 | CAP | SCS | CCIE;
          TBCCR2 = turn_around + w_ticks;
          // TBCCR2 = rtimer_arch_now_dco() + ((RTX_SFD_DRIFT + FIXED_TURN_AROUND_TICK + FIXED_ACK_TICK) * t_cap[1] / t_cap[0]) << 7;
          // TBCCR2 = rtimer_arch_now_dco() + rtx_time.turnaround_time + rtx_time.ack_time;
          TBCCTL2 = CCIE;
          rtx_time.channel_detection += detect_duration;
          rtx_status = S_TX_ACK;
        } else {
          // TODO: detect again
          radio_flush_tx();
          // while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
          // radio_flush_tx();
          // enable rising edge interrupt to capture RTX
          TBCCTL1 = CM_1 | CAP | SCS | CCIE;
          rtx_status = S_TX_DETECT;
          atomic {
            // cc2420_rx_start();
            cc2420_signal_detect(TBCCR1);
          }
        }

        rtx_time.pkt_send++;
        tx_time_update(&rtx_time, tx_duration);
      } else if ( rtx_status == S_CI_SFD && ( !(P4IN & (1 << 1)) ) ) {
#ifdef RADIO_X_DEBUG
        debug = 55;
        printf_u8(1, &debug);
#endif
        /** CI transmission end **/
        detect_duration = TBCCR1;
        // turn off reception exception timer
        TBCCTL3 &= ~(CCIE | CCIFG);

        // TBCCTL1 = CM_1 | CAP | SCS | CCIE;
        // TBCCTL1 &= ~(CCIE | CCIFG);
        TBCCTL1 = CAP;

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

          // cc2420_ack_wait_tx();
          // manually inline
          // if ( !(dco_drift_compensate(t_cap, EXTRA_EXECUTE_TICK_RTX)) ) {
            /*
            rtx_status = S_TX_DETECT;
            TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
            detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
            return;
            */
          // }

          w_ticks = (RTX_SFD_DRIFT + FIXED_TURN_AROUND_TICK + FIXED_ACK_TICK) * 1024 / dco_cap;

          detect_duration = rtimer_arch_now_dco() - detect_duration;
          // printf_u16(1, &detect_duration);
          turn_around = rtimer_arch_now_dco();

          TBCCTL1 = CM_1 | CAP | SCS | CCIE;
          TBCCR2 = turn_around + w_ticks;
          // TBCCR2 = turn_around + rtx_time.turnaround_time + rtx_time.ack_time;
          TBCCTL2 = CCIE;
          rtx_time.channel_detection += detect_duration;
          rtx_status = S_CI_ACK;
        } else {
          radio_flush_tx();
          // while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
          // radio_flush_tx();
          // TODO : possible tx pointer swap
          if ((tx_setting->size != 0) && (swap_tx_buf != NULL)) {
            p_tx_buf = swap_tx_buf;
            swap_tx_buf = NULL;
            rtx_status = S_TX_DETECT;
            TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
            TBCCTL5 = CCIE;
          } else {
            TBCCTL1 = CM_1 | CAP | SCS | CCIE;
            rtx_status = S_RX_DETECT;
            // TBCCR5 = TBCCR1 + LISTENING_TAIL;
            TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
            // detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
          }
        }

        rtx_time.pkt_send++;
      } else if ( (rtx_status == S_RX_DETECT || rtx_status == S_TX_DETECT) && (P4IN & (1 << 1)) ) {
#ifdef RADIO_X_DEBUG
        debug = 66;
        printf_u8(1, &debug);
#endif
        /** data packet prepare to receive and start reception exception timer **/
        // turn off the detection waiting timer
        detect_duration = TBCCR1 - detect_duration;
        rx_duration = TBCCR1;

        TBCCTL5 &= ~(CCIE | CCIFG);

        // decline the rx when the irs delay is up to 1024
        if (TB1_irq > 1024) {
          radio_flush_rx();
          rtx_status = S_RX_DETECT;
          // TBCCR5 = TBCCR1 + LISTENING_TAIL;
          TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
          rtx_time.channel_detection += detect_duration;
          detect_duration = TBCCR1;
          TBCCTL5 = CCIE;
        } else {
          TBCCTL1 = CM_2 | CAP | SCS | CCIE;
          /**According to the Glossy, Rx timeout : packet duration + 200 us packet duration : 32 us * packet_length, 1 DCO tick ~ 1 us**/
          TBCCR3 = TBCCR1 + ( pkt_length * 32 + 200 ) * 4;
          abortion_duration = TBCCR1;
          // TBCCTL3 &= ~CCIFG;
          TBCCTL3 = CCIE;

          cc2420_begin_rx();

          rtx_time.channel_detection += detect_duration;
          // turnaround time is already included in detection time!
        }
      } else if ( (rtx_status == S_RX_ACK || rtx_status == S_TX_ACK || rtx_status == S_CI_ACK) && (P4IN & (1 << 1)) ) {
#ifdef RADIO_X_DEBUG
        debug = 77;
        printf_u8(1, &debug);
#endif
        /** transmissin of ack starts**/
        // turn off ack waiting timer
        turn_around = TBCCR1 - turn_around;
        ack_duration = TBCCR1;

        // printf_u16(1, &turn_around);
        // printf_u16(1, &TB1_irq);

        // make sure the received ack is a valid one.
        if (turn_around < 1500 || turn_around > 1600) {
          // something smell bad, flush RXFIFO and wait ack timer fire
          radio_flush_rx();
          // clear and reenable interrupt capture
          // TBCCTL1 &= ~(CCIE | CCIFG);
          TBCCTL1 = CM_1 | CAP | SCS | CCIE;
        } else {

          TBCCTL2 &= ~(CCIE | CCIFG);

          // turn on SFD abortion timer
          TBCCR3 = TBCCR1 + ( ACK_LENGTH * 32 + 200 ) * 4;
          abortion_duration = TBCCR1;
          // TBCCTL3 &= ~CCIFG;
          TBCCTL3 = CCIE;

          // measure the DCO drift
          /*
          {
            uint16_t t_cap[2];
            uint16_t t_next_cap[2];
            uint8_t i;
            CAPTURE_NEXT_CLOCK_TICK(t_cap[0], t_cap[1]);
            TACCTL2 = CCIS0 | CM_1 | CAP | SCS;
            for (i = 0; i < 7; i++) {
              TACCTL2 &= ~CCIFG;
              while (!(TACCTL2 & CCIFG));
            }
            TACCTL2 = 0;
            CAPTURE_NEXT_CLOCK_TICK(t_next_cap[0], t_next_cap[1]);
            t_cap[0] = t_next_cap[0] - t_cap[0];
            t_cap[1] = t_next_cap[1] - t_cap[1];
            printf_u16(2, t_cap);
          }

          TBCCTL1 = CAP;
          */
          TBCCTL1 = CM_2 | CAP | SCS | CCIE;

#ifdef RADIO_X_DEBUG
{
        // uint16_t debug_ta = rtx_time.turnaround_time;
        // uint16_t debug_ad = rtx_time.ack_time;
        // printf_u16(1, &turn_around);
        // printf_u16(1, &debug_ta);
        // printf_u16(1, &debug_ad);
}
#endif

          turnaround_time_update(&rtx_time, turn_around);
          rtx_time.pkt_turnaround++;
        }
      } else if ( rtx_status == S_CI_SFD && (P4IN & (1 << 1))) {
#ifdef RADIO_X_DEBUG
        debug = 88;
        printf_u8(1, &debug);
#endif
        // TODO : turn_arround calc
        /** CI transmission start **/
        TBCCTL1 = CM_2 | CAP | SCS | CCIE;

        TBCCR3 = TBCCR1 +  ( pkt_length * 32 + 200 ) * 4;
        abortion_duration = TBCCR1;
        // TBCCTL3 &= ~CCIFG;
        TBCCTL3 = CCIE;

        rtx_time.pkt_turnaround++;
      } else if ( (rtx_status == S_TX_SFD) && (P4IN & (1 << 1))) {
#ifdef RADIO_X_DEBUG
        debug = 99;
        printf_u8(1, &debug);
#endif
        // turn_around = TBCCR1 - turn_around;
        // printf_u16(1, &turn_around);
        /** packet transmission start **/
        tx_duration = TBCCR1;

        TBCCTL1 = CM_2 | CAP | SCS | CCIE;

        TBCCR3 = TBCCR1 +  ( pkt_length * 32 + 200 ) * 4;
        abortion_duration = TBCCR1;
        // TBCCTL3 &= ~CCIFG;
        TBCCTL3 = CCIE;

        // update the dco drift for later waiting time calibration
        dco_drift_update();

        rtx_time.pkt_turnaround++;
      } else {
      // error status, abort current process
        radio_flush_tx();
        // while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
        // radio_flush_tx();
        radio_flush_rx();
        rtx_status = S_RX_DETECT;
        TBCCTL1 = CM_1 | CAP | SCS | CCIE;
        // TBCCR5 = TBCCR1 + LISTENING_TAIL;
        TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
        detect_duration = TBCCR1;
        TBCCTL5 = CCIE;
      }
    } else {
      switch(tbiv) {
        case 4:
          detect_duration = TBCCR2;
          TBCCTL2 &= ~(CCIE | CCIFG);
          // TBCCTL1 &= ~(CCIE | CCIFG);
          TBCCTL1 = CAP;
          // printf_u16(1, &TB2_irq);
        #ifdef RADIO_X_DEBUG
          // printf_u16(1, &TB2_irq);
          debug = 4;
          printf_u8(1, &debug);
        #endif
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

            // if ( !dco_drift_compensate(t_cap, EXTRA_EXECUTE_TICK_ACK) ) {

            // }

            /**
             * strobe TX when S_RX_ACK might hamper the radio to receive the SFD reception of new packet
             **/
            cc2420_strobe_tx();
            // enable interrupt capture
            // TBCCTL1 = CM_1 | CAP | SCS | CCIE;

            if (rtx_status == S_RX_ACK) {
            // does not successful or need to ack the received data
              if ( (tx_setting->size != 0) && (p_tx_buf != NULL) ) {
                rtx_status = S_TX_SFD;
                // cc2420_strobe_tx();
                cc2420_load_tx();
                tx_counter = 0;
              } else {
                radio_flush_tx();
                while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
                radio_flush_tx();
                if (continuous_duplicate > DUPLICATE_COUNT) {
                  // TBCCTL1 &= ~(CCIE | CCIFG);
                  TBCCTL1 = CAP;
                  rtx_status = S_RTX_IDLE;
                  // adding small tail to end the radio process
                  TBCCR5 = rtimer_arch_now_dco() + 128;
                  // detect_duration = TBCCR2;
                  TBCCTL5 = CCIE;
                } else {
                  rtx_status = S_RX_DETECT;
                //{
                //  uint16_t debug_delay = rtimer_arch_now_dco();
                //  debug_delay = debug_delay - TBCCR2;
                //  printf_u16(1, &debug_delay);
                //}
                // TBCCR5 = TBCCR2 + LISTENING_TAIL;
                  TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
                  // detect_duration = TBCCR2;
                  TBCCTL5 = CCIE;
                }
              }
            } else if (rtx_status == S_TX_ACK) {
              // if (p_tx_buf != NULL) {
              rtx_status = S_TX_SFD;
              // cc2420_strobe_tx();
              // }
              tx_counter++;
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
                  while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
                  radio_flush_tx();
                  rtx_status = S_TX_DETECT;
                  // TBCCR5 = TBCCR2 + LISTENING_TAIL;
                  TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
                  detect_duration = TBCCR2;
                  TBCCTL5 = CCIE;
                }
              }
            } else if (rtx_status == S_CI_ACK) {
              // if (p_tx_buf != NULL) {
              rtx_status = S_CI_SFD;
              // cc2420_strobe_tx();
              // }
              tx_counter++;
              write_ram(CC2420_RAM_TXFIFO, sizeof(cc2420_header_t)+sizeof(rtx_setting_t), &tx_counter, 1);
              if (tx_counter == PREAMBLE_PACKET_LENGTH) {
                if ((tx_setting->size != 0) && (swap_tx_buf != NULL)) {
                  rtx_status = S_TX_SFD;
                  p_tx_buf = swap_tx_buf;
                  cc2420_load_tx();
                  swap_tx_buf = NULL;
                  tx_counter = 0;
                } else {
                  radio_flush_tx();
                  while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
                  radio_flush_tx();
                  rtx_status = S_RX_DETECT;
                  // TBCCR5 = TBCCR2 + LISTENING_TAIL;
                  TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
                  detect_duration = TBCCR2;
                  TBCCTL5 = CCIE;
                }
              }
            } else {
              // TODO: exception handling
              radio_flush_tx();
              while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
              radio_flush_tx();
              if (continuous_duplicate > DUPLICATE_COUNT) {
                // TBCCTL1 &= ~(CCIE | CCIFG);
                TBCCTL1 = CAP;
                rtx_status = S_RTX_IDLE;
                // adding small tail to end the radio process
                TBCCR5 = rtimer_arch_now_dco() + 128;
                TBCCTL5 = CCIE;
              } else {
                rtx_status = S_RX_DETECT;
                TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
                TBCCTL5 = CCIE;
              }
            }
          } else {
            if ( (tx_setting->size != 0) && (p_tx_buf != NULL) ) {
              /**
               * no synchronous transmission, should not transmit further
              cc2420_strobe_tx();
               */
              tx_counter++;
              if (tx_counter == PREAMBLE_PACKET_LENGTH) {
                tx_setting->size--;
                if (tx_setting->size != 0) {
                  p_tx_buf = (message_t*)((uint8_t*)p_tx_buf + sizeof(message_t));
                  tx_counter = 0;
                  // enable interrupt capture
                  TBCCTL1 = CM_1 | CAP | SCS | CCIE;
                  rtx_status = S_TX_DETECT;
                  atomic {
                    cc2420_signal_detect(TBCCR2);
                  }
                } else {
                  // radio_flush_tx();
                  // enable interrupt capture
                  TBCCTL1 = CM_1 | CAP | SCS | CCIE;
                  rtx_status = S_TX_DETECT;
                  // TBCCR5 = TBCCR2 + LISTENING_TAIL;
                  TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
                  detect_duration = TBCCR2;
                  TBCCTL5 = CCIE;
                }
              } else {
                // radio_flush_tx();
                // if (strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)) {
                //   printf_u8(1, &tx_counter);
                // }
                rtx_status = S_TX_DETECT;
                // tx_counter = 0;
                atomic {
                  // cc2420_rx_start();
                  cc2420_signal_detect(TBCCR2);
                }
              }
            } else {
              // radio_flush_tx();
              // enable interrupt capture
              TBCCTL1 = CM_1 | CAP | SCS | CCIE;
              rtx_status = S_RX_DETECT;
              // TBCCR5 = TBCCR2 + LISTENING_TAIL;
              TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
              detect_duration = TBCCR2;
              TBCCTL5 = CCIE;
            }
          }

          rtx_time.pkt_ack++;
          rtx_time.pkt_turnaround++;
          break;
        case 6:
        #ifdef RADIO_X_DEBUG
           debug = 6;
           printf_u8(1, &debug);
/*{
           uint16_t register_debug = TBCCTL3;
           printf_u16(1, &register_debug);
}*/
        #endif
          // TBCCTL1 &= ~(CCIE | CCIFG);
          TBCCTL1 = CAP;
          TBCCTL3 &= ~(CCIE | CCIFG);
        // SFD abortion or reception exception
          radio_flush_tx();
          // while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
          // radio_flush_tx();
          radio_flush_rx();
          if ( (tx_setting->size != 0) && (p_tx_buf != NULL) ) {
            rtx_status = S_TX_DETECT;
          } else {
            rtx_status = S_RX_DETECT;
          }
          abortion_duration = TBCCR3 - abortion_duration;
          rtx_time.channel_detection += abortion_duration;
          TBCCTL1 = CM_1 | CAP | SCS | CCIE;
          // TBCCR5 = TBCCR3 + LISTENING_TAIL;
          TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
          detect_duration = TBCCR3;
          TBCCTL5 = CCIE;
          break;
        case 10:
        #ifdef RADIO_X_DEBUG
           debug = 10;
           printf_u8(1, &debug);
        #endif
        // disable timer first
          TBCCTL1 &= ~(CCIE | CCIFG);
          TBCCTL2 &= ~(CCIE | CCIFG);
          TBCCTL3 &= ~(CCIE | CCIFG);
          TBCCTL5 &= ~(CCIE | CCIFG);
          radio_flush_rx();
        // data frame is pending
          if ( (tx_setting->size != 0) && (rtx_status == S_TX_DETECT) ) {
            // enable interrupt capture
            // TBCCTL1 = CM_1 | CAP | SCS | CCIE;
            rtx_status = S_TX_SFD;
            cc2420_strobe_tx();
            cc2420_load_tx();
            return;
          }
          // TBCCTL1 &= ~(CCIE | CCIFG);
          // TBCCTL2 &= ~(CCIE | CCIFG);
          // TBCCTL3 &= ~(CCIE | CCIFG);
          // radio_flush_rx();
        // no packet in channel is detected
          detect_duration = TBCCR5 - detect_duration;
          rtx_time.channel_detection += detect_duration;
          rtx_status = S_RTX_IDLE;
          TBCCTL1 = CAP;
          signal LplTime.timeRadio(&rtx_time);
          if (m_rx_buf.received) {
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
    for (i = 0; (i < SIGNAL_DETECT_PERIOD) && (!(TBCCTL1 & CCIFG)); i++) {
      // atomic { rssi_list[i] = cc2420_get_rssi(); }
      rssi_list[i] = cc2420_get_rssi();
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
    if ( (pos_number > RSSI_UP_THRESHOLD) && (!(TBCCTL1 & CCIFG)) ) {
      int pos_avr = pos_sum / pos_number;
      // printf_int(1, &pos_avr);
      // printf_int(1, &rssi_max);
      // PAPR is small so it is probably ZigBee. Start the tail timer.
      if (rssi_max - pos_avr < RSSI_PAPR_THRESHOLD) {
        TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
        TBCCTL5 = CCIE;
        return;
      }
      if ( rtx_status == S_TX_DETECT ) {
        // the rx should be disabled to eliminate the influence of received signal interrupt
        // cc2420_rx_stop();
        rtx_status = S_TX_SFD;
        rtx_time.channel_detection += rtimer_arch_now_dco() - detect_duration;
        cc2420_strobe_tx();
        cc2420_load_tx();
        return;
      }
      if ( rtx_status == S_RX_DETECT ) {
/*
        TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
        TBCCTL5 = CCIE;
        return;
*/
        //TODO: signal LPL layer to sleep
        detect_duration = rtimer_arch_now_dco() - detect_duration;
        rtx_time.channel_detection += detect_duration;
        rtx_status = S_RTX_IDLE;
        // TBCCTL1 &= ~(CCIE | CCIFG);
        TBCCTL1 = CAP;
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
      }
    }
    // update the noise floor
    if ( !(TBCCTL1 & CCIFG) ) {
      noise_floor_sum = noise_floor_sum / noise_number;
      noise_floor = (8 * noise_floor + 2 * noise_floor_sum) / 10;
      // printf_int(1, &noise_floor);
      if ( rtx_status == S_TX_DETECT ) {
        // cc2420_rx_stop();
        rtx_status = S_TX_SFD;
        rtx_time.channel_detection += rtimer_arch_now_dco() - detect_duration;
        cc2420_strobe_tx();
        cc2420_load_tx();
        return;
      }
      if ( rtx_status == S_RX_DETECT) {
        //TODO: signal LPL layer to sleep
        detect_duration = rtimer_arch_now_dco() - detect_duration;
        rtx_time.channel_detection += detect_duration;
        rtx_status = S_RTX_IDLE;
        // TBCCTL1 &= ~(CCIE | CCIFG);
        TBCCTL1 = CAP;
        signal LplTime.timeRadio(&rtx_time);
        if (m_rx_buf.received) {
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

  static inline void cc2420_begin_rx() {
    uint8_t* p_header = get_packet_header( m_rx_buf.p_rx_buf );
    uint16_t crc = 0xFFFF;
    uint8_t crc_header_size = CC2420_X_HEADER_LENGTH - 4;
    rtx_status = S_RX_RECEIVE;

    while (((P1IN & (1 << 3)) == 0)) {
      if (TBCCTL3 & CCIFG) {
        radio_flush_rx();
        TBCCTL1 = CAP;
        return;
      }
    }

    fast_read_one( p_header );

    // printf_u8(1, p_header);

    if (p_header[0] != pkt_length) {
      /** debug **
      uint8_t read_debug = 11;
      while (read_debug) {
        while (((P1IN & (1 << 3)) == 0));
        fast_continue_read_one( p_header + rx_readbytes );
        printf_u8(1, p_header+rx_readbytes);
        read_debug--;
        rx_readbytes++;
      }
      ** debug **/
      TBCCTL3 &= ~(CCIFG | CCIE);
      radio_flush_rx();
      rtx_status = S_RX_DETECT;
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;
      // TBCCR5 = TBCCR1 + LISTENING_TAIL;
      TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
      detect_duration = TBCCR1;
      TBCCTL5 = CCIE;
      return;
    }

    // dco drift update for the waiting time calibration
    dco_drift_update();

    rx_readbytes = 1;

    // leave enough time to mitigate the influence of atomic execution in cc2420_signal_detect()
    while (rx_readbytes <= pkt_length - 4) {

      while (((P1IN & (1 << 3)) == 0)) {
        if (TBCCTL3 & CCIFG) {
          radio_flush_rx();
          TBCCTL1 = CAP;
          return;
        }
      }

      fast_continue_read_one( p_header + rx_readbytes );

      if (rx_readbytes <= crc_header_size) {
        uint8_t tmp_val = *(p_header + rx_readbytes);
        uint8_t i;
        crc = crc ^ (tmp_val << 8);
        for(i = 0; i < 8; i++) {
          if (crc & 0x8000)
            crc = (crc << 1) ^ CRC_POLY;
          else
            crc <<= 1;
        }
        crc &= 0xFFFF;
        if (rx_readbytes == crc_header_size)
          crc ^= 0x0000;
      }

      rx_readbytes++;

      /** deal with the packet header here **/
      if (rx_readbytes == CC2420_X_HEADER_LENGTH) {
        rtx_setting_t* m_ptr_setting = (rtx_setting_t*)get_packet_setting(m_rx_buf.p_rx_buf);
        // receive the data, acknowledge the packet, else abort the ongoing ack transmission
        if ( (!m_ptr_setting->ack)
         && (m_ptr_setting->addr != TOS_NODE_ID)
         && (! ((m_ptr_setting->addr == OPPORTUNISTIC_ROUTING_ADDR)
             && (m_ptr_setting->metric - local_metric > m_ptr_setting->progress))) ) {
          ack_mark = FALSE;
        } else {
          ack_mark = TRUE;
        }
        if (crc != m_ptr_setting->crc) {
          ack_mark = FALSE;
          printf_u16(1, &crc);
        }
        //  else {
        //    printf_u16(1, &crc);
        //  }
      }
    }
    CC2420_SPI_DISABLE();
  }

  static inline void cc2420_end_rx() {
    uint8_t i;
    bool duplicate = FALSE;
    uint8_t* p_buf = get_packet_header(m_rx_buf.p_rx_buf);

    rtx_status = S_RX_ACK;

    // fast_continue_read_tail( p_buf+rx_readbytes, pkt_length-rx_readbytes+1 );
    fast_read_any( p_buf+rx_readbytes, pkt_length-rx_readbytes+1 );
    rx_readbytes = pkt_length + 1;

    if (p_buf[pkt_length] >> 7) {
      rtx_setting_t* m_ptr_setting = (rtx_setting_t*)get_packet_setting(m_rx_buf.p_rx_buf);

      if (!ack_mark) {
        // no ack transmission triggered
        // radio_flush_tx();
        if (m_ptr_setting->addr != AM_BROADCAST_ADDR) {
          // overheard packet
          signal Snoop.receive(m_rx_buf.p_rx_buf, (void*)get_packet_payload(m_rx_buf.p_rx_buf), pkt_length);
          radio_flush_rx();
          return;
        }
        // the ack waiting timer is running, deal with this after ACK is transmitted
      }
      /* else {
        // strobe( CC2420_SACK );
        if (strobe( CC2420_SNOP ) & BV(CC2420_TX_ACTIVE)) {
          // while (!(P4IN & (1 << 1)));
          debug_interval = rtimer_arch_now_dco();
          while (!(TBCCTL1 & CCIFG));
          debug_interval = rtimer_arch_now_dco() - debug_interval;
          time_interval = rtimer_arch_now_dco();
          while (P4IN & (1 << 1));
          time_interval = rtimer_arch_now_dco() - time_interval;
          printf_u16(1, &debug_interval);
          printf_u16(1, &time_interval);
        }
      }*/

      //** Too long to execute here, no left time to terminate the ACK transmission
      // receive the data, acknowledge the packet, else abort the ongoing ack transmission

      /**
      if ((!m_ptr_setting->ack)
       || ( (m_ptr_setting->addr != TOS_NODE_ID)
         && ( (m_ptr_setting->addr != OPPORTUNISTIC_ROUTING_ADDR)
           || (m_ptr_setting->metric - local_metric < m_ptr_setting->progress)))
       || (!(m_ptr_setting->ci && (m_ptr_setting->hop != 0)))) {
        radio_flush_tx();
        if (m_ptr_setting->addr != AM_BROADCAST_ADDR) {
          // overheard packet
          signal Snoop.receive(m_rx_buf.p_rx_buf, (void*)get_packet_payload(m_rx_buf.p_rx_buf), pkt_length);
          radio_flush_rx();
          return;
        }
        // the ack waiting timer is running, deal with this after ACK is transmitted
      }
      **/

      // duplicate check
      for (i = 0; i < m_rx_buf.occ_size; i++) {
        cc2420_header_t* p_header = (cc2420_header_t*)get_packet_header(&rx_buf[(m_rx_buf.pos_buf + i) % RX_BUFFER_SIZE]);
        if ((((cc2420_header_t*)p_buf)->dsn == p_header->dsn) && (((cc2420_header_t*)p_buf)->src == p_header->src)) {
          duplicate = TRUE;
          break;
        }
      }
      if (!duplicate) {
        // clear the continuous duplicate counter
        continuous_duplicate = 0;
        // set the rx setting
        m_ptr_setting->hop--;
        /** too long to hard copy the setting
        memcpy(&rx_setting, m_ptr_setting, sizeof(rtx_setting_t));
        **/
        rx_setting = m_ptr_setting;
        // successfully receive one data packet
        m_rx_buf.received = TRUE;
        // buffer swapping
        m_rx_buf.occ_size++;
        if (m_rx_buf.occ_size != m_rx_buf.max_size) {
          m_rx_buf.p_rx_buf = &rx_buf[(m_rx_buf.pos_buf + m_rx_buf.occ_size) % RX_BUFFER_SIZE];
          /** deal with this after ACK is transmitted **/
        } else {
          // no extra buffer for further data packet receiving, deal with this after ACK is transmitted
        }
      } else {
        // printf_u8(1, &continuous_duplicate);
        continuous_duplicate++;
        // if (continuous_duplicate > DUPLICATE_COUNT) {
        //  TBCCTL1 &= ~(CCIE | CCIFG);
        //  TBCCTL1 = CAP;
        // }
        // received duplicate
        signal Snoop.receive(m_rx_buf.p_rx_buf, (void*)get_packet_payload(m_rx_buf.p_rx_buf), pkt_length);
      }
      // flush the remained unkown bytes in the RxBuffer
      // radio_flush_rx();
    } else {
      radio_flush_tx();
      // while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
      // radio_flush_tx();
      radio_flush_rx();
      // the ack waiting timer is running, deal with this after ACK is transmitted
    }
  }

  static inline void cc2420_expired_end_rx() {
    uint8_t i;
    bool duplicate = FALSE;
    uint8_t* p_buf = get_packet_header(m_rx_buf.p_rx_buf);

    // miss the time to align the ack timming, reset the status to detect the following pacekt
    // rtx_status = S_RX_ACK;
    rtx_status = S_RX_DETECT;

    fast_continue_read_tail( p_buf+rx_readbytes, pkt_length-rx_readbytes+1 );
    rx_readbytes = pkt_length + 1;

    // since without ack, drop all packets
    // signal Snoop.receive(m_rx_buf.p_rx_buf, (void*)get_packet_payload(m_rx_buf.p_rx_buf), pkt_length);
  }

  static inline void cc2420_ack_strobe_rx() {
    /** rx not end, should not flush RxFIFO **/
    // radio_flush_rx();
    // TBCCTL1 &= ~CCIFG;
    
    // if ( !dco_drift_compensate(t_cap, EXTRA_EXECUTE_TICK_RTX) ) {
      /*
      rtx_status = S_RX_DETECT;
      TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
      detect_duration = TBCCR1;
      TBCCTL5 = CCIE;
      return;
      */
    // }

    w_ticks = (FIXED_TURN_AROUND_TICK + FIXED_ACK_TICK) * 1024 / dco_cap;

    detect_duration = rtimer_arch_now_dco() - detect_duration;
    turn_around = rtimer_arch_now_dco();
    // turn_around = TBCCR1;
    // printf_u16(1, &detect_duration);
    
    if (ack_mark) {
      // turn_around = rtimer_arch_now_dco();
/*
      asm volatile ("nop");
      asm volatile ("nop");
      asm volatile ("nop");
      asm volatile ("nop");
      asm volatile ("nop");
      asm volatile ("nop");
      asm volatile ("nop");
      asm volatile ("nop");
*/
      strobe( CC2420_SACK );
      // enable rising edge interrupt capture to receive ACK
      TBCCTL1 = CM_1 | CAP | SCS | CCIE;
/*
      {
        uint8_t debug_mark = 222;
        printf_u8(1, &debug_mark);
      }
*/
    }
    // radio_flush_tx();
#ifdef RADIO_X_DEBUG
    // time_interval = rtimer_arch_now_dco();
    // while (!(P4IN & (1 << 1)));
    // while ((P4IN & (1 << 1)));
    // time_interval = rtimer_arch_now_dco() - time_interval;
    // printf_u16(1, &time_interval);
#endif
    // if (strobe( CC2420_SNOP) & BV(CC2420_TX_ACTIVE))
    // call Leds.led1On();

    // dco drift update for the waiting time calibration
    // dco_drift_update(t_cap);

    TBCCR2 = turn_around + w_ticks;
    // TBCCR2 = turn_around + ((FIXED_TURN_AROUND_TICK + FIXED_ACK_TICK) * t_cap[1] / t_cap[0]) << 7;
    // TBCCR2 = turn_around + rtx_time.turnaround_time + rtx_time.ack_time;
    // TBCCR2 = turn_around + 4096;
    TBCCTL2 = CCIE;
    rtx_time.channel_detection += detect_duration;
  }

  static inline void cc2420_load_tx() {
    fast_write_any( get_packet_header(p_tx_buf), pkt_length );
    if (strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)) {
      radio_flush_tx();
      // TODO : exception of radio tx
      rtx_status = S_TX_DETECT;
      TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
      detect_duration = rtimer_arch_now_dco();
      TBCCTL5 = CCIE;
    }
  }

  static inline void cc2420_strobe_tx() {
    // avoid the concurrent rx processing, this should not be happened
    // radio_flush_rx();
    // TBCCTL1 &= ~CCIFG;
    detect_duration = rtimer_arch_now_dco() - detect_duration;
    // turn_around = rtimer_arch_now_dco();
    strobe( CC2420_STXON );
    // enable rising edge interrupt capture
    TBCCTL1 = CM_1 | CAP | SCS | CCIE;
    rtx_time.channel_detection += detect_duration;
  }

  static inline void cc2420_ack_wait_tx() {
    // avoid the concurrent rx processing, this should not be happened
    // radio_flush_rx();
    // TBCCTL1 &= ~CCIFG;
    detect_duration = rtimer_arch_now_dco() - detect_duration;
    turn_around = rtimer_arch_now_dco();
    // printf_u16(1, &detect_duration);
    TBCCTL1 = CM_1 | CAP | SCS | CCIE;
    TBCCR2 = turn_around + rtx_time.turnaround_time + rtx_time.ack_time;
    // TBCCR2 = turn_around + 4096;
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
          while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
          radio_flush_tx();
          rtx_status = S_TX_DETECT;
          // TBCCR5 = TBCCR1 + LISTENING_TAIL;
          TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
          detect_duration = TBCCR1;
          TBCCTL5 = CCIE;
        }
      } else {
        tx_counter++;
        if (tx_counter == PREAMBLE_PACKET_LENGTH) {
          tx_setting->size--;
          if (tx_setting->size != 0) {
            rtx_status = S_TX_DETECT;
            tx_counter = 0;
            p_tx_buf = (message_t*)((uint8_t*)p_tx_buf + sizeof(message_t));
            atomic {
              // open the radio to do channel sampling
              // cc2420_rx_start();
              cc2420_signal_detect(TBCCR1);
            }
          } else {
            radio_flush_tx();
            while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
            radio_flush_tx();
            rtx_status = S_RX_DETECT;
            // TBCCR5 = TBCCR1 + LISTENING_TAIL;
            TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
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
      // TODO: stop transmission or continuously transmission till rereach the maximum counter
      if (tx_counter == PREAMBLE_PACKET_LENGTH) {
        radio_flush_tx();
        while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
        radio_flush_tx();
        rtx_status = S_RX_DETECT;
        // TBCCR5 = TBCCR1 + LISTENING_TAIL;
        TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
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
          atomic {
            // enable rx to do channel sampling
            // cc2420_rx_start();
            cc2420_signal_detect(TBCCR1);
          }
        } else {
          radio_flush_tx();
          // while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
          // radio_flush_tx();
          rtx_status = S_RX_DETECT;
          // TBCCR5 = TBCCR1 + LISTENING_TAIL;
          TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
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
            atomic {
              // cc2420_rx_start();
              cc2420_signal_detect(TBCCR1);
            }
          } else {
            radio_flush_tx();
            // while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
            // radio_flush_tx();
            rtx_status = S_RX_DETECT;
            // TBCCR5 = TBCCR1 + LISTENING_TAIL;
            TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
            detect_duration = TBCCR1;
            TBCCTL5 = CCIE;
          }
        } else {
          rtx_status = S_TX_DETECT;
          tx_counter = 0;
          atomic {
            // cc2420_rx_start();
            cc2420_signal_detect(TBCCR1);
          }
        }
      }
    }

    if (rtx_status == S_CI_ACK) {
      radio_flush_tx();
      // while (!(strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW)));
      // radio_flush_tx();
      rtx_status = S_RX_DETECT;
      // TBCCR5 = TBCCR1 + LISTENING_TAIL;
      TBCCR5 = rtimer_arch_now_dco() + LISTENING_TAIL;
      detect_duration = TBCCR1;
      TBCCTL5 = CCIE;
    }
  }

  static inline void dco_drift_update() {
  uint32_t next_dco_cap;
  uint8_t i;

  CAPTURE_NEXT_CLOCK_TICK(dco_cap, w_ticks);

  // keep the duration for 7 bytes
  TACCTL2 = CCIS0 | CM_1 | CAP | SCS;
  for (i = 0; i < 7; i++) {
    TACCTL2 &= ~CCIFG;
    while (!(TACCTL2 & CCIFG));
  }
  TACCTL2 = 0;

  CAPTURE_NEXT_CLOCK_TICK(next_dco_cap, w_ticks);
  
  dco_cap = next_dco_cap - dco_cap;
}

  command void OppoRouting.setLocalMetric(uint16_t metric) {
    atomic local_metric = metric;
  }

  default async event void LplTime.timeRadio(rtx_time_compensation_t* p_rtx_time) {}
  default async event void LplTime.timeCompensated(uint16_t time, rtx_time_compensation_t* p_rtx_time) {}

  default event message_t* Snoop.receive(message_t* msg, void* payoload, uint8_t len) { return msg; }
}
