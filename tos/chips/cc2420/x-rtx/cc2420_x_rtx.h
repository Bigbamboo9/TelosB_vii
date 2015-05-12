#ifndef _CC2420_X_RTX_H_
#define _CC2420_X_RTX_H_

#include "AM.h"
#include "message.h"
#include "CC2420.h"

/** Get the realtime DCO counter **/
#define rtimer_arch_now_dco()  (TBR)
/** Maximum size of data receiving buffer **/
#define RX_BUFFER_SIZE         7
/** Length of IEEE 802.15.4 ACK frame **/
#define ACK_LENGTH             5
#define OPPORTUNISTIC_ROUTING_ADDR 0xFFFE
/** Interval between two adjacent preamble packets transmission **/
/** 128 us turnaround + 160 us 5 bytes data transmission **/
#define ACK_WAITING_PERIOD     288
/** Maximum number of the packets in preamble **/
#define PREAMBLE_PACKET_LENGTH 64   // when packet size is 64 bytes and sleep period is 128 ms
/** Maximum number of RSSI sampling (128 + 160 + 128) / 32 + 9 **/
#define SIGNAL_DETECT_PERIOD   22
/** Tail listening period **/
#define LISTENING_TAIL         288
/** Radio keep awake only when the number of rssi samples that are 3dBm larger than the noisefloor is larger than this threshold **/
#define RSSI_UP_THRESHOLD      7
/** Rssi PAPR threshold **/
#define RSSI_PAPR_THRESHOLD    2
/** CI Hop threshold **/
#define CI_HOP_THRESHOLD       7
/* DELTA_2 assumes an ACLK of 32768 Hz */
#define DELTA_II               ((MSP430_CPU_SPEED) / 32768)


/* -------------------------- Clock Capture ------------------------- */
/**
 * \brief Capture next low-frequency clock tick and DCO clock value at that instant.
 * \param t_cap_h variable for storing value of DCO clock value
 * \param t_cap_l variable for storing value of low-frequency clock value
 */
#define CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l) do {\
                /* Enable capture mode for timers B6 and A2 (ACLK) */\
                TBCCTL6 = CCIS0 | CM_1 | CAP | SCS; \
                TACCTL2 = CCIS0 | CM_1 | CAP | SCS; \
                /* Wait until both timers capture the next clock tick */\
                while (!((TBCCTL6 & CCIFG) && (TACCTL2 & CCIFG))); \
                /* Store the capture timer values */\
                t_cap_h = TBCCR6; \
                t_cap_l = TACCR2; \
                /* Disable capture mode */\
                TBCCTL6 = 0; \
                TACCTL2 = 0; \
} while (0)

typedef enum {
  S_RTX_IDLE,
  /** Tx States **/
  S_TX_DETECT,
  S_TX_SFD,
  S_TX_ACK,
  /** Rx States **/
  S_RX_DETECT,
  S_RX_RECEIVE,
  S_RX_ACK,
  /** CI States **/
  S_CI_SFD,
  S_CI_ACK
} cc2420_rtx_state_t;

typedef nx_struct {
  // receive: whether batched transmission
  nx_bool batched;
  // receive: whether acknoledgement the received packet
  nx_bool ack;
  // transmit: priority set
  nx_bool priority;
  // transmit: number of packets need to transmit
  nx_uint8_t size;
  nx_uint16_t addr;
  nx_uint16_t metric;
  nx_uint16_t progress;
  nx_bool ci;
  // ci : hop constraint
  nx_uint8_t hop;
  nx_uint8_t preamble_dsn;
} rtx_setting_t;

typedef struct {
  bool received;
  uint8_t max_size;
  uint8_t occ_size;
  uint8_t pos_buf;
  message_t* p_rx_buf;
} rx_buffer_t;

typedef struct {
  //the factor between 32,768Hz and SMCLK
  uint16_t calibration_factor;
  // ticks SMCLK
  uint16_t pkt_rtx_time;
  uint16_t ack_time;
  uint16_t turnaround_time;
  // the number of packet RTx of individual radio open
  uint8_t pkt_recv;
  uint8_t pkt_send;
  // the length of channel detect and tail of individual radio open in ticks SMCLK
  uint32_t channel_detection;
  // the number of packet ack of individual radio open
  uint8_t pkt_ack;
  // the number of turn_around of individual radio open
  uint8_t pkt_turnaround;
  // total radio on time in 32,768Hz
  uint32_t radio_on_time;
  uint32_t tail_total_time;
  uint32_t rtx_total_time;
  uint32_t ack_total_time;
  uint32_t turnaround_total_time;
} rtx_time_compensation_t;

static inline void timer_initialization() {
  // TimerB1 for SFD capture, set edge and interrupt enable later
  TBCCTL1 = CAP;
  // TimerB2 for ACK timer, compare mode
  TBCCTL2 = 0;
  // TimerB3 for SFD abortion and reception exception timer, compare mode
  TBCCTL3 = 0;
  // TimerB4 is reserved
  TBCCTL4 = 0;
  // TimerB6 and TimerA2 for time calibration, capture mode
  TBCCTL6 = CAP;
  TACCTL2 = CAP;
  // TimerB5 for receiving and transmiting tail timer, compare mode
  TBCCTL5 = 0;
}

static inline uint32_t update_calibration_factor() {
  uint16_t cap_h0, cap_h1;
  uint16_t cap_l0, cap_l1;
  CAPTURE_NEXT_CLOCK_TICK(cap_h0, cap_l0);
  CAPTURE_NEXT_CLOCK_TICK(cap_h1,cap_l1);
  return (((cap_l1-cap_l0) << 16) + (cap_h1-cap_h0));
}

static inline void msp430_sync_dco() {
  uint16_t last;
  uint16_t diff;
 
  /* Capture on ACLK for TBCCR6 */
  TBCCTL6 = CCIS0 | CM0 | CAP | SCS;

  while (1) {
    // wait for next Capture
    TBCCTL6 &= ~CCIFG;
    while(!(TBCCTL6 & CCIFG));
    last = TBCCR6;
 
    TBCCTL6 &= ~CCIFG;
    // wait for next Capture - and calculate difference
    while(!(TBCCTL6 & CCIFG));
    diff = TBCCR6 - last;
   
    /* resynchronize the DCO speed if not at target */
    if(DELTA_II < diff) {        /* DCO is too fast, slow it down */
      DCOCTL--;
      if(DCOCTL == 0xFF) {              /* Did DCO role under? */
        BCSCTL1--;
      }
    } else if (DELTA_II > diff) {
      DCOCTL++;
      if(DCOCTL == 0x00) {              /* Did DCO role over? */
        BCSCTL1++;
      }
    } else {
      break;
    }
  }
}


/** Detect the potential ongoing packet transmission by being awake and checking SFD for two adjacent SFD period **/
//static inline void cc2420_signal_detect(uint16_t time);
/** Start to read data from RxFIFO after the SFD up edge is detected during receiving process **/
//static inline void cc2420_begin_rx();
/** Finish the lefted data transfer, send ack back and determine the further settings after the SFD down edge is detected during receiving process **/
//static inline void cc2420_end_rx();
/** Strobe the software ACK transmission **/
//static inline void cc2420_ack_strobe_rx();
/** Finish the ack sending after the SFD down edge is detected during the receiving process **/
//static inline void cc2420_ack_end_rx();
/** Load the data from memory to TxFIFO after the Tx strbe **/
//static inline void cc2420_load_tx();
/** Strobe the TXON to initialize the transmission process **/
//static inline void cc2420_strobe_tx();
/** ACK waiting after the falling edge SFD is detected during transmitting process **/
//static inline void cc2420_ack_wait_tx();
/** falling edge SFD is detected, ACK has been received during transmission **/
//static inline void cc2420_ack_rx();
/** dealing with the timing exception after ack receiving **/
//static inline void cc2420_ack_rx_except();

#endif
