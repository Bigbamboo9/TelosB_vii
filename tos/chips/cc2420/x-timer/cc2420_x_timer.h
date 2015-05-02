#ifndef _CC2420_X_TIMER_H_
#define _CC2420_X_TIMER_H_

#include "cc2420_x_rtx.h"

#define INVALID_TIME_STAMP 0XFFFF

/*
 * /brief define the operations on timer registers
 */

#define rtimer_arch_now_dco() (TBR)

/*
 * This means that delay(i) will delay the CPU for CONST + 3x cycles.
 */
void clock_delay(unsigned int i) {
  asm("add #-1, r15");
  asm("jnz $-2");
}

static inline void tx_time_update(rtx_time_compensation_t* rtc, uint16_t time) {
  rtc->pkt_rtx_time = (rtc->pkt_rtx_time * 8 + time * 2) / 10;
}

static inline void rx_time_update(rtx_time_compensation_t* rtc, uint16_t time) {
  rtc->pkt_rtx_time = (rtc->pkt_rtx_time * 8 + time * 2) / 10;
}

static inline void ack_time_update(rtx_time_compensation_t* rtc, uint16_t time) {
  rtc->ack_time = time;
}

static inline void turnaround_time_update(rtx_time_compensation_t* rtc, uint16_t time) {
  rtc->turnaround_time = time;
}

#endif
