#include "serial_fast_print.h"

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

module TestDcoDriftC {
    uses interface Boot;
    uses interface Timer<TMilli> as MilliTimer;
}
implementation {
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
          if(DELTA_II < diff) {
          /* DCO is too fast, slow it down */
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
              TBCCTL6 &= ~CCIFG;
              break;
          }
      }
  }

    event void Boot.booted() {
        uart_init();
        call MilliTimer.startPeriodic(1024);
        /*
        while (1) {
            uint16_t t_cap[2];
            uint16_t t_next_cap[2];
            uint8_t i;
            CAPTURE_NEXT_CLOCK_TICK(t_cap[0], t_cap[1]);
            TACCTL2 = CCIS0 | CM_1 | CAP | SCS;
            for (i = 0; i < 22; i++) {
                TACCTL2 &= ~CCIFG;
                while (!(TACCTL2 & CCIFG));
            }
            TACCTL2 = 0;
            CAPTURE_NEXT_CLOCK_TICK(t_next_cap[0], t_next_cap[1]);
            t_cap[0] = t_next_cap[0] - t_cap[0];
            t_cap[1] = t_next_cap[1] - t_cap[1];
            printf_u16(2, t_cap);
        }
        */
    }

    event void MilliTimer.fired() {
        uint16_t t_cap[2];
        uint16_t t_next_cap[2];
        uint8_t i;
        msp430_sync_dco();
        CAPTURE_NEXT_CLOCK_TICK(t_cap[0], t_cap[1]);
        TACCTL2 = CCIS0 | CM_1 | CAP | SCS;
        for (i = 0; i < 22; i++) {
            TACCTL2 &= ~CCIFG;
            while (!(TACCTL2 & CCIFG));
        }
        TACCTL2 = 0;
        CAPTURE_NEXT_CLOCK_TICK(t_next_cap[0], t_next_cap[1]);
        t_cap[0] = t_next_cap[0] - t_cap[0];
        t_cap[1] = t_next_cap[1] - t_cap[1];
        printf_u16(2, t_cap);
    }
}
