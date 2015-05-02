#include "serial_fast_print.h"
#include "cc2420_x_control.h"
#include "cc2420_x_spi.h"
#include "cc2420_x_rtx.h"
#include "cc2420_x_timer.h"

#ifndef CAPTURE_NEXT_CLOCK_TICK
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
#endif

module TestSpiC {
  uses {
    interface Boot;
    interface Leds;
  }
}
implementation {
  interrupt_status_t ie_status;
  uint16_t delta;
  uint16_t rssi;
  uint16_t status;

  event void Boot.booted() {
    disable_other_interrupts(&ie_status);
    uart_init();
    cc2420_init();
    cc2420_rx_start();
    call Leds.led0On();

    while(1) {
      delta = TBR;
      rssi = 0xff & get_register(CC2420_RSSI);
      delta = TBR - delta;
      status = strobe(CC2420_SNOP);
      printf_u16(1, &status);
      printf_u16(1, &rssi);
      printf_u16(1, &delta);
      delta = 22222;
      printf_u16(1, &delta);
    }
  }
}

