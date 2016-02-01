/*
 * Copyright (c) 2006 Washington University in St. Louis.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *
 * This application is used to test the basic functionality of the printf service.
 * Calls to the standard c-style printf command are made to print various strings
 * of text over the serial line.  Only upon calling printfflush() does the
 * data actually get sent out over the serial line.
 *
 * @author Kevin Klues (klueska@cs.wustl.edu)
 * @version $Revision: 1.11 $
 * @date $Date: 2010-06-29 22:07:25 $
 */

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

module TestPrintfC @safe() {
  uses {
    interface Boot;
    interface Timer<TMilli>;
    interface Leds;
  }
}
implementation {

  uint8_t dummyVar1[8];
  uint16_t dummyVar2[8];

  event void Boot.booted() {
	uart_init();
	call Timer.startPeriodic(1000);
  }

  event void Timer.fired() {
        uint8_t i = 0;
        uint16_t* p;
        uint16_t t_cap_h0 = 0;
        uint16_t t_cap_h1 = 0;
        uint16_t t_cap_l0 = 0;
        uint16_t t_cap_l1 = 0;

        call Leds.led1Toggle();
        for (i=0;i<8;i+=1) {
		dummyVar1[i] = 100 + i;
		dummyVar2[i] = 2222 + i;
	}
        for (i=0;i<8;i+=1) {
          p = &dummyVar2[(7+i)%8];
          printf_u16(1, p);
        }
	// printf_u8(8, dummyVar1);
	// printf_u16(8, dummyVar2);
        // CAPTURE_NEXT_CLOCK_TICK(t_cap_h0, t_cap_l0);
        // CAPTURE_NEXT_CLOCK_TICK(t_cap_h1, t_cap_l1);
        // dummyVar2[0] = t_cap_h1 - t_cap_h0;
        // dummyVar2[1] = t_cap_l1 - t_cap_l0;
        // printf_u16(2, dummyVar2);

        // t_cap_l0 = 65534;
        // t_cap_h0 = 2;
        // t_cap_h1 = (t_cap_h0 - t_cap_l0) % 7;
        // t_cap_h1 = t_cap_h0 - t_cap_l0 - 2;
        // printf_u16(1, &t_cap_h1);
  }
}

