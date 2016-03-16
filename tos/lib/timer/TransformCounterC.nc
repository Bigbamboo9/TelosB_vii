//$Id: TransformCounterC.nc,v 1.6 2010-06-29 22:07:50 scipio Exp $

/* Copyright (c) 2000-2003 The Regents of the University of California.  
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
 * - Neither the name of the copyright holder nor the names of
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
 * TransformCounterC decreases precision and/or widens an Counter.
 *
 * <p>See TEP102 for more details.
 *
 * @param to_precision_tag A type indicating the precision of the transformed
 *   Counter.
 * @param to_size_type The type for the width of the transformed Counter.
 * @param from_precision_tag A type indicating the precision of the original
 *   Counter.
 * @param from_size_type The type for the width of the original Counter.
 * @param bit_shift_right Original time units will be 2 to the power 
 *   <code>bit_shift_right</code> larger than transformed time units.
 * @param upper_count_type A type large enough to store the upper bits --
 *   those needed above from_size_type after its shift right to fill
 *   to_size_type.
 *
 * @author Cory Sharp <cssharp@eecs.berkeley.edu>
 */

#include "serial_fast_print.h"

generic module TransformCounterC(
  typedef to_precision_tag,
  typedef to_size_type @integer(),
  typedef from_precision_tag,
  typedef from_size_type @integer(),
  uint8_t bit_shift_right,
  typedef upper_count_type @integer(),
  bool isdco) @safe()
{
  provides interface Counter<to_precision_tag,to_size_type> as Counter;
  uses interface Counter<from_precision_tag,from_size_type> as CounterFrom;
  uses interface RadioTimerUpdate;
}
implementation
{
  upper_count_type m_upper;
  uint16_t current_counter;
  uint32_t radio_on_time;

  enum
  {
    LOW_SHIFT_RIGHT = bit_shift_right,
    HIGH_SHIFT_LEFT = 8*sizeof(from_size_type) - LOW_SHIFT_RIGHT,
    NUM_UPPER_BITS = 8*sizeof(to_size_type) - 8*sizeof(from_size_type) + bit_shift_right,
    // 1. hack to remove warning when NUM_UPPER_BITS == 8*sizeof(upper_count_type)
    // 2. still provide warning if NUM_UPPER_BITS > 8*sizeof(upper_count_type)
    // 3. and allow for the strange case of NUM_UPPER_BITS == 0
    OVERFLOW_MASK = NUM_UPPER_BITS ? ((((upper_count_type)2) << (NUM_UPPER_BITS-1)) - 1) : 0,
  };

  async command to_size_type Counter.get()
  {
    to_size_type rv = 0;
    atomic
    {
      upper_count_type high = m_upper;
      from_size_type low = call CounterFrom.get();
      if (call CounterFrom.isOverflowPending())
      {
	    // If we signalled CounterFrom.overflow, that might trigger a
	    // Counter.overflow, which breaks atomicity.  The right thing to do
	    // increment a cached version of high without overflow signals.
	    // m_upper will be handled normally as soon as the out-most atomic
	    // block is left unless Clear.clearOverflow is called in the interim.
	    // This is all together the expected behavior.
	    high++;
	    low = call CounterFrom.get();
      }
      {
	    to_size_type high_to = high;
	    to_size_type low_to = low >> LOW_SHIFT_RIGHT;
	    rv = (high_to << HIGH_SHIFT_LEFT) | low_to;
      }
    }
    return rv;
  }

  // isOverflowPending only makes sense when it's already part of a larger
  // async block, so there's no async inside the command itself, where it
  // wouldn't do anything useful.

  async command bool Counter.isOverflowPending()
  {
    return ((m_upper & OVERFLOW_MASK) == OVERFLOW_MASK)
	    && call CounterFrom.isOverflowPending();
  }

  // clearOverflow also only makes sense inside a larger atomic block, but we
  // include the inner atomic block to ensure consistent internal state just in
  // case someone calls it non-atomically.

  async command void Counter.clearOverflow()
  {
    atomic
    {
      if (call Counter.isOverflowPending())
      {
	    m_upper++;
	    call CounterFrom.clearOverflow();
      }
    }
  }

  async event void CounterFrom.overflow()
  {
    atomic
    {
      m_upper++;
      if ((m_upper & OVERFLOW_MASK) == 0)
	    signal Counter.overflow();
    }
  }

  event void RadioTimerUpdate.startRadioTime() {
  atomic{
    if (!isdco && (bit_shift_right == 5)) {
      uint32_t high = m_upper;
      uint32_t low;

      if (call CounterFrom.isOverflowPending()) {
        high++;
      }
      current_counter = call CounterFrom.get();
      low = current_counter;
      radio_on_time = (high << HIGH_SHIFT_LEFT) | (low >> LOW_SHIFT_RIGHT);
    }
  }
  }

  // count is in SMCLK ticks
  event void RadioTimerUpdate.counterUpdate(uint32_t count, uint16_t factor) {
    atomic {
      // only for 32KHz 16 to TMilli 32
      if (!isdco && (bit_shift_right == 5)) {
        // factor is approximate 128, add 1 tick as the guard time
        // upper_count_type add_new = (count >> 7) >> (8 * sizeof(from_size_type));
        // upper_count_type add_new = (count >> 7) + current_counter + 1;
        uint32_t add_new = (count >> 7) + current_counter;
        uint8_t i;
        add_new = add_new >> 16;
        for (i = 0; i < add_new; i++) {
          m_upper++;
          if ((m_upper & OVERFLOW_MASK) == 0)
            signal Counter.overflow();
        }
/*
        {
          uint16_t debug_u16 = add_new;
          printf_u16(1, &debug_u16);
        }
*/
      }
/* else {
        upper_count_type add_new = count >> (8 * sizeof(from_size_type));
        upper_count_type i;
        for (i = 0; i < add_new; i++) {
          m_upper++;
          if ((m_upper & OVERFLOW_MASK) == 0)
            signal Counter.overflow();
        }
      }
*/
    }
  }
  
  event void RadioTimerUpdate.triggerUpdate() {
  atomic {
    uint32_t high;
    uint32_t low;
    uint32_t current_time;

    high = m_upper;
    // checke whether the compenstation miss any overflow, should not count any pending overflow
/*
    if (call CounterFrom.isOverflowPending()) {
      high++;
    }
*/
    low = call CounterFrom.get();

    current_time = (high << HIGH_SHIFT_LEFT) | (low >> LOW_SHIFT_RIGHT);

    if (current_time < radio_on_time) {
      m_upper++;
      if ((m_upper & OVERFLOW_MASK) == 0)
        signal Counter.overflow();
    }
  }
  }
}

