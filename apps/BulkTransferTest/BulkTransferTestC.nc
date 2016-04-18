// $Id: RadioCountToLedsC.nc,v 1.6 2008/06/24 05:32:31 regehr Exp $

/*									tab:4
 * "Copyright (c) 2000-2005 The Regents of the University  of California.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */

#include "Timer.h"
#include "BulkTransferTest.h"
#include "serial_fast_print.h"

/**
 * Implementation of the RadioCountToLeds application. RadioCountToLeds
 * maintains a 4Hz counter, broadcasting its value in an AM packet
 * every time it gets updated. A RadioCountToLeds node that hears a counter
 * displays the bottom three bits on its LEDs. This application is a useful
 * test to show that basic AM communication and timers work.
 *
 * @author Philip Levis
 * @date   June 6 2005
 */

module BulkTransferTestC @safe() {
  uses {
    interface Leds;
    interface Boot;
    interface Receive;
    interface AMSend;
    interface LocalTime<TMilli>;
    interface StdControl as AMControl;
    interface Packet;
    interface LplxPacket;
    interface PacketAcknowledgements as Acks;
    interface CC2420xControl;
    interface CC2420xProfile;
  }
}
implementation {

  message_t packet[22];

  bool locked;
  // sequence number
  uint16_t counter = 0;
  int total_data_size = 512;
  uint8_t frag_size = 16;

  uint32_t radio_on_time;
  uint8_t rec_packet = 0;

  task void packet_forward();
  task void packet_receive();
  task void radio_turn_on();
  task void print_radio_on_time();

  event void Boot.booted() {
    // uint16_t sr_value;
    // uint16_t tick_val_0;
    // uint16_t tick_val_1;
    locked = FALSE;
    // locked = TRUE;
    memset((uint8_t*)packet, 0x0, 22*sizeof(message_t));
    uart_init();
/*
    tick_val_0 = TBR;
    tick_val_1 = TBR;
    printf_u16(1, &tick_val_0);
    printf_u16(1, &tick_val_1);
*/
    call AMControl.start();
    if (TOS_NODE_ID == 0) {
      // call MilliTimer.startOneShot(1024);
      post packet_forward();
    }
/*
    call Leds.led1On();
    sr_value = READ_SR;
    printf_u16(1, &sr_value);
    atomic {
      sr_value = READ_SR;
      printf_u16(1, &sr_value);
    }
*/
  }

  task void packet_forward() {
    uint16_t dest_addr = TOS_NODE_ID + 1;
    uint8_t i;
    // call Leds.led2Toggle();
    if (locked)
      return;
    else {
      for (i = 0; i < frag_size; i++) {
        bulk_msg_t* rcm = (bulk_msg_t*)call Packet.getPayload(packet+i, sizeof(bulk_msg_t));
        if (rcm == NULL)
          return;

        call LplxPacket.setPacketBulk(packet+i, frag_size);
        rcm->counter = counter;
        counter++;
        rcm->time = call LocalTime.get();
      }
      if (call AMSend.send(dest_addr, packet, sizeof(bulk_msg_t)) == SUCCESS)
        locked = TRUE;
    }
  }

  task void packet_receive() {
    switch (TOS_NODE_ID) {
      case 1:
        call CC2420xControl.setChannel(26);
        break;
      case 2:
        call CC2420xControl.setChannel(22);
        break;
      case 3:
        call CC2420xControl.setChannel(19);
        break;
    }
    call CC2420xControl.turnRadioOn();
  }

  task void radio_turn_on() {
    call CC2420xControl.turnRadioOn();
  }

  task void print_radio_on_time() {
    uint16_t h_rot;
    uint16_t l_rot;
    radio_on_time = call CC2420xProfile.getRadioOnTime();
    h_rot = radio_on_time >> 16;
    printf_u16(1, &h_rot);
    l_rot = radio_on_time & 0xFFFF;
    printf_u16(1, &l_rot);
  }

  event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len) {
    uint8_t i;
    // memcpy((uint8_t*)packet, (uint8_t*)bufPtr, 4*sizeof(message_t));

    for (i = 0; i < frag_size; i++) {
      // bulk_msg_t* rcm = (bulk_msg_t*)call Packet.getPayload(bufPtr+i, sizeof(bulk_msg_t));
      // uint16_t seq_num = rcm->counter;
      // printf_u16(1, &seq_num);

      rec_packet++;
    }

    switch (TOS_NODE_ID) {
      case 1:
        call CC2420xControl.setChannel(22);
        break;
      case 2:
        call CC2420xControl.setChannel(19);
        break;
      case 3:
        call CC2420xControl.setChannel(15);
        break;
    }

    if (TOS_NODE_ID != 4) {
      post packet_forward();
    } else {
      post radio_turn_on();
      if (rec_packet == total_data_size) {
/*
        uint16_t h_rot;
        uint16_t l_rot;
        radio_on_time = call CC2420xProfile.getRadioOnTime();
        h_rot = radio_on_time >> 16;
        printf_u16(1, &h_rot);
        l_rot = radio_on_time & 0xFFFF;
        printf_u16(1, &l_rot);
*/
        post print_radio_on_time();
      }
    }

    return bufPtr;
  }

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    uint8_t i;
    call Leds.led0Toggle();
    if (packet == bufPtr) {
      locked = FALSE;
      for (i = 0; i < frag_size; i++) {
        if (call Acks.wasAcked(bufPtr+i)) {
          // bulk_msg_t* rcm = (bulk_msg_t*)call Packet.getPayload(bufPtr+i, sizeof(bulk_msg_t));
          // uint16_t seq_num = rcm->counter;
          // printf_u16(1, &seq_num);
          if (TOS_NODE_ID == 0)
            total_data_size--;
          // call Leds.led0Toggle();
        } else {
          counter--;
        }
      }
      if (TOS_NODE_ID != 0) {
          if (rec_packet == total_data_size) {
/*
            uint16_t h_rot;
            uint16_t l_rot;
            radio_on_time = call CC2420xProfile.getRadioOnTime();
            h_rot = radio_on_time >> 16;
            printf_u16(1, &h_rot);
            l_rot = radio_on_time & 0xFFFF;
            printf_u16(1, &l_rot);
*/
            post print_radio_on_time();
          }
          post packet_receive();
          return;
      } else {
        if (total_data_size > 0) {
          post packet_forward();
        }
      }
    }
  }
}




