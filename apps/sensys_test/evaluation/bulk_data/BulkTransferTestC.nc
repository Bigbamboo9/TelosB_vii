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
#include "pr.h"

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
    interface SplitControl as AMControl;
    interface SplitControl as SerialControl;
    interface Packet;
    interface PacketAcknowledgements as Acks;
    interface SwitchChannel;
    interface PowerSet;
  }
}
implementation {

  message_t packet;

  bool locked;
  // sequence number
  uint16_t counter = 0;
  int total_data_size = 512;
  uint8_t frag_size = 4;
  uint16_t total_rec = 0;

  uint8_t rec_counter = 0;
  uint32_t total_time;

  task void packet_forward();
  task void packet_receive();
  task void radio_turn_on();

  event void Boot.booted() {
    call SerialControl.start();
  }

  event void SerialControl.startDone(error_t err) {
    if (err != SUCCESS) {
      call SerialControl.start();
    } else {
      call AMControl.start();
    }
  }

  event void AMControl.startDone(error_t err) {
    if (err != SUCCESS) {
      call AMControl.start();
      return;
    }
    locked = FALSE;
    memset((uint8_t*)&packet, 0x0, sizeof(message_t));
    if (TOS_NODE_ID == 0) {
      pr("start tx\n");
      total_time = call LocalTime.get();
      post packet_forward();
    }
  }

  event void AMControl.stopDone(error_t err) { } 
  event void SerialControl.stopDone(error_t err) { }

  task void packet_forward() {
    uint16_t dest_addr = TOS_NODE_ID + 1;
    if (locked)
      return;
    else {
      bulk_msg_t* rcm = (bulk_msg_t*)call Packet.getPayload(&packet, sizeof(bulk_msg_t));
      if (rcm == NULL)
        return;

      rcm->counter = counter;
      counter++;
      rcm->time = call LocalTime.get();

      call Acks.requestAck(&packet);
      if (call AMSend.send(dest_addr, &packet, sizeof(bulk_msg_t)) == SUCCESS)
        locked = TRUE;
    }
  }

  task void packet_receive() {
    switch (TOS_NODE_ID) {
      case 1:
        call SwitchChannel.setNewChannel(26);
        break;
      case 2:
        call SwitchChannel.setNewChannel(22);
        break;
      case 3:
        call SwitchChannel.setNewChannel(19);
        break;
    }
  }

  task void radio_turn_on() {
    call PowerSet.turnRadioOn();
  }

  event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len) {
    bulk_msg_t* rcm = (bulk_msg_t*)call Packet.getPayload(bufPtr, sizeof(bulk_msg_t));
    uint16_t seq_num = rcm->counter;

    // pr("%d\n", seq_num);
    if (total_rec == 0) {
      pr("start rx\n");
    }

    total_rec++;

    if (total_rec == total_data_size) {
      pr("end rx\n");
    }

    rec_counter++;

    if (rec_counter != frag_size) { return bufPtr; }

    switch (TOS_NODE_ID) {
      case 1:
        call SwitchChannel.setNewChannel(22);
        break;
      case 2:
        call SwitchChannel.setNewChannel(19);
        break;
      case 3:
        call SwitchChannel.setNewChannel(15);
        break;
    }

    if (TOS_NODE_ID != 4) {    
      post packet_forward();
    } else {
      // post radio_turn_on();
    }

    return bufPtr;
  }

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    call Leds.led0Toggle();
    if (&packet == bufPtr) {
      locked = FALSE;
      if (call Acks.wasAcked(bufPtr)) {
        bulk_msg_t* rcm = (bulk_msg_t*)call Packet.getPayload(bufPtr, sizeof(bulk_msg_t));
        uint16_t seq_num = rcm->counter;
        // pr("%d\n", seq_num);
        if (TOS_NODE_ID == 0) {
          total_data_size--;
        } else {
          rec_counter--;
        }
      } else {
        counter--;
      }
      if (TOS_NODE_ID != 0) {
        if (rec_counter == 0) {
          if ( total_rec == total_data_size ) {
            pr("end tx\n");
          }
          post packet_receive();
        } else {
          post packet_forward();
        }
      } else {
        if (total_data_size > 0) {
          post packet_forward();
        } else {
          total_time = call LocalTime.get() - total_time;
          pr("total %u\n", total_time);
        }
      }
    }
  }

  event void PowerSet.radioIsOn() {};
}




