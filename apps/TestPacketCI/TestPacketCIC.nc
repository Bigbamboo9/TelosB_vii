#include "../../tos/include/pr.h"

module TestPacketCIC {
  uses {
    interface Leds;
    interface Boot;
    interface SplitControl as AMControl;

    interface AMSend as CommandSend;
    interface Receive as SlaveReceive;

    interface Timer<TMilli> as MilliTimer;
  }
} implementation {
  message_t m_data;
  uint16_t max_counter = 200;
  uint16_t rec_counter = 0;

  task void null_task() { post null_task(); }

  event void Boot.booted() {
    memset(&m_data, 0, sizeof(message_t));
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      if (TOS_NODE_ID == 0) {
        // call MilliTimer.startOneShot( 1000 );
        call MilliTimer.startPeriodic( 1000 );
      }
      post null_task();
    } else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) { }

  event void MilliTimer.fired() {
    data_msg_t* m_p_data = (data_msg_t*)call CommandSend.getPayload(&m_data, sizeof(data_msg_t));
    uint8_t i = 0;
    for (i = 0; i < 40; i++) {
      *((uint8_t*)m_p_data + i) = i;
    }
    if (call CommandSend.send(AM_BROADCAST_ADDR, &m_data, sizeof(data_msg_t)) == SUCCESS) {
      call Leds.led1Toggle();
      max_counter--;
      if (max_counter <= 0) {
        call MilliTimer.stop();
        pr("rec: %d\n", rec_counter);
      }
    }
    call Leds.led0On();
  }

  event message_t* SlaveReceive.receive(message_t* msg, void* payload, uint8_t len) {
    call Leds.led2Toggle();
    rec_counter++;
    pr("lft: %d, got: %d\n", max_counter, rec_counter);
    return msg;
  }

  event void CommandSend.sendDone(message_t* msg, error_t err) {
    if (err == SUCCESS) {
      call Leds.led1Toggle();
    }
  }
}
