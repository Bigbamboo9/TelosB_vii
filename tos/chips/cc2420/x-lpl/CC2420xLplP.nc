#include "cc2420_x_control.h"
#include "cc2420_x_packet.h"
#include "cc2420_x_rtx.h"
#include "cc2420_x_lpl.h"

module CC2420xLplP {
  provides interface Init;
  provides interface StdControl as RadioControl;
  provides interface Send;
  provides interface BulkSend;
  provides interface Receive;
  provides interface RadioTimerUpdate;

  uses interface LplSend as SubSend;
  uses interface LplReceive as SubReceive;
  uses interface LplTime;
  uses interface Timer<TMilli> as SleepTimer;
  uses interface Random;

  uses Leds;
} implementation {
  lpl_x_state_t lpl_status;
  interrupt_status_t ie_status;
  rtx_setting_t tx_status;
  uint8_t lpl_dsn;
  uint32_t radio_time_perround;
  uint32_t radio_start_time;

  command error_t Init.init() {
    lpl_status = LPL_X_CLOSE;
    lpl_dsn = TOS_NODE_ID % 0xFF;
    return SUCCESS;
  }

  command error_t RadioControl.start() {
    atomic {
      cc2420_init();
      cc2420_rx_stop();
      lpl_status = LPL_X_IDLE;
      call SleepTimer.startOneShot(CC2420_LPL_PERIOD);
    }
    return SUCCESS;
  }

  command error_t RadioControl.stop() {
    atomic {
      cc2420_stop();
      lpl_status = LPL_X_CLOSE;
      call SleepTimer.stop();
    }
    return SUCCESS;
  }

  event void SleepTimer.fired() {
    if (lpl_status == LPL_X_IDLE) {
      atomic {
        lpl_status = LPL_X_RX;
        disable_other_interrupts(&ie_status);
        // keep the dco accurate!
        msp430_sync_dco();
        call LplReceive.rxInit();
        radio_start_time = rtimer_arch_now_dco();
        cc2420_rx_start();
      }
      call SubReceive.rxOn();
    }
  }

  command error_t Send.send(message_t* msg, uint8_t len) {
    if (lpl_status != LPL_X_IDLE) {
      return EBUSY;
    }
    call SleepTimer.stop();
    atomic {
      lpl_status = LPL_X_TX;
      set_packet_header(msg, lpl_dsn);
      set_payload_length(msg, len);
      set_tx_setting(msg, &tx_status);
      disable_other_interrupts(&ie_status);
      // keep the dco accurate!
      msp430_sync_dco();
      call LplReceive.rxInit();
      cc2420_rx_start();
    }
    return (call SubSend.send(msg, &tx_status)) ;
  }

  command error_t Send.cancel(message_t* msg) {
    // the data transmission can not be cancel after STXON is strobed unless the TxFIFO is underflow. Flush the TxFIFO, close the rx and reset the status.
    if (lpl_status == LPL_X_TX) {
      atomic {
        radio_flush_tx();
        if ( strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW) ) {
          radio_flush_tx();
        }
        cc2420_rx_stop();
        enable_other_interrupts(&ie_status);
      }
    }
  }
  
  command uint8_t Send.maxPayloadLength() {
    // TODO: fixed packet size minus the header and footer size
    return (CC2420_X_PACKET_SIZE-1-sizeof(rtx_setting_t));
  }
  
  command void* Send.getPayload(message_t* msg, uint8_t len) {
    // TODO: return the second byte of payload as actrual payload, the first byte set as the length
    return get_packet_payload(msg);
  }

  command error_t BulkSend.send(message_t* msg, uint8_t len) {
    uint8_t i;
    
    if (lpl_status != LPL_X_IDLE) {
      return EBUSY;
    }
    call SleepTimer.stop();
    atomic {
      rtx_setting_t* p_ts = get_packet_setting(msg);
      uint8_t size = p_ts->size;
      lpl_status = LPL_X_TX;
      for (i = 0; i < size; i++) {
        set_packet_header(msg+i, lpl_dsn);
        set_payload_length(msg+i, len);
        lpl_dsn++;
      }
      set_tx_setting(msg, &tx_status);
      disable_other_interrupts(&ie_status);
      call LplReceive.rxInit();
      cc2420_rx_start();
    }
    return (call SubSend.send(msg, &tx_status)) ;
  }

  command error_t BulkSend.cancel(message_t* msg) {
    if (lpl_status == LPL_X_TX) {
      atomic {
        radio_flush_tx();
        if ( strobe(CC2420_SNOP) & BV(CC2420_TX_UNDERFLOW) ) {
          radio_flush_tx();
        }
        cc2420_rx_stop();
        enable_other_interrupts(&ie_status);
      }
    }
  }

  event void SubSend.sendDone(message_t* msg, rtx_setting_t* ts, error_t error) {
    // TODO: deal with the transmited data packets
    if (lpl_status != LPL_X_TX)
      return;
    atomic {
      radio_flush_tx();
      lpl_dsn++;
    }
    // signal up-layer sendDone
    if (ts->size == 1) {
      signal Send.sendDone(msg, error);
    } else {
      signal BulkSend.sendDone(msg, error);
    }
  }

  event void SubReceive.receive(message_t* msg, uint8_t size) {
    // TODO: deal with the received data packets
    // it is possible to receive data during data transmission
    uint8 i;
    if (lpl_status == LPL_X_RX) {
      atomic {
        radio_flush_rx();
      }
    }
    // deal with the received packets, signal up-layer packets received
    for (i=0; i<size; i++) {
      signal Receive.receive(msg+i, get_packet_payload(msg+i), SUCCESS);
    }
  }

  event void LplTime.timeRadio(rtx_time_compensation_t* rtx_time) {
    if (lpl_status == LPL_X_IDLE)
      return;
    atomic {
      cc2420_rx_stop();
      radio_time_perround = (rtx_time.pkt_recv + rtx_time.pkt_send) * rtx_time.pkt_rtx_time
                          + rtx_time.pkt_ack * rtx_time.ack_time
                          + rtx_time.pkt_turnaround * rtx_time.turnaround_time
                          + rtx_time.channel_detection;
      rtx_time.radio_on_time += radio_time_perround;
      rtx_time.tail_total_time += rtx_time.channel_detection;
      rtx_time.rtx_total_time += (rtx_time.pkt_recv + rtx_time.pkt_send) * rtx_time.pkt_rtx_time;
      rtx_time.ack_total_time += rtx_time.pkt_ack * rtx_time.ack_time;
      rtx_time.turnaround_total_time += rtx_time.pkt_turnaround * rtx_time.turnaround_time;
    }
  }

  event void LplTime.timeCompensated(uint16_t time, rtx_time_compensation_t* rtx_time) {
    if (lpl_status == LPL_X_IDLE)
        return;
    atomic {
      // TODO: compensate the frozen timer
      signal RadioTimerUpdate.counterUpdate(radio_time_perround+time+radio_start_time, rtx_time.calibration_factor);
      signal RadioTimerUpdate.triggerUpdate();
      if (lpl_status == LPL_X_RX)
        call SleepTimer.startOneShot(CC2420_LPL_PERIOD);
      lpl_status = LPL_X_IDLE;
      enable_other_interrupts(&ie_status);
    }
  }

  default event void RadioTimerUpdate.triggerUpdate() {}
  default event void RadioTimerUpdate.counterUpdate(uint32_t count, uint16_t factor) {}
}
