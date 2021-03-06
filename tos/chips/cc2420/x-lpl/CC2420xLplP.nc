#include "cc2420_x_control.h"
#include "cc2420_x_spi.h"
#include "cc2420_x_packet.h"
#include "cc2420_x_rtx.h"
#include "cc2420_x_lpl.h"

#include "serial_fast_print.h"

module CC2420xLplP {
  provides interface Init;
  provides interface StdControl as RadioControl;
  provides interface Send;
  provides interface BulkSend;
  provides interface Receive;
  provides interface RadioTimerUpdate;
  provides interface CC2420xControl;

  uses interface LplSend as SubSend;
  uses interface LplReceive as SubReceive;
  uses interface LplTime;
  uses interface Timer<TMilli> as SleepTimer;
  uses interface Random;

  uses interface Leds;
} implementation {
  lpl_x_state_t lpl_status;
  interrupt_status_t ie_status;
  rtx_setting_t tx_status;
  bool bulk_or_not;
  uint8_t lpl_dsn;
  uint8_t tx_size;
  uint32_t radio_time_perround;
  uint32_t radio_start_time;

  uint16_t print_low;
  uint16_t print_high;

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

  command void CC2420xControl.turnRadioOn() {
    if (lpl_status == LPL_X_IDLE) {
      atomic {
        if (call SubReceive.rxOn() != SUCCESS) {
          call SleepTimer.startOneShot(CC2420_LPL_PERIOD);
          return;
        }
        call SleepTimer.stop();
        lpl_status = LPL_X_RX;
        disable_other_interrupts(&ie_status);
        signal RadioTimerUpdate.startRadioTime();
        call SubReceive.rxInit();
        call SubReceive.rxEnable();
        // call SubReceive.txDetect();
      }
    }
  }

  command void CC2420xControl.setChannel(uint8_t channel) {
    uint16_t setting = get_register(CC2420_FSCTRL);
    setting &= 0xFE00;
    setting |= ( 0x1FFF & (357 + 5 * (channel - 11)) );
    set_register(CC2420_FSCTRL, setting);
  }

  event void SleepTimer.fired() {
    if (lpl_status == LPL_X_IDLE) {
      atomic {
        if (call SubReceive.rxOn() != SUCCESS) {
          call SleepTimer.startOneShot(CC2420_LPL_PERIOD);
          return;
        }
        lpl_status = LPL_X_RX;
        disable_other_interrupts(&ie_status);
        signal RadioTimerUpdate.startRadioTime();
        // keep the dco accurate! However, the timely adjustment surprisingly make the dco unstable. Disable it.
        // msp430_sync_dco();
        call SubReceive.rxInit();
        // radio_start_time = rtimer_arch_now_dco();
        // restart the radio from idle to rx mode, avoid the early comming SFD
        call SubReceive.rxEnable();
        call SubReceive.txDetect();
      }
    }
  }

  command error_t Send.send(message_t* msg, uint8_t len) {
    if (lpl_status != LPL_X_IDLE) {
      return EBUSY;
    }
    call SleepTimer.stop();
    atomic {
      lpl_status = LPL_X_TX;
      bulk_or_not = FALSE;
      set_packet_header(msg, lpl_dsn);
      set_payload_length(msg, len);
      init_packet_metadata(msg);
      set_tx_setting(msg, &tx_status);
      set_packet_header_crc(msg);
      disable_other_interrupts(&ie_status);
      signal RadioTimerUpdate.startRadioTime();
      // keep the dco accurate! However, the timely adjustment surprisingly make the dco unstable. Disable it.
      // msp430_sync_dco();
      call SubReceive.rxInit();
      // start the radio at low-layer to reduce the change of early comming SFD
      // cc2420_rx_start();
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
      rtx_setting_t* p_ts = (rtx_setting_t*)get_packet_setting(msg);
      tx_size = p_ts->size;
      lpl_status = LPL_X_TX;
      bulk_or_not = TRUE;
      for (i = 0; i < tx_size; i++) {
        set_packet_header(msg+i, lpl_dsn);
        set_payload_length(msg+i, len);
        init_packet_metadata(msg+i);
        set_packet_header_crc(msg+i);
        lpl_dsn++;
      }
      set_tx_setting(msg, &tx_status);
      disable_other_interrupts(&ie_status);
      call SubReceive.rxInit();
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

  async event void SubSend.sendDone(message_t* msg, rtx_setting_t* ts, error_t error) {
    // TODO: deal with the transmited data packets
    if (lpl_status != LPL_X_TX)
      return;
    atomic {
      radio_flush_tx();
      lpl_dsn++;
    }
    // signal up-layer sendDone
    // if (ts->size == 1) {
    if (!bulk_or_not) {
      signal Send.sendDone(msg, error);
    } else {
      signal BulkSend.sendDone(msg-tx_size+1, error);
    }
  }

  async event void SubReceive.receive(message_t* msg, uint8_t size) {
    // TODO: deal with the received data packets
    // it is possible to receive data during data transmission
    uint8_t i;
    if (lpl_status == LPL_X_RX) {
      atomic {
        radio_flush_rx();
      }
    }
    // deal with the received packets, signal up-layer packets received
    // printf_u8(1, &size);
    // TODO : signal multiple event may not all be handled
    // for (i=0; i<size; i++) {
    //   signal Receive.receive(msg+i, get_packet_payload(msg+i), SUCCESS);
    // }
    signal Receive.receive(msg, get_packet_payload(msg), SUCCESS);
    call SubReceive.rxBuffSet();
  }

  async event void LplTime.timeRadio(rtx_time_compensation_t* rtx_time) {
    if (lpl_status == LPL_X_IDLE)
      return;
    atomic {
      cc2420_rx_stop();
      radio_time_perround = (rtx_time->pkt_recv + rtx_time->pkt_send) * rtx_time->pkt_rtx_time
                          + rtx_time->pkt_ack * rtx_time->ack_time
                          + rtx_time->pkt_turnaround * rtx_time->turnaround_time
                          + rtx_time->channel_detection;
      rtx_time->radio_on_time += radio_time_perround;
      rtx_time->tail_total_time += rtx_time->channel_detection;
      rtx_time->rtx_total_time += (rtx_time->pkt_recv + rtx_time->pkt_send) * rtx_time->pkt_rtx_time;
      rtx_time->ack_total_time += rtx_time->pkt_ack * rtx_time->ack_time;
      rtx_time->turnaround_total_time += rtx_time->pkt_turnaround * rtx_time->turnaround_time;

      print_high = radio_time_perround >> 16;
      // printf_u16(1, &print_high);
      print_low = radio_time_perround & 0xFFFF;
      // printf_u16(1, &print_low);

    }
  }

  async event void LplTime.timeCompensated(uint16_t time, rtx_time_compensation_t* rtx_time) {
    if (lpl_status == LPL_X_IDLE)
        return;
    atomic {
      // TODO: compensate the frozen timer
      signal RadioTimerUpdate.counterUpdate(radio_time_perround+time+radio_start_time, rtx_time->calibration_factor);
      // signal RadioTimerUpdate.counterUpdate(radio_time_perround+time, rtx_time->calibration_factor);
      // signal RadioTimerUpdate.triggerUpdate();
      lpl_status = LPL_X_IDLE;
      enable_other_interrupts(&ie_status);
      // check the possible missing overflow
      signal RadioTimerUpdate.triggerUpdate();
      // restart the channel timer after RTx
      call SleepTimer.startOneShot(CC2420_LPL_PERIOD);
    }
  }

  default event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) { return msg; }
  default event void Send.sendDone(message_t* msg, error_t err) {}
  default event void BulkSend.sendDone(message_t* msg, error_t err) {}

  default event void RadioTimerUpdate.startRadioTime() {}
  default event void RadioTimerUpdate.triggerUpdate() {}
  default event void RadioTimerUpdate.counterUpdate(uint32_t count, uint16_t factor) {}
}
