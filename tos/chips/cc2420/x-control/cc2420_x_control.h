#ifndef _CC2420_X_CONTROL_H_
#define _CC2420_X_CONTROL_H_

#include "CC2420.h"
#include "cc2420_x_spi.h"
#include "cc2420_x_timer.h"
#include "serial_fast_print.h"

#ifndef CC2420_X_PACKET_SIZE
#define CC2420_X_PACKET_SIZE 77
#endif

#ifndef CC2420_X_DEF_RFPOWER
#define CC2420_X_DEF_RFPOWER 2
#endif

#ifndef CC2420_X_DEF_CHANNEL
#define CC2420_X_DEF_CHANNEL 15
#endif


/*
 * /brief define the operations on radio, interrupt enable and io control registers.
 */
#define FIFO_P 0  /* P1.0 - Input */
#define FIFO   3  /* P1.3 - Input */
#define CCA    4  /* P1.4 - Input */

#define SFD     1 /* P4.1 - Input  */
#define VREG_EN 5 /* P4.5 - Output */
#define RESET_N 6 /* P4.6 - Output */

#define CC2420_FIFOP_INT_INIT() do {\
  P1IES &= ~BV(FIFO_P);\
  P1IFG &= ~BV(FIFO_P);\
} while(0)
#define CC2420_FIFOP_INT_DISABLE() ( P1IE &= ~BV(FIFO_P) )
#define CC2420_FIFOP_INT_ENABLE() ( P1IE |= BV(FIFO_P) )
/* CC2420 reset pin. */
#define SET_RESET_INACTIVE() ( P4OUT |= BV(RESET_N) )
#define SET_RESET_ACTIVE() ( P4OUT &= ~BV(RESET_N) )
/* CC2420 voltage regulator enable pin. */
#define SET_VREG_ACTIVE() ( P4OUT |= BV(VREG_EN) )
#define SET_VREG_INACTIVE() ( P4OUT &= ~BV(VREG_EN) )

/* CC2420 status byte P29 in datasheet */
enum cc2420_status_byte {
  CC2420_XOSC16M_STABLE = 6,
  CC2420_TX_UNDERFLOW   = 5,
  CC2420_ENC_BUSY       = 4,
  CC2420_TX_ACTIVE      = 3,
  CC2420_LOCK           = 2,
  CC2420_RSSI_VALID     = 1,
};

// struct to record the current interrupt register status
typedef struct {
  uint16_t ie1;
  uint16_t ie2;
  uint16_t p1ie;
  uint16_t p2ie;  
} interrupt_status_t;
// disable all other interrupts after the radio is turned on
static inline void disable_other_interrupts(interrupt_status_t* status) {
  status->ie1 = IE1;
  status->ie2 = IE2;
  status->p1ie = P1IE;
  status->p2ie = P2IE;
  // disable USARTx
  IE1 = 0;
  IE2 = 0;
  // disable IOPx
  P1IE = 0;
  P2IE = 0;
  // disable DMA
  DMA0CTL &= ~DMAIE;
  DMA1CTL &= ~DMAIE;
  DMA2CTL &= ~DMAIE;
  // disable comparator_A
  CACTL1 &= ~CAIE;
  // disable TimerA_VECTORx
  TACTL   &= ~TAIE;
  TACCTL0 &= ~CCIE;
  TACCTL1 &= ~CCIE;
  TACCTL2 &= ~CCIE;
  // disable TimerB_VECTORx except TBCCTL1
  TBCCTL0 &= ~CCIE;
  // TimerB overflow is always disabled
  TBCTL   &= ~TBIE;
  // set SFD pin
  P4SEL |= (1 << SFD);
  // enable rising edge interrupt
  TBCCTL1 = CM_1 | CAP | SCS | CCIE;
  TBCCTL1 &= ~CCIFG; 
}
// enable all other interrupts after the radio is turned off
static inline void enable_other_interrupts(interrupt_status_t* status) {
  IE1 = status->ie1;
  IE2 = status->ie2;
  P1IE = status->p1ie;
  P2IE = status->p2ie;
  DMA0CTL &= ~DMAIFG;
  DMA0CTL |= DMAIE;
  DMA1CTL &= ~DMAIFG;
  DMA1CTL |= DMAIE;
  DMA2CTL &= ~DMAIFG;
  DMA2CTL |= DMAIE;
  CACTL1  &= ~CAIFG;
  CACTL1  |= CAIE;
  TACTL   &= ~TAIFG;
  TACTL   |= TAIE;
  TACCTL0 &= ~CCIFG;
  TACCTL0 |= CCIE;
  TACCTL1 &= ~CCIFG;
  TACCTL1 |= CCIE;
  TACCTL2 &= ~CCIFG;
  TACCTL2 |= CCIE;
  TBCTL   &= ~TBIFG;
  TBCTL   |= TBIE;
  TBCCTL0 &= ~CCIFG;
  TBCCTL0 |= CCIE;
}
// Transmit Control Register
static inline void cc2420_tx_setting() {
  // all by defaults, set the tx_power
  uint16_t setting = get_register(CC2420_TXCTRL);
  setting &= ( (CC2420_X_DEF_RFPOWER & 0x1F) << CC2420_TXCTRL_PA_LEVEL );
  set_register(CC2420_TXCTRL, setting);
}
// Operating frequency
static inline void cc2420_channel_setting() {
  uint16_t setting = get_register(CC2420_FSCTRL);
  setting &= 0xFE00;
  setting |= ( 0x1FFF & (357 + 5 * (CC2420_X_DEF_CHANNEL - 11)) );
  set_register(CC2420_FSCTRL, setting);
}
// Modem Control Register
static inline void cc2420_mod_setting() {
  // reserved frame types(100, 101, 110, 111) are rejected by address recognition as default
  // disable autoack and hardware for anycasting
  uint16_t setting = get_register(CC2420_MDMCTRL0);
  setting &= ~BV(CC2420_MDMCTRL0_ADR_DECODE);
  set_register(CC2420_MDMCTRL0, setting);
  // set CORR_THR as 20
  setting = get_register(CC2420_MDMCTRL1);
  setting |= ( (20 & 0x1F) << CC2420_MDMCTRL1_CORR_THR );
  set_register(CC2420_MDMCTRL1, setting);
}
// Receive Control Register
static inline void cc2420_rx_setting() {
  // controls reference bias current to RX bandpass filters 3uA (recommanded setting)
  uint16_t setting = get_register(CC2420_RXCTRL1);
  setting |= BV(CC2420_RXCTRL1_RXBPF_LOCUR);
  set_register(CC2420_RXCTRL1, setting);
}
// Security Control Register
static inline void cc2420_sec_setting() {
  // clear RXFIFO protection to better utilize rx_fifo
  uint16_t setting = get_register(CC2420_SECCTRL0);
  setting &= ~BV(CC2420_SECCTRL0_RXFIFO_PROTECTION);
  set_register(CC2420_SECCTRL0, setting);
}
// I/O Configuration Register
static inline void cc2420_io_setting() {
  // reverse CCA polarity: CCA = 1 if rssi > thr
  // set FIFOP as the maximum
  uint16_t setting = get_register(CC2420_IOCFG0);
  setting |= ( BV(CC2420_IOCFG0_CCA_POLARITY) | (127 << CC2420_IOCFG0_FIFOP_THR) );
  set_register(CC2420_IOCFG0, setting);
}
// RSSI and CCA Status and Control Register
static inline int cc2420_get_rssi() {
  int rssi = ( 0xff & get_register(CC2420_RSSI) );
  if (rssi > 128) {
    rssi = rssi - 256 - 45;
  } else {
    rssi = rssi - 45;
  }
  return rssi;
}
// RX_FIFO FLUSH
static inline void radio_flush_rx() {
  uint8_t dummy;
  fast_read_one(&dummy);
  strobe(CC2420_SFLUSHRX);
  strobe(CC2420_SFLUSHRX);
}
// TX_FIFO FLUSH
static inline void radio_flush_tx() {
  strobe(CC2420_SFLUSHTX);
}
// Radio Initialization
void cc2420_init() {
  // initialize SPI registers
  cc2420_spi_init();
  // all input by default, set these as output
  P4DIR |= BV(CSN) | BV(VREG_EN) | BV(RESET_N);
  // set these as input
  P4DIR &= ~( BV(SFD) );
  P1DIR &= ~( BV(FIFO_P) | BV(FIFO) | BV(CCA) );
  // disable spi transmission
  CC2420_SPI_DISABLE();
  // -- disable and clear FIFOP interrupt --
  CC2420_FIFOP_INT_DISABLE();
  CC2420_FIFOP_INT_INIT();
  // turn on voltage regulator and reset
  SET_VREG_ACTIVE();
  SET_RESET_ACTIVE();
  // voltage regulator startup time Typ. 0.3ms Max 0.6ms
  clock_delay(1024);
  SET_RESET_INACTIVE();
  // turn on the crystal oscillator
  strobe(CC2420_SXOSCON);
  // oscillator stable
  while ( !( strobe(CC2420_SNOP) & BV(CC2420_XOSC16M_STABLE) ) );
  // register settings
  cc2420_mod_setting();
  cc2420_sec_setting();
  cc2420_channel_setting();
  cc2420_rx_setting();
  cc2420_tx_setting();
  cc2420_io_setting();
  // flush rx_fifo
  radio_flush_rx();
}
// totally close the radio
void cc2420_stop() {
  strobe(CC2420_SXOSCOFF);
  SET_VREG_INACTIVE();
}
// turn the radio from IDLE to RX mode
static inline void cc2420_rx_start() {
  strobe(CC2420_SRXON);
}
// turn the radio from RX mode to IDLE
static inline void cc2420_rx_stop() {
  strobe(CC2420_SRFOFF);
}

#endif
