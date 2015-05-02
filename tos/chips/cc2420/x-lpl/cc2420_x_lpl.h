#ifndef _CC2420_X_LPL_H_
#define _CC2420_X_LPL_H_

#ifndef CC2420_LPL_PERIOD
#define CC2420_LPL_PERIOD 128
#endif

// define the states
typedef enum {
  // radio is not open
  LPL_X_CLOSE,
  // radio rx is closed
  LPL_X_IDLE,
  // turn on the rx to wait the SFD preamble
  // LPL_X_RX_SFD,
  // keep the rx on for data receive
  LPL_X_RX,
  // turn on both rx and tx to transmit data
  LPL_X_TX,
} lpl_x_state_t;

#endif
