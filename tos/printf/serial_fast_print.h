#ifndef _SERIAL_FAST_PRINT_H_
#define _SERIAL_FAST_PRINT_H_

#define PRINT_U8_DELIMITER  0x22;
#define PRINT_U16_DELIMITER 0x77;

void uart_init() {
  static unsigned char uart_inited = 0;

  if (uart_inited)
    return;

  P3SEL |= 0xc0;           //P3.6 UTXD1 P3.7 URXD1
  U1CTL  = CHAR + SWRST;
  U1TCTL = 0x20;
  // --1MHz SMCLK--
  // U1BR0  = 0x09;
  // U1BR1  = 0x00;
  // U1MCTL = 0x10;
  // --------------
  // **4Mhz SMCLK**
  U1BR0  = 0x24;
  U1BR1  = 0x00;
  U1MCTL = 0x29;
  // **************
  ME2    = UTXE1;
  U1CTL &= ~SWRST;
}

static inline void uart_fast_tx(uint8_t byte) {
  while (!(IFG2 & UTXIFG1));
  U1TXBUF = byte; 
}

static inline void printf_u8(uint8_t var, uint8_t* byte) {
  uint8_t idx;
  // uart_fast_tx(PRINT_U8_DELIMITER);
  uart_fast_tx(0x22);
  uart_fast_tx(var);
  for (idx = 0; idx < var; idx++) {
    uart_fast_tx(byte[idx]);
  }
}

static inline void printf_u16(uint8_t var, uint16_t* word) {
  uint8_t high;
  uint8_t low;
  uint8_t idx;
  //uart_fast_tx(PRINT_U16_DELIMITER);
  uart_fast_tx(0x77);
  uart_fast_tx(var);
  for (idx = 0; idx < var; idx++) {
    low = word[idx] & 0x00FF;
    high = word[idx] >> 8;
    uart_fast_tx(high);
    uart_fast_tx(low);
  }
}

static inline void printf_int(uint8_t var, int* value) {
  uint8_t high;
  uint8_t low;
  uint8_t idx;
  uint16_t tmp_value;
  uart_fast_tx(0x44);
  uart_fast_tx(var);
  for (idx = 0; idx < var; idx++) {
    tmp_value = (uint16_t)value[idx];
    low = tmp_value & 0x00FF;
    high = tmp_value >> 8;
    uart_fast_tx(high);
    uart_fast_tx(low);
  }
}
#endif
