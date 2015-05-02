/**
 * CounterMicro16C provides at 16-bit counter at 1048576 ticks per second
 **/

configuration CounterMicro16C {
  provides interface Counter<TMicro, uint16_t>;
} implementation {
  components Msp430CounterMicroC as CounterFrom;
  Counter = CounterFrom;
}
