/**
 * CounterMicro32C provides at 32-bit counter at 4194304 ticks per second
 **/

configuration CounterMicro32C {
  provides interface Counter<TMicro, uint32_t>;
} implementation {
  components Msp430CounterMicroC as CounterFrom;
  components new TransformCounterC(TMicro, uint32_t, TMicro, uint16_t, 0, uint16_t, TRUE) as Transform;

  Counter = Transform;

  Transform.CounterFrom -> CounterFrom;
}
