/**
 * HilTimerMicroC provides a parameterized interface to a virtualized
 * millisecond timer. TimerMicroC in tos/system/ uses this component to
 * allocate new timers.
 **/

configuration HilTimerMicroC {
  provides interface Init;
  provides interface Timer<TMicro> as TimerMicro[ uint8_t ];
  provides interface LocalTime<TMicro>;
} implementation {
  components new AlarmMicro32C();
  components new AlarmToTimerC(TMicro);
  components new VirtualizeTimerC(TMicro, uniqueCount(UQ_TIMER_MICRO));
  components new CounterToLocalTimeC(TMicro);
  components CounterMicro32C;

  Init = AlarmMicro32C;
  TimerMicro = VirtualizeTimerC;
  LocalTime = CounterToLocalTimeC;

  VirtualizeTimerC.TimerFrom -> AlarmToTimerC;
  AlarmToTimerC.Alarm -> AlarmMicro32C;
  CounterToLocalTimeC.Counter -> CounterMicro32C;
}
