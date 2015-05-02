/**
 * AlarmMicroC is the alarm for async Micro alarms
 **/

generic configuration AlarmMicro32C() {
  provides interface Init;
  provides interface Alarm<TMicro, uint32_t>;
} implementation {
  components new AlarmMicro16C() as AlarmC;
  components CounterMicro32C as Counter;
  components new TransformAlarmC(TMicro, uint32_t, TMicro, uint16_t, 0) as Transform;

  Init = AlarmC;
  Alarm = Transform;

  Transform.AlarmFrom -> AlarmC;
  Transform.Counter -> Counter;

  components LedsC;
  Transform.Leds -> LedsC;
}
