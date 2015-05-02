/**
 * AlarmMicroC is the alarm for async Micro alarms
 **/

generic configuration AlarmMicro16C() {
  provides interface Init;
  provides interface Alarm<TMicro, uint16_t>;
} implementation {
  components new Msp430TimerMicroC() as Msp430Timer;
  components new Msp430AlarmC(TMicro) as Msp430Alarm;
  components LedsC;

  Init = Msp430Alarm;
  Alarm = Msp430Alarm;

  Msp430Alarm.Msp430Timer -> Msp430Timer;
  Msp430Alarm.Msp430TimerControl -> Msp430Timer;
  Msp430Alarm.Msp430Compare -> Msp430Timer;
  Msp430Alarm.Leds -> LedsC;
}
