configuration CC2420xLplC {
  provides interface StdControl as RadioControl;
  provides interface Send;
  provides interface BulkSend;
  provides interface Receive;
  provides interface Receive as Snoop;
  provides interface OppoRouting;

  provides interface RadioTimerUpdate;
} implementation {
  components MainC;
  components CC2420xLplP;
  components CC2420xRTxP;

  RadioControl = CC2420xLplP;
  Send = CC2420xLplP;
  BulkSend = CC2420xLplP;
  Receive = CC2420xLplP;
  Snoop = CC2420xRTxP;
  RadioTimerUpdate = CC2420xLplP;
  OppoRouting = CC2420xRTxP;

  MainC.SoftwareInit -> CC2420xLplP;
  MainC.SoftwareInit -> CC2420xRTxP;
  
  CC2420xLplP.SubSend -> CC2420xRTxP;
  CC2420xLplP.SubReceive -> CC2420xRTxP;
  CC2420xLplP.LplTime -> CC2420xRTxP;

  components new TimerMilliC() as SleepTimer;
  CC2420xLplP.SleepTimer -> SleepTimer;

  components RandomC;
  CC2420xLplP.Random -> RandomC;

  components LedsC;
  CC2420xLplP.Leds -> LedsC;
  CC2420xRTxP.Leds -> LedsC;
}
