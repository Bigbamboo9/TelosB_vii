configuration CC2420xLplC {
  provides interface StdControl as RadioControl;
  provides interface Send;
  provides interface BulkSend;
  provides interface Receive;
  provides interface RadioTimerUpdate;
} implementation {
  components MainC;
  components CC2420xLplP;
  components CC2420xRTxP;

  RadioControl = CC2420xLplP;
  Send = CC2420xLplP;
  BulkSend = CC2420xLplP;
  Receive = CC2420xLplP;
  RadioTimerUpdate = CC2420xLplP;

  MainC->SoftwareInit -> CC2420xLplP;
  MainC->SoftwareInit -> CC2420xRTxP;
  
  CC2420xLplP.SubSend -> CC2420xRTxP;
  CC2420xLplP.SubReceive -> CC2420xRTxP;
  CC2420xLplP.LplTime -> CC2420xRTxP;

  components new TimerMilliC as SleepTimer;
  CC2420xLplP.SleepTimer -> SleepTimer;

  components RandomC;
  CC2420xLplP.Random -> RandomC;
}