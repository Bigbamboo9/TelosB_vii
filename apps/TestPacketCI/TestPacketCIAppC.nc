#include "TestPacketCI.h"

configuration TestPacketCIAppC {

} implementation {
  components MainC, TestPacketCIC as App, LedsC;
  components ActiveMessageC;
  components new TimerMilliC();
  components new AMSenderC(0x16);
  components new AMReceiverC(0x16);

  App.Boot -> MainC.Boot;
  App.Leds -> LedsC;
  App.AMControl -> ActiveMessageC;
  App.CommandSend -> AMSenderC;
  App.SlaveReceive -> AMReceiverC;
  App.MilliTimer -> TimerMilliC;
}
