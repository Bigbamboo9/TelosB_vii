configuration TestDcoDriftAppC {}
implementation {
      components MainC, TestDcoDriftC as App;
      components new TimerMilliC();
      App.Boot -> MainC.Boot;
      App.MilliTimer -> TimerMilliC;
}
