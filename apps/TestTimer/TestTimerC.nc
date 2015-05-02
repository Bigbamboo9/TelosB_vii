#include "pr.h"

module TestTimerC {
  uses interface LocalTime<TMicro> as MicroTime;
  // uses interface Timer<TMilli> as MilTimer;
  uses interface Timer<TMicro> as MicTimer;
  uses interface Boot;
  uses interface Leds;
} implementation {
  uint32_t current_time;
  uint32_t counter;

  task void null_task();
  task void print_task();

  event void Boot.booted() {
    // call MilTimer.startPeriodic(1024);
    call MicTimer.startPeriodic(1048567L*4L);
    post null_task();
    current_time = call MicroTime.get();
    counter = 0;
  }
/*
  event void MilTimer.fired() {
    // uint32_t diff = call MicroTime.get() - current_time;
    // pr("%ld\n", diff);
    call Leds.led0Toggle();
    // current_time = call MicroTime.get();
  }
*/
  event void MicTimer.fired() {
    counter++;
    call Leds.led2Toggle();
    post print_task();
  }

  task void null_task() {
    post null_task();
  }

  task void print_task() {
    pr("%ld\n", counter);
  }
}
