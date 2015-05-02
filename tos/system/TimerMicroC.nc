/**
 * The virtualized microsecond timer abstraction.
 **/

#include "Timer.h"

generic configuration TimerMicroC() {
  provides interface Timer<TMicro>;
} implementation {
  components TimerMicroP;
  Timer = TimerMicroP.TimerMicro[unique(UQ_TIMER_MICRO)];
}
