interface RadioTimerUpdate {
  event void triggerTimer();
  event void counterUpdate(uint32_t count, uint16_t factor);
}
