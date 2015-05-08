interface RadioTimerUpdate {
  event void triggerUpdate();
  event void counterUpdate(uint32_t count, uint16_t factor);
}
