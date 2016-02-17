interface RadioTimerUpdate {
  event void startRadioTime();
  event void triggerUpdate();
  event void counterUpdate(uint32_t count, uint16_t factor);
}
