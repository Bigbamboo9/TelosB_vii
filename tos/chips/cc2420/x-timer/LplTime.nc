interface LplTime {
  event void timeRadio(rtx_time_compensation_t* rtx_time)
  event void timeCompensated(uint16_t time, rtx_time_compensation_t* rtx_time);
}
