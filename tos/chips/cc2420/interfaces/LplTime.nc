interface LplTime {
  async event void timeRadio(rtx_time_compensation_t* p_rtx_time);
  async event void timeCompensated(uint16_t time, rtx_time_compensation_t* p_rtx_time);
}
