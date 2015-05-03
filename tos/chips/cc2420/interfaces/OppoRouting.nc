interface OppoRouting {
  command void setLocalMetric(uint16_t metric);
  command void setOppoRouting(message_t* m, uint16_t metric, uint16_t progress);
}
