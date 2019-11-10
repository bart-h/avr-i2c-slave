#ifndef H_DATA_H
#define H_DATA_H
#include "my-config.h"
#include "keyled.h"
#include <stdint.h>

union exported_data_union
{
  uint8_t i2c_slave_send_buffer[8];
  struct __attribute__((packed)) {
    uint16_t battery_voltage;
    int8_t   keyled_counters[KEYLED_KEY_NUM_INPUTS];
  };
};

extern union exported_data_union exported_data;

#endif
  
