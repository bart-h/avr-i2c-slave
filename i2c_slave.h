#ifndef H_I2C_SLAVE_H
#define H_I2C_SLAVE_H

extern uint8_t i2c_slave_send_buffer[6] ;
extern uint8_t i2c_slave_receive_buffer[6];
extern uint8_t i2c_slave_cmd;

void i2c_slave_init(void);

#endif
