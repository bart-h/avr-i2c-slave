/* implements twi slave mode for attiny2313 */

/*
 * This code is pretty straightforward, but with a small twist concerning acking our address.
 *
 * A master will signal a start condition by pulling SDA low when CLK is high. 
 * This will trigger the start condition interrupt on our device.
 * After some time, the master will pull CLK low as well. From this point on, our device 
 * (or any other on the bus) will keep the CLK low until the start condition is released.
 * When CLK goes high, the master will have put the high bit of the address on SDA, and
 * things will continue normally (by clocking the other bits as well).
 *
 * The problem now is that in the start condition (SC) ISR the state of CLK is not defined, and that
 * the USI counts 16 flanks max. Other code that I've seen waits in the ISR until CLK goes low, probably
 * so that the next 8 address bits can be read by 16 flanks. Unfortunately, this waiting could take 
 * upto forever if the master goes nuts. So I handle it differently. 
 *
 * The main ingredients are reading bits at the positive clock edge, and only reading the first 7 bits
 * of the address initially. So depending on the state of CLK in the SC ISR, we wait for 15 (CLK hi) or
 * 14 (CLK lo) flanks. Since 7 of these flanks are positive, 7 bits are read.
 *
 * But 7 bits of address is sufficient to see if we are addressed or not. If not, we go off bus (idle).
 * If yes, we setup the ISR to do a combined output and input. Such that the RW bit is read on the 
 * first clock pulse , and SDA is pulled low on the next clock, acknowledging our address.
 *
 * This way, an invalid address is handled in 2 interrupts (SC and overflow after 14-15 flanks), 
 * and acknowledging a valid address is handled in 3 interrupts (SC, overflow after 14-15 flanks 
 * and overflow after 4 flanks). Other code has the same number of interrupts, but with a loop in 
 * the SC ISR to wait for the CLK to go down.
 */

#include "my-config.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "event.h"
#include "debug.h"
#include "data.h"

#define I2C_PORT PORTB
#define I2C_PIN PINB
#define I2C_DDR DDRB
#define I2C_SDA 5
#define I2C_CLK 7

enum { I2C_IDLE=0x00,
       I2C_ADDRESS=0x01,
       I2C_READ_WRITE=0x02,
       I2C_CONTROL=0x03,

       I2C_RECEIVE_1=0x10,
       I2C_RECEIVE_2=0x11,
       I2C_RECEIVE_3=0x12,
       I2C_RECEIVE_4=0x13,
       I2C_RECEIVE_5=0x14,
       I2C_RECEIVE_6=0x15,

       I2C_SEND_1=0x20,
       I2C_SEND_2=0x21,
       I2C_SEND_3=0x22,
       I2C_SEND_4=0x23,
       I2C_SEND_5=0x24,
       I2C_SEND_6=0x25,
       I2C_SEND_7=0x26,
       I2C_SEND_MAX=0x2f,
       
       I2C_ACK_RECEIVE=0x40, /* acknowledge the receive */
       I2C_WAIT_ACK1=0x80,
       I2C_WAIT_ACK2=0x40,
       I2C_WAIT_ACK=0xC0,
};


/* this is used to communicate with the outside world */
/* do not use this, but use a global buffer where all modules publish data. */
/*uint8_t i2c_slave_send_buffer[6] ;*/

#define i2c_slave_send_buffer exported_data.i2c_slave_send_buffer

uint8_t i2c_slave_receive_buffer[6];
uint8_t i2c_slave_cmd;

static uint8_t i2c_slave_state;

static inline __attribute__ ((always_inline))
void set_sda_input(void)
{
  I2C_DDR &= ~_BV(I2C_SDA);
}

static inline __attribute__ ((always_inline))
void set_sda_output(void)
{
  I2C_DDR |= _BV(I2C_SDA);
}

static inline __attribute__((always_inline))
void set_sda_high(void)
{
  I2C_PORT |= _BV(I2C_SDA);
}

static inline __attribute__((always_inline))
void set_sda_low(void)
{
  I2C_PORT &= ~_BV(I2C_SDA);
}

static inline __attribute__ ((always_inline))
void set_status_register( uint8_t sif, uint8_t oif, uint8_t flanks )
{
  USISR = (sif ? _BV(USISIF) : 0) | (oif ? _BV(USIOIF) : 0 ) | (16-flanks);  
}

static inline __attribute__ ((always_inline))
void release_start_condition(void)
{
  USISR |= _BV(USISIF);
}

static inline __attribute__ ((always_inline))
void release_overflow_condition(void)
{
  USISR |= _BV(USIOIF);
}

static inline __attribute__ ((always_inline))
void set_control_register( uint8_t start, uint8_t overflow, uint8_t clock )
{
  USICR =
    (start ? _BV(USISIE) : 0)| /* enable start condition interrupt, obviously */
    (overflow ? _BV(USIOIE) : 0) | /* overflow interrupt */
    _BV(USIWM1) | _BV(USIWM0) | /* twi mode */
    (clock? _BV(USICS1) : 0 ) | /* clock on positive edge */
    0;
}

static inline __attribute__ ((always_inline))
void disable_overflow_interrupt(void)
{
  USICR &= ~_BV(USIOIE);
}

static inline __attribute__ ((always_inline))
void enable_overflow_interrupt(void)
{
  USICR |= _BV(USIOIE);
}

static inline __attribute__ ((always_inline))
void i2c_slave_go_idle(void)
{
  /* make input */
  set_sda_input();
  set_sda_high();
  
  /* clear start, clear overflow, dummy flanks */
  set_status_register( 1, 1, 16 );
  /* disable interrupts except start condition */
  set_control_register( 1, 0, 0);
  i2c_slave_state = I2C_IDLE;
}

void i2c_slave_init(void)
{
  /* CLK setup here. 
   * SDA is setup by i2c_slave_go_idle.
   */
  I2C_DDR |= _BV(I2C_CLK);
  /* set output bit to 1, to let the USI control things */
  I2C_PORT |= _BV(I2C_CLK);
  i2c_slave_go_idle();
}

/*
 * Start function: the master forced a start condition.
 */

ISR(USI_START_vect,ISR_BLOCK)
{
  /* Start condition.
   * Master might still have to clock up at this point
   * Once it goes down, USI will keep it down, as long as we did not clear 
   * the start condition.
   */

  /* make sure that sda is in the right mode, as a start condition can happen
   * virtually anytime.
   */
  set_sda_input();
  set_sda_high();
  
  /* Next will be the 7 address bits (14 flanks on clock) */

  /* To prepare for the next step after receiving address, we need to "output"
   * 1s, so put those in the DR.
   */
  USIDR = 0xff;
  
  /* Assume clock is still high, so we prepare for 15 flanks, and fix it later. */
  set_status_register(0,1,15);

  /* Enable full mode */
  set_control_register(1,1,1);

  /* Setup for receiving address */
  i2c_slave_state = I2C_ADDRESS;

  /* Check if clock is down */
  if ((I2C_PIN & _BV(I2C_CLK))==0) {
    /* Line went down already, so only 14 flanks are needed now. 
     * Doing check like this will prevent racing.
     */
    /* Set 14 flanks, but also release the start condition since 
     * we are here.
     */
    set_status_register(0,1,14);
  }
  release_start_condition();
}


#define ALLOW_INTERRUPT 0
/*
 * Overflow function: (some) data was transferred.
 */
ISR(USI_OVERFLOW_vect,ISR_BLOCK)
{
  uint8_t data;
  
  data = USIDR;
  debug_puthex(data);

#if ALLOW_INTERRUPT       
  /* disable interrupt, so we can  */
  disable_overflow_interrupt();
  sei();
#endif
  
  if (i2c_slave_state & I2C_WAIT_ACK1) {
    /* We send a byte. Now try to get an acknowledge. */
    if (i2c_slave_state & I2C_WAIT_ACK2) {
      i2c_slave_state &= ~I2C_WAIT_ACK2;
      set_sda_input();
      set_status_register( 0, 1, 2 );
      goto go_leave;
    } else {
      /* check if the master acked */
      if (data & 1) {
	/* NACK */
	goto go_idle;
      } else {
	i2c_slave_state &= ~I2C_WAIT_ACK;
	set_sda_output();
	debug_putchar('$');
      }
    }
  } else if (i2c_slave_state & I2C_ACK_RECEIVE) {
    /* Done acknowledging our reception */
    i2c_slave_state &= ~I2C_ACK_RECEIVE;
    if (i2c_slave_state == I2C_IDLE) {
      goto go_idle;
    }
    /* release sda and set to input */
    set_sda_high();
    set_sda_input();
    /* wait for 16 flanks */
    debug_putchar('R');
    set_status_register( 0, 1, 16 );
    goto go_leave;
  }
  
  switch( i2c_slave_state ) {
  case I2C_ADDRESS:
    /* First 7 address bits are received. */
    if (data==(0x80|(I2C_SLAVE_ADDR>>1))) {
      /* The address, now get our send-receive bit */
      i2c_slave_state = I2C_READ_WRITE;
      /* As this is our address, we can have the hardware do the acknowledge. */

      /* Enable output. 
       * This is okay, as the start ISR made sure that we are outputing "1", i.e. nothing.
       */
      set_sda_output();
      /* Clock is down. 
       * Next clock up will shift RW bit into DR0, and 0 to DR7
       * Next clock down will latch DR7 into SDA. -> ACK
       * Next clock up will shift our 0 into DR0, and 1 out to DR7
       * Next clock down will latch DR7 into SDA, i.e. release SDA again.
       */
      USIDR = 0xbf;
      /* Wait for 4 flanks. */
      set_status_register( 0, 1, 4 );
      debug_putchar('A');
      goto go_leave;
    }
    /* not us, get off the bus */
    goto go_idle;
    break;
  case I2C_READ_WRITE:
    debug_putchar('#');
    /* This is the tail end of the address phase.
     * Only the RW bit was still needed, and the ACK was
     * already dealt with by the hardware.
     */
    /* RW bit is the second bit */
    if (data&2) {
      /* master requests data */
      i2c_slave_state = I2C_SEND_1;
      debug_putchar('w');
      // we are ready to send now
      goto go_send;
    } else {
      /* master will send more */
      set_sda_input();
      i2c_slave_state = I2C_CONTROL;
      /* prepare to receive 8 bits */
      set_status_register(0,1,16);
      debug_putchar('r');
      goto go_leave;
      return;
    }
    break;
  case I2C_CONTROL:
    /* get our control value */
    debug_putchar('C');
    i2c_slave_cmd = data;
    switch(data) {
    case 0x01: /* set pwm colors */
      i2c_slave_state = I2C_RECEIVE_6;
      goto go_ack_receive;
      break;
    case 0x10: /* read voltage */
      i2c_slave_state = I2C_IDLE;
      event_trigger( EVENT_I2C_SLAVE_CMD );
      goto go_ack_receive;
      break;
    case 0x20: /* read string */
      i2c_slave_send_buffer[0] = 0xde;
      i2c_slave_send_buffer[1] = 0xad;
      i2c_slave_send_buffer[2] = 0xbe;
      i2c_slave_send_buffer[3] = 0xef;
      i2c_slave_send_buffer[4] = 0xcc;
      i2c_slave_send_buffer[5] = 0xaa;
      goto go_ack_receive;
    default:
      goto go_idle;
    }
    break;
  case I2C_RECEIVE_1 ... I2C_RECEIVE_6:
    i2c_slave_receive_buffer[ i2c_slave_state - I2C_RECEIVE_1 ] = data;
    if (i2c_slave_state == I2C_RECEIVE_1) {
      /* got all our bytes */
      event_trigger( EVENT_I2C_SLAVE_CMD );
      i2c_slave_state = I2C_IDLE;
    } else {
      --i2c_slave_state;
    }
    goto go_ack_receive;
    break;
  case I2C_SEND_1 ... I2C_SEND_MAX:
  go_send:
    // here we start sending...
    
    USIDR = i2c_slave_send_buffer[ i2c_slave_state - I2C_SEND_1 ];
    set_status_register( 0, 1, 16 );

    if (i2c_slave_state == sizeof(i2c_slave_send_buffer) ) {
      i2c_slave_state = I2C_IDLE; /* wait for ack, and go idle */
    } else {
      ++i2c_slave_state;
    }
    /* Setup so we wait for an ACK afterwards */
    i2c_slave_state |= I2C_WAIT_ACK;
    debug_putchar('T');
    goto go_leave;
    break;
    
  case I2C_IDLE:
  default:
    goto go_idle;
  }

 go_idle:
  i2c_slave_go_idle();
  debug_putchar('i');
 go_leave:
#if ALLOW_INTERRUPT       
  cli();
  enable_overflow_interrupt();
#endif
  return;

 go_ack_receive:
  /* we need to acknowledge our receiving... */
  i2c_slave_state |= I2C_ACK_RECEIVE;

  debug_putchar('@');
  /* clock is down --> bring sda down to ack*/
  set_sda_output();
  set_sda_low();
  /* interrupt after two flanks */
  set_status_register( 0, 1, 2 );
#if ALLOW_INTERRUPT       
  cli();
  enable_overflow_interrupt();
#endif
  return;
}
