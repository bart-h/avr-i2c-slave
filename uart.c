#include "my-config.h"
#include <avr/io.h>
#include <stdlib.h>

void uart_init(void)
{
#if F_CPU < 2000000UL && defined(U2X)
  UCSRA = _BV(U2X);             /* improve baud rate error by using 2x clk */
  UBRRL = (F_CPU / (8UL * UART_BAUD)) - 1;
#else
  UBRRL = (F_CPU / (16UL * UART_BAUD)) - 1;
#endif
  //  UCSRB = _BV(TXEN) | _BV(RXEN); /* tx/rx enable */
  UCSRB = _BV(TXEN); /* tx enable */
}

void uart_putchar( char c )
{
  loop_until_bit_is_set(UCSRA, UDRE);
  UDR = c;
}

static void uart_put_nibble( char c )
{
  if (c<10)
    uart_putchar(c+'0');
  else
    uart_putchar(c+'A'-10);
}

void uart_puthex( char c )
{
  uart_put_nibble( (c>>4)&0x0f );
  uart_put_nibble( c&0x0f );
}

void uart_print_number( int number )
{
  div_t arg;
  char buf[10];
  uint8_t c;
  arg.quot = (number<0) ? -number : number;
  for( c=7; ; c--) {
    arg = div( arg.quot, 10 );
    buf[c] = '0'+arg.rem;
    if (arg.quot==0)
      break;
  }
  if (number<0)
    uart_putchar('-');
  for( ; (c<8) ; c++) {
    uart_putchar(buf[c]);
  }
}

void uart_newline(void)
{
  uart_putchar('\r');
  uart_putchar('\n');
}
