#include "debug.h"
#include "uart.h"

#ifdef DEBUG_DO_DEBUG

char debug_buffer[DEBUG_BUFFER_LENGTH];
volatile char debug_in_ptr=0;
char debug_out_ptr=0;

static void debug_put_nibble( char c )
{
  if (c<10)
    debug_putchar(c+'0');
  else
    debug_putchar(c+'a'-10);
}

void debug_puthex( char c )
{
  debug_put_nibble( (c>>4)&0x0f );
  debug_put_nibble( c&0x0f );
}

void debug_output(void)
{
    if (debug_out_ptr!=debug_in_ptr) {
        debug_out_ptr=(debug_out_ptr+1)&(DEBUG_BUFFER_LENGTH-1);
	uart_putchar(debug_buffer[debug_out_ptr]);
    }
}
	    
#endif
