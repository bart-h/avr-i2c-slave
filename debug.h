#ifndef H_DEBUG_H
#define H_DEBUG_H

#include "my-config.h"

#define DEBUG_BUFFER_LENGTH	32

#ifdef DEBUG_DO_DEBUG

extern char debug_buffer[DEBUG_BUFFER_LENGTH];
extern volatile char debug_in_ptr;

inline __attribute__((always_inline)) void debug_putchar( char c )
{
    debug_buffer[debug_in_ptr=(debug_in_ptr+1)&(DEBUG_BUFFER_LENGTH-1)]=c;
}

extern void debug_puthex(char c );
extern void debug_output(void);
#else
inline __attribute__((always_inline)) void debug_putchar(char c)
{
	c=1;
}
inline __attribute__((always_inline)) void debug_puthex(char hex)
{
	hex=1;
}
inline __attribute__((always_inline)) void debug_output(void)
{
}

#endif

#endif
