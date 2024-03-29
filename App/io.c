/*
 * File:		io.c
 * Purpose:		Serial Input/Output routines
 *
 * Notes:       TERMINAL_PORT defined in <board>.h
 */

#include "common.h"
#include "MK60_uart.h"


/********************************************************************/
char in_char (void)
{
    char ch;
    uart_getchar(FIRE_PORT,&ch);
    return ch;
}
/********************************************************************/
void out_char (char ch)
{
    uart_putchar(FIRE_PORT, ch);
}
/********************************************************************/
/*int char_present (void)
{
	return uart_getchar_present(FIRE_PORT);
}*/
/********************************************************************/
