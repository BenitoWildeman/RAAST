#include <msp430.h>

int putchar(int c) {
    while (!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = c;
    return c;
}

void serial_init() {
	P1SEL0 = BIT4 | BIT5;       // configure UART pins
	UCA0CTL1 |= UCSWRST;        // disable UART
	UCA0CTL1 |= UCSSEL__SMCLK;  // clock source SMCLK (=8MHz)
	UCA0CTL0 = 0;               // no parity, 8 data bits, 1 stop bit
	UCA0ABCTL = 0;              // no auto baud rate detection
	/*
	N = 8MHz / 9600 = 833.33
	UCOS16 = 1
	UCBR = int(833.33 / 16) = 52
	UCBRF = int((833.33 / 16 - 52) * 16) = 1
	UCBRS = 0x49 for .33 (table 22-4)
	*/
	UCA0BRW = 52;               // settings for 9600 baud
	UCA0MCTLW_H = 0x49;         // UCBRS
	UCA0MCTLW_L = 0x11;         // UCBRF and UCOS16
	UCA0CTL1 &= ~UCSWRST;       // enable UART
}
