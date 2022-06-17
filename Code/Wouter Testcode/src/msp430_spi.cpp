#include <msp430.h> 

void spi_init() {
	P2SEL0 |= BIT4 | BIT5 | BIT6; // data sheet table 6-19: P2SELx set to 01 for SPI for P2.4, P2.5, and P2.6
//    P3SEL0 |= BIT1;	              // P3SELx set to 01 for P3.1 (STE)
    UCA1CTLW0 = UCSWRST;          // eUSCI logic set to reset state
    UCA1CTLW0 |= UCMST            // master mode
               | UCSYNC           // synchronous mode
               | UCCKPL           // clock polarity: active high
               | UCSSEL__SMCLK    // clock source: SMCLK
               | UCMSB;            // MSB first
//               | UCMODE1          // 4-pin SPI with UCxSTE active low: Slave enabled when UCxSTE = 0
//               | UCSTEM;          // STE pin is used to generate the enable signal for a 4-wire slave
    UCA1BRW = 0x01;               // bit clock prescaler 1
    UCA1CTLW0 &= ~UCSWRST;        // release eUSCI reset for operation
    PM5CTL0 &= ~LOCKLPM5;         // clear LOCKLPM5 bit to let settings take effect
}

unsigned char spi_transfer(unsigned char inb)
{
    while (!(UCA1IFG & UCTXIFG)); // wait for empty buffer
    UCA1TXBUF = inb;              // start transfer, clear UCTXIFG
    while (!(UCA1IFG & UCRXIFG)); // wait for transfer complete
	return UCA1RXBUF;
}
