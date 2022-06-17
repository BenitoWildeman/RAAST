/*
UART (serial connection to PC) uses pins UCA0RXD (P1.5) and UCA0TXD (P1.4)
SPI uses pins UCA1SIMO (SI: P2.6), UCA1SOMI (SO: P2.5), UCA1STE (CS: P3.1), UCA1CLK (SCK: P2.4) and INT: P1.3
I2C uses pins UCB0SCL (P1.3) and UCB0SDA (P1.2)
*/

#include <msp430.h> 
#include <stdio.h>
#include "msp430_trim.h"
#include "msp430_serial.h"
#include "msp430_spi.h"
#include "mcp2515.h"

uint32_t rid;
uint8_t mext;
uint8_t irq, buf[8];
volatile int i;
volatile uint16_t sleep_counter;
#define SLEEP_COUNTER 0xffff

int main() {
	WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer

    Factory_Trim();
	Software_Trim();

    serial_init();
    printf("Serial port initialised\n");

	can_init();
    printf("MCP2515 initialised\n");

	if (can_speed(125000, 1, 1) < 0) {
		printf("Error setting speed\n");
	} else {
        printf("Speed set\n");
    }

	can_rx_setmask(0, 0x000000FF, 1);
	can_rx_setfilter(0, 0, 0x00000080);
	can_rx_mode(0, MCP2515_RXB0CTRL_MODE_RECV_STD_OR_EXT);

	can_ioctl(MCP2515_OPTION_LOOPBACK, 1);
	can_ioctl(MCP2515_OPTION_ONESHOT, 1);

	sleep_counter = SLEEP_COUNTER;

	while(1) {
		if (mcp2515_irq & MCP2515_IRQ_FLAGGED) {
			irq = can_irq_handler();
			if (irq & MCP2515_IRQ_RX && !(irq & MCP2515_IRQ_ERROR)) {
				i = can_recv(&rid, &mext, buf);
				if (i > 0) {
                    printf("Message received: ID: %x; Data: ", rid);
                    for (int j = 0; j < i; j++) printf("%02x", buf[j]);
                    putchar('\n');
				}
			} else if (irq & MCP2515_IRQ_ERROR) {
				can_r_reg(MCP2515_CANINTF, &mext, 1);
                printf("CANINTF: %d\n", mext);
				can_r_reg(MCP2515_EFLG, &mext, 1);
                printf("EFLG: %d\n", mext);
			}
		}
		if (!sleep_counter) {
			buf[0] = '1';
			buf[1]++;
			if (can_send(0x00000080, 1, buf, 2, 3) < 0) {
				printf("Error sending message\n");
			} else {
				printf("Message send\n");
			}
			sleep_counter = SLEEP_COUNTER;
		}
        sleep_counter--;
	}
    return 0;
}

// ISR for PORT1
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void) {
	if (P1IFG & CAN_IRQ_PORTBIT) {
		P1IFG &= ~CAN_IRQ_PORTBIT;
		mcp2515_irq |= MCP2515_IRQ_FLAGGED;
	}
}
