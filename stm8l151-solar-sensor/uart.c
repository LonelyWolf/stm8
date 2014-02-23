#include "iostm8l152c6.h"
#include "stdint.h"
#include "string.h"
#include "uart.h"


void UART_Init(void) {
    PC_DDR_bit.DDR3 = 1; // Set TX pin (PC3) as output
    PC_CR1_bit.C13  = 1; // Configure PC3 as push-pull
    PC_CR2_bit.C23  = 1; // 10MHz output
    PC_ODR_bit.ODR3 = 1; // Set 1 for output (remove trash in first byte transfered)

    CLK_PCKENR1_bit.PCKEN15 = 1; // Enable USART peripherial (PCKEN15)

    USART1_CR1 = 0; // USARTD enable, Parity control disable
    USART1_CR3 = 0; // 1 STOP bit
    USART1_BRR2 = 0x04; // 19200bps at 2MHz CPU clock
    USART1_BRR1 = 0x03;
    USART1_CR4 = 0; // USART address = 0
    USART1_CR2_bit.TEN = 1; // Transmitter enable
}

void UART_SendChar(char ch) {
    while (!(USART1_SR_bit.TXE)); // Wait until data register is empty
    USART1_DR = ch; // Push byte into USART data register
    while (!(USART1_SR_bit.TC)); // Wait until transmit complete
}

void UART_SendChars(char ch, uint8_t count) {
    uint8_t i;

    for (i = 0; i < count; i++) UART_SendChar(ch);
}

void UART_SendInt(int32_t num) {
    char str[10]; // 10 chars max for INT32_MAX
    int i = 0;
    if (num < 0) {
        UART_SendChar('-');
	num *= -1;
    }
    do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
    for (i--; i >= 0; i--) UART_SendChar(str[i]);
}

void UART_SendHex8(uint16_t num) {
    UART_SendChar(HEX_CHARS[(num >> 4)   % 0x10]);
    UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendHex16(uint16_t num) {
    uint8_t i;
    for (i = 12; i > 0; i -= 4) UART_SendChar(HEX_CHARS[(num >> i) % 0x10]);
    UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendHex32(uint32_t num) {
    uint8_t i;
    for (i = 28; i > 0; i -= 4)	UART_SendChar(HEX_CHARS[(num >> i) % 0x10]);
    UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendStr(char *str) {
    while (*str) UART_SendChar(*str++);
}

void UART_SendBuf(char *buf, uint16_t bufsize) {
    uint16_t i;
    for (i = 0; i < bufsize; i++) UART_SendChar(*buf++);
}

void UART_SendBufPrintable(char *buf, uint16_t bufsize, char subst) {
    uint16_t i;
    char ch;
    for (i = 0; i < bufsize; i++) {
        ch = *buf++;
	UART_SendChar(ch > 32 ? ch : subst);
    }
}

void UART_SendBufHex(char *buf, uint16_t bufsize) {
    uint16_t i;
    char ch;
    for (i = 0; i < bufsize; i++) {
        ch = *buf++;
	UART_SendChar(HEX_CHARS[(ch >> 4)   % 0x10]);
	UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
    }
}
