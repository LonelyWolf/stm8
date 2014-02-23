#include "stdint.h"
#include "iostm8l051f3.h"
#include "stm8l051.h"

#include "nRF24.h"


uint8_t nRF24_RX_addr[nRF24_RX_ADDR_WIDTH] = {'W','o','l','k','T'};
uint8_t nRF24_TX_addr[nRF24_TX_ADDR_WIDTH] = {'W','o','l','k','T'};


// Check the specified SPI flag
FlagStatus SPI_GetFlagStatus(SPI_FLAG_TypeDef flag) {
    return ((SPI1_SR & (uint8_t)flag) != (uint8_t)RESET) ? SET : RESET;
}

// Transmit data byte through the SPI
void SPI_SendData(uint8_t data) {
    SPI1_DR = data;
}

// Return most recent received data by the SPI
uint8_t SPI_ReceiveData(void) {
    return (uint8_t)SPI1_DR;
}

// GPIO and SPI initialization
void nRF24_init() {
    // IRQ  --> PB2
    // CE   <-- PB3
    // CSN  <-- PB4
    // SCK  <-- PB5
    // MOSI <-- PB6
    // MISO --> PB7

    // SCK,MOSI,CSN,CE pins set as output fiwth push-pull at 10MHz
    PB_DDR |= 0x78; // Set PB3..PB6 as output
    PB_CR1 |= 0x78; // Configure PB3..PB6 as output with push-pull
    PB_CR2 |= 0x78; // Set 10MHz output speed for PB3..PB6 pins
    //PB_CR2 &= ~(0x78); // Set 2MHz output speed for PB3..PB6 pins

    // MISO pin set as input with pull-up
    PB_DDR_bit.DDR7 = 0; // Set PB7 as input
    PB_CR1_bit.C17  = 1; // Configure PB7 as input with pull-up
    PB_CR2_bit.C27  = 0; // Disable external interrupt for PB7

    // IRQ pin set as input with pull-up
    PC_DDR_bit.DDR0 = 0; // Input
    PC_CR1_bit.C10  = 1; // Pull-up
    PC_CR2_bit.C20  = 0; // Disable external interrupt

    // Configure SPI
    CLK_PCKENR1_bit.PCKEN14 = 1; // Enable SPI peripherial (PCKEN14)

    /*
    SPI1_CR1_bit.BR       = 0; // Baud = f/2 (1MHz at 2MHz CPU)
    SPI1_CR1_bit.CPHA     = 0; // CPHA = 1st edge
    SPI1_CR1_bit.CPOL     = 0; // CPOL = low (SCK low when idle)
    SPI1_CR1_bit.LSBFIRST = 0; // first bit is MSB
    SPI1_CR1_bit.MSTR     = 1; // Master configuration
    SPI1_CR1_bit.SPE      = 0; // Peripherial enabled
    */
    SPI1_CR1 = 0x04; // SPI: MSB first, Baud=f/2, Master, CPOL=low, CPHA=1st edge
    //SPI1_CR1 = 0x24; // SPI: MSB first, Baud=f/32, Master, CPOL=low, CPHA=1st edge

    /*
    SPI1_CR2_bit.BDM    = 0; // 2-line unidirectional data mode
    SPI1_CR2_bit.BD0E   = 0; // don't care when BDM set to 0
    SPI1_CR2_bit.RXOnly = 0; // Full duplex
    SPI1_CR2_bit.SSI    = 1; // Master mode
    SPI1_CR2_bit.SSM    = 1; // Software slave management enabled
    */
    SPI1_CR2 = 0x03; // SPI: 2-line mode, full duplex, SSM on (master mode)

    SPI1_CR1_bit.SPE = 1; // SPI peripherial enabled

    CSN_H();
    CE_L(); // CE pin low -> power down mode at startup
}

// Send/Receive data to nRF24L01 via SPI
// input:
//   data - byte to send
// output: received byte from nRF24L01
uint8_t nRF24_ReadWrite(uint8_t data) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET); // Wait while DR register is not empty
    SPI_SendData(data); // Send byte to SPI
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET); // Wait to receive byte
    return SPI_ReceiveData(); // Read byte from SPI bus
}

// Write new value to register
// input:
//   reg - register number
//   value - new value
// output: nRF24L01 status
uint8_t nRF24_RWReg(uint8_t reg, uint8_t value) {
    uint8_t status;

    CSN_L();
    status = nRF24_ReadWrite(reg); // Select register
    nRF24_ReadWrite(value); // Write value to register
    CSN_H();

    return status;
}

// Read nRF24L01 register
// input:
//   reg - register number
// output: register value
uint8_t nRF24_ReadReg(uint8_t reg) {
    uint8_t value;

    CSN_L();
    nRF24_ReadWrite(reg);
    value = nRF24_ReadWrite(0);
    CSN_H();

    return value;
}

// Get data from nRF24L01 into buffer
// input:
//   reg - register number
//   pBuf - pointer to buffer
//   count - bytes count
// output: nRF24L01 status
uint8_t nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    uint8_t status,i;

    CSN_L();
    status = nRF24_ReadWrite(reg);
    for (i = 0; i < count; i++) pBuf[i] = nRF24_ReadWrite(0);
    CSN_L();

    return status;
}

// Send buffer to nRF24L01
// input:
//   reg - register number
//   pBuf - pointer to buffer
//   count - bytes count
// output: nRF24L01 status
uint8_t nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    uint8_t status,i;

    CSN_L();
    status = nRF24_ReadWrite(reg);
    for (i = 0; i < count; i++) nRF24_ReadWrite(*pBuf++);
    CSN_H();

    return status;
}

// Check if nRF24L01 present (send byte sequence, read it back and compare)
// return:
//   0 - looks like an nRF24L01 is online
//   1 - received sequence differs from original
uint8_t nRF24_Check(void) {
    uint8_t txbuf[5] = { 'W','o','l','k','?' };
    uint8_t rxbuf[5];
    uint8_t i;

    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,txbuf,5); // Write fake TX address
    nRF24_ReadBuf(nRF24_REG_TX_ADDR,rxbuf,5); // Try to read TX_ADDR register
    for (i = 0; i < 5; i++) if (rxbuf[i] != txbuf[i]) return 1;

    return 0;
}

// Put nRF24L01 in RX mode
void nRF24_RXMode(uint8_t RX_PAYLOAD) {
    CE_L();
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable ShockBurst for data pipe 0
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_RXADDR,0x01); // Enable data pipe 0
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,0x6E); // Set frequency channel 110 (2.510MHz)
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RX_PW_P0,RX_PAYLOAD); // Set RX payload length
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,0x06); // Setup: 1Mbps, 0dBm, LNA off
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0F); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PRX
    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P0,nRF24_RX_addr,nRF24_RX_ADDR_WIDTH); // Set static RX address
    CE_H();
    // Delay_us(10); // Hold CE high at least 10us
}

// Put nRF24L01 in TX mode
void nRF24_TXMode(void) {
    CE_L();
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable ShockBurst for data pipe 0
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_SETUP_RETR,0x1A); // Auto retransmit: wait 500us, 10 retries
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,0x6E); // Set frequency channel 110 (2.510MHz)
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,0x06); // Setup: 1Mbps, 0dBm, LNA off
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0E); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PTX
}

// Receive data packet
// input:
//   pBuf - buffer for received data
// return:
//   0 if no data received and nonzero otherwise
uint8_t nRF24_RXPacket(uint8_t* pBuf) {

    return 0;
}

// Send data packet
// input:
//   pBuf - buffer with data to send
// return:
//   nRF24_MASK_MAX_RT - if transmit failed with maximum auto retransmit count
//   nRF24_MAX_TX_DS - if transmit succeed
//   contents of STATUS register otherwise
uint8_t nRF24_TXPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD) {
    uint8_t status;

    CE_L();
    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,nRF24_TX_addr,nRF24_TX_ADDR_WIDTH); // Set static TX address
    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P0,nRF24_RX_addr,nRF24_RX_ADDR_WIDTH); // Set static RX address for auto ack
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable auto acknowledgement for data pipe 0
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_SETUP_RETR,0x1A); // Automatic retransmission: wait 500us, 10 retries
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,0x6E); // Set frequency channel 110 (2.510MHz)
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,0x07); // Setup: 1Mbps, 0dBm, LNA on
    nRF24_WriteBuf(nRF24_CMD_W_TX_PAYLOAD,pBuf,TX_PAYLOAD); // Write specified buffer to FIFO
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0E); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PTX
    CE_H(); // CE pin high => Start transmit
    // Delay_us(10); // Must hold CE at least 10us
    while(PC_IDR_bit.IDR0 != 0); // Wait for IRQ from nRF24L01
    CE_L();
    status = nRF24_ReadReg(nRF24_REG_STATUS); // Read status register
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status); // Reset TX_DS and MAX_RT bits
    if (status & nRF24_MASK_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit. FIFO is not removed.
        nRF24_RWReg(nRF24_CMD_FLUSH_TX,0xFF); // Flush TX FIFO buffer
        return nRF24_MASK_MAX_RT;
    };
    if (status & nRF24_MASK_TX_DS) {
        // Transmit ok
        nRF24_RWReg(nRF24_CMD_FLUSH_TX,0xFF); // Flush TX FIFO buffer
        return nRF24_MASK_TX_DS;
    }

    // Some banana happens
    return status;
}

// Put nRF24 in Power Down mode
void nRF24_PowerDown(void) {
    uint8_t conf;

    CE_L(); // CE pin to low
    conf = nRF24_ReadReg(nRF24_REG_CONFIG);
    conf &= ~(1<<1); // Clear PWR_UP bit
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,conf); // Go Power down mode
}

// Wake nRF24 from Power Down mode (usually wakes to Standby-I mode within 1.5ms)
void nRF24_Wake(void) {
    uint8_t conf;

    conf = nRF24_ReadReg(nRF24_REG_CONFIG);
    conf |= (1<<1); // Set PWR_UP bit
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,conf); // Wakeup
    // Delay_ms(2); // Wakeup from Power Down to Standby-I mode takes 1.5ms
}
