#include <stdint.h>
#include <iostm8l051f3.h>

#include "stm8l051.h"
#include "nRF24.h"


// If defined wait for IRQ in polling mode, WFI otherwise
//#define IRQ_POLL


const uint8_t RX_PW_PIPES[6] = {
    nRF24_REG_RX_PW_P0,
    nRF24_REG_RX_PW_P1,
    nRF24_REG_RX_PW_P2,
    nRF24_REG_RX_PW_P3,
    nRF24_REG_RX_PW_P4,
    nRF24_REG_RX_PW_P5
};
const uint8_t RX_ADDR_PIPES[6] = {
    nRF24_REG_RX_ADDR_P0,
    nRF24_REG_RX_ADDR_P1,
    nRF24_REG_RX_ADDR_P2,
    nRF24_REG_RX_ADDR_P3,
    nRF24_REG_RX_ADDR_P4,
    nRF24_REG_RX_ADDR_P5
};


// GPIO and SPI initialization
void nRF24_init() {
    // IRQ  --> PC1
    // CE   <-- PB3
    // CSN  <-- PB4
    // SCK  <-- PB5
    // MOSI <-- PB6
    // MISO --> PB7

    // SCK,MOSI,CSN,CE pins (PB3..PB6) set as output with push-pull at 10MHz
    PB_DDR |= 0x78; // Output
    PB_CR1 |= 0x78; // Push-pull
    PB_CR2 |= 0x78; // 10MHz output speed

    // MISO pin (PB7) set as input with pull-up
    PB_DDR_bit.DDR7 = 0; // Input
    PB_CR1_bit.C17  = 1; // Pull-up
    PB_CR2_bit.C27  = 0; // Disable external interrupt

    // IRQ pin (PC1) set as input with pull-up
    PC_DDR_bit.DDR1 = 0; // Input
    PC_CR1_bit.C11  = 1; // Pull-up
    PC_CR2_bit.C21  = 0; // Disable external interrupt

    // Configure SPI
    CLK_PCKENR1_bit.PCKEN14 = 1; // Enable SPI peripheral (PCKEN14)

    /*
    SPI1_CR1_bit.BR       = 0; // Baud = f/2 (1MHz at 2MHz CPU)
    SPI1_CR1_bit.CPHA     = 0; // CPHA = 1st edge
    SPI1_CR1_bit.CPOL     = 0; // CPOL = low (SCK low when idle)
    SPI1_CR1_bit.LSBFIRST = 0; // first bit is MSB
    SPI1_CR1_bit.MSTR     = 1; // Master configuration
    SPI1_CR1_bit.SPE      = 0; // Peripheral enabled
    */
    SPI1_CR1 = 0x04; // SPI: MSB first, Baud=f/2, Master, CPOL=low, CPHA=1st edge

    /*
    SPI1_CR2_bit.BDM    = 0; // 2-line unidirectional data mode
    SPI1_CR2_bit.BD0E   = 0; // don't care when BDM set to 0
    SPI1_CR2_bit.RXOnly = 0; // Full duplex
    SPI1_CR2_bit.SSI    = 1; // Master mode
    SPI1_CR2_bit.SSM    = 1; // Software slave management enabled
    SPI1_CR2_bit.CRCEN  = 0; // CRC disabled
    */
    SPI1_CR2 = 0x03; // SPI: 2-line mode, full duplex, SSM on (master mode), no CRC

    SPI1_CR1_bit.SPE = 1; // SPI peripheral enabled

    CSN_H();
    CE_L(); // CE pin low -> power down mode at startup

    nRF24_ClearIRQFlags();
}

// Transmit byte via SPI
// input:
//   data - byte to send
// return: received byte from SPI
uint8_t SPI1_SendRecv(uint8_t data) {
    uint8_t rcv;

    SPI1_DR = data; // Send byte to SPI (TXE cleared)
    while (!(SPI1_SR_bit.RXNE)); // Wait until byte is received
    rcv = SPI1_DR; // Read received byte (RXNE cleared)
    while (!(SPI1_SR_bit.TXE)); // Wait for TXE flag --> transmit buffer is empty
    while (SPI1_SR_bit.BSY); // Wait until the transmission is complete

    return rcv;
}

// Write new value to the nRF24L01 register
// input:
//   reg - register number
//   value - new value
// return: nRF24L01 status
void nRF24_WriteReg(uint8_t reg, uint8_t value) {
    CSN_L();
    SPI1_SendRecv(nRF24_CMD_WREG | reg); // Select register
    SPI1_SendRecv(value); // Write value to register
    CSN_H();
}

// Read value of the nRF24L01 register
// input:
//   reg - register number
// return: register value
uint8_t nRF24_ReadReg(uint8_t reg) {
    uint8_t value;

    CSN_L();
    SPI1_SendRecv(reg & 0x1f); // Select register to read from
    value = SPI1_SendRecv(nRF24_CMD_NOP); // Read register value
    CSN_H();

    return value;
}

// Read specified amount of data from the nRF24L01 into data buffer
// input:
//   reg - register number
//   pBuf - pointer to the data buffer
//   count - number of bytes to read
void nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    CSN_L();
    SPI1_SendRecv(reg); // Send buffer address
    SPI1_DR = nRF24_CMD_NOP; // Transmit first dummy byte (clears the TXE flag)
    while (--count) {
        while (!(SPI1_SR_bit.TXE)); // Wait until TX buffer is empty
        while (!(SPI1_SR_bit.RXNE)); // Wait while RX buffer is empty
        *pBuf++ = SPI1_DR; // Read received byte into buffer (clears the RXNE flag)
        SPI1_DR = nRF24_CMD_NOP; // Transmit dummy byte
    }
    while (!(SPI1_SR_bit.RXNE)); // Wait while RX buffer is empty
    *pBuf++ = SPI1_DR; // Read last received byte
    while (!(SPI1_SR_bit.TXE)); // Wait until TX buffer is empty
    while (SPI1_SR_bit.BSY); // Wait until the transmission is complete
    CSN_H();
}

// Send data buffer to the nRF24L01
// input:
//   reg - register number
//   pBuf - pointer to the data buffer
//   count - number of bytes to write
void nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    CSN_L();
    SPI1_SendRecv(nRF24_CMD_WREG | reg); // Send buffer address
    SPI1_DR = *pBuf++; // Transmit first byte (clears the TXE the flag)
    while (--count) {
        while (!(SPI1_SR_bit.TXE)); // Wait until TX buffer is empty
        while (!(SPI1_SR_bit.RXNE)); // Wait while RX buffer is empty
        (void)SPI1_DR; // Clear the RXNE flag
        SPI1_DR = *pBuf++; // Transmit byte
    }
    while (!(SPI1_SR_bit.RXNE)); // Wait while RX buffer is empty
    (void)SPI1_DR; // Clear the RXNE flag
    while (!(SPI1_SR_bit.TXE)); // Wait until TX buffer is empty
    while (SPI1_SR_bit.BSY); // Wait until the transmission is complete
    CSN_H();
}

// Check if nRF24L01 present (send byte sequence, read it back and compare)
// return:
//   1 - looks like an nRF24L01 is online
//   0 - received sequence differs from original
uint8_t nRF24_Check(void) {
    uint8_t rxbuf[5];
    uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;
    uint8_t i = 5;

    nRF24_WriteBuf(nRF24_REG_TX_ADDR,ptr,5); // Write fake TX address
    nRF24_ReadBuf(nRF24_REG_TX_ADDR,rxbuf,5); // Read TX_ADDR register
    for (i = 0; i < 5; i++) if (rxbuf[i] != *ptr++) return 0;

    return 1;
}

// Set nRF24L01 frequency channel
// input:
//   RFChannel - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
// Note, what part of the OBSERVER_TX register called "PLOS_CNT" will be cleared!
void nRF24_SetRFChannel(uint8_t RFChannel) {
    nRF24_WriteReg(nRF24_REG_RF_CH,RFChannel);
}

// Flush nRF24L01 TX FIFO buffer
void nRF24_FlushTX(void) {
	nRF24_WriteReg(nRF24_CMD_FLUSH_TX,0xFF);
}

// Flush nRF24L01 RX FIFO buffer
void nRF24_FlushRX(void) {
	nRF24_WriteReg(nRF24_CMD_FLUSH_RX,0xFF);
}

// Put nRF24L01 in TX mode
// input:
//   RetrCnt - Auto retransmit count on fail of AA (1..15 or 0 for disable)
//   RetrDelay - Auto retransmit delay 250us+(0..15)*250us (0 = 250us, 15 = 4000us)
//   RFChan - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
//   DataRate - Set data rate: nRF24_DataRate_1Mbps or nRF24_DataRate_2Mbps
//   TXPower - RF output power (-18dBm, -12dBm, -6dBm, 0dBm)
//   CRCS - CRC encoding scheme (nRF24_CRC_[off | 1byte | 2byte])
//   Power - power state (nRF24_PWR_Up or nRF24_PWR_Down)
//   TX_Addr - buffer with TX address
//   TX_Addr_Width - size of the TX address (3..5 bytes)
void nRF24_TXMode(uint8_t RetrCnt, uint8_t RetrDelay, uint8_t RFChan, nRF24_DataRate_TypeDef DataRate,
                  nRF24_TXPower_TypeDef TXPower, nRF24_CRC_TypeDef CRCS, nRF24_PWR_TypeDef Power, uint8_t *TX_Addr,
                  uint8_t TX_Addr_Width) {
    uint8_t rreg;

    CE_L();
    nRF24_ReadReg(0x00); // Dummy read
    nRF24_WriteReg(nRF24_REG_SETUP_AW,TX_Addr_Width - 2); // Set address width
    nRF24_WriteBuf(nRF24_REG_TX_ADDR,TX_Addr,TX_Addr_Width); // Set static TX address
    nRF24_WriteReg(nRF24_REG_RF_SETUP,(uint8_t)DataRate | (uint8_t)TXPower); // Setup register
    nRF24_WriteReg(nRF24_REG_CONFIG,(uint8_t)CRCS | (uint8_t)Power | nRF24_PRIM_TX); // Config register
    nRF24_SetRFChannel(RFChan); // Set frequency channel (OBSERVER_TX part PLOS_CNT will be cleared)
    rreg = nRF24_ReadReg(nRF24_REG_EN_AA);
    nRF24_WriteReg(nRF24_REG_SETUP_RETR,(RetrDelay << 4) | (RetrCnt & 0x0f)); // Auto retransmit settings
    if (RetrCnt) {
        // Enable auto acknowledgment for data pipe 0
        rreg |= nRF24_ENAA_P0;
        // Static RX address of the PIPE0 must be same as TX address for auto ack
        nRF24_WriteBuf(nRF24_REG_RX_ADDR_P0,TX_Addr,TX_Addr_Width);
    } else {
        // Disable auto acknowledgment for data pipe 0
        rreg &= ~nRF24_ENAA_P0;
    }
    nRF24_WriteReg(nRF24_REG_EN_AA,rreg);
}

// Put nRF24L01 in RX mode
// input:
//   PIPE - RX data pipe (nRF24_RX_PIPE[0..5])
//   PIPE_AA - auto acknowledgment for data pipe (nRF24_ENAA_P[0..5] or nRF24_ENAA_OFF)
//   RFChan - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
//   DataRate - Set data rate (nRF24_DataRate_[250kbps,1Mbps,2Mbps])
//   CRCS - CRC encoding scheme (nRF24_CRC_[off | 1byte | 2byte])
//   RX_Addr - buffer with TX address
//   RX_Addr_Width - size of TX address (3..5 byte)
//   RX_PAYLOAD - receive buffer length
//   TXPower - RF output power for ACK packets (-18dBm, -12dBm, -6dBm, 0dBm)
void nRF24_RXMode(nRF24_RX_PIPE_TypeDef PIPE, nRF24_ENAA_TypeDef PIPE_AA, uint8_t RFChan,
                  nRF24_DataRate_TypeDef DataRate, nRF24_CRC_TypeDef CRCS, uint8_t *RX_Addr, uint8_t RX_Addr_Width,
                  uint8_t RX_PAYLOAD, nRF24_TXPower_TypeDef TXPower) {
    uint8_t rreg;

    CE_L();
    nRF24_ReadReg(nRF24_CMD_NOP); // Dummy read
    rreg = nRF24_ReadReg(nRF24_REG_EN_AA);
    if (PIPE_AA != nRF24_ENAA_OFF) {
        // Enable auto acknowledgment for given data pipe
        rreg |= (uint8_t)PIPE_AA;
    } else {
        // Disable auto acknowledgment for given data pipe
        rreg &= ~(1 << (uint8_t)PIPE);
    }
    nRF24_WriteReg(nRF24_REG_EN_AA,rreg);
    rreg = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
    nRF24_WriteReg(nRF24_REG_EN_RXADDR,rreg | (1 << (uint8_t)PIPE)); // Enable given data pipe
    nRF24_WriteReg(RX_PW_PIPES[(uint8_t)PIPE],RX_PAYLOAD); // Set RX payload length
    nRF24_WriteReg(nRF24_REG_RF_SETUP,(uint8_t)DataRate | (uint8_t)TXPower); // SETUP register
    nRF24_WriteReg(nRF24_REG_CONFIG,(uint8_t)CRCS | nRF24_PWR_Up | nRF24_PRIM_RX); // Config register
    nRF24_SetRFChannel(RFChan); // Set frequency channel
    nRF24_WriteReg(nRF24_REG_SETUP_AW,RX_Addr_Width - 2); // Set of address widths (common for all data pipes)
    nRF24_WriteBuf(RX_ADDR_PIPES[(uint8_t)PIPE],RX_Addr,RX_Addr_Width); // Set static RX address for given data pipe
    nRF24_ClearIRQFlags();
    nRF24_FlushRX();
    CE_H(); // RX mode
}

// Send data packet
// input:
//   pBuf - buffer with data to send
//   TX_PAYLOAD - buffer size
// return:
//   nRF24_TX_XXX values
nRF24_TX_PCKT_TypeDef nRF24_TXPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD) {
    uint8_t status;

#ifdef IRQ_POLL
    uint32_t wait = nRF24_WAIT_TIMEOUT;

    // This part is for simple wait for IRQ (polling the nRF24L01 IRQ pin)

    // Release CE pin (in case if it still high)
    CE_L();
    // Transmit data from the specified buffer to the TX FIFO
    nRF24_WriteBuf(nRF24_CMD_W_TX_PAYLOAD,pBuf,TX_PAYLOAD);
    // CE pin high => Start transmit (must hold pin at least 10us)
    CE_H();
    // Wait for and IRQ from nRF24L01
    while(PC_IDR_bit.IDR1 && --wait); // Wait for IRQ from nRF24L01
    if (!wait) return nRF24_TX_TIMEOUT;
    // Release CE pin
    CE_L();
#else
    // This part is for more power saving (waiting for nRF24L01 IRQ in sleep mode)

    // Enable external interrupt
    PC_CR2_bit.C21 = 1;
    // Release CE pin (in case if it still high)
    CE_L();
    // Transmit data from the specified buffer to the TX FIFO
    nRF24_WriteBuf(nRF24_CMD_W_TX_PAYLOAD,pBuf,TX_PAYLOAD);
    // CE pin high => Start transmit (must hold pin at least 10us)
    CE_H();
    // In theory there should be a WFE instruction instead of WFI, but according to the ST errata sheet
    // my be "incorrect code execution when WFE instruction is interrupted by ISR or event"
    // So WFI executed here and EXTI1 IRQ bit cleared in EXTI1_IRQHandler() procedure
    asm("WFI"); // Wait for the transmission ends (IRQ from nRF24L01)
    CE_L(); // Release CE pin (Standby-II -> Standby-I)
    PC_CR2_bit.C21 = 0; // Disable external interrupt
#endif

    // Common part further...

    // Read the status register
    status = nRF24_ReadReg(nRF24_REG_STATUS);
    // Clear pending IRQ flags
    nRF24_WriteReg(nRF24_REG_STATUS,status | 0x70);
    if (status & nRF24_MASK_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit. FIFO is not removed.
        nRF24_FlushTX();

        return nRF24_TX_MAXRT;
    };
    if (status & nRF24_MASK_TX_DS) {
        // Transmit successful
        return nRF24_TX_SUCCESS;
    }

    // Some banana happens
    nRF24_FlushTX();
    nRF24_ClearIRQFlags();

    return nRF24_TX_ERROR;
}

// Read data packet from the nRF24L01
// input:
//   pBuf - buffer for received data
//   RX_PAYLOAD - buffer size
// return:
//   nRF24_RX_PCKT_PIPE[0..5] - packet received from specific data pipe
//   nRF24_RX_PCKT_ERROR - RX_DR bit was not set
//   nRF24_RX_PCKT_EMPTY - RX FIFO is empty
nRF24_RX_PCKT_TypeDef nRF24_RXPacket(uint8_t * pBuf, uint8_t RX_PAYLOAD) {
    uint8_t status;
    nRF24_RX_PCKT_TypeDef result = nRF24_RX_PCKT_ERROR;

    status = nRF24_ReadReg(nRF24_REG_STATUS); // Read the status register
    if (status & nRF24_MASK_RX_DR) {
        // RX_DR bit set (Data ready RX FIFO interrupt)
        result = (nRF24_RX_PCKT_TypeDef)((status & 0x0e) > 1); // Pipe number
        if ((uint8_t)result < 6) {
            // Read received payload from RX FIFO buffer
            nRF24_ReadBuf(nRF24_CMD_R_RX_PAYLOAD,pBuf,RX_PAYLOAD);
            // Clear pending IRQ flags
            nRF24_WriteReg(nRF24_REG_STATUS,status | 0x70);
            // Check if RX FIFO is empty and flush it if not
            status = nRF24_ReadReg(nRF24_REG_FIFO_STATUS);
            if (status & nRF24_FIFO_RX_EMPTY) nRF24_FlushRX();

            return result; // Data pipe number
        } else {
            // RX FIFO is empty
            return nRF24_RX_PCKT_EMPTY;
        }
    }

    // Some banana happens
    nRF24_FlushRX(); // Flush the RX FIFO buffer
    nRF24_ClearIRQFlags();

    return result;
}

// Clear pending IRQ flags
void nRF24_ClearIRQFlags(void) {
    uint8_t status;

    status = nRF24_ReadReg(nRF24_REG_STATUS);
    nRF24_WriteReg(nRF24_REG_STATUS,status | 0x70);
}

// Put nRF24 in Power Down mode
void nRF24_PowerDown(void) {
    uint8_t conf;

    CE_L(); // CE pin to low
    conf  = nRF24_ReadReg(nRF24_REG_CONFIG);
    conf &= ~(1<<1); // Clear PWR_UP bit
    nRF24_WriteReg(nRF24_REG_CONFIG,conf); // Go Power down mode
}

// Wake nRF24 from Power Down mode
// note: with external crystal it wake to Standby-I mode within 1.5ms
void nRF24_Wake(void) {
    uint8_t conf;

    conf = nRF24_ReadReg(nRF24_REG_CONFIG) | (1<<1); // Set PWR_UP bit
    nRF24_WriteReg(nRF24_REG_CONFIG,conf); // Wake-up
}

// Configure RF output power in TX mode
// input:
//   TXPower - RF output power (-18dBm, -12dBm, -6dBm, 0dBm)
void nRF24_SetTXPower(nRF24_TXPower_TypeDef TXPower) {
    uint8_t rf_setup;

    rf_setup  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
    rf_setup &= 0xf9; // Clear RF_PWR bits
    nRF24_WriteReg(nRF24_REG_RF_SETUP,rf_setup | (uint8_t)TXPower);
}
