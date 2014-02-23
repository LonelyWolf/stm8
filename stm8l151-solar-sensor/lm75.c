#include "iostm8l151k6.h"

#include "stm8l15x.h"
#include "stdint.h"
#include "intrinsics.h"

#include "lm75.h"


// Enable or disable I2C acknowledge feature
void I2C_AcknowledgeConfig(FunctionalState newstate) {
    I2C1_CR2_bit.ACK = (newstate == ENABLE) ? 1 : 0;
}

// Generate I2C communication START condition
void I2C_GenerateSTART(FunctionalState newstate) {
    I2C1_CR2_bit.START = (newstate == ENABLE) ? 1 : 0;
}

// Generate I2C communication STOP condition
void I2C_GenerateSTOP(FunctionalState newstate) {
    I2C1_CR2_bit.STOP = (newstate == ENABLE) ? 1 : 0;
}

// Select the specified I2C Ack position
//   I2C_AckPosition_Current - Ack on the current byte
//   I2C_AckPosition_Next - Ack on the next byte (for 2 bytes reception)
// Must be called before any data reception starts
void I2C_AckPositionConfig(I2C_AckPosition_TypeDef I2C_AckPosition) {
    I2C1_CR2_bit.POS = 0; // Clear Ack position
    I2C1_CR2_bit.POS = I2C_AckPosition; // Configure the specified Ack position
}

// Transmit 7-bit address of slave device
void I2C_Send7bitAddress(uint8_t addr, I2C_Direction_TypeDef dir) {
    // Clear direction bit in addr for just in case and set the specified direction
    I2C1_DR = (addr & 0xFE) | (uint8_t)dir;
}

// Checks whether the last I2C event is equal to the one passed as parameter
// This function must be called once because some flags can be cleared by read the registers
// reutrn SUCCESS if last event was equal to specified and ERROR otherwise
ErrorStatus I2C_CheckEvent(I2C_Event_TypeDef I2C_Event) {
    uint16_t last;
    uint8_t f1,f2;

    if (I2C_Event == I2C_EVENT_SLAVE_ACK_FAILURE) {
        last = (uint16_t)I2C1_SR2_bit.AF << 2;
    } else {
        f1 = I2C1_SR1;
        f2 = I2C1_SR3;
        last = (((uint16_t)f2 << 8) | (uint16_t)f1);
    }

    return (last == I2C_Event) ? SUCCESS : ERROR;
}

// Check state of the specified I2C flag.
FlagStatus I2C_GetFlagStatus(I2C_FLAG_TypeDef I2C_FLAG) {
    uint8_t reg;

    // Get desired register
    switch (I2C_FLAG >> 8) {
    case 0x01:
        reg = (uint8_t)I2C1_SR1;
        break;
    case 0x02:
        reg = (uint8_t)I2C1_SR2;
        break;
    case 0x03:
        reg = (uint8_t)I2C1_SR3;
        break;
    }

    return ((reg & (uint8_t)I2C_FLAG) != 0) ? SET : RESET;
}

// Send a byte by writing in the I2C DR register
void I2C_SendData(uint8_t data) {
    I2C1_DR = data;
}

// Return most recent received data
uint8_t I2C_ReceiveData(void) {
    return (uint8_t)I2C1_DR;
}

// Configure on/off specified I2C interrupt
void I2C_ITConfig(I2C_IT_TypeDef I2C_IT, FunctionalState newstate) {
    if (newstate == DISABLE) {
        I2C1_ITR &= (uint8_t)(~(uint8_t)I2C_IT);
    } else {
        I2C1_ITR |= (uint8_t)I2C_IT;
    }
}

// Init I2C
// return:
//   1 - if it was I2C init timeout
//   0 - if I2C init OK
uint8_t LM75_Init(void) {
    CLK_PCKENR1 |= (1<<3); // Enable I2C peripherial (PCKEN13)

    I2C1_CR1_bit.PE = 0; // Disable I2C to configure TRISER
    I2C1_FREQR = 0x02; // 2MHz peripherial clock frequency (WTF is this?)
    I2C1_CCRH = 0x00; // Standard mode I2C, clear CCRH
    I2C1_CCRL  = 0x00; // Clear CCRL

    I2C1_TRISER = 0x03; // Maximum rise time: = [1000ns/(1/input_clock.10e6)]+1 (3 for 2MHz CPU)

    /*
    I2C_CCRH = 0x00;
    I2C_CCRL = 0x0A; // 100kHz I2C on 2MHz CPU
    */

    ///*
    I2C1_CCRH = 0x00;
    I2C1_CCRL = 0x64; // 10kHz I2C on 2MHz CPU
    //*/

    /*
    I2C_CCRH = 0x03;
    I2C_CCRL = 0xFF; // 1kHz I2C on 2MHz CPU
    */

    I2C1_CR1_bit.PE = 1; // Enable I2C
    I2C1_CR2_bit.ACK = 1; // Acknowledge enable
    I2C1_OARL = 0x00; // Own 7-bit address = 0 (don't care in master mode)
    I2C1_OARH = 0x00; // 7-bit addressing mode
    I2C1_OARH_bit.ADDCONF = 1; // ADDCONF bit must always be written as '1'

    int16_t tmOut = 0x7FFF; // Dummy I2C timeout timer
    while (I2C_GetFlagStatus(I2C_FLAG_BUSY) && (tmOut > 0)) tmOut--; // Wait for I2C to become free

    return (tmOut > 0) ? 0 : 1;
}

// Read 16-bit LM75 register
uint16_t LM75_ReadReg(uint8_t reg) {
    uint16_t value;

    I2C_AcknowledgeConfig(ENABLE); // Enable I2C acknowledgment
    I2C_GenerateSTART(ENABLE);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
    I2C_Send7bitAddress(LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
    I2C_SendData(reg); // Send register address
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
    I2C_GenerateSTART(ENABLE); // Send repeated START condition (aka Re-START)
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
    I2C_Send7bitAddress(LM75_ADDR,I2C_Direction_Receiver); // Send slave address for READ
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
    value = (I2C_ReceiveData() << 8); // Receive high byte
    I2C_AcknowledgeConfig(DISABLE); // Disable I2C acknowledgment
    I2C_GenerateSTOP(ENABLE); // Send STOP condition
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
    value |= I2C_ReceiveData(); // Receive low byte

    return value;
}

// Write 16-bit LM75 register
void LM75_WriteReg(uint8_t reg, uint16_t value) {
    I2C_AcknowledgeConfig(ENABLE); // Enable I2C acknowledgment
    I2C_GenerateSTART(ENABLE);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
    I2C_Send7bitAddress(LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
    I2C_SendData(reg); // Send register address
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
    I2C_SendData((uint8_t)(value >> 8)); // Send high byte
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
    I2C_SendData((uint8_t)value); // Send low byte
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
    I2C_GenerateSTOP(ENABLE);
}

// Read value from LM75 configuration register (8 bit)
uint8_t LM75_ReadConf(void) {
    uint8_t value;

    I2C_AcknowledgeConfig(ENABLE); // Enable I2C acknowledgment
    I2C_GenerateSTART(ENABLE);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
    I2C_Send7bitAddress(LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
    I2C_AckPositionConfig(I2C_AckPosition_Current);
    I2C_SendData(LM75_REG_CONF); // Send register address
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
    I2C_GenerateSTART(ENABLE); // Send repeated START condition (aka Re-START)
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
    I2C_Send7bitAddress(LM75_ADDR,I2C_Direction_Receiver); // Send slave address for READ
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
    I2C_AcknowledgeConfig(DISABLE); // Disable I2C acknowledgment
    I2C_GenerateSTOP(ENABLE); // Send STOP condition
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
    value = I2C_ReceiveData();

    return value;
}

// Write value to LM75 configuration register  (8 bit)
void LM75_WriteConf(uint8_t value) {
    I2C_AcknowledgeConfig(ENABLE); // Enable I2C acknowledgment
    I2C_GenerateSTART(ENABLE);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
    I2C_Send7bitAddress(LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
    I2C_SendData(LM75_REG_CONF); // Send register address
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
    I2C_SendData(value); // Send CONFIG register value
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
    I2C_SendData(0x00); // Send Dummy byte (datasheet WTF?)
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
    I2C_GenerateSTOP(ENABLE);
}

// Set LM75 shutdown mode
// newstate:
//    ENABLE = put LM75 into powerdown mode
//    DISABLE = wake up LM75
//void LM75_Shutdown(FunctionalState newstate) {
void LM75_Shutdown(void) {
    uint8_t value;

    value = LM75_ReadConf();
    LM75_WriteConf(value | 0x01);
}

void LM75_Wake(void) {
    uint8_t value;

    value = LM75_ReadConf();
    LM75_WriteConf(value & 0xFE);
}

// Read temperature readings from LM75 in decimal format
// IIIF where:
//   III - integer part
//   F   - fractional part
// e.g. 355 means 35.5C
int16_t LM75_Temperature(void) {
    uint16_t raw;
    int16_t temp;

    raw = LM75_ReadReg(LM75_REG_TEMP) >> 7;
    if (raw & 0x0100) {
        // Negative temperature
        temp = -10 * (((~(uint8_t)(raw & 0xFE) + 1) & 0x7F) >> 1) - (raw & 0x01) * 5;
    } else {
        // Positive temperature
        temp = ((raw & 0xFE) >> 1) * 10 + (raw & 0x01) * 5;
    }

    return temp;
}
