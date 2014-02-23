typedef enum {
    I2C_AckPosition_Current = (uint8_t)0x00,   // Acknowledge on the current byte
    I2C_AckPosition_Next    = (uint8_t)0x01    // Acknowledge on the next byte
} I2C_AckPosition_TypeDef;

typedef enum {
    I2C_Direction_Transmitter = (uint8_t)0x00,  // Transmission direction
    I2C_Direction_Receiver    = (uint8_t)0x01   // Reception direction
} I2C_Direction_TypeDef;

typedef enum {
    // Slave mode
    I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED= (uint16_t)0x0682,  // EV1: TRA, BUSY, TXE and ADDR flags
    I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED   = (uint16_t)0x0202,  // EV1: BUSY and ADDR flags
    I2C_EVENT_SLAVE_BYTE_RECEIVED              = (uint16_t)0x0240,  // EV2: BUSY and RXNE flags
    I2C_EVENT_SLAVE_BYTE_TRANSMITTED           = (uint16_t)0x0684,  // EV3: TRA, BUSY, TXE and BTF flags
    I2C_EVENT_SLAVE_STOP_DETECTED              = (uint16_t)0x0010,  // EV4: STOPF flag
    I2C_EVENT_SLAVE_ACK_FAILURE                = (uint16_t)0x0004,  // EV3_2: AF flag

    // Master mode
    I2C_EVENT_MASTER_MODE_SELECT               = (uint16_t)0x0301,  // EV5: BUSY, MSL and SB flags
    I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED = (uint16_t)0x0782,  // EV6: BUSY, MSL, ADDR, TXE and TRA flags
    I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED    = (uint16_t)0x0302,  // EV6: BUSY, MSL and ADDR flags
    I2C_EVENT_MASTER_BYTE_RECEIVED             = (uint16_t)0x0340,  // EV7: BUSY, MSL and RXNE flags
    I2C_EVENT_MASTER_BYTE_TRANSMITTING         = (uint16_t)0x0780,  // EV8: TRA, BUSY, MSL, TXE flags
    I2C_EVENT_MASTER_BYTE_TRANSMITTED          = (uint16_t)0x0784,  // EV8_2: TRA, BUSY, MSL, TXE and BTF flags
    I2C_EVENT_MASTER_MODE_ADDRESS10            = (uint16_t)0x0308   // EV9: BUSY, MSL and ADD10 flags
} I2C_Event_TypeDef;

typedef enum {
    // SR1 register flags
    I2C_FLAG_TXE       = (uint16_t)0x0180,  // Transmit Data Register Empty flag
    I2C_FLAG_RXNE      = (uint16_t)0x0140,  // Read Data Register Not Empty flag
    I2C_FLAG_STOPF     = (uint16_t)0x0110,  // Stop detected flag
    I2C_FLAG_ADD10     = (uint16_t)0x0108,  // 10-bit Header sent flag
    I2C_FLAG_BTF       = (uint16_t)0x0104,  // Data Byte Transfer Finished flag
    I2C_FLAG_ADDR      = (uint16_t)0x0102,  // Address Sent/Matched (master/slave) flag
    I2C_FLAG_SB        = (uint16_t)0x0101,  // Start bit sent flag

    // SR2 register flags
    I2C_FLAG_WUFH     = (uint16_t)0x0220,  // Wake Up From Halt Flag
    I2C_FLAG_OVR      = (uint16_t)0x0208,  // Overrun/Underrun flag
    I2C_FLAG_AF       = (uint16_t)0x0204,  // Acknowledge Failure Flag
    I2C_FLAG_ARLO     = (uint16_t)0x0202,  // Arbitration Loss Flag
    I2C_FLAG_BERR     = (uint16_t)0x0201,  // Misplaced Start or Stop condition

    // SR3 register flags
    I2C_FLAG_GENCALL  = (uint16_t)0x0310,  // General Call header received Flag
    I2C_FLAG_TRA      = (uint16_t)0x0304,  // Transmitter Receiver Flag
    I2C_FLAG_BUSY     = (uint16_t)0x0302,  // Bus Busy Flag
    I2C_FLAG_MSL      = (uint16_t)0x0301   // Master Slave Flag
} I2C_FLAG_TypeDef;

typedef enum {
    I2C_IT_ERR             = (uint16_t)0x0001, 	// Error Interruption
    I2C_IT_EVT             = (uint16_t)0x0002, 	// Event Interruption
    I2C_IT_BUF             = (uint16_t)0x0004, 	// Buffer Interruption

    // SR1 register flags
    I2C_IT_TXE             = (uint16_t)0x1680, 	// Transmit Data Register Empty
    I2C_IT_RXNE            = (uint16_t)0x1640, 	// Read Data Register Not Empty
    I2C_IT_STOPF           = (uint16_t)0x1210, 	// Stop detected
    I2C_IT_ADD10           = (uint16_t)0x1208, 	// 10-bit Header sent
    I2C_IT_BTF             = (uint16_t)0x1204, 	// Data Byte Transfer Finished
    I2C_IT_ADDR            = (uint16_t)0x1202, 	// Address Sent/Matched (master/slave)
    I2C_IT_SB              = (uint16_t)0x1201, 	// Start bit sent

    // SR2 register flags
    I2C_IT_WUFH            = (uint16_t)0x2220, 	// Wake Up From Halt
    I2C_IT_OVR             = (uint16_t)0x2108, 	// Overrun/Underrun
    I2C_IT_AF              = (uint16_t)0x2104, 	// Acknowledge Failure
    I2C_IT_ARLO            = (uint16_t)0x2102, 	// Arbitration Loss
    I2C_IT_BERR            = (uint16_t)0x2101  	// Misplaced Start or Stop condition
} I2C_IT_TypeDef;


/* LM75 defines */
#define LM75_ADDR                     0x90 // LM75 address

/* LM75 registers */
#define LM75_REG_TEMP                 0x00 // Temperature
#define LM75_REG_CONF                 0x01 // Configuration
#define LM75_REG_THYS                 0x02 // Hysteresis
#define LM75_REG_TOS                  0x03 // Overtemperature shutdown


uint8_t LM75_Init(void);

void LM75_WriteReg(uint8_t reg, uint16_t value);
uint16_t LM75_ReadReg(uint8_t reg);
uint8_t LM75_ReadConf(void);
void LM75_WriteConf(uint8_t value);

//void LM75_Shutdown(FunctionalState newstate);
void LM75_Shutdown();
void LM75_Wake();
int16_t LM75_Temperature(void);
