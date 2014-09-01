// nRF24L01 data rate
typedef enum {
    nRF24_DataRate_250kbps = (uint8_t)0x20, // 250kbps data rate
    nRF24_DataRate_1Mbps   = (uint8_t)0x00, // 1Mbps data rate
    nRF24_DataRate_2Mbps   = (uint8_t)0x08  // 2Mbps data rate
} nRF24_DataRate_TypeDef;

// nRF24L01 RF output power in TX mode
typedef enum {
    nRF24_TXPower_18dBm = (uint8_t)0x00, // -18dBm
    nRF24_TXPower_12dBm = (uint8_t)0x02, // -12dBm
    nRF24_TXPower_6dBm  = (uint8_t)0x04, //  -6dBm
    nRF24_TXPower_0dBm  = (uint8_t)0x06  //   0dBm
} nRF24_TXPower_TypeDef;

// nRF24L01 CRC encoding scheme
typedef enum {
    nRF24_CRC_off   = (uint8_t)0x00, // CRC disabled
    nRF24_CRC_1byte = (uint8_t)0x08, // 1-byte CRC
    nRF24_CRC_2byte = (uint8_t)0x0c  // 2-byte CRC
} nRF24_CRC_TypeDef;

// nRF24L01 power control
typedef enum {
    nRF24_PWR_Up   = (uint8_t)0x02, // Power up
    nRF24_PWR_Down = (uint8_t)0x00  // Power down
} nRF24_PWR_TypeDef;

// nRF24L01 RX/TX control
typedef enum {
    nRF24_PRIM_RX = (uint8_t)0x01, // PRX
    nRF24_PRIM_TX = (uint8_t)0x00  // PTX
} nRF24_PRIM_TypeDef;

// nRF24L01 auto acknowledgement settings
typedef enum {
    nRF24_ENAA_OFF  = 0x00, // auto acknowledgement disabled
    nRF24_ENAA_P0   = 0x01, // enable auto acknowledgement data pipe 0
    nRF24_ENAA_P1   = 0x02, // enable auto acknowledgement data pipe 1
    nRF24_ENAA_P2   = 0x04, // enable auto acknowledgement data pipe 2
    nRF24_ENAA_P3   = 0x08, // enable auto acknowledgement data pipe 3
    nRF24_ENAA_P4   = 0x10, // enable auto acknowledgement data pipe 4
    nRF24_ENAA_P5   = 0x20, // enable auto acknowledgement data pipe 5
} nRF24_ENAA_TypeDef;

////////////////////////////////////////////////////////////////////////////////////////////////


// Chip Enable Activates RX or TX mode
#define CE_L() PB_ODR_bit.ODR3 = 0;
#define CE_H() PB_ODR_bit.ODR3 = 1;

// Chip Select
#define CSN_L() PB_ODR_bit.ODR4 = 0;
#define CSN_H() PB_ODR_bit.ODR4 = 1;

// nRF24L0 commands
#define nRF24_CMD_RREG             0x00  // R_REGISTER -> Read command and status registers
#define nRF24_CMD_WREG             0x20  // W_REGISTER -> Write command and status registers
#define nRF24_CMD_R_RX_PAYLOAD     0x61  // R_RX_PAYLOAD -> Read RX payload
#define nRF24_CMD_W_TX_PAYLOAD     0xA0  // W_TX_PAYLOAD -> Write TX payload
#define nRF24_CMD_FLUSH_TX         0xE1  // FLUSH_TX -> Flush TX FIFO
#define nRF24_CMD_FLUSH_RX         0xE2  // FLUSH_RX -> Flush RX FIFO
#define nRF24_CMD_REUSE_TX_PL      0xE3  // REUSE_TX_PL -> Reuse last transmitted payload
#define nRF24_CMD_NOP              0xFF  // No operation (to read status register)

// nRF24L0 registers
#define nRF24_REG_CONFIG           0x00  // Configuration register
#define nRF24_REG_EN_AA            0x01  // Enable "Auto acknowledgment"
#define nRF24_REG_EN_RXADDR        0x02  // Enable RX addresses
#define nRF24_REG_SETUP_AW         0x03  // Setup of address widths
#define nRF24_REG_SETUP_RETR       0x04  // Setup of automatic retranslation
#define nRF24_REG_RF_CH            0x05  // RF channel
#define nRF24_REG_RF_SETUP         0x06  // RF setup register
#define nRF24_REG_STATUS           0x07  // Status register
#define nRF24_REG_OBSERVE_TX       0x08  // Transmit observe register
#define nRF24_REG_CD               0x09  // Carrier detect
#define nRF24_REG_RX_ADDR_P0       0x0A  // Receive address data pipe 0
#define nRF24_REG_RX_ADDR_P1       0x0B  // Receive address data pipe 1
#define nRF24_REG_RX_ADDR_P2       0x0C  // Receive address data pipe 2
#define nRF24_REG_RX_ADDR_P3       0x0D  // Receive address data pipe 3
#define nRF24_REG_RX_ADDR_P4       0x0E  // Receive address data pipe 4
#define nRF24_REG_RX_ADDR_P5       0x0F  // Receive address data pipe 5
#define nRF24_REG_TX_ADDR          0x10  // Transmit address
#define nRF24_REG_RX_PW_P0         0x11  // Number of bytes in RX payload id data pipe 0
#define nRF24_REG_RX_PW_P1         0x12  // Number of bytes in RX payload id data pipe 1
#define nRF24_REG_RX_PW_P2         0x13  // Number of bytes in RX payload id data pipe 2
#define nRF24_REG_RX_PW_P3         0x14  // Number of bytes in RX payload id data pipe 3
#define nRF24_REG_RX_PW_P4         0x15  // Number of bytes in RX payload id data pipe 4
#define nRF24_REG_RX_PW_P5         0x16  // Number of bytes in RX payload id data pipe 5
#define nRF24_REG_FIFO_STATUS      0x17  // FIFO status register
#define nRF24_REG_DYNPD            0x1C  // Enable dynamic payload length
#define nRF24_REG_FEATURE          0x1D  // Feature register

// nRF24L0 bits
#define nRF24_MASK_RX_DR           0x40  // Mask interrupt caused by RX_DR
#define nRF24_MASK_TX_DS           0x20  // Mask interrupt caused by TX_DS
#define nRF24_MASK_MAX_RT          0x10  // Mask interrupt caused by MAX_RT
#define nRF24_FIFO_RX_EMPTY        0x01  // RX FIFO empty flag
#define nRF24_FIFO_RX_FULL         0x02  // RX FIFO full flag


// Function prototypes
void nRF24_init(void);

uint8_t nRF24_RWReg(uint8_t reg, uint8_t value);
uint8_t nRF24_ReadReg(uint8_t reg);
uint8_t nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);
uint8_t nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);

uint8_t nRF24_Check(void);

void nRF24_SetRFChannel(uint8_t RFChannel);
void nRF24_TXMode(uint8_t RetrCnt, uint8_t RetrDelay, nRF24_ENAA_TypeDef ENAA, uint8_t RFChan,
                  nRF24_DataRate_TypeDef DataRate, nRF24_TXPower_TypeDef TXPower, nRF24_CRC_TypeDef CRCS,
                  nRF24_PWR_TypeDef PWR, uint8_t *TX_Addr, uint8_t TX_Addr_Width);
void nRF24_TXPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD);
void nRF24_PowerDown(void);
void nRF24_Wake(void);
void nRF24_SetTXPower(nRF24_TXPower_TypeDef TXPower);
void nRF24_ClearIRQFlags(void);
