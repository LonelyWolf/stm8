#define HEX_CHARS      "0123456789ABCDEF"


void UART_Init(void);

void UART_SendChar(char ch);
void UART_SendChars(char ch, uint8_t count);

void UART_SendInt(int32_t num);
void UART_SendUInt(uint32_t num);
void UART_SendHex8(uint16_t num);
void UART_SendHex16(uint16_t num);
void UART_SendHex32(uint32_t num);

void UART_SendStr(char *str);

void UART_SendBuf(char *buf, uint16_t bufsize);
void UART_SendBufPrintable(char *buf, uint16_t bufsize, char subst);
void UART_SendBufHex(char *buf, uint16_t bufsize);
