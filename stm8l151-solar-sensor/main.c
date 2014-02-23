#include "stdint.h"
#include "iostm8l151k6.h"

#include "stm8l15x.h"
#include "uart.h"
#include "nRF24.h"
#include "lm75.h"
#include "rtc.h"


//#define DEBUG


#define TX_PAYLOAD           18   // nRF24L01 TX payload length


uint8_t i;
uint16_t temp; // Temperature readings
uint32_t cntr; // Send counter
uint8_t buf[TX_PAYLOAD]; // nRF24L01 payload buffer
uint16_t factory_vref;
uint16_t vrefint;
uint32_t master_freq; // CPU frequency
uint32_t LSI_freq; // LSI frequency
uint8_t throttle_v; // vrefint measure throttle
uint8_t throttle_f; // LSI clock measure throttle
RTC_TimeTypeDef time;
RTC_DateTypeDef date;
RTC_TimeBCDTypeDef time_raw;
RTC_DateBCDTypeDef date_raw;


static uint8_t CRC7_one(uint8_t t, uint8_t data) {
	const uint8_t g = 0x89;
	uint8_t i;

	t ^= data;
	for (i = 0; i < 8; i++) {
		if (t & 0x80) t ^= g;
		t <<= 1;
	}

	return t;
}

uint8_t CRC7_buf(const uint8_t * p, uint8_t len) {
	uint8_t j,crc = 0;

	for (j = 0; j < len; j++) crc = CRC7_one(crc,p[j]);

	return crc >> 1;
}

void UART_SendInt0(int32_t num) {
    char str[10]; // 10 chars max for INT32_MAX
    int i = 0;

    if (num < 0) {
        UART_SendChar('-');
	num *= -1;
    }
    if ((num < 10) & (num >= 0)) UART_SendChar('0');
    do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
    for (i--; i >= 0; i--) UART_SendChar(str[i]);
}

// RTC_WAKEUP IRQ handle
#pragma vector=RTC_WAKEUP_vector
__interrupt void RTC_IRQHandler(void) {
    RTC_ISR2_bit.WUTF = 0; // Clear wakeup timer interrupt flag
}

// Init Vreint ADC channel
void ADC_Vrefint_Init(void) {
    ADC1_CR1_bit.ADON = 1; // Enable ADC
    ADC1_TRIGR1_bit.VREFINTON = 1; // Enable internal reference voltage
    ADC1_SQR1_bit.CHSEL_S28 = 1; // Enable CHSEL_SVREFINT fast ADC channel
    ADC1_CR3 = 0x80; // Sampling time = 48 ADC clock cycles, disable analog watchdog channels
    ADC1_SQR1_bit.DMAOFF = 1; // DMA off
}

// Measure Vrefint
uint16_t ADC_Vrefint_Measure(void) {
    uint16_t adc_res;
    uint16_t value = 0;
    uint8_t cntr;

    for (cntr = 0; cntr < 4; cntr++) {
        ADC1_CR1_bit.START = 1; // Start ADC conversion, by software trigger
        while (!ADC1_SR_bit.EOC); // Wait for the conversion ends
        adc_res  = (ADC1_DRH << 8); // Get ADC converted data
        adc_res |= ADC1_DRL;
        value += adc_res;
        if (cntr) value >>= 1;
    }

    return value;
}

// Disable Vrefint ADC channel and ADC module
void ADC_Vrefint_Disable() {
    ADC1_TRIGR1_bit.VREFINTON = 0; // Disable internal reference voltage
    ADC1_SQR1_bit.CHSEL_S28 = 0; // Disable CHSEL_SVREFINT fast ADC channel
    ADC1_CR1_bit.ADON = 0; // Disable ADC
}


int main( void ) {
    CLK_PCKENR2_bit.PCKEN27 = 0; // Disable Boot ROM clock

    PB_DDR_bit.DDR1 = 1; // Set PB1 as output
    PB_CR1_bit.C11 = 1; // Configure PB1 push-pull
    PB_ODR_bit.ODR1 = 1; // LED on

#ifdef DEBUG
    UART_Init();
#endif

#ifdef DEBUG
    UART_SendStr("STM8L151K6T6 is online\n");
    UART_SendStr("Init I2C ... ");
#endif
    if (LM75_Init()) {
#ifdef DEBUG
	UART_SendStr("timeout.\n");
	UART_SendStr("MCU will now halt.\n");
#endif
	while(1);
    } else {
#ifdef DEBUG
        UART_SendStr("ok\n");
#endif
    }

    nRF24_init(); // Init SPI interface for nRF24L01 communications
#ifdef DEBUG
    UART_SendStr("nRF24L01 check ... ");
#endif
    if (nRF24_Check() != 0) {
#ifdef DEBUG
	UART_SendStr("wrong answer from SPI device.\n");
	UART_SendStr("MCU will now halt.\n");
#endif
	while(1);
    } else {
#ifdef DEBUG
        UART_SendStr("ok\n");
#endif
    }
    nRF24_TXMode(); // Configure nRF24L01 for TX mode
    for (i = 0; i < TX_PAYLOAD; i++) buf[i] = 0x00;
    nRF24_PowerDown(); // Put nRF24L01 into power down mode

    temp = LM75_Temperature(); // Get temperature readings
    LM75_Shutdown(ENABLE); // Shut down LM75
#ifdef DEBUG
    UART_SendStr("T.RAW=");
    UART_SendHex16(temp);
    UART_SendStr("\n");
#endif

    master_freq = CLK_GetClockFreq();
#ifdef DEBUG
    UART_SendStr("CPU: ");
    UART_SendInt(master_freq);
    UART_SendStr("Hz\n");
#endif

    LSI_freq = CLK_GetLSIFreq(master_freq);
#ifdef DEBUG
    UART_SendStr("LSI: ");
    UART_SendInt(LSI_freq);
    UART_SendStr("kHz\n");
#endif

    // Configure RTC
    RTC_Init(); // Init RTC (hardware reset only in power on sequence!)
    RTC_TuneClock(LSI_freq);
    RTC_WakeupConfig(RTC_WUC_RTCCLK_Div16); // RTC wakeup = LSI/RTCDIV/16
    RTC_WakeupTimerSet((74*5) - 1); // 5 seconds wakeup
    RTC_WakeupSet(ENABLE);
    RTC_WakeupIT(ENABLE);

    // If VREFINT not set on factory, assign standard value
    if (Factory_VREFINT != 0) {
        factory_vref = 0x0600 | (uint16_t)Factory_VREFINT;
    } else {
        factory_vref = 0x0687;
    }
    CLK_PCKENR2_bit.PCKEN20 = 1; // Enable ADC peripherial (PCKEN20)
    ADC_Vrefint_Init();
    vrefint = ADC_Vrefint_Measure();
    ADC_Vrefint_Disable();
    vrefint = (((uint32_t)factory_vref*300)/vrefint);
#ifdef DEBUG
    UART_SendStr("Vcc: ");
    UART_SendInt(vrefint / 100); UART_SendChar('.');
    UART_SendInt(vrefint % 100); UART_SendStr("V\n");
    UART_SendChars('-',80); UART_SendChar('\n');
#endif

    throttle_v = 0;
    throttle_f = 0;

    // Main loop ^_^
    cntr = 0;
    while(1) {
        PB_ODR_bit.ODR1 = 1; // LED on

        cntr++; // Iterations counter (since power on)

#ifdef DEBUG
        UART_SendStr("cntr=");
        UART_SendInt(cntr);
#endif

        LM75_Wake(); // Wake LM75 (temperature conversion starts automatically)

        // Since temperature conversion takes approximately 150ms, the MCU will sleep
        // a bit longer and then wake up to get readings.
        RTC_WakeupTimerSet(12); // Trigger wakeup timer for about 160ms
        asm("halt");

        // Acquire temperature readings and shutdown the LM75
        temp = LM75_Temperature();
        LM75_Shutdown();

        // Trigger Wakeup timer for 5 minutes
        //RTC_WakeupTimerSet((74*10) - 1); // 10 seconds wakeup
        RTC_WakeupTimerSet(22265); // 5 minutes wakeup

#ifdef DEBUG
        UART_SendStr(" T.RAW="); UART_SendHex16(temp);
        UART_SendStr(" => "); UART_SendInt(temp / 10);
        UART_SendChar('.'); UART_SendInt(temp % 10);
        UART_SendChar('C');
#endif

        // Wake nRF24L01 (it goes into Standby-I mode, this takes about 1.5ms)
        nRF24_Wake();

        // Measure Vrefint and compute Vcc
        throttle_v++;
        if (throttle_v > 5) {
            ADC_Vrefint_Init();
            vrefint = ADC_Vrefint_Measure();
            ADC_Vrefint_Disable();
            vrefint = (((uint32_t)factory_vref*300)/vrefint);
            throttle_v = 0;
        }
#ifdef DEBUG
        UART_SendStr(" Vcc=");
        UART_SendInt(vrefint / 100); UART_SendChar('.');
        UART_SendInt(vrefint % 100); UART_SendStr("V");
#endif

        // Measure real LSI frequency and tune RTC clock
        throttle_f++;
        if (throttle_f > 5) {
            LSI_freq = CLK_GetLSIFreq(master_freq);
            RTC_TuneClock(LSI_freq);
            throttle_f = 0;
        }
#ifdef DEBUG
        UART_SendStr(" LSI=");
        UART_SendInt(LSI_freq);
        UART_SendStr("kHz");
#endif

        // Get date and time from RTC (sort of uptime)
        time_raw = RTC_GetTimeBCD();
        date_raw = RTC_GetDateBCD();
#ifdef DEBUG
        time = RTC_GetTime();
        date = RTC_GetDate();
        UART_SendStr(" Uptime:[");
        UART_SendInt0(time.Hours); UART_SendChar(':');
        UART_SendInt0(time.Minutes); UART_SendChar(':');
        UART_SendInt0(time.Seconds); UART_SendChar('@');
        UART_SendInt0(date.Day); UART_SendChar('.');
        UART_SendInt0(date.Month); UART_SendStr(".20");
        UART_SendInt0(date.Year); UART_SendChar(']');
#endif

        // Prepare data packet
        buf[0]  = (uint8_t)(temp >> 8); // Temperature
        buf[1]  = (uint8_t)temp;
        buf[2]  = (uint8_t)(cntr >> 24); // Iterations counter
        buf[3]  = (uint8_t)(cntr >> 16);
        buf[4]  = (uint8_t)(cntr >> 8);
        buf[5]  = (uint8_t)cntr;
        buf[6]  = (uint8_t)(vrefint >> 8); // Vcc
        buf[7]  = (uint8_t)vrefint;
        buf[8]  = (uint8_t)time_raw.RTC_TR1; // RAW Time
        buf[9]  = (uint8_t)time_raw.RTC_TR2;
        buf[10] = (uint8_t)time_raw.RTC_TR3;
        buf[11] = (uint8_t)date_raw.RTC_DR1; // RAW Date
        buf[12] = (uint8_t)date_raw.RTC_DR2;
        buf[13] = (uint8_t)date_raw.RTC_DR3;
        buf[14] = (uint8_t)(LSI_freq >> 8); // LSI frequency
        buf[15] = (uint8_t)LSI_freq;
        // buf[16] will be filled later with OBSERVER_TX register value
        buf[TX_PAYLOAD - 1] = CRC7_buf(&buf[0],TX_PAYLOAD - 1); // Put packet CRC byte

        // Send packet
        i = nRF24_TXPacket(buf,TX_PAYLOAD);
#ifdef DEBUG
        UART_SendStr(" Buf=[");
        UART_SendBufHex((char*)&buf[0],TX_PAYLOAD);
        UART_SendStr("] TX=");
        UART_SendHex8(i);
#endif

        // Put OBSERVER_TX value into buffer for next iteration
        i = nRF24_ReadReg(nRF24_REG_OBSERVE_TX);
        buf[16] = i;
#ifdef DEBUG
        UART_SendStr(" OBSERVE_TX=");
        UART_SendHex8(i >> 4); UART_SendChar(':');
        UART_SendHex8(i & 0x0F); UART_SendChar('\n');
#endif

        // Put nRF24L01 into power down mode
        nRF24_PowerDown();

        PB_ODR_bit.ODR1 = 0; // LED off

        // Enable Wakeup timer and put MCU into Active-Halt mode for 5 minutes
        asm("halt");
    }
}
