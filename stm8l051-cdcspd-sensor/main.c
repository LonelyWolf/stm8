//////////////////////////
////// STM8L051F3P6 //////
//////////////////////////


//#define DEBUG             // CPU clock 2MHz, UART spam, Speed/Cadence calculating
#define SOFT_DEBOUNCE     // If defined - use software debounce, no debounce otherwise


///////////////////////////////////////////////////////////////////////////////


#include "stdint.h"
#include "iostm8l051f3.h"

#include "stm8l051.h"
#ifdef DEBUG
    #include "uart.h"
    #include "math.h"
#endif
#include "nRF24.h"
#include "rtc.h"


///////////////////////////////////////////////////////////////////////////////


#define TIM2LSE              // If defined - TIM2 will be clocked from LSE
#define TIM3LSE              // If defined - TIM3 will be clocked from LSE

#define TX_PAYLOAD       16  // nRF24L01 Payload length
#define RF_CHANNEL       90  // nRF24L01 channel (90ch = 2490MHz)
#define RF_RETR          10  // nRF24L01 TX retransmit count (max 15)

#define TIM_DEBOUNCE     60  // Debounce delay, higher value means lowest high speed can be measured.
#define TIM_TIMEOUT    6000  // Magnetic reed impulse timeout (roughly measured in milliseconds)

#define WU_TIMER         63  // Wakeup timer Period = (WU_TIMER + 1) * 0,015625 seconds (63 = 1 second)


///////////////////////////////////////////////////////////////////////////////


uint8_t i; // Really need to comment this?

uint16_t vrefint; // Last measured internal voltage
uint16_t factory_vref; // Factory measured internal voltage with 3V external

volatile uint16_t cntr_EXTI1 = 0;  // EXTI1 impulse counter
volatile uint16_t cntr_EXTI2 = 0;  // EXTI2 impulse counter

volatile uint16_t tim2_diff = 0; // TIM2 difference between two pulses on EXTI1
volatile uint16_t tim3_diff = 0; // TIM3 difference between two pulses on EXTI2

volatile uint16_t tim2 = 0; // TIM2 overflows counter
volatile uint16_t tim3 = 0; // TIM3 overflows counter

uint16_t packets_lost = 0; // Lost packets counter

const nRF24_TXPower_TypeDef tx_rf_powers[4] = { nRF24_TXPower_18dBm,
                                                nRF24_TXPower_12dBm,
                                                nRF24_TXPower_6dBm,
                                                nRF24_TXPower_0dBm };
uint8_t tx_power = 0; // RF output power

#ifdef DEBUG
    float speed;
    float spd_f, spd_i;
    uint32_t cdc;
#endif

uint8_t buf[TX_PAYLOAD]; // nRF24L01 payload buffer
uint8_t prev_observe_TX = 0; // Last value of nRF24L01 OBSERVE_TX register
uint16_t cntr_wake = 0; // Wakeup counter (for debug purposes)


///////////////////////////////////////////////////////////////////////////////


// RTC_WAKEUP IRQ handle
#pragma vector=RTC_WAKEUP_vector
__interrupt void RTC_IRQHandler(void) {
    CPU_CFG_GCR_bit.AL = 0; // Set main activation level
    RTC_ISR2_bit.WUTF = 0; // Clear wakeup timer interrupt flag
}

// EXTI1 IRQ handle
#pragma vector=EXTI1_vector
__interrupt void EXTI1_IRQHandler(void) {
    TIM2_CR1_bit.CEN = 1; // Enable TIM2 counter

#ifdef SOFT_DEBOUNCE
    if (tim2 > TIM_DEBOUNCE) {
        cntr_EXTI1++;
        if (cntr_EXTI1 > 1) tim2_diff = tim2;
        tim2 = 0;
        TIM2_EGR_bit.UG = 1; // Reinitialize TIM2
    }
#else
    cntr_EXTI1++;
    if (cntr_EXTI1 > 1) tim2_diff = tim2;

    tim2 = 0;
    TIM2_EGR_bit.UG = 1; // Reinitialize TIM2
#endif

    if (!RTC_CR2_bit.WUTE) RTC_WakeupSet(ENABLE); // Enable wakeup timer
    EXTI_SR1_bit.P1F = 1; // Clear EXTI1 IRQ flag
}

// EXTI2 IRQ handle
#pragma vector=EXTI2_vector
__interrupt void EXTI2_IRQHandler(void) {
    TIM3_CR1_bit.CEN = 1; // Enable TIM3 counter

#ifdef SOFT_DEBOUNCE
    if (tim3 > TIM_DEBOUNCE) {
        cntr_EXTI2++;
        if (cntr_EXTI2 > 1) tim3_diff = tim3;

        tim3 = 0;
        TIM3_EGR_bit.UG = 1; // Reinitialize TIM3
    }
#else
    cntr_EXTI2++;
    if (cntr_EXTI2 > 1) tim3_diff = tim3;

    tim3 = 0;
    TIM3_EGR_bit.UG = 1; // Reinitialize TIM3
#endif

    if (!RTC_CR2_bit.WUTE) RTC_WakeupSet(ENABLE); // Enable wakeup timer
    EXTI_SR1_bit.P2F = 1; // Clear EXTI2 IRQ flag
}

// TIM2 Update/Overflow/Trigger/Break interrupt IRQ handle
#pragma vector=TIM2_OVR_UIF_vector
__interrupt void TIM2_UIF_IRQHandler(void) {
    tim2++;
    if (tim2 > TIM_TIMEOUT) {
        TIM2_CR1_bit.CEN = 0; // Disable TIM2 counter
        cntr_EXTI1 = 0;
        tim2_diff  = 0;
    }
    TIM2_SR1_bit.UIF = 0; // Clear TIM2 update interrupt flag
}

// TIM3 Update/Overflow/Trigger/Break interrupt IRQ handle
#pragma vector=TIM3_OVR_UIF_vector
__interrupt void TIM3_UIF_IRQHandler(void) {
    tim3++;
    if (tim3 > TIM_TIMEOUT) {
        TIM3_CR1_bit.CEN = 0; // Disable TIM3 counter
        cntr_EXTI2 = 0;
        tim3_diff  = 0;
    }
    TIM3_SR1_bit.UIF = 0; // Clear TIM3 update interrupt flag
}


///////////////////////////////////////////////////////////////////////////////


// Init Vreint ADC channel
void ADC_Vrefint_Init(void) {
    ADC1_CR1_bit.ADON = 1; // Enable ADC
    ADC1_TRIGR1_bit.VREFINTON = 1; // Enable internal reference voltage
    ADC1_SQR1_bit.CHSEL_S28 = 1; // Enable CHSEL_SVREFINT fast ADC channel
    ADC1_CR3 = 0x80; // Sampling time = 48 ADC clock cycles, disable analog watchdog channels
    ADC1_SQR1_bit.DMAOFF = 1; // DMA off
}

// Measure Vrefint
uint16_t ADC_Vrefint_Measure(uint8_t count) {
    uint16_t adc_res;
    uint16_t value = 0;
    uint8_t cntr;

    // Measure voltage "count" times and calculate rough average value
    for (cntr = 0; cntr < count - 1; cntr++) {
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


///////////////////////////////////////////////////////////////////////////////


int main(void)
{
    CLK_PCKENR2_bit.PCKEN27 = 0; // Disable Boot ROM

    // Configure unused GPIO ports as input with pull-up for powersaving
    // PB0
    PB_ODR_bit.ODR0 = 0; // Latch "0" in input register
    PB_DDR_bit.DDR0 = 0; // Set as input
    PB_CR1_bit.C10  = 1; // Input with Pull-up
    PB_CR2_bit.C20  = 0; // External interrupt disabled
    // PD0
    PD_ODR_bit.ODR0 = 0; // Latch "0" in input register
    PD_DDR_bit.DDR0 = 0; // Set as input
    PD_CR1_bit.C10  = 1; // Input with Pull-up
    PD_CR2_bit.C20  = 0; // External interrupt disabled
    // PC1
    PD_ODR_bit.ODR1 = 0; // Latch "0" in input register
    PD_DDR_bit.DDR1 = 0; // Set as input
    PD_CR1_bit.C11  = 1; // Input with Pull-up
    PD_CR2_bit.C21  = 0; // External interrupt disabled
    // PC4
    PC_ODR_bit.ODR4 = 0; // Latch "0" in input register
    PC_DDR_bit.DDR4 = 0; // Set as output
    PC_CR1_bit.C14  = 1; // Input with Pull-up
    PC_CR2_bit.C24  = 0; // External interrupt disabled

    // Magnetic reed inputs
    // PB1 = Reed#1
    // PB2 = Reed#2
    PB_ODR_bit.ODR1 = 1; // Latch "1" in input register
    PB_DDR_bit.DDR1 = 0; // Set PB1 as input
    PB_CR1_bit.C11  = 0; // Floating input (it requires an external pull-up resistor)
    PB_CR2_bit.C21  = 1; // Enable external interrupt
    PB_ODR_bit.ODR2 = 1; // Latch "1" in input register
    PB_DDR_bit.DDR2 = 0; // Set PB2 as input
    PB_CR1_bit.C12  = 0; // Floating input (it requires an external pull-up resistor)
    PB_CR2_bit.C22  = 1; // Enable external interrupt

#ifdef DEBUG
    // UART pinout
    // PA2 - UART_TX
    // PA3 - UART_RX
    SYSCFG_RMPCR1_bit.USART1TR_REMAP = 0x01; // USART1 remap: TX on PA2 and RX on PA3
    UART_Init();
    UART_SendStr("Speed & Cadence RF sensor (C) Wolk 2014\n");
#endif

    // nRF24L01 pinout
    // PB3 - CE
    // PB4 - NSS
    // PB5 - SCK
    // PB6 - MOSI
    // PB7 - MISO
    // PC0 - IRQ
    nRF24_init(); // Init SPI interface for nRF24L01 communications
#ifdef DEBUG
    UART_SendStr("nRF24L01 check ... ");
    if (nRF24_Check() != 0) {
	UART_SendStr("wrong answer from SPI device.\n");
	UART_SendStr("MCU will now halt.\n");
	while(1);
    } else {
        UART_SendStr("ok\n");
    }
#else
    if (nRF24_Check()) {
        // No answer from nRF24L01 -> go deep sleep mode
        while(1) asm("halt");
    }
#endif
    // Configure nRF24L01 for TX mode:
    // RF_RETR retransmits with 500us delay
    // RF channel 90 (2490MHz), -18dBm TX power, LNA gain enabled, 2-byte CRC
    // Power down mode initially
    // 1Mbps - gives 3dB better receiver sensitivity compared to 2Mbps
    nRF24_TXMode(RF_RETR,1,RF_CHANNEL,nRF24_DataRate_1Mbps,nRF24_TXPower_18dBm,nRF24_CRC_on,nRF24_CRC_2byte,nRF24_PWR_Down);
    for (i = 0; i < TX_PAYLOAD; i++) buf[i] = 0x00; // it's obvious :)

    // Configure RTC
#ifdef DEBUG
    UART_SendStr("RTC init ... ");
    RTC_Init();
    UART_SendStr("ok\n");
#else
    RTC_Init();
#endif
    RTC_WakeupConfig(RTC_WUC_RTCCLK_Div16); // RTC wakeup clock = LSE/RTCDIV/16

    // If VREFINT is not set on factory, assign average standard value
    if (Factory_VREFINT != 0) {
        factory_vref = 0x0600 | (uint16_t)Factory_VREFINT;
    } else {
        factory_vref = 0x0687;
    }
    CLK_PCKENR2_bit.PCKEN20 = 1; // Enable ADC peripherial (PCKEN20)
    ADC_Vrefint_Init();
    vrefint = ADC_Vrefint_Measure(5);
    ADC_Vrefint_Disable();
    CLK_PCKENR2_bit.PCKEN20 = 0; // Disable ADC peripherial
    vrefint = (((uint32_t)factory_vref * 300) / vrefint);
#ifdef DEBUG
    UART_SendStr("Vcc: ");
    UART_SendInt(vrefint / 100); UART_SendChar('.');
    UART_SendInt(vrefint % 100); UART_SendStr("V\n");
    UART_SendChars('-',80); UART_SendChar('\n');
#endif

    // Timers initialization
#ifdef TIM2LSE
    CLK_PCKENR1_bit.PCKEN10 = 1; // Enable TIM2 peripherial
    SYSCFG_RMPCR2_bit.TIM2TRIGLSE_REMAP = 1; // TIM2 trigger controlled by LSE (OSC32_IN)
    TIM2_PSCR = 0; // Prescaler is off
    TIM2_ARRH = 0x00; // Auto-reload value (32768 / 33 -> 0.9929696969)
    TIM2_ARRL = 0x20;
    TIM2_ETR = 0x40; // External trigger: enabled, non-inverted, prescaler off, filter off
    TIM2_IER_bit.UIE = 1; // Update interrupt enable
    TIM2_EGR_bit.UG = 1; // Generate UEV (update event) to reload TIM2 and set the prescaler
#else
    CLK_PCKENR1_bit.PCKEN10 = 1; // Enable TIM2 peripherial
    TIM2_PSCR = 4; // Prescaler is 16
    TIM2_ARRH = 0; // Auto-reload value (TIM2 overflow in 1.0008ms at 2MHz)
    TIM2_ARRL = 123;
    TIM2_IER_bit.UIE = 1; // Update interrupt enable
    TIM2_EGR_bit.UG = 1; // Generate UEV (update event) to reload TIM2 and set the prescaler
#endif

#ifdef TIM3LSE
    CLK_PCKENR1_bit.PCKEN11 = 1; // Enable TIM3 peripherial
    SYSCFG_RMPCR2_bit.TIM3TRIGLSE_REMAP = 1; // TIM3 trigger controlled by LSE (OSC32_IN)
    TIM3_PSCR = 0; // Prescaler is off
    TIM3_ARRH = 0x00; // Auto-reload value (32768 / 33 -> 0.9929696969)
    TIM3_ARRL = 0x20;
    TIM3_ETR = 0x40; // External trigger: enabled, non-inverted, prescaler off, filter off
    TIM3_IER_bit.UIE = 1; // Update interrupt enable
    TIM3_EGR_bit.UG = 1; // Generate UEV (update event) to reload TIM3 and set the prescaler
#else
    CLK_PCKENR1_bit.PCKEN11 = 1; // Enable TIM3 peripherial
    TIM3_PSCR = 4; // Prescaler is 16
    TIM3_ARRH = 0; // Auto-reload value (TIM3 overflow in 1.0008ms at 2MHz)
    TIM3_ARRL = 123;
    TIM3_IER_bit.UIE = 1; // Update interrupt enable
    TIM3_EGR_bit.UG = 1; // Generate UEV (update event) to reload TIM3 and set the prescaler
#endif

    // Configure external interrupts
    CPU_CFG_GCR_bit.AL = 1; // Interrupt-only activation level (IRET causes the CPU to go back to Halt mode)
    EXTI_CR1_bit.P1IS = 0x02; // EXTI1 on falling edge
    EXTI_CR1_bit.P2IS = 0x02; // EXTI2 on falling edge
    ITC_SPR3_bit.VECT9SPR  = 0x03; // IRQ level 3 (high priority)
    ITC_SPR3_bit.VECT10SPR = 0x03; // IRQ level 3 (high priority)
    asm("rim"); // Enable global interrupts (enable priorities)

    // Configure wakeup timer
    RTC_WakeupTimerSet(WU_TIMER); // 1 second wakeup
    RTC_WakeupIT(ENABLE); // Enable wakeup interrupt
    RTC_WakeupSet(DISABLE); // Disable wakeup timer

    // Configure system clock
#ifdef DEBUG
    CLK_CKDIVR_bit.CKM = 0x03; // System clock source /8 (2MHz from HSI)
#else
    CLK_CKDIVR_bit.CKM = 0x07; // System clock source /128 (125kHz from HSI)
#endif

    // Go deep sleep mode
    asm("halt");

    // Main loop ^_^
    while(1) {
        // Wake nRF24L01 (Power Down -> Standby-I mode)
        // This will take about 1.5ms with internal nRF24L01 oscillator or 150us with external.
        // So here delay is not necessary since nRF24L01 uses external oscillator.
        nRF24_Wake();

        cntr_wake++; // Count wakeups for debug purposes

#ifdef DEBUG
        UART_SendStr("Wakeups: ");
        UART_SendUInt(cntr_wake);

        UART_SendStr("   EXTI: ");
        UART_SendUInt(cntr_EXTI1);
        UART_SendChar(':');
        UART_SendUInt(cntr_EXTI2);

        UART_SendStr("   DIFF: ");
        UART_SendUInt(tim2_diff);
        UART_SendChar(':');
        UART_SendUInt(tim3_diff);

        UART_SendStr("   SPD: ");
#ifdef TIM2LSE
        speed = 206.0 * (992.9696969 * 0.036) / tim2_diff;
#else
        speed = (206.0 / (tim2_diff / 1008.0)) * 0.036;
#endif
        spd_f = modff(speed, &spd_i);
        UART_SendUInt((uint32_t)spd_i);
        UART_SendChar('.');
        UART_SendUInt((uint32_t)(spd_f * 10));
        UART_SendChar(':');
#ifdef TIM3LSE
        speed = 206.0 * (992.9696969 * 0.036) / tim3_diff;
#else
        speed = (206.0 / (tim3_diff / 1008.0)) * 0.036;
#endif
        spd_f = modff(speed, &spd_i);
        UART_SendUInt((uint32_t)spd_i);
        UART_SendChar('.');
        UART_SendUInt((uint32_t)(spd_f * 10));

        UART_SendStr("   CDC: ");
        cdc = (uint32_t)((60.0 / tim2_diff) * 1008.0);
        UART_SendUInt(cdc);
        UART_SendChar(':');
        cdc = (uint32_t)((60.0 / tim3_diff) * 1008.0);
        UART_SendUInt(cdc);

        UART_SendStr("   TIM: ");
        UART_SendStr(TIM2_CR1_bit.CEN ? "on" : "off");
        UART_SendChar(':');
        UART_SendStr(TIM3_CR1_bit.CEN ? "on" : "off");
#endif

        // Measure supply voltage
        if (cntr_wake % 25 == 0) {
            CLK_PCKENR2_bit.PCKEN20 = 1; // Enable ADC peripherial (PCKEN20)
            ADC_Vrefint_Init();
            vrefint = ADC_Vrefint_Measure(5);
            ADC_Vrefint_Disable();
            CLK_PCKENR2_bit.PCKEN20 = 0; // Disable ADC peripherial
            vrefint = (((uint32_t)factory_vref * 300) / vrefint);
        }
#ifdef DEBUG
        UART_SendStr("   Vcc: ");
        UART_SendInt(vrefint / 100); UART_SendChar('.');
        UART_SendInt(vrefint % 100); UART_SendChar('V');
        UART_SendChar('\n');
#endif

        // Prepare data packet for nRF24L01
        buf[0]  = cntr_EXTI1 >> 8;
        buf[1]  = cntr_EXTI1 & 0xff;
        buf[2]  = cntr_EXTI2 >> 8;
        buf[3]  = cntr_EXTI2 & 0xff;
        buf[4]  = tim2_diff >> 8;
        buf[5]  = tim2_diff & 0xff;
        buf[6]  = tim3_diff >> 8;
        buf[7]  = tim3_diff & 0xff;
        buf[8]  = vrefint >> 8;
        buf[9]  = vrefint & 0xff;
        buf[10] = prev_observe_TX;
        buf[11] = cntr_wake >> 8;
        buf[12] = cntr_wake & 0xff;
        buf[13] = packets_lost >> 8;
        buf[14] = packets_lost & 0xff;
        buf[15] = tx_power;

        nRF24_SetRFChannel(RF_CHANNEL); // Set RF channel to clean "PLOS_CNT" part of the OBERVER_TX register
        i = nRF24_TXPacket(buf,TX_PAYLOAD);
        prev_observe_TX = nRF24_ReadReg(nRF24_REG_OBSERVE_TX);
        nRF24_PowerDown(); // Standby-I -> Power down

        packets_lost += prev_observe_TX >> 4;

        // Lame adaptive TX power: increase power if it was retransmissions and decrease otherwise
        if ((prev_observe_TX & 0x0f) > 0) {
            if (tx_power < 3) nRF24_SetTXPower(tx_rf_powers[tx_power++]);
        } else {
            if (tx_power > 0) nRF24_SetTXPower(tx_rf_powers[tx_power--]);
        }

#ifdef DEBUG
        UART_SendStr("  nRF24: ");
        UART_SendHex8(i);
        UART_SendStr("   OTX: ");
        UART_SendHex8(prev_observe_TX);
        UART_SendStr("   P.lost: ");
        UART_SendInt(packets_lost);
        UART_SendStr("   TXpwr: ");
        UART_SendInt(tx_power);
        UART_SendChar('\n');
#endif

        if (TIM2_CR1_bit.CEN || TIM3_CR1_bit.CEN) {
#ifdef DEBUG
            UART_SendStr("WFI\n");
#endif
            CPU_CFG_GCR_bit.AL = 1; // Set interrupt-only activation level
            asm("wfi"); // Wait mode
        } else {
#ifdef DEBUG
            UART_SendStr("HALT\n");
#endif
            RTC_WakeupSet(DISABLE); // Disable wakeup timer
            CPU_CFG_GCR_bit.AL = 0; // Set main activation level
            asm("halt"); // Halt mode
        }
    }
}
