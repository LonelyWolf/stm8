#include "stdint.h"
#include "iostm8l051f3.h"

#include "stm8l051.h"
#include "uart.h"
#include "nRF24.h"
#include "rtc.h"

#include "math.h"


///////////////////////////////////////////////////////////////////////////////


#define DEBUG
#define SOFT_DEBOUNCE     // If defined - use software debounce, no debounce otherwise

#define TIM2LSE           // If defined - TIM2 will be clocked from LSE
#define TIM3LSE           // If defined - TIM3 will be clocked from LSE

#define TX_PAYLOAD    13  // nRF24L01 Payload length


///////////////////////////////////////////////////////////////////////////////


uint8_t i;

uint16_t vrefint;
uint16_t factory_vref;

uint16_t cntr_EXTI1 = 0;  // EXTI1 impulse counter
uint16_t cntr_EXTI2 = 0;  // EXTI2 impulse counter

volatile uint16_t tim2_diff = 0;
volatile uint16_t tim3_diff = 0;

volatile uint16_t tim2 = 0;
volatile uint16_t tim3 = 0;

float speed;
float spd_f, spd_i;
uint32_t cdc;

uint8_t buf[TX_PAYLOAD]; // nRF24L01 payload buffer

uint8_t prev_observe_TX = 0; // Last value of nRF24L01 OBSERVE_TX register

uint16_t cntr_wake = 0; // Wakeup counter (for debug purposes)


///////////////////////////////////////////////////////////////////////////////


// RTC_WAKEUP IRQ handle
#pragma vector=RTC_WAKEUP_vector
__ramfunc __interrupt void RTC_IRQHandler(void) {
    CPU_CFG_GCR_bit.AL = 0; // Set main activation level
    RTC_ISR2_bit.WUTF = 0; // Clear wakeup timer interrupt flag
}

// EXTI1 IRQ handle
#pragma vector=EXTI1_vector
__interrupt void EXTI1_IRQHandler(void) {
    TIM2_CR1_bit.CEN = 1; // Enable TIM2 counter

#ifdef SOFT_DEBOUNCE
    if (tim2 > 60) {
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

    if (!RTC_CR2_bit.WUTE) {
        RTC_WakeupIT(ENABLE); // Enable wakeup interrupt
        RTC_WakeupSet(ENABLE); // Enable wakeup timer
    }
    EXTI_SR1_bit.P1F = 1; // Clear EXTI1 IRQ flag
}

// EXTI2 IRQ handle
#pragma vector=EXTI2_vector
__interrupt void EXTI2_IRQHandler(void) {
    TIM3_CR1_bit.CEN = 1; // Enable TIM3 counter

#ifdef SOFT_DEBOUNCE
    if (tim3 > 60) {
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

    if (!RTC_CR2_bit.WUTE) {
        RTC_WakeupIT(ENABLE); // Enable wakeup interrupt
        RTC_WakeupSet(ENABLE); // Enable wakeup timer
    }
    EXTI_SR1_bit.P2F = 1; // Clear EXTI2 IRQ flag
}

// TIM2 Update/Overflow/Trigger/Break interrupt IRQ handle
#pragma vector=TIM2_OVR_UIF_vector
__ramfunc __interrupt void TIM2_UIF_IRQHandler(void) {
    tim2++;
    if (tim2 > 6000) {
        TIM2_CR1_bit.CEN = 0; // Disable TIM2 counter
        cntr_EXTI1 = 0;
        tim2_diff  = 0;
    }
    TIM2_SR1_bit.UIF = 0; // Clear TIM2 update interrupt flag
}

// TIM3 Update/Overflow/Trigger/Break interrupt IRQ handle
#pragma vector=TIM3_OVR_UIF_vector
__ramfunc __interrupt void TIM3_UIF_IRQHandler(void) {
    tim3++;
    if (tim3 > 6000) {
        TIM3_CR1_bit.CEN = 0; // Disable TIM3 counter
        cntr_EXTI2 = 0;
        tim3_diff  = 0;
    }
    TIM3_SR1_bit.UIF = 0; // Clear TIM3 update interrupt flag
}


///////////////////////////////////////////////////////////////////////////////


// Init Vreint ADC channel
__ramfunc void ADC_Vrefint_Init(void) {
    ADC1_CR1_bit.ADON = 1; // Enable ADC
    ADC1_TRIGR1_bit.VREFINTON = 1; // Enable internal reference voltage
    ADC1_SQR1_bit.CHSEL_S28 = 1; // Enable CHSEL_SVREFINT fast ADC channel
    ADC1_CR3 = 0x80; // Sampling time = 48 ADC clock cycles, disable analog watchdog channels
    ADC1_SQR1_bit.DMAOFF = 1; // DMA off
}

// Measure Vrefint
__ramfunc uint16_t ADC_Vrefint_Measure(uint8_t count) {
    uint16_t adc_res;
    uint16_t value = 0;
    uint8_t cntr;

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
__ramfunc void ADC_Vrefint_Disable() {
    ADC1_TRIGR1_bit.VREFINTON = 0; // Disable internal reference voltage
    ADC1_SQR1_bit.CHSEL_S28 = 0; // Disable CHSEL_SVREFINT fast ADC channel
    ADC1_CR1_bit.ADON = 0; // Disable ADC
}


///////////////////////////////////////////////////////////////////////////////


int main( void )
{
    CLK_PCKENR2_bit.PCKEN27 = 0; // Disable Boot ROM

    // Magnetic reed inputs
    // PB1 = Reed#1
    // PB2 = Reed#2
    PB_ODR_bit.ODR1 = 1; // Latch "1" in input register (is this really necessary?)
    PB_DDR_bit.DDR1 = 0; // Set PB1 as input
    PB_CR1_bit.C11  = 1; // Pull-up (does it have sense here?)
    PB_CR2_bit.C21  = 1; // Enable external interrupt
    PB_ODR_bit.ODR2 = 1; // Latch "1" in input register (is this really necessary?)
    PB_DDR_bit.DDR2 = 0; // Set PB2 as input
    PB_CR1_bit.C12  = 1; // Pull-up (does it have sense here?)
    PB_CR2_bit.C22  = 1; // Enable external interrupt

    // UART pinout
    // PA2 - UART_TX
    // PA3 - UART_RX
    SYSCFG_RMPCR1_bit.USART1TR_REMAP = 0x01; // USART1 remap: TX on PA2 and RX on PA3
    UART_Init();
    UART_SendStr("STM8L051F3P6 is online\n");

    // nRF24L01 pinout
    // PB3 - CE
    // PB4 - NSS
    // PB5 - SCK
    // PB6 - MOSI
    // PB7 - MISO
    // PC0 - IRQ
    nRF24_init(); // Init SPI interface for nRF24L01 communications
    UART_SendStr("nRF24L01 check ... ");
    if (nRF24_Check() != 0) {
	UART_SendStr("wrong answer from SPI device.\n");
	UART_SendStr("MCU will now halt.\n");
	while(1);
    } else {
        UART_SendStr("ok\n");
    }
    nRF24_TXMode(); // Configure nRF24L01 for TX mode

    UART_SendStr("nRF24:");
    i = nRF24_TXPacket(buf,TX_PAYLOAD);
    UART_SendHex8(i);
    prev_observe_TX = nRF24_ReadReg(nRF24_REG_OBSERVE_TX);
    UART_SendStr("   OTX:");
    UART_SendHex8(prev_observe_TX);
    UART_SendChar('\n');

    nRF24_PowerDown(); // Put nRF24L01 into power down mode
    for (i = 0; i < TX_PAYLOAD; i++) buf[i] = 0x00;

    // Configure RTC
    UART_SendStr("RTC init ... ");
    RTC_Init(); // Init RTC (hardware reset only in power on sequence!)
    UART_SendStr("ok\n");
    RTC_WakeupConfig(RTC_WUC_RTCCLK_Div16); // RTC wakeup = LSE/RTCDIV/16

    // If VREFINT not set on factory, assign standard value
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
    UART_SendStr("Vcc: ");
    UART_SendInt(vrefint / 100); UART_SendChar('.');
    UART_SendInt(vrefint % 100); UART_SendStr("V\n");
    UART_SendChars('-',80); UART_SendChar('\n');

    // Timers initialization
#ifdef TIM2LSE
    CLK_PCKENR1_bit.PCKEN10 = 1; // Enable TIM2 peripherial
    SYSCFG_RMPCR2_bit.TIM2TRIGLSE_REMAP = 1; // TIM2 trigger controlled by LSE (OSC32_IN)
    TIM2_PSCR = 0; // Prescaler is off
    TIM2_ARRH = 0x00; // Auto-reload value (32768 / 33 -> 0.9929696969)
    TIM2_ARRL = 0x20;
    TIM2_ETR = 0x40; // External trigger: enabled, non-inverted, prescaler off, filter off
    TIM2_IER_bit.UIE = 1; // Update interrupt enable
    //TIM2_CR1_bit.CEN = 1; // Enable TIM2 counter
    TIM2_EGR_bit.UG = 1; // Generate UEV (update event) to reload TIM2 and set the prescaler
#else
    CLK_PCKENR1_bit.PCKEN10 = 1; // Enable TIM2 peripherial
    TIM2_PSCR = 4; // Prescaler is 16
    TIM2_ARRH = 0; // Auto-reload value (TIM2 overflow in 1.0008ms at 2MHz)
    TIM2_ARRL = 123;
    TIM2_IER_bit.UIE = 1; // Update interrupt enable
    //TIM2_CR1_bit.CEN = 1; // Enable TIM2 counter
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
    //TIM3_CR1_bit.CEN = 1; // Enable TIM3 counter
    TIM3_EGR_bit.UG = 1; // Generate UEV (update event) to reload TIM3 and set the prescaler
#else
    CLK_PCKENR1_bit.PCKEN11 = 1; // Enable TIM3 peripherial
    TIM3_PSCR = 4; // Prescaler is 16
    TIM3_ARRH = 0; // Auto-reload value (TIM3 overflow in 1.0008ms at 2MHz)
    TIM3_ARRL = 123;
    TIM3_IER_bit.UIE = 1; // Update interrupt enable
    //TIM3_CR1_bit.CEN = 1; // Enable TIM3 counter
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
    RTC_WakeupTimerSet(63); // 1 second wakeup
    //RTC_WakeupTimerSet(127); // 2 seconds wakeup
    RTC_WakeupIT(DISABLE); // Disable wakeup interrupt
    RTC_WakeupSet(DISABLE); // Disable wakeup timer

    // Go deep sleep mode
    asm("halt");

    // Main loop ^_^
    while(1) {
        cntr_wake++; // Count wakeups for debug purposes

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

        UART_SendStr("   TIM2:");
        UART_SendStr(TIM2_CR1_bit.CEN ? "on" : "off");
        UART_SendStr("   TIM3:");
        UART_SendStr(TIM3_CR1_bit.CEN ? "on" : "off");

        // Measure supply voltage
        if (cntr_wake % 25 == 0) {
            CLK_PCKENR2_bit.PCKEN20 = 1; // Enable ADC peripherial (PCKEN20)
            ADC_Vrefint_Init();
            vrefint = ADC_Vrefint_Measure(5);
            ADC_Vrefint_Disable();
            CLK_PCKENR2_bit.PCKEN20 = 0; // Disable ADC peripherial
            vrefint = (((uint32_t)factory_vref * 300) / vrefint);
        }
        UART_SendStr("   Vcc:");
        UART_SendInt(vrefint / 100); UART_SendChar('.');
        UART_SendInt(vrefint % 100); UART_SendChar('V');

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

        UART_SendStr("   nRF24:");
        i = nRF24_TXPacket(buf,TX_PAYLOAD);
        UART_SendHex8(i);
        prev_observe_TX = nRF24_ReadReg(nRF24_REG_OBSERVE_TX);
        UART_SendStr("   OTX:");
        UART_SendHex8(prev_observe_TX);

        UART_SendChar('\n');

        if (TIM2_CR1_bit.CEN || TIM3_CR1_bit.CEN) {
            CPU_CFG_GCR_bit.AL = 1; // Set interrupt-only activation level
            UART_SendStr("WFI\n");
            asm("wfi"); // Wait mode
        } else {
            UART_SendStr("HALT\n");
            RTC_WakeupIT(DISABLE); // Disable wakeup interrupt
            RTC_WakeupSet(DISABLE); // Disable wakeup timer
            nRF24_PowerDown(); // Powerdown nRF24L01
            CPU_CFG_GCR_bit.AL = 0; // Set main activation level
            asm("halt"); // Active-Halt mode
            nRF24_Wake(); // Wake nRF24L01 (Standby-I mode), this take about 1.5ms
        }
    }
}
