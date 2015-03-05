#ifndef __MAIN_H
#define __MAIN_H

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

// The VREFINT_Factory_CONV byte represents the LSB of the VREFINT 12-bit
// ADC conversion result. The MSB have a fixed value: 0x06
__near __no_init const unsigned char Factory_VREFINT @ 0x4910;

#endif // __MAIN_H