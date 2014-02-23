typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus, BitAction;

// The VREFINT_Factory_CONV byte represents the LSB of the VREFINT 12-bit
// ADC conversion result. The MSB have a fixed value: 0x06
__near __no_init const unsigned char Factory_VREFINT @ 0x4910;