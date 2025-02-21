#include <inttypes.h>

#define PIN(bank, num) (((bank) - 'A') << 8 | num)
#define PINNUM(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
#define BIT(x) (1UL << (x))

struct gpio {
    volatile uint32_t GPIOx_CRL,GPIOx_CRH, GPIOx_IDR, GPIOx_ODR, 
        GPIOx_BSRR, GPIOx_BRR, GPIOx_LCKR;
};

#define GPIO(bank) ((struct gpio *) (0x40010800 + 0x400 * bank))

enum cnf {GPIO_MODE_INPUT_ANALOG, GPIO_MODE_INPUT_FLOAT, GPIO_MODE_INPUT_PULL_PUSH};
enum mode_output {GPIO_MODE_OUTPUT_PUSH_PULL, GPIO_MODE_OUTPUT_OPEN_DRAIN,
    GPIO_MODE_OUTPUT_AF_PUSH_PULL, GPIO_MODE_OUTPUT_AF_OPEN_DRAIN};
enum mode_input {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_CLOCK_SPEED_10, 
    GPIO_MODE_OUTPUT_CLOCK_SPEED_2, GPIO_MODE_OUTPUT_CLOCK_SPEED_50};

static inline void gpio_set_mode(uint16_t pin, uint8_t cnf, uint8_t mode) {
    struct gpio *gpio = GPIO(PINBANK(pin));
    uint8_t n = PINNUM(pin);
    uint8_t full_mode = (cnf << 2) | mode;
    if (n <= 15)
    {
        gpio->GPIOx_CRL &= ~(15U << (n * 4));
        gpio->GPIOx_CRL |= (full_mode << (n * 4));
    } else {
        gpio->GPIOx_CRH &= ~(15U << (n*4));
        gpio->GPIOx_CRH |= (full_mode << (n * 4));
    }
}

struct rcc {
    volatile uint32_t RCC_CR, RCC_CFGR, RCC_CIR, RCC_APB2RSTR, RCC_APB1RSTR,
    RCC_AHBENR, RCC_APB2ENR, RCC_APB1ENR, RCC_BDCR, RCC_CSR, RCC_AHBSTR, RCC_CFGR2;

};

#define RCC ((struct rcc *) 0x40021000)

struct exti {
    volatile uint32_t EXTI_IMR, EXTI_EMR, EXTI_RTSR, EXTI_FTSR, EXTI_SWIER,
    EXTI_PR;
};
    
#define EXTI ((struct exti *) 0x40010400)



struct nvic {
    volatile uint32_t ISER[8];
    volatile uint32_t ICER[8];
    volatile uint32_t ISPR[8];
    volatile uint32_t ICPR[8];
    volatile uint32_t IABR[8];
    volatile uint8_t  IPR[240];
};

#define NVIC ((struct nvic *) 0xE000E100)

struct tim2 {
    volatile uint32_t TIM2_CR1, TIM2_CR2, TIM2_SMCR, TIM2_DIER, TIM2_SR,
     TIM2_EGR, TIM2_CCMR1, TIM2_CCMR2, TIM2_CCER, TIM2_CNT, TIM2_PSC, 
     TIM2_ARR, TIM2_RCR, TIM2_CCR1, TIM2_CCR2, TIM2_CCR3, TIM2_CCR4, 
     TIM2_BDTR, TIM2_DCR, TIM2_DMAR;
};

#define TIM2 ((struct tim2 *) 0x40000000)