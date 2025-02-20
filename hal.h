#include <inttypes.h>

#define PIN(bank, num) (((bank) - 'A') << 8 | num)
#define PINNUM(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct gpio {
    volatile uint32_t GPIOx_CRL,GPIOx_CRH, GPIOx_IDR, GPIOx_ODR, 
        GPIOx_BSRR, GPIOx_BRR, GPIOx_LCKR;
};

#define GPIO(bank) ((struct gpio *) (0x40010800 + 0x400 * bank))

enum {GPIO_MODE_INPUT_ANALOG, GPIO_MODE_INPUT_FLOAT, GPIO_MODE_INPUT_PULL_PUSH,
    GPIO_MODE_INPUT_RESERVED};
enum {GPIO_MODE_OUTPUT_PUSH_PULL, GPIO_MODE_OUTPUT_OPEN_DRAIN,
    GPIO_MODE_OUTPUT_AF_PUSH_PULL, GPIO_MODE_OUTPUT_AF_OPEN_DRAIN};
enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_CLOCK_SPEED_10, 
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