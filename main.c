#include <inttypes.h>
#include <hal.h>

int main(void) {
    //set pins
    uint16_t ir_receiver = PIN('B', 12);
    uint16_t esp_tx = PIN('A', 10);
    uint16_t esp_rx = PIN('A', 9);
    uint16_t mpu_scl = PIN('B', 6);
    uint16_t mpu_sda = PIN('B', 7);

    // enable clocks
    RCC->RCC_APB2ENR |= (3UL << 2); //enable ports A and B clocks
    RCC->RCC_APB2ENR |= 1UL; //enable alternate function IO clock
    RCC->RCC_APB2ENR |= (1UL << 14); //enable USART1 clock
    RCC->RCC_APB1ENR |= (1UL << 21); //enable I2C1 clock
    RCC->RCC_APB1ENR |= 1UL; // enable timer clock

    //set I2C Pins to high
    GPIO(PINBANK(mpu_scl))->GPIOx_ODR |= BIT(PINNUM(mpu_scl));
    GPIO(PINBANK(mpu_sda))->GPIOx_ODR |= BIT(PINNUM(mpu_sda));

    // set GPIO Pins to correct modes
    gpio_set_mode(ir_receiver, GPIO_MODE_INPUT_FLOAT, GPIO_MODE_INPUT);
    gpio_set_mode(esp_tx, GPIO_MODE_INPUT_FLOAT, GPIO_MODE_INPUT);
    gpio_set_mode(esp_rx, GPIO_MODE_OUTPUT_AF_PUSH_PULL, GPIO_MODE_OUTPUT_CLOCK_SPEED_50);
    gpio_set_mode(mpu_scl, GPIO_MODE_OUTPUT_AF_OPEN_DRAIN, GPIO_MODE_OUTPUT_CLOCK_SPEED_50);
    gpio_set_mode(mpu_sda, GPIO_MODE_OUTPUT_AF_OPEN_DRAIN, GPIO_MODE_OUTPUT_CLOCK_SPEED_50);

    //unmask EXTI register
    EXTI->EXTI_IMR |= BIT(PINNUM(ir_receiver));

    //set rising and falling interrupt
    EXTI->EXTI_RTSR |= BIT(PINNUM(ir_receiver));
    EXTI->EXTI_FTSR |= BIT(PINNUM(ir_receiver));

    //enable 10-15 interrupts and clear pending interrupts
    NVIC->ISER[0] |= (1UL << 40);
    NVIC->ICPR[0] |= (1UL << 40);

    // Configure Timer for 1µs resolution (72MHz / 72 = 1MHz)
    TIM2->TIM2_PSC = 71;
    TIM2->TIM2_ARR = 0xFFFFFFFF; // Auto-reload to max
    TIM2->TIM2_CR1 |= 1UL; // Start timer

    return 0;
}

//Interrupt Handler for IR Receiver
void EXTI15_10_IRQHandler(void) {
    if (EXTI->EXTI_PR & BIT(12)) {
        EXTI->EXTI_PR |= BIT(12); // Clear the interrupt flag
        
        // Add your IR receiving logic here
    }
}

__attribute__((naked,noreturn)) void _reset(void) {
    // memset .bss to zero, and copy .data section to RAM region
    extern long _sdata, _edata, _sbss, _ebss, _sidata;
    for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
    for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;
    
    main();
    while(1) (void) 0;  
}

extern void _estack(void);

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 91])(void) = {
    _estack, _reset};
