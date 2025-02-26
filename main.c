#include <inttypes.h>
#include <hal.h>

// Global variables for capturing IR signal
#define MAX_EDGES 100
static volatile uint32_t edges_duration[MAX_EDGES];
static volatile uint8_t edge_index = 0;
static volatile uint32_t start_time = 0;

//Pins
uint16_t ir_receiver = PIN('B', 12);
uint16_t esp_tx = PIN('A', 10);
uint16_t esp_rx = PIN('A', 9);
uint16_t mpu_scl = PIN('B', 6);
uint16_t mpu_sda = PIN('B', 7);
uint16_t ir_led = PIN('B', 13);
//in the future, esp8266 will be used to communicate with wifi to change the time the on/off function should run

int main(void) {

    // enable clocks
    RCC->RCC_APB2ENR |= (3UL << 2); //enable ports A and B clocks
    RCC->RCC_APB2ENR |= 1UL; //enable alternate function IO clock
    RCC->RCC_APB2ENR |= (1UL << 14); //enable USART1 clock
    RCC->RCC_APB1ENR |= (1UL << 21); //enable I2C1 clock
    RCC->RCC_APB1ENR |= 1UL; // enable timer clock

    //set I2C & IR Receiver Pins to high
    GPIO(PINBANK(mpu_scl))->GPIOx_ODR |= BIT(PINNUM(mpu_scl));
    GPIO(PINBANK(mpu_sda))->GPIOx_ODR |= BIT(PINNUM(mpu_sda));
    GPIO(PINBANK(ir_receiver))->GPIOx_ODR |= BIT(PINNUM(ir_receiver));

    // set GPIO Pins to correct modes
    gpio_set_mode(ir_receiver, GPIO_MODE_INPUT_PULL_PUSH, GPIO_MODE_INPUT);
    gpio_set_mode(esp_tx, GPIO_MODE_INPUT_FLOAT, GPIO_MODE_INPUT);
    gpio_set_mode(esp_rx, GPIO_MODE_OUTPUT_AF_PUSH_PULL, GPIO_MODE_OUTPUT_CLOCK_SPEED_50);
    gpio_set_mode(mpu_scl, GPIO_MODE_OUTPUT_AF_OPEN_DRAIN, GPIO_MODE_OUTPUT_CLOCK_SPEED_50);
    gpio_set_mode(mpu_sda, GPIO_MODE_OUTPUT_AF_OPEN_DRAIN, GPIO_MODE_OUTPUT_CLOCK_SPEED_50);
    gpio_set_mode(ir_led, GPIO_MODE_OUTPUT_AF_PUSH_PULL, GPIO_MODE_OUTPUT_CLOCK_SPEED_2); //May need to not use AF (AF if I use TIM2 to trigger, otherwise software controlled is just regular)

    //unmask EXTI register
    EXTI->EXTI_IMR |= BIT(PINNUM(ir_receiver));

    //set rising and falling interrupt
    EXTI->EXTI_RTSR |= BIT(PINNUM(ir_receiver));
    EXTI->EXTI_FTSR |= BIT(PINNUM(ir_receiver));

    //enable 10-15 and RTC interrupts and clear pending interrupts
    NVIC->ISER[1] |= (1UL << (40 - 32));
    NVIC->ICPR[1] |= (1UL << (40 - 32));
    NVIC->ISER[0] |= (1UL << (3));
    NVIC->ICPR[0] |= (1UL << (3));

    // Configure Timer
    TIM2->TIM2_PSC = 71;
    TIM2->TIM2_ARR = 0xFFFFFFFF; // Auto-reload to max
    TIM2->TIM2_CR1 |= 1UL; // Start timer

    //Initialize RTC, manually set alarm to be 10:30 PM
    rtc_init();
    RTC->RTC_CRL |= (1UL << 4);  // Enter Configuration Mode
    RTC->RTC_ALRH = (81000 >> 16); // Set alarm high bits (22:30)
    RTC->RTC_ALRL = (81000 & 0xFFFF); // Set alarm low bits
    RTC->RTC_CRL &= ~(1UL << 4); // Exit Configuration Mode
    RTC->RTC_CRH |= (1UL << 1);  // Enable RTC alarm interrupt

    while(1){}

    return 0;
}

//Interrupt Handler for IR Receiver
void EXTI15_10_IRQHandler(void) {

    if (EXTI->EXTI_PR & BIT(PINNUM(ir_receiver))) {
        uint32_t end_time = TIM2->TIM2_CNT;
        EXTI->EXTI_PR |= BIT(PINNUM(ir_receiver));
        
        //Algorithm for receiving IR Signals:
        //Every low pulse, set the start time; every high pulse, record the start time
        //The total pulse time (end - start) is then put into edges_duration where index 0 = the first pulse, index 1 = second, etc...
        if(GPIO(PINBANK(ir_receiver))->GPIOx_IDR & BIT(PINNUM(ir_receiver))) {
            start_time = TIM2->TIM2_CNT;
        } else {
            edges_duration[edge_index] = end_time - start_time;
            edge_index++;
        }
        
    }
}

//Interrupt Handler for IR Transmitter once RTC time matches time given from esp8266
void USART1_IRQHandler(void) {
    //TODO
}

void RTC_IRQHandler(void) {
    //TODO use IR data from reciever to transmit from transmitter
    //then, reset the RTC to trigger on the next day at the same time by resetting the RTC to 0.
    //Later, once connected to the esp8266, set up an esp interrupt to trigger a RTC reset to 0 at midnight so the interrupt will trigger daily at the set time
}

void Default_Handler(void) {
    while (1);
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
__attribute__((section(".vectors"))) void (*const tab[16 + 60])(void) = {
    _estack, _reset,
    [2 ... 16 + 59] = Default_Handler,
    [16 + 40] = EXTI15_10_IRQHandler
};
