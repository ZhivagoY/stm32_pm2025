#include <stdint.h>
#include <stm32f10x.h>

volatile uint32_t delay_time = 180000000;
volatile uint32_t speeds[] = {180000000, 90000000, 45000000, 22500000, 11250000};
volatile uint8_t speed_index = 0;
volatile uint8_t button_was_pressed = 0;

uint8_t is_button_pressed() {
    if ((GPIOA->IDR & GPIO_IDR_IDR0) == 0) {
        if(!button_was_pressed) {
            button_was_pressed = 1;
            return 1;
        }
    } else {
        button_was_pressed = 0;
    }
    return 0;
}

void delay(uint32_t ticks) {
	for (int i = 0; i < ticks; i++) {
		__NOP();
	}
}

void try_change_frequency() {
    if (is_button_pressed()) {
        speed_index = (speed_index + 1) % 5;
        delay_time = speeds[speed_index];
    }
    delay(300000);
}

void delay_with_check(uint32_t ticks) {
	uint32_t checks_count = delay_time / 300000;
    for (int i = 0; i < checks_count; i++) {
        try_change_frequency();
    }
}

int main(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
    
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13_1;

    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;
    GPIOA->ODR |= GPIO_ODR_ODR0;
    
    while(1) {
        GPIOC->ODR ^= GPIO_ODR_ODR13;
        delay_with_check(delay_time);
    }
}