#ifndef GPIO_H
#define GPIO_H
#include <stdio.h>

#define BUZZ_PIN 2
#define VSENS_PIN 34

void gpio_configure();
float get_bat_volt();
void start_beep(uint32_t freq);
void stop_beep();
#endif