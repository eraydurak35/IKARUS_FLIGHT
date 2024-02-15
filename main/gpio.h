#ifndef GPIO_H
#define GPIO_H
#include <stdio.h>

#define RC_PPM_PIN 35
#define BUZZ_PIN 2
#define VSENS_PIN 34

void gpio_configure();
float get_bat_volt();
#endif