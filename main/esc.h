#ifndef ESC_H
#define ESC_H

#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"

#define DSHOT_ESC_RESOLUTION_HZ 40000000
#define MOTOR_LB_PIN 27
#define MOTOR_LT_PIN 4  // 35 input only // 23 didnt work
#define MOTOR_RB_PIN 26
#define MOTOR_RT_PIN 25

void dshot_esc_init();
void write_throttle(uint16_t thr1, uint16_t thr2, uint16_t thr3, uint16_t thr4);

#endif