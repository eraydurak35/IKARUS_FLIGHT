#ifndef FILTERS_H
#define FILTERS_H

#include <stdio.h>
#include "typedefs.h"

void fir_filter_init(fir_filter_t *fir);
void fir_filter(fir_filter_t *fir, float *value);
void fir_filter_custom_gain(fir_filter_t *fir, const float *gain, float *value);

void notch_filter_init(notch_filter_t *notch);
void notch_configure(float cf, float bw, notch_filter_t *notch);
void notch_filter(notch_filter_t *notch, float *value);

int16_t meadianFilter(int16_t new_data);
#endif