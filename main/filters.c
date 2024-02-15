#include <stdio.h>
#include "filters.h"
#include "math.h"
#include "string.h"

static const float fir_gain[5] =
    {
        0.199736703181241426f,
        0.200131617212090052f,
        0.200263359213336989f,
        0.200131617212090052f,
        0.199736703181241426f};

int cmpfunc(const void *a, const void *b);

int16_t meadianFilter(int16_t new_data)
{
    static const uint8_t size = 11;
    static int16_t buffer[11];
    static int16_t sorted_buffer[11];

    for (uint8_t i = 0; i < size - 1; i++)
    {
        buffer[i] = buffer[i + 1];
    }
    buffer[size - 1] = new_data;

    memcpy(sorted_buffer, buffer, size * sizeof(int16_t));
    /*  for (uint8_t i = 0; i < size; i++)
        {
            sorted_buffer[i] = buffer[i];
        }
    */
    // sort ascending order
    qsort(sorted_buffer, size, sizeof(int16_t), cmpfunc);
    // std::sort(sorted_buffer, sorted_buffer + size);
    return sorted_buffer[5];
}

int cmpfunc(const void *a, const void *b)
{
    return (*(int *)a - *(int *)b);
}

void fir_filter_init(fir_filter_t *fir)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        fir[i].size = 7;
        fir[i].index = 0;
    }
}

void fir_filter(fir_filter_t *fir, float *value)
{
    fir->buffer[fir->index] = *value;
    fir->index++;

    if (fir->index == fir->size)
        fir->index = 0;

    float temp = 0;
    uint8_t sumIndex = fir->index;

    for (uint8_t n = 0; n < fir->size; n++)
    {
        if (sumIndex > 0)
            sumIndex--;
        else
            sumIndex = fir->size - 1;
        temp += fir_gain[n] * fir->buffer[sumIndex];
    }
    *value = temp;
}

void fir_filter_custom_gain(fir_filter_t *fir, const float *gain, float *value)
{
    fir->buffer[fir->index] = *value;
    fir->index++;

    if (fir->index == fir->size)
        fir->index = 0;

    float temp = 0;
    uint8_t sumIndex = fir->index;

    for (uint8_t n = 0; n < fir->size; n++)
    {
        if (sumIndex > 0)
            sumIndex--;
        else
            sumIndex = fir->size - 1;
        temp += gain[n] * fir->buffer[sumIndex];
    }
    *value = temp;
}

void notch_filter_init(notch_filter_t *notch)
{
    for (uint8_t i = 0; i < 18; i++)
    {
        notch[i].sample_rate = 1000.0f;
        notch_configure(200.0f, 25.0f, &notch[i]);
    }
}

// Q = notchCenterFreq_hz / bandwidth_hz
void notch_configure(float cf, float bw, notch_filter_t *notch)
{
    float Q = cf / bw;
    float omega = 2.0f * M_PI * cf / notch->sample_rate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * Q);

    notch->b0 = 1.0f;
    notch->b1 = -2.0f * cs;
    notch->b2 = 1.0f;
    notch->a0 = 1.0f + alpha;
    notch->a1 = -2.0f * cs;
    notch->a2 = 1.0f - alpha;

    // prescale flter constants
    notch->b0 /= notch->a0;
    notch->b1 /= notch->a0;
    notch->b2 /= notch->a0;
    notch->a1 /= notch->a0;
    notch->a2 /= notch->a0;
}

// perform one filtering step
void notch_filter(notch_filter_t *notch, float *value)
{
    static float x = 0;
    static float y = 0;
    x = *value;
    y = notch->b0 * x + notch->b1 * notch->x1 + notch->b2 * notch->x2 - notch->a1 * notch->y1 - notch->a2 * notch->y2;
    notch->x2 = notch->x1;
    notch->x1 = x;
    notch->y2 = notch->y1;
    notch->y1 = y;
    *value = y;
}
