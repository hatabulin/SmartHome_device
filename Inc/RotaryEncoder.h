#ifndef ROTARYENCODER_H
#define ROTARYENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include "usbd_def.h"

//#define MAX(x, y) (((x) > (y)) ? (x) : (y))
//#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ENC_ACCEL_THRESHOLD 5

typedef struct RotaryEncoder
{
    uint16_t min_value;
    uint16_t max_value;
    uint8_t pulses_per_dedent;
    uint8_t acceleration;
    uint8_t cur_accel;
    uint8_t readings;
    uint16_t counter;
    uint8_t value;
    uint8_t controller;
    bool dirty;

} RotaryEncoder;

void init_encoder(RotaryEncoder* enc);
uint8_t read_encoder(RotaryEncoder* enc);
uint8_t read_encoder2(RotaryEncoder* enc);
void read_encoders(RotaryEncoder* enc, uint8_t num_enc);

#endif  /* ROTARYENCODER_H */
