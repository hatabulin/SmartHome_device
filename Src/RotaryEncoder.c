//#include <libopencm3/stm32/gpio.h>
#include "RotaryEncoder.h"
#include "stm32f1xx_hal.h"

const int8_t enc_states[] = {
    0,  /* 00 00 */
    -1, /* 00 01 */
    1,  /* 00 10 */
    0,  /* 00 11 */
    1,  /* 01 00 */
    0,  /* 01 01 */
    0,  /* 01 10 */
    -1, /* 01 11 */
    -1, /* 10 00 */
    0,  /* 10 01 */
    0,  /* 10 10 */
    1,  /* 10 11 */
    0,  /* 11 00 */
    1,  /* 11 01 */
    -1, /* 11 10 */
    0   /* 11 11 */
};


void init_encoder(RotaryEncoder* enc)
{
    enc->min_value = 0;
    enc->max_value = 99;
    enc->pulses_per_dedent = 4;
    enc->acceleration = 20;
    enc->cur_accel = 0;
    enc->readings = 0;
    enc->counter = 0;
    enc->value = 1;
    enc->controller = 1;
    enc->dirty = false;
};


void read_encoders(RotaryEncoder* enc, uint8_t num_enc)
{
    int8_t enc_incr;
    int16_t value;

   	enc->dirty = false;
    if (num_enc == 0)	enc_incr = read_encoder(enc); else enc_incr = read_encoder2(enc);
    if (enc->cur_accel)
    enc->cur_accel--;

    if (enc_incr)
    {
    	enc->cur_accel = MIN(255, enc->cur_accel + enc->acceleration);
        enc->counter = MIN(enc->max_value * enc->pulses_per_dedent,MAX(enc->min_value * enc->pulses_per_dedent,enc->counter + (1 + (enc->cur_accel >> ENC_ACCEL_THRESHOLD)) * enc_incr));
        value = enc->counter / enc->pulses_per_dedent;

        if (enc->value != value)
        {
        	enc->dirty = true;
            enc->value = value;
        }
    }
}

/* returns change in encoder state (-1,0,1) */
uint8_t read_encoder(RotaryEncoder* enc) {
    uint8_t reading = 0;

    if (HAL_GPIO_ReadPin(ENC_A_DOOR_GPIO_Port,ENC_A_DOOR_Pin) == 0) reading = 0x02;
    if (HAL_GPIO_ReadPin(ENC_B_DOOR_GPIO_Port,ENC_B_DOOR_Pin) == 0) reading |= 0x01;
    // shift previous state 2 bits to left
    enc->readings <<= 2;

    // Add current state to bit 0 and 1
    if (reading & 0x01)//(1 << enc->pin_a))
        enc->readings |= 1;
    if (reading & 0x02) //(1 << enc->pin_b))
        enc->readings |= 2;

    return (enc_states[(enc->readings & 0x0f)]);
}

uint8_t read_encoder2(RotaryEncoder* enc) {
    uint8_t reading = 0;

    if (HAL_GPIO_ReadPin(ENC_A_UI_GPIO_Port,ENC_A_UI_Pin) == 0) reading = 0x02;
    if (HAL_GPIO_ReadPin(ENC_B_UI_GPIO_Port,ENC_B_UI_Pin) == 0) reading |= 0x01;
    // shift previous state 2 bits to left
    enc->readings <<= 2;

    // Add current state to bit 0 and 1
    if (reading & 0x01)//(1 << enc->pin_a))
        enc->readings |= 1;
    if (reading & 0x02) //(1 << enc->pin_b))
        enc->readings |= 2;

    return (enc_states[(enc->readings & 0x0f)]);
}
