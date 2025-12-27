/*
 * encoder.c
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#include "encoder.h"
#include "stm32g4xx_hal.h"
#include "foc_math.h"
#include "motor_controller_conf.h"

//Global I2C busy flag for encoder
uint8_t encoder_i2c_busy = 0;



HAL_StatusTypeDef Encoder_init(Encoder *encoder, I2C_HandleTypeDef *hi2c) {
  encoder->hi2c = hi2c;
//  encoder->i2c_update_counter = 0;

  encoder->cpr = ENCODER_DIRECTION * (1 << ENCODER_PRECISION_BITS);  // 12 bit precision


  //debugging to find out why cpr = 0 at later stage

  printf("ENCODER_DIRECTION=%d, ENCODER_PRECISION_BITS=%d, CPR=%ld\r\n",
         ENCODER_DIRECTION, ENCODER_PRECISION_BITS, encoder->cpr);




  encoder->position_offset = 0.f;

  // defaults to be 2000 Hz cutoff, out of 10 kHz loop
  encoder->velocity_filter_alpha = 0.7153904566639707f;

  encoder->position_raw = 0;
  encoder->n_rotations = 0;

  encoder->position = 0.f;
  encoder->velocity = 0.f;

  Encoder_resetFluxOffset(encoder);

  HAL_StatusTypeDef status = HAL_ERROR;
  while (status) {
    HAL_I2C_Init(encoder->hi2c);

    // wait for I2C device to power up
    HAL_Delay(100);

    status = HAL_I2C_Mem_Read(encoder->hi2c, AS5600_I2C_ADDR << 1, AS5600_ANGLE_ADDR, I2C_MEMADD_SIZE_8BIT, encoder->i2c_buffer, 2, 100);
  }

  return status;
}

void Encoder_resetFluxOffset(Encoder *encoder) {
  encoder->n_rotations = 0;
  encoder->flux_offset = 0.f;
  memset((uint8_t *)encoder->flux_offset_table, 0, ENCODER_LUT_ENTRIES*sizeof(float));
}





HAL_StatusTypeDef Encoder_update(Encoder *encoder) {
  // commutation frequency (10 kHz) should be slower than I2C transfer speed (~12.86 kHz)




  // Read the raw reading from the I2C sensor, the range should be [0, cpr-1].
  // safety check to handle encoder data frame mismatch error
//  uint16_t raw_reading = (((uint16_t)encoder->i2c_buffer[0]) << 8) | encoder->i2c_buffer[1];
//  if (raw_reading >= abs(encoder->cpr)) {
//    return 0x04;
//  }

  // TODO: implement encoder lut-table Linearization



  // I2C takes ~77.75 us (12.86 kHz) to finish one transaction
//  HAL_I2C_Master_Receive_IT(encoder->hi2c, AS5600_I2C_ADDR << 1, encoder->i2c_buffer, 2);



  //from ChatGPT, get data by polling instead of _IT method

  HAL_I2C_Master_Receive(encoder->hi2c, AS5600_I2C_ADDR << 1, encoder->i2c_buffer, 2, 5);

  uint16_t raw_reading = (((uint16_t)encoder->i2c_buffer[0]) << 8) | encoder->i2c_buffer[1];
  if (raw_reading >= abs(encoder->cpr)) {
    return 0x04;
  }




  // Calculate the change in reading
  int16_t reading_delta = encoder->position_raw - raw_reading;

  // Handle multi-rotation crossing.
  if (abs(reading_delta) >= abs(encoder->cpr / 2)) {
    encoder->n_rotations += ((encoder->cpr * reading_delta) > 0) ? 1 : -1;
  }
  encoder->position_raw = raw_reading;

  // Convert the raw position to position in radians (rad)
  float position = (((float)raw_reading / (float)encoder->cpr) + encoder->n_rotations) * (M_2PI_F);

  // Update the delta position
  float delta_position = position - encoder->position;
  encoder->position = position;



/*

  // ========== NEW UNWRAP LOGIC (robust, continuous) ==========

  // Compute difference with wrap-around correction
  int32_t diff = (int32_t)raw_reading - (int32_t)encoder->position_raw;

  if (diff > encoder->cpr / 2)
      diff -= encoder->cpr;
  else if (diff < -encoder->cpr / 2)
      diff += encoder->cpr;

  // Update continuous position (radians)
  float delta_position = (float)diff * (M_2PI_F / (float)encoder->cpr);
  encoder->position += delta_position;

  // Save raw reading for next cycle
  encoder->position_raw = raw_reading;

  encoder->n_rotations = 0;

*/

  // Update the filtered velocity
  float velocity = delta_position * (float)COMMUTATION_FREQ;
  encoder->velocity += encoder->velocity_filter_alpha * (velocity - encoder->velocity);

  return HAL_OK;
}



/*

HAL_StatusTypeDef Encoder_update(Encoder *encoder)
{
    extern uint8_t encoder_i2c_busy;

    // 1. If previous async read finished, process result
    if (!encoder_i2c_busy)
    {
        uint16_t raw =
            ((uint16_t)encoder->i2c_buffer[0] << 8) |
             encoder->i2c_buffer[1];

        if (raw < abs(encoder->cpr))
        {
            int32_t prev = encoder->position_raw;
            int32_t diff = raw - prev;
            int32_t half = encoder->cpr / 2;

            // rollover
            if (diff >  half) diff -= encoder->cpr;
            if (diff < -half) diff += encoder->cpr;

            // glitch filtering
            float delta_rad = diff * (M_2PI_F / encoder->cpr);
            if (fabsf(delta_rad) < 0.1f)   // allowable per 100us
            {
                encoder->position_raw = raw;
                encoder->n_rotations += diff > half ? -1 :
                                        diff < -half ?  1 : 0;

                float pos = ((float)raw / encoder->cpr
                              + encoder->n_rotations) * M_2PI_F;

                float vel = (pos - encoder->position) * COMMUTATION_FREQ;
                encoder->velocity += encoder->velocity_filter_alpha * (vel - encoder->velocity);
                encoder->position = pos;
            }
        }

        // 2. Launch next async read
        if (HAL_I2C_Master_Receive_IT(encoder->hi2c,
                                      AS5600_I2C_ADDR << 1,
                                      encoder->i2c_buffer,
                                      2) == HAL_OK)
        {
            encoder_i2c_busy = 1;
        }
    }

    return HAL_OK;
}


*/














