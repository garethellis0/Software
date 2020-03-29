#pragma once

#include "firmware_new/boards/frankie_v1/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h"
#include "firmware_new/boards/frankie_v1/main.h"
#include "firmware_new/boards/frankie_v1/io/io_adc.h

// TODO: jdoc here
void io_adc_manager_init(ADC_HandleTypeDef*[] adc_handles, size_t num_adc_handles);

// TODO: jdoc here
void io_adc_manager_addAdc(Adc_t* adc);

// TODO: jdoc here
void io_adc_manager_adcCallback();
