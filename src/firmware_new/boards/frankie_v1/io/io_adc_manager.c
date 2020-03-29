#include "firmware_new/boards/frankie_v1/io/io_adc_manager.h"

#define MAX_NUMBER_OF_ADC_HANDLES 10

static ADC_HandleTypeDef* _adc_handles[MAX_NUMBER_OF_ADC_HANDLES];
static size_t _num_adc_handles;

void io_adc_manager_init(ADC_HandleTypeDef*[] adc_handles, size_t num_adc_handles)
{
    if (num_adc_handles > MAX_NUMBER_OF_ADC_HANDLES)
    {
        // TODO: should we change other assertions throughout the `io` layer to use
        //       `Error_Handler()` instead? Or maybe just return an error code?
        Error_Handler();
    }

    _num_adc_handles = num_adc_handles;

    for (size_t i = 0; i < num_adc_handles; i++)
    {
        _adc_handles[i] = adc_handles[i];
    }
}

void io_adc_manager_addAdc(Adc_t* adc) {}

void io_adc_manager_adcCallback() {}
