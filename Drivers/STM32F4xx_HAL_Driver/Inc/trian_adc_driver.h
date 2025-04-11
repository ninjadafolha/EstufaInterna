/**
  ******************************************************************************
  * @file    trian_adc_driver.h
  * @brief   This file contains the headers of the ADC driver.
  ******************************************************************************
  * @attention
  * Company: Trian 
  * 
  *
  ******************************************************************************
  */

#ifndef __TRIAN_ADC_DRIVER_H
#define __TRIAN_ADC_DRIVER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"




void ADC_Init(uint8_t pinValue);
void ADC_conversion(void);
int ADC_read(void);



#ifdef __cplusplus
}
#endif

#endif /* __TRIAN_ADC_DRIVER_H */