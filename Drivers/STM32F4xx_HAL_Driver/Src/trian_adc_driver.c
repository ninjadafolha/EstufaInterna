/**
  ******************************************************************************
  * @file    trian_adc_driver.c
  * @author  Trian Application Team
  * @brief   ADC module driver.
  *    
  * 
  *
  *    
  *
  *      
  @verbatim     
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]


  @endverbatim
  ******************************************************************************
  * @attention
  *

  *
  ******************************************************************************
  */ 


#include "stm32f4xx_hal.h"
#include "trian_adc_driver.h"

typedef struct
{
  /* data */
}ADC_PinConf_t;


void ADC_Init(uint8_t pinValue){
    RCC->AHB1ENR |= (1<<0); //activate AHB bus for PORT A
    
    GPIOA->MODER |= (3<<(pinValue*2)); // Pin 0 as analog
  
    RCC->APB2ENR |= (1<<8); // Enable clock source for ADC1
    ADC1->CR1 |= (1 << 24); // set 10 bit ADC
    ADC1->CR1 &= ~(1 <<25); // set 10 bit ADC
  
    ADC1->CR2 &= ~(1<<0); // ADC Disable
    ADC1->SQR3 |= (pinValue<<0); // Enable ADC 0th channel
    ADC1->CR2 |= (1<<0); // ADC ON
  
  }
  
  void ADC_conversion(void){
    ADC1->CR2 |= (1<<30); // start ADC conversion
  
  }
  
  int ADC_read(void){
    while(!(ADC1->SR & (1<<1))){
      //wait for conversion
    }
    return (ADC1->DR);
  }
  