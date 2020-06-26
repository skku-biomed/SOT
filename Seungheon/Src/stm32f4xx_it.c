/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stm32_hal_legacy.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal.h" //Delay를 넣으며 추가한 것으로 딜레이를 사용하지 않으면 없어져도 됩니다.



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

int mux_switch = 1;
uint16_t InjectedADC[6];
uint16_t testADC[16];
uint8_t DevADC[2][6];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */


extern ADC_HandleTypeDef hadc1;
extern  ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern UART_HandleTypeDef huart3;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  HAL_GPIO_WritePin(GPIOB, PB12_Pin, GPIO_PIN_SET);  // 시간간격측정

        
  //mux switch와 관련된 조건들을 간소화해보았습니다. 제가 확인하긴 했지만 확인해주시면 감사하겠습니다.
  
 for(mux_switch = 1 ; mux_switch < 14; mux_switch++)
  {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
        
      
     

  switch(mux_switch)
  {          
          
  case 1: 
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
             
  case 2:  
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);      

  case 3:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);    

  case 4:
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);     

  case 5:     
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);  
   
  case 6: 
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_12, GPIO_PIN_SET);      
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
   
  case 7:
     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_SET);
     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  case 8:      
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  case 9: 
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);      
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
  
  case 10:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_13, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  case 11:      
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11 | GPIO_PIN_13, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
            
  case 12:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_13, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  case 13:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
      
      //14,15,16 안쓰신다고 하셔서 주석처리하고 for 문의 반복조건도 수정했습니다.
/*
  case 14:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  case 15:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
      
  case 16:
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
 
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
*/
    
 }

    

  
    
  
      
      //adc conversion 시작
  
      HAL_ADCEx_InjectedStart(&hadc1);
      HAL_ADCEx_InjectedStart(&hadc2);
      HAL_ADCEx_InjectedStart(&hadc3);
      HAL_ADCEx_InjectedStart(&hadc1);
      HAL_ADCEx_InjectedStart(&hadc2);
      HAL_ADCEx_InjectedStart(&hadc3);

         if(__HAL_ADC_GET_FLAG(&hadc1,ADC_FLAG_JSTRT)&&__HAL_ADC_GET_FLAG(&hadc1,ADC_FLAG_JEOC))
        {
          
            testADC[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            InjectedADC[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
                     
            DevADC[0][0]=testADC[0]>>8;
            DevADC[1][0]=testADC[0];
            DevADC[0][1]=InjectedADC[1]>>8;
            DevADC[1][1]=InjectedADC[1];
            
        }
    
       if(__HAL_ADC_GET_FLAG(&hadc2,ADC_FLAG_JSTRT)&&__HAL_ADC_GET_FLAG(&hadc2,ADC_FLAG_JEOC))
        {
            InjectedADC[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            InjectedADC[3] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
                      
            DevADC[0][2]=InjectedADC[2]>>8;
            DevADC[1][2]=InjectedADC[2];
            DevADC[0][3]=InjectedADC[3]>>8;
            DevADC[1][3]=InjectedADC[3];
           
        }

        if(__HAL_ADC_GET_FLAG(&hadc3,ADC_FLAG_JSTRT)&&__HAL_ADC_GET_FLAG(&hadc3,ADC_FLAG_JEOC))
        {
            InjectedADC[4] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
            InjectedADC[5] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);   
            
            DevADC[0][4]=InjectedADC[4]>>8;
            DevADC[1][4]=InjectedADC[4];
            DevADC[0][5]=InjectedADC[5]>>8;
            DevADC[1][5]=InjectedADC[5];

        }    
        
        
        //------------------------------------------------------
        //uart 통신 시작


       HAL_UART_Transmit(&huart3, &DevADC[1][0], 1, 1);
       HAL_UART_Transmit(&huart3, &DevADC[1][0], 1, 1);
        
       HAL_UART_Transmit(&huart3, &DevADC[0][1], 1, 1);
       HAL_UART_Transmit(&huart3, &DevADC[1][1], 1, 1);
        
       HAL_UART_Transmit(&huart3, &DevADC[0][2], 1, 1);
       HAL_UART_Transmit(&huart3, &DevADC[1][2], 1, 1);
        
       HAL_UART_Transmit(&huart3, &DevADC[0][3], 1, 1);
       HAL_UART_Transmit(&huart3, &DevADC[1][3], 1, 1);
        
       HAL_UART_Transmit(&huart3, &DevADC[0][4], 1, 1);
       HAL_UART_Transmit(&huart3, &DevADC[1][4], 1, 1);
        
       HAL_UART_Transmit(&huart3, &DevADC[0][5], 1, 1);
       HAL_UART_Transmit(&huart3, &DevADC[1][5], 1, 1);


        

       
        
        //case 1 이후의 명령을 복구함
       if(mux_switch == 1){  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); }
        
        
       
}
      
        
  //HAL_ADCEx_InjectedStart(&hadc1);
  //HAL_ADCEx_InjectedStart(&hadc2);
  //HAL_ADCEx_InjectedStart(&hadc3);
  
  /*if(__HAL_ADC_GET_FLAG(&hadc1,ADC_FLAG_JSTRT)&&__HAL_ADC_GET_FLAG(&hadc1,ADC_FLAG_JEOC)&&
  __HAL_ADC_GET_FLAG(&hadc2,ADC_FLAG_JSTRT)&&__HAL_ADC_GET_FLAG(&hadc2,ADC_FLAG_JEOC)&&
  __HAL_ADC_GET_FLAG(&hadc3,ADC_FLAG_JSTRT)&&__HAL_ADC_GET_FLAG(&hadc3,ADC_FLAG_JEOC)){
    {        
            InjectedADC[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            InjectedADC[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
            InjectedADC[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            InjectedADC[3] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
            InjectedADC[4] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
            InjectedADC[5] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);            
            
    } 
  
  }
      __NOP();
  // 넣으면 5ms마다 한번, 안 넣으면 10ms마다 한번
  */

 // HAL_ADCEx_InjectedStart(&hadc1);
 // HAL_ADCEx_InjectedStart(&hadc2);
 // HAL_ADCEx_InjectedStart(&hadc3);

  

        /* if(__HAL_ADC_GET_FLAG(&hadc1,ADC_FLAG_JSTRT)&&__HAL_ADC_GET_FLAG(&hadc1,ADC_FLAG_JEOC))
        {
          
            InjectedADC[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            InjectedADC[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
            printf("1) %u  ", InjectedADC[0]);
            printf("%u\n", InjectedADC[1]);
        }
    
       if(__HAL_ADC_GET_FLAG(&hadc2,ADC_FLAG_JSTRT)&&__HAL_ADC_GET_FLAG(&hadc2,ADC_FLAG_JEOC))
        {
            InjectedADC[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
            InjectedADC[3] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
            //printf("2) %u  ", InjectedADC[2]);
            //printf("%u\t", InjectedADC[3]);
        }

        if(__HAL_ADC_GET_FLAG(&hadc3,ADC_FLAG_JSTRT)&&__HAL_ADC_GET_FLAG(&hadc3,ADC_FLAG_JEOC))
        {
            InjectedADC[4] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
            InjectedADC[5] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);          
            //printf("3) %u  ", InjectedADC[4]);
            //printf("%u\n", InjectedADC[5]);

        }       

*/
  
        
        HAL_GPIO_WritePin(GPIOB, PB12_Pin, GPIO_PIN_RESET); //시간간격 측정  총 2.6ms 소요

        
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */






/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
