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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/

extern DMA_HandleTypeDef hdma_sdio;
extern SD_HandleTypeDef hsd;
extern TIM_HandleTypeDef htim6;


/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  
}

/**
  * @brief This function handles Hard fault interrupt.
  */
/**
* @brief This function handles Hard fault interrupt.
*/
void hard_fault_handler_c (unsigned int * hardfault_args)
{
unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;
 
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);
 
  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);
  
  printf ("\n\n[Hard fault handler - all numbers in hex]\r\n");
  printf ("R0 = %x\r\n", stacked_r0);
  printf ("R1 = %x\r\n", stacked_r1);
  printf ("R2 = %x\r\n", stacked_r2);
  printf ("R3 = %x\r\n", stacked_r3);
  printf ("R12 = %x\r\n", stacked_r12);
  printf ("LR [R14] = %x  subroutine call return address\r\n", stacked_lr);
  printf ("PC [R15] = %x  program counter\r\n", stacked_pc);
  printf ("PSR = %x\r\n", stacked_psr);
  printf ("BFAR = %x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
  printf ("CFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
  printf ("HFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
  printf ("DFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
  printf ("AFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
  printf ("SCB_SHCSR = %x\r\n", SCB->SHCSR);
  
  while (1);
}

//intial save fault handler then branch to  hard_fault_handler_c to dump to uart 1 
void HardFault_Handler(void)
{
	
	__asm volatile (
		"tst LR, #4       \n"
		"ite EQ       \n"
		"mrseq R0,MSP       \n"
		"mrsne R0, PSP       \n"
		"b hard_fault_handler_c \n"
		);
	
  __asm volatile (
    " movs r0,#4       \n"
    " movs r1, lr      \n"
    " tst r0, r1       \n"
    " beq _MSP         \n"
    " mrs r0, psp      \n"
    " b _HALT          \n"
  "_MSP:               \n"
    " mrs r0, msp      \n"
  "_HALT:              \n"
    " ldr r1,[r0,#20]  \n"
    " bkpt #0          \n"
  );
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{

  while (1)
  {
 
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  
  while (1)
  {
  
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{

  while (1)
  {
  
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
 
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  
 // HAL_IncTick();
 HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_sdio);

}

void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}


void SDIO_IRQHandler(void)
{
  /* USER CODE BEGIN SDIO_IRQn 0 */

  /* USER CODE END SDIO_IRQn 0 */
  HAL_SD_IRQHandler(&hsd);
  /* USER CODE BEGIN SDIO_IRQn 1 */

  /* USER CODE END SDIO_IRQn 1 */
}