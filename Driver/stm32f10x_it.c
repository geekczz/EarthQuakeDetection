/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "GLOBAL.h"
#include "math.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	bspdata.systime++;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
uint32_t Counter_I = 0;
uint32_t COunter_J = 0;
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		
		ADXL313_Get3AxisData();
		mainData.MixAccData = sqrtf((adxl313_data.Acc_AxisX * adxl313_data.Acc_AxisX) + (adxl313_data.Acc_AxisY * adxl313_data.Acc_AxisY));
		
		if(mainData.MixAccData >= mainData.MixThresholdValue)
		{
			mainData.AccFlag = 1;
			if(mainData.BuzzerFlag == 1)
			{
				if(mainData.AccFirst == 0) //保证加速度计只报警一次
				{
					mainData.AccFirst = 1;
					CHARGER_OFF; //关断负载开关
					
					for(Counter_I = 0;Counter_I<1000;Counter_I++)
					{
						__nop();
					}
					RELAY_ON;
					
					for(Counter_I = 0;Counter_I<3000;Counter_I++)
					{
						for(COunter_J = 0;COunter_J<1500;COunter_J++)
						{
							__nop();
						}
					}
					
					//关闭电磁铁
					RELAY_OFF;
					for(Counter_I = 0;Counter_I<1000;Counter_I++)
					{
						__nop();
					}
					CHARGER_ON;
				}
			}
		}
  }
}

void EXTI0_IRQHandler(void)
{
	uint8_t Datatemp = 0;
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
		Datatemp = ADXL313_GetRegData(REG_INT_SOURCE);
		
		ADXL313_Get3AxisData();
    mainData.AccFlag = 1;
		
		if(mainData.BuzzerFlag == 1)
		{
			if(mainData.AccFirst == 0) //保证加速度计只报警一次
			{
				mainData.AccFirst = 1;
				CHARGER_OFF; //关断负载开关
				for(Counter_I = 0;Counter_I<1000;Counter_I++)
				{
					__nop();
				}
				//打开电磁铁
				RELAY_ON;
				
				for(Counter_I = 0;Counter_I<3000;Counter_I++)
				{
					for(COunter_J = 0;COunter_J<1000;COunter_J++)
					{
						__nop();
					}
				}
				
				//关闭电磁铁
				RELAY_OFF;
				for(Counter_I = 0;Counter_I<1000;Counter_I++)
				{
					__nop();
				}
				CHARGER_ON;
			}
		}
	

    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	uint8_t Datatemp = 0;
  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
		Datatemp = ADXL313_GetRegData(REG_INT_SOURCE);
		//需要编写策略处理在函数体内多次触发的问题，否则中断后维持高电平，无法连续触发
		ADXL313_Get3AxisData();
    /* Clear the  EXTI line 9 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line7);
  }
}
//void TIM1_UP_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
//	{
//		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
//		
//		
//	}
//}
/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
