

/* Includes ------------------------------------------------------------------*/
#include "system_define.h"
/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
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
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
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
}

/**
  * @brief  This function handles External lines 4 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {  
    /* Clear the EXTI line 11 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line11);
  } 
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    /* Clear the EXTI line 12 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line12);							//Reset Angle
  }
}
/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	if((TIM_GetFlagStatus(TIM3,TIM_FLAG_Update)) != RESET )
	{
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
	}
}

void TIM2_IRQHandler(void)
{
	/* Clear TIM2 Capture compare interrupt pending bit */
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* Clear TIM1 COM pending bit */
	if((TIM_GetFlagStatus(TIM1,TIM_FLAG_Update)) != RESET )
	{
		TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);	
	}

	if((TIM_GetFlagStatus(TIM1,TIM_FLAG_Break)) != RESET )
	{
		TIM_ClearITPendingBit(TIM1, TIM_FLAG_Break);
		Close_PWM();		
		TIM_CtrlPWMOutputs(TIM1, DISABLE);
		ShortI_Flag=1;
		Motor_State=WAIT;		
	}
}

void DMA1_Channel1_IRQHandler(void) 
{
		/* Test DMA1 TC flag */
	if((DMA_GetFlagStatus(DMA1_FLAG_TC1)) != RESET ) 
	{
		/* Clear DMA TC flag */
		DMA_ClearFlag(DMA1_FLAG_TC1);
			
		if(Motor_State==INIT)
		{
			ADCTemp_Init(ADC_Tab);
		}
		else
		{			
			LED1_ON();
			SensorlessFOCRUN();
			LED1_OFF();
		}
		SVM_Angle=EstimParm.qRho;
		//DAC_SetChannel1Data(DAC_Align_12b_R,SVM_Angle/16);			
		
		if(++T2ms_Temp>T2MSTEMP)			//2ms
		{	
			T2ms_Temp=0;
			T2ms_Flag=1;
		}else{}
		
		if(++T100ms_Temp>T100MSTEMP)	//100ms 
		{				
			T100ms_Temp=0;
			T100ms_Flag=1;
		}else{}
	}
}
/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
		if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
		{
			USART_ClearFlag(USART1, USART_IT_RXNE);			
    	/* Read one byte from the receive data register */
			UART_FromPc();
  	}

  	if(USART_GetITStatus(USART1, USART_IT_TC) == SET)
  	{       
    	/* Clear the USART1 transmit interrupt */
			USART_ClearFlag(USART1, USART_IT_TC);
  	} 
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
