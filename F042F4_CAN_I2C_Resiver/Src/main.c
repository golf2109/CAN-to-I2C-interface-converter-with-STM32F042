/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t xBuffer[3],rrr,www1,www2,send;
uint8_t UART_Buffer[3]={0x31,0x32,0x0D};
uint8_t DEVADDR;

uint32_t CAN_ReceiveMessage1 = 0;
uint32_t CAN_ReceiveMessage2 = 0;

typedef struct  {
  unsigned int   id;                    /* 29 bit identifier */
  unsigned char  data[8];               /* Data field */
  unsigned char  len;                   /* Length of data field in bytes */
  unsigned char  format;                /* 0 - STANDARD, 1- EXTENDED IDENTIFIER */
  unsigned char  type;                  /* 0 - DATA FRAME, 1 - REMOTE FRAME */
} CAN_msg;

CAN_msg       CAN_RxMsg;                      /* CAN message for receiving */                                

uint32_t      CAN_RxRdy = 0;              /* CAN HW received a message */

static uint32_t CAN_filterIdx[2] = {0,0};        /* static variable for the filter index */

uint32_t      CAN_msgId     = 0;

#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define DATA_FRAME       0
#define REMOTE_FRAME     1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//==============================================================================
void CAN_wrFilter (uint32_t ctrl, uint32_t id, uint8_t format)  {
  
  if (CAN_filterIdx[ctrl-1] > 13) {                 /* check if Filter Memory is full*/
    return;
  }
                                            /* Setup identifier information  */
  if (format == STANDARD_FORMAT)  {         /*   Standard ID                 */
      CAN_msgId |= (uint32_t)(id << 21) | CAN_ID_STD;
  }  else  {                                /*   Extended ID                 */
      CAN_msgId |= (uint32_t)(id <<  3) | CAN_ID_EXT;
  }

  CAN->FMR  |=   CAN_FMR_FINIT;            /* set initMode for filter banks */
  CAN->FA1R &=  ~(1UL << CAN_filterIdx[ctrl-1]);   /* deactivate filter             */

                                            /* initialize filter             */
  CAN->FS1R |= (uint32_t)(1 << CAN_filterIdx[ctrl-1]);     /* set 32-bit scale configuration    */
	
	/*! !* To disable the CAN Filters: Swap the comment on the next two lines and the two lines after the next line. */
	CAN->FM1R |= (uint32_t)(1 << CAN_filterIdx[ctrl-1]);   /* set to 32-bit Identifier List mode */
	//CAN->FM1R |= 0x0;    							/* DISABLES FILTERS  set to 32-bit Identifier Mask mode */
	
  CAN->sFilterRegister[CAN_filterIdx[ctrl-1]].FR1 = 0; //CAN_msgId; /*  32-bit identifier   */
	
	/* !!*  To disable the CAN Filters: Swap the comment on the next two lines.    */
  CAN->sFilterRegister[CAN_filterIdx[ctrl-1]].FR2 = CAN_msgId; 	/*  32-bit identifier (33) for identifier mode  */
	//CAN->sFilterRegister[CAN_filterIdx[ctrl-1]].FR2 = 0; 	/*  DISABLES FILTERS  32-bit MASK for mask mode */
   
   
  CAN->FFA1R &= ~(uint32_t)(1 << CAN_filterIdx[ctrl-1]);   /* assign filter to FIFO 0  */
  CAN->FA1R  |=  (uint32_t)(1 << CAN_filterIdx[ctrl-1]);   /* activate filter          */
	
  CAN->FMR &= ~CAN_FMR_FINIT;              /* reset initMode for filterBanks*/
	
	CAN_filterIdx[ctrl-1]++;                       /* increase filter index  */
}

//===========================================================================================
void CAN_rdMsg (uint32_t ctrl, CAN_msg *msg)  {
                                              /* Read identifier information  */
  if ((CAN->sFIFOMailBox[0].RIR & CAN_ID_EXT) == 0) {
    msg->format = STANDARD_FORMAT;
    msg->id     = 0x000007FF & (CAN->sFIFOMailBox[0].RIR >> 21);
  } else {
    msg->format = EXTENDED_FORMAT;
    msg->id     = 0x1FFFFFFF & (CAN->sFIFOMailBox[0].RIR >> 3);
  }
                                              /* Read type information        */
  if ((CAN->sFIFOMailBox[0].RIR & CAN_RTR_REMOTE) == 0) {
    msg->type =   DATA_FRAME;
  } else {
    msg->type = REMOTE_FRAME;
  }
                                              /* Read number of rec. bytes    */
  msg->len     = (CAN->sFIFOMailBox[0].RDTR      ) & 0x0F;
                                              /* Read data bytes              */
  msg->data[0] = (CAN->sFIFOMailBox[0].RDLR      ) & 0xFF;
  msg->data[1] = (CAN->sFIFOMailBox[0].RDLR >>  8) & 0xFF;
  msg->data[2] = (CAN->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
  msg->data[3] = (CAN->sFIFOMailBox[0].RDLR >> 24) & 0xFF;

  msg->data[4] = (CAN->sFIFOMailBox[0].RDHR      ) & 0xFF;
  msg->data[5] = (CAN->sFIFOMailBox[0].RDHR >>  8) & 0xFF;
  msg->data[6] = (CAN->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
  msg->data[7] = (CAN->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

  CAN->RF0R |= CAN_RF0R_RFOM0;             /* Release FIFO 0 output mailbox */
}
//=============================================================================
void read_CAN(void)
{
CAN_rdMsg (1, &CAN_RxMsg);
}	

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
 CAN->IER |= CAN_IER_FMPIE0;								/* enable FIFO 0 msg pending IRQ    */ 
 CAN_wrFilter (1, 199, STANDARD_FORMAT);    /* filter reception of msgs */                            
 CAN->MCR &= ~CAN_MCR_INRQ;             		/* normal operating mode, reset INRQ*/
 while (CAN->MSR & CAN_MCR_INRQ);						/* start CAN Controller  */                       

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);		//on Power LED
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);			//off CAN LED

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
    if (CAN_RxRdy == 1)  																														// rx msg on CAN Ctrl 
			{                          
				CAN_RxRdy = 0;																															//clear CAM flag

				xBuffer[0] = 0x0;																														//write byte to potentiometr 0
				xBuffer[1] = CAN_ReceiveMessage1;							
				DEVADDR=0x50; 																															//asdress of digital potentiometr
				www1=HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) DEVADDR, xBuffer, 2, 10);  	//write to address  (2-number of bytes)      
				//!!!       www1 = 0 if Transmit OK
				xBuffer[0] = 0x10;																													//write byte to potentiometr 1				
				xBuffer[1] = CAN_ReceiveMessage2;							
				DEVADDR=0x50; 																															//asdress of digital potentiometr
				www2=HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) DEVADDR, xBuffer, 2, 10);  	//write to address  (2-number of bytes)	
				//!!!        www2 = 0 if Transmit OK				
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);																			//toggle CAN LED
			}		
//==========================================================================================================================
// 			TEST for digi pot
//==========================================================================================================================
//for (rrr=0;rrr<=255;rrr++)
//{
//			xBuffer[0] = 0x0;																													//write byte to potentiometr 0				
//			xBuffer[1] = rrr;							
//			DEVADDR=0x5e; 																														//asdress of digital potentiometr
//			www1=HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) DEVADDR, xBuffer, 2, 10);  //write to address  (2-number of bytes)																														//memory write delay	
//			xBuffer[0] = 0x10;																													//write byte to potentiometr 1				
//			xBuffer[1] = rrr;							
//			DEVADDR=0x5e; 																														//asdress of digital potentiometr
//			www2=HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) DEVADDR, xBuffer, 2, 10);  //write to address  (2-number of bytes)		
//HAL_Delay(200);  //delay for debug			
//}	
//===========================================================================================================================
	
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_4TQ;
  hcan.Init.BS1 = CAN_BS1_8TQ;
  hcan.Init.BS2 = CAN_BS2_3TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CANSTA_Pin|POWSTA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CANSTA_new_Pin|POSTA_new_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CANSTA_Pin POWSTA_Pin */
  GPIO_InitStruct.Pin = CANSTA_Pin|POWSTA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CANSTA_new_Pin POSTA_new_Pin */
  GPIO_InitStruct.Pin = CANSTA_new_Pin|POSTA_new_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
