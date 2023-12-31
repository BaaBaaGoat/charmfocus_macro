/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 通信协议
typedef struct{
	uint32_t preamble:24;
	uint32_t type:8;
	uint32_t Timestamp;
	uint32_t KeyStat;
} KeyMessage_t;//当有按键被按下时,下位机报给上位机

typedef struct{
	uint32_t preamble:24;
	uint32_t type:8;
	uint32_t Timestamp;
	uint32_t KeyStat;
} KeyOperation_t;//上位机发给下位机要求按下指定的按键
typedef enum{
	OP_PRESS,
	OP_WAIT,
	OP_JUMP,
} Operator_t;
typedef __packed struct {
	Operator_t Operator;
	int16_t Operand;
} Command_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAGICNUMBER 0x16DB2E
#define FRAME_KEYMESSAGE   0x01
#define FRAME_KEYOPERATION 0x02
#define FRAME_KEYPROGRAM   0x03
// 按键扫描码表
// 游戏机按键 可读可写
#define SCANCODE_TOUCH 0x00000001
#define SCANCODE_SQUARE 0x00000002
#define SCANCODE_X 0x00000004
#define SCANCODE_CIRCLE 0x00000008
#define SCANCODE_TRIANGLE 0x00000010
#define SCANCODE_UP 0x00000020
#define SCANCODE_DOWN 0x00000040
#define SCANCODE_LEFT 0x00000080
#define SCANCODE_RIGHT 0x00000100
#define SCANCODE_L1 0x00000200
#define SCANCODE_R1 0x00000400
#define SCANCODE_L2 0x00000800
#define SCANCODE_R2 0x00001000
//#define SCANCODE_L3 0x00002000
//#define SCANCODE_R3 0x00004000
// 背键 只读
#define SCANCODE_BACKKEY1 0x00010000
#define SCANCODE_BACKKEY2 0x00020000
#define SCANCODE_BACKKEY3 0x00040000
#define SCANCODE_BACKKEY4 0x00080000
#define SCANCODE_BACKKEYLEFT 0x00100000
#define SCANCODE_BACKKEYRIGHT 0x00200000


#define SCANCODE_ALL  0x003F07FF
#define SCANCODE_GAMEPAD  0x000007FF
#define SCANCODE_BACKKEY  0x003F0000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
const struct{
	Command_t prog_BACKKEY1[1024];
	Command_t prog_BACKKEY2[1024];
	Command_t prog_BACKKEY3[1024];
	Command_t prog_BACKKEY4[1024];
//	Command_t prog_BACKKEYLEFT[1024];
//	Command_t prog_BACKKEYRIGHT[1024];
} ProgramSet = {
	.prog_BACKKEY1 = {
		{OP_PRESS,SCANCODE_SQUARE},
		{OP_WAIT,41},
		{OP_PRESS,SCANCODE_CIRCLE},
		{OP_WAIT,19},
		{OP_PRESS,0},
		{OP_WAIT,38},
		{OP_JUMP,5}
	},
	.prog_BACKKEY2 = {
		{OP_PRESS,SCANCODE_R2},
		{OP_JUMP,2}
	},
	.prog_BACKKEY3 = {
		{OP_JUMP,1}
	},
	.prog_BACKKEY4 = {
		{OP_PRESS,SCANCODE_L2},
		{OP_JUMP,2}
	},
//	.prog_BACKKEYLEFT = {
//		{OP_JUMP,1}
//	},
//	.prog_BACKKEYRIGHT = {
//		{OP_JUMP,1}
//	},
};
// TODO 做从电脑download脚本的接口
//TODO 做从电脑下发按键的接口
uint32_t KeyStat,KeyPressed,KeyReleased;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t ReadKeyStatus(){
	uint32_t stat=0;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_TOUCH_GPIO_Port,GPIO_GAMEPAD_TOUCH_Pin))      stat |=  SCANCODE_TOUCH;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_R1_GPIO_Port,GPIO_GAMEPAD_R1_Pin))            stat |=  SCANCODE_R1;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_UP_GPIO_Port,GPIO_GAMEPAD_UP_Pin))            stat |=  SCANCODE_UP;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_LEFT_GPIO_Port,GPIO_GAMEPAD_LEFT_Pin))        stat |=  SCANCODE_LEFT;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_DOWN_GPIO_Port,GPIO_GAMEPAD_DOWN_Pin))        stat |=  SCANCODE_DOWN;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_RIGHT_GPIO_Port,GPIO_GAMEPAD_RIGHT_Pin))      stat |=  SCANCODE_RIGHT;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_TRIANGLE_GPIO_Port,GPIO_GAMEPAD_TRIANGLE_Pin))stat |=  SCANCODE_TRIANGLE;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_SQUARE_GPIO_Port,GPIO_GAMEPAD_SQUARE_Pin))    stat |=  SCANCODE_SQUARE;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_X_GPIO_Port,GPIO_GAMEPAD_X_Pin))              stat |=  SCANCODE_X;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_CIRCLE_GPIO_Port,GPIO_GAMEPAD_CIRCLE_Pin))    stat |=  SCANCODE_CIRCLE;
	if(!HAL_GPIO_ReadPin(GPIO_GAMEPAD_L1_GPIO_Port,GPIO_GAMEPAD_L1_Pin))            stat |=  SCANCODE_L1;
	   
	if(!HAL_GPIO_ReadPin(BACKKEY1_GPIO_Port,BACKKEY1_Pin))            stat |=  SCANCODE_BACKKEY1;
	if(!HAL_GPIO_ReadPin(BACKKEY2_GPIO_Port,BACKKEY2_Pin))            stat |=  SCANCODE_BACKKEY2;
	if(!HAL_GPIO_ReadPin(BACKKEY3_GPIO_Port,BACKKEY3_Pin))            stat |=  SCANCODE_BACKKEY3;
	if(!HAL_GPIO_ReadPin(BACKKEY4_GPIO_Port,BACKKEY4_Pin))            stat |=  SCANCODE_BACKKEY4;
	if(!HAL_GPIO_ReadPin(BACKKEY_LEFT_GPIO_Port,BACKKEY_LEFT_Pin))    stat |=  SCANCODE_BACKKEYLEFT;
	if(!HAL_GPIO_ReadPin(BACKKEY_RIGHT_GPIO_Port,BACKKEY_RIGHT_Pin))  stat |=  SCANCODE_BACKKEYRIGHT;
	
	uint16_t adcvalue[4];
	HAL_ADC_Start_DMA(&hadc1, (void*)adcvalue, 4);
	HAL_ADC_PollForConversion(&hadc1,10);
	if(adcvalue[0] < 0x0200)  stat |=  SCANCODE_R2;
	if(adcvalue[1] < 0x0200)  stat |=  SCANCODE_L2;
	return stat;
	
}
uint32_t ReadDejitteredKeyStat(){
	static uint32_t newKeyStat[2];
	newKeyStat[1] = newKeyStat[0];
	newKeyStat[0] = ReadKeyStatus();
	// 如果连续2次读取均稳定,则更新状态;否则不更新
	uint32_t newKeyStat_hi=newKeyStat[1] & newKeyStat[0];
	KeyPressed = (~KeyStat) & newKeyStat_hi & SCANCODE_ALL;
	KeyStat|=newKeyStat_hi;
	uint32_t newKeyStat_lo=~(newKeyStat[1] | newKeyStat[0]);
	KeyReleased = (KeyStat) & newKeyStat_lo &SCANCODE_ALL;
	KeyStat&=(~newKeyStat_lo);
	KeyStat&=SCANCODE_ALL;
	return KeyStat;
}
void WriteKeyStat(uint32_t KeyCode){
	HAL_GPIO_WritePin(GPIO_GAMEPAD_TOUCH_GPIO_Port,GPIO_GAMEPAD_TOUCH_Pin,(KeyCode & SCANCODE_TOUCH)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_R1_GPIO_Port,GPIO_GAMEPAD_R1_Pin,(KeyCode & SCANCODE_R1)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_UP_GPIO_Port,GPIO_GAMEPAD_UP_Pin,(KeyCode & SCANCODE_UP)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_LEFT_GPIO_Port,GPIO_GAMEPAD_LEFT_Pin,(KeyCode & SCANCODE_LEFT)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_DOWN_GPIO_Port,GPIO_GAMEPAD_DOWN_Pin,(KeyCode & SCANCODE_DOWN)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_RIGHT_GPIO_Port,GPIO_GAMEPAD_RIGHT_Pin,(KeyCode & SCANCODE_RIGHT)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_TRIANGLE_GPIO_Port,GPIO_GAMEPAD_TRIANGLE_Pin,(KeyCode & SCANCODE_TRIANGLE)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_SQUARE_GPIO_Port,GPIO_GAMEPAD_SQUARE_Pin,(KeyCode & SCANCODE_SQUARE)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_X_GPIO_Port,GPIO_GAMEPAD_X_Pin,(KeyCode & SCANCODE_X)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_CIRCLE_GPIO_Port,GPIO_GAMEPAD_CIRCLE_Pin,(KeyCode & SCANCODE_CIRCLE)?0:1);
	HAL_GPIO_WritePin(GPIO_GAMEPAD_L1_GPIO_Port,GPIO_GAMEPAD_L1_Pin,(KeyCode & SCANCODE_L1)?0:1);
	HAL_GPIO_WritePin(ANALOG_GAMEPAD_L2_GPIO_Port,ANALOG_GAMEPAD_L2_Pin,(KeyCode & SCANCODE_L2)?0:1);
	HAL_GPIO_WritePin(ANALOG_GAMEPAD_R2_GPIO_Port,ANALOG_GAMEPAD_R2_Pin,(KeyCode & SCANCODE_R2)?0:1);
}
const int FrameInterval = 10;// 10ms按键扫描间隔




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ANALOG_GAMEPAD_R2_Pin|ANALOG_GAMEPAD_L2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_WritePin(ANALOG_GAMEPAD_R2_GPIO_Port,ANALOG_GAMEPAD_R2_Pin,1);
	HAL_GPIO_WritePin(ANALOG_GAMEPAD_R2_GPIO_Port,ANALOG_GAMEPAD_L2_Pin,1);
	HAL_GPIO_Init(ANALOG_GAMEPAD_R2_GPIO_Port, &GPIO_InitStruct);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 static uint8_t DEBUGDATA[200];
 uint32_t t=HAL_GetTick();
 enum{
	 STAT_IDLE,
	 STAT_BACKKEY1,
	 STAT_BACKKEY2,
	 STAT_BACKKEY3,
	 STAT_BACKKEY4,
	 STAT_BACKKEY_LEFT,
	 STAT_BACKKEY_RIGHT
 }status;
 uint16_t PC,A,B;
 
 
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(HAL_GetTick() > FrameInterval + t){
			t=HAL_GetTick();
			uint32_t stat = ReadDejitteredKeyStat();
			//如果有操作 发送消息给上位机
			if(KeyPressed|KeyReleased){
				KeyMessage_t msg = {MAGICNUMBER,FRAME_KEYMESSAGE,HAL_GetTick(),stat};
				CDC_Transmit_FS((void*)&msg,sizeof(msg));
			}
	
			// 按键状态机
			switch(status){
				case STAT_IDLE:{
					if(KeyPressed & SCANCODE_BACKKEY1){
						A=B=PC=0;
						status = STAT_BACKKEY1;
					}
					if(KeyPressed & SCANCODE_BACKKEY2){
						A=B=PC=0;
						status = STAT_BACKKEY2;
					}
					if(KeyPressed & SCANCODE_BACKKEY3){
						A=B=PC=0;
						status = STAT_BACKKEY3;
					}
					if(KeyPressed & SCANCODE_BACKKEY4){
						A=B=PC=0;
						status = STAT_BACKKEY4;
					}
				}break;
				case STAT_BACKKEY1:{
					if(KeyReleased & SCANCODE_BACKKEY1){
						WriteKeyStat(0);
						status = STAT_IDLE;
					}else {
						Command_t cmd = ProgramSet.prog_BACKKEY1[PC];
						switch(cmd.Operator){
							case OP_PRESS:WriteKeyStat(cmd.Operand);break;
							case OP_WAIT:PC--;B++;if(B>=cmd.Operand){PC++;B=0;}break;
							case OP_JUMP:PC-=cmd.Operand;break;
						}
					}
					PC++;
				}break;
				case STAT_BACKKEY2:{
					if(KeyReleased & SCANCODE_BACKKEY2){
						WriteKeyStat(0);
						status = STAT_IDLE;
					}else {
						Command_t cmd = ProgramSet.prog_BACKKEY2[PC];
						switch(cmd.Operator){
							case OP_PRESS:WriteKeyStat(cmd.Operand);break;
							case OP_WAIT:PC--;B++;if(B>=cmd.Operand){PC++;B=0;}break;
							case OP_JUMP:PC-=cmd.Operand;break;
						}
					}
					PC++;
				}break;
				case STAT_BACKKEY3:{
					if(KeyReleased & SCANCODE_BACKKEY3){
						WriteKeyStat(0);
						status = STAT_IDLE;
					}else {
						Command_t cmd = ProgramSet.prog_BACKKEY3[PC];
						switch(cmd.Operator){
							case OP_PRESS:WriteKeyStat(cmd.Operand);break;
							case OP_WAIT:PC--;B++;if(B>=cmd.Operand){PC++;B=0;}break;
							case OP_JUMP:PC-=cmd.Operand;break;
						}
					}
					PC++;
				}break;
				case STAT_BACKKEY4:{
					if(KeyReleased & SCANCODE_BACKKEY4){
						WriteKeyStat(0);
						status = STAT_IDLE;
					}else {
						Command_t cmd = ProgramSet.prog_BACKKEY4[PC];
						switch(cmd.Operator){
							case OP_PRESS:WriteKeyStat(cmd.Operand);break;
							case OP_WAIT:PC--;B++;if(B>=cmd.Operand){PC++;B=0;}break;
							case OP_JUMP:PC-=cmd.Operand;break;
						}
					}
					PC++;
				}break;
				default:status = STAT_IDLE;break;
			}
		}
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_GAMEPAD_TOUCH_Pin|GPIO_GAMEPAD_R1_Pin|GPIO_GAMEPAD_UP_Pin|GPIO_GAMEPAD_LEFT_Pin
                          |GPIO_GAMEPAD_DOWN_Pin|GPIO_GAMEPAD_RIGHT_Pin|GPIO_GAMEPAD_SQUARE_Pin|GPIO_GAMEPAD_X_Pin
                          |GPIO_GAMEPAD_CIRCLE_Pin|GPIO_GAMEPAD_L1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_GAMEPAD_TRIANGLE_GPIO_Port, GPIO_GAMEPAD_TRIANGLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_A_PLUS_B_Pin|LED_FIST_Pin|LED_BULLET_Pin|LED_BULLET_A_Pin
                          |LED_USB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_GAMEPAD_TOUCH_Pin GPIO_GAMEPAD_R1_Pin GPIO_GAMEPAD_UP_Pin GPIO_GAMEPAD_LEFT_Pin
                           GPIO_GAMEPAD_DOWN_Pin GPIO_GAMEPAD_RIGHT_Pin GPIO_GAMEPAD_SQUARE_Pin GPIO_GAMEPAD_X_Pin
                           GPIO_GAMEPAD_CIRCLE_Pin GPIO_GAMEPAD_L1_Pin */
  GPIO_InitStruct.Pin = GPIO_GAMEPAD_TOUCH_Pin|GPIO_GAMEPAD_R1_Pin|GPIO_GAMEPAD_UP_Pin|GPIO_GAMEPAD_LEFT_Pin
                          |GPIO_GAMEPAD_DOWN_Pin|GPIO_GAMEPAD_RIGHT_Pin|GPIO_GAMEPAD_SQUARE_Pin|GPIO_GAMEPAD_X_Pin
                          |GPIO_GAMEPAD_CIRCLE_Pin|GPIO_GAMEPAD_L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_GAMEPAD_TRIANGLE_Pin */
  GPIO_InitStruct.Pin = GPIO_GAMEPAD_TRIANGLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_GAMEPAD_TRIANGLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_A_PLUS_B_Pin LED_FIST_Pin LED_BULLET_Pin LED_BULLET_A_Pin
                           LED_USB_Pin */
  GPIO_InitStruct.Pin = LED_A_PLUS_B_Pin|LED_FIST_Pin|LED_BULLET_Pin|LED_BULLET_A_Pin
                          |LED_USB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB3 BACKKEY1_Pin BACKKEY3_Pin
                           BACKKEY2_Pin BACKKEY4_Pin BACKKEY_LEFT_Pin BACKKEY_RIGHT_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_3|BACKKEY1_Pin|BACKKEY3_Pin
                          |BACKKEY2_Pin|BACKKEY4_Pin|BACKKEY_LEFT_Pin|BACKKEY_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
