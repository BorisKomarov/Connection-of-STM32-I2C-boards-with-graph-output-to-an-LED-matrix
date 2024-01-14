/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */
uint8_t datatest[16];
uint8_t fulldata[64];

volatile int msec = 0;
float time = 0.001;
float k = 1;
float psi = 0.10;
float w = 0;
float T = 0.9;
float e = 2.7;
float h = 0;
int counter = 0;
float Arrhk[500];
float Arrtk[500];
float hk=0;
float tk=0;
float x_scale = 0.1;
int x_scale_counter = 0;
float temp_h = 0;
int flag=0;
float step_h = 0.150;
float step_time = 0.3;
int counter_temp=0;;
float hktemp = 0;
uint8_t Arr_control[4][16];
uint8_t mask = 0b10000000;
int row=0;
int col = 0;
uint8_t data_00[16];
uint8_t data_01[16];
uint8_t data_10[16];
uint8_t data_11[16];
int A;
int B;
int AB_counter=0;
int Matrix_Transfer_Flag = 1;
uint8_t testdata[16] = {0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111,0b00001111};
uint8_t cleardata[16] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t testdata2[16] = {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10};
uint8_t testdata3[16] = {0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33};
uint8_t cleardat = 0x30;
float temp_hk;
float temp_tk;
uint8_t diodes_byte;
uint8_t shifted_bit;
int btn;
float pressed_flag;
float a;
float imp = 0;
float imp_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//  for (int i =0; i<16;i++){
//	  data_11[i] =0xff;
//	  data_10[i] =0xff;
//	  data_01[i] =0xff;
//	  data_00[i] =0xff;
//  }
//  w = sqrt(1-pow(psi,2))/T;
//  a = psi/T;
//  pressed_flag = 0;
//  imp_time = 0;
//  imp = 0;

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
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//	  btn = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
//	  	  	if(btn){
//	  	  		pressed_flag = 1;
//	  	  		imp_time = time;
//	  	  	}
//
//
//
//	  	  h = (k * (  1-pow(e,-psi*time/T) * (cos(w*time)+  (psi/sqrt(1-pow(psi,2)))*sin(w*time) )  ) )+pressed_flag*(k/1.7*((w+pow(a,2))/w)*sin(w*(time-imp_time))*pow(e,-a*(time-imp_time)));//расчёт значение переходной функции
//		  hk = round(h / step_h) * step_h; //квантование  значений функции
//		  tk = round(time / step_time) * step_time; //квантование  времени
//		  if(tk!= temp_tk || hk!= temp_hk){
//			  temp_tk = tk;
//			  temp_hk = hk;
//			  col = round(tk/step_time);
//			  row = round(hk/step_h);
//			  diodes_byte = ~(mask>>(col%8));
//			  if(col<=31){
//				  if(row%4==0){
//					  data_11[4*(col/8)+row/4]&=diodes_byte;
//				  }
//				  else if(row%4==1){
//					  data_01[4*(col/8)+row/4]&=diodes_byte;
//				  }
//				  else if(row%4==2){
//					  data_10[4*(col/8)+row/4]&=diodes_byte;
//				  }
//				  else if(row%4==3){
//					  data_00[4*(col/8)+row/4]&=diodes_byte;
//				  }
//			  }
//			  else{
//				  for(int i =0;i<12;i++){
//					  shifted_bit = (data_11[i+4] >> 7) & 1;
//					  data_11[i]<<=1;
//					  data_11[i]|=shifted_bit;
//				  }
//				  for(int i =0;i<12;i++){
//					  shifted_bit = (data_10[i+4] >> 7) & 1;
//					  data_10[i]<<=1;
//					  data_10[i]|=shifted_bit;
//				  }
//				  for(int i =0;i<12;i++){
//					  shifted_bit = (data_01[i+4] >> 7) & 1;
//					  data_01[i]<<=1;
//					  data_01[i]|=shifted_bit;
//				  }
//				  for(int i =0;i<12;i++){
//					  shifted_bit = (data_00[i+4] >> 7) & 1;
//					  data_00[i]<<=1;
//					  data_00[i]|=shifted_bit;
//				  }
//
//
//
//
//
//				  for(int i =0;i<3;i++){
//					  data_11[12+i]<<=1;
//					  data_11[12+i]|=0b00000001;
//				  }
//				  for(int i =0;i<3;i++){
//					  data_10[12+i]<<=1;
//					  data_10[12+i]|=0b00000001;
//				  }
//				  for(int i =0;i<3;i++){
//					  data_01[12+i]<<=1;
//					  data_01[12+i]|=0b00000001;
//				  }
//				  for(int i =0;i<3;i++){
//					  data_00[12+i]<<=1;
//					  data_00[12+i]|=0b00000001;
//				  }
//
//
//
//				  if(row%4==0){
//
//					  shifted_bit =((diodes_byte <<col%8)>>7) & 1;
//					  shifted_bit|=0b11111110;
//					  data_11[12+row/4]&=shifted_bit;
//				  }
//				  if(row%4==1){
//
//					  shifted_bit =((diodes_byte <<col%8)>>7) & 1;
//					  shifted_bit|=0b11111110;
//					  data_01[12+row/4]&=shifted_bit;
//				  }
//				  if(row%4==2){
//
//					  shifted_bit =((diodes_byte <<col%8)>>7) & 1;
//					  shifted_bit|=0b11111110;
//					  data_10[12+row/4]&=shifted_bit;
//				  }
//				  if(row%4==3){
//
//					  shifted_bit =((diodes_byte <<col%8)>>7) & 1;
//					  shifted_bit|=0b11111110;
//					  data_00[12+row/4]&=shifted_bit;
//				  }
//
//
//			  }
//
//
//
//		  }
//		  time+=0.01;


	  HAL_I2C_Master_Receive_IT(&hi2c2, 10<<1, &fulldata, 64);

	  for(int i = 0; i<16;i++){

		  data_00[i] = fulldata[i];
	  }
	  for(int i = 0; i<16;i++){
		  data_01[i] = fulldata[16+i];
	  }
	  for(int i = 0; i<16;i++){
		  data_10[i] = fulldata[32+i];
	  }
	  for(int i = 0; i<16;i++){
		  data_11[i] = fulldata[48+i];
	  }


	  //	//смена значений A и B
	      A = (AB_counter % 2 == 0) ? 1 : 0;
	      B = (AB_counter / 2 % 2 == 0) ? 1 : 0;
	      AB_counter = (AB_counter + 1) % 4;



	  HAL_GPIO_WritePin(NOE_GPIO_Port, NOE_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, A);
	  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, B);

      //Отправка данных в зависимости от уровней A и B
      if(A==1 && B==1){
      	HAL_SPI_Transmit(&hspi2, &data_11, 16,10);
    	  //HAL_SPI_Transmit(&hspi2, &datatest, 16,10);
      }
      if(A==0 && B==1){
      	HAL_SPI_Transmit(&hspi2, &data_01, 16,10);
    	  //HAL_SPI_Transmit(&hspi2, &datatest, 16,10);
      }
      if(A==1 && B==0){
      	HAL_SPI_Transmit(&hspi2, &data_10, 16,10);
    	  //HAL_SPI_Transmit(&hspi2, &datatest, 16,10);
      }
      if(A==0 && B==0){
      	HAL_SPI_Transmit(&hspi2, &data_00, 16,10);
    	  //HAL_SPI_Transmit(&hspi2, &datatest, 16,10);
      }

	  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(NOE_GPIO_Port, NOE_Pin, GPIO_PIN_SET);
	  HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SCLK_Pin|NOE_Pin|NOE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A_Pin */
  GPIO_InitStruct.Pin = A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B_Pin */
  GPIO_InitStruct.Pin = B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCLK_Pin NOE_Pin NOE1_Pin */
  GPIO_InitStruct.Pin = SCLK_Pin|NOE_Pin|NOE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
