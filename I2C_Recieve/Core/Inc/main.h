/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_Pin GPIO_PIN_0
#define BTN_GPIO_Port GPIOA
#define A_Pin GPIO_PIN_5
#define A_GPIO_Port GPIOC
#define B_Pin GPIO_PIN_1
#define B_GPIO_Port GPIOB
#define SCLK_Pin GPIO_PIN_7
#define SCLK_GPIO_Port GPIOE
#define NOE_Pin GPIO_PIN_9
#define NOE_GPIO_Port GPIOE
#define NOE1_Pin GPIO_PIN_11
#define NOE1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
