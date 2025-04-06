/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attentio n
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root.                                                           directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "Remote_control.h"
#include "car_drive.h"
#include "string.h"
#include "Encoder.h"
#include "turn.h"
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

/* USER CODE BEGIN PV */
volatile uint8_t is_open = 0;   // 0: �ر�, 1: ��
volatile uint8_t CH3,CH4,CH5,CH6,CH7; // ���ڴ洢ң������ֵ
volatile int16_t ECD;  // ���ڴ洢��������ֵ
uint8_t prev_CH3 = 0, prev_CH4 = 0,prev_CH5 = 0, prev_CH6 = 0,prev_CH7 = 0;     // ���ڴ洢��һ�ε�ֵ
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  ���Ƽ̵����ͷ�����
  * @param  Gpiox: GPIOx��ֵ
  * @param  GPIO_Pin: GPIO_Pin��ֵ
  * @param  PinState: GPIO_PinState��ֵ
  * @retval None
  */
void handle_relay_and_buzzer(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    // �����������ʱ��Ȼ����Ƽ̵���
    // ���������ӷ���������ʱ�Ĵ���
    HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin, GPIO_PIN_RESET);  // ��������
    HAL_Delay(500);  // ��ʱ100ms
    HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin, GPIO_PIN_SET);  // ��������
    // ���Ƽ̵�����״̬
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
}
/**
  * @brief  ���CH3��CH4��ֵ�����CH3Ϊ1��CH4Ϊexpected_CH4�����л��̵���״̬
  * @param  CH3: CH3��ֵ
  * @param  CH4: CH4��ֵ
  * @param  expected_CH4: CH4������ֵ,Ҳ���ǿ�������
  * @param  switchFlag: �����ж��Ƿ񳤰��ļ�����
  * @param  is_open: �̵�����״̬
  * @retval None
  */
void check_and_toggle_relay(uint8_t expected_CH4, uint8_t* switchFlag, volatile uint8_t* is_open) {
    GPIO_PinState newState;
    CH3 = CH3_GetDuty();  // ��ȡCH3��ֵ
    CH4 = CH4_GetDuty();  // ��ȡCH4��ֵ
    if (CH3 == 1 && CH4 == expected_CH4) {
        (*switchFlag)++;
    } else {
        *switchFlag = 0;  // ���ü�����
    }

    if (*switchFlag == 50) {    // ����2��
        newState = *is_open ? GPIO_PIN_RESET : GPIO_PIN_SET;
        handle_relay_and_buzzer(eleRelay1_GPIO_Port, eleRelay1_Pin, newState);
        *is_open = !(*is_open);  // �л�״̬ 
        *switchFlag = 0;  // ���ü�����   
        }
}

void update_oled_display(uint8_t CH3, uint8_t CH4, uint8_t CH5, uint8_t CH6,uint8_t CH7) {
    if (CH3 != prev_CH3) {
        OLED_ShowNum(24, 0, CH3, 4, OLED_6X8);
        prev_CH3 = CH3;
    }
    if (CH4 != prev_CH4) {
        OLED_ShowNum(24, 8, CH4, 4, OLED_6X8);
        prev_CH4 = CH4;
    }
    if (CH5 != prev_CH5) {
        OLED_ShowNum(24, 16, CH5, 4, OLED_6X8);
        prev_CH5 = CH5;
    }
    if (CH6 != prev_CH6) {
        OLED_ShowNum(24, 24, CH6, 4, OLED_6X8);
        prev_CH6 = CH6;
    }
    if (CH7 != prev_CH7) {
        OLED_ShowNum(24, 32, CH7, 4, OLED_6X8);
        prev_CH7 = CH7;
    }
    OLED_Update();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t switchFlag = 0;   // �����ж��Ƿ񳤰�

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
  MX_DAC_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  // Uart_Rxagain(&huart1);  // ���¿������ڽ����жϣ�����Ϊ��Ҫ�����Ĵ��ں�
  OLED_Init();
  CH_Init();
  Car_Init();   // ��ʼ��С������dac
  Encoder_open();  // ��ʼ��������
  Step_Motor_Init();  // ��ʼ���������
  __HAL_TIM_SET_COUNTER(&htim5, 0);  //���������ֵ
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  OLED_ShowString(0, 0, "CH3:0000", OLED_6X8);  //��ʾCH3��
  OLED_ShowString(0, 8, "CH4:0000", OLED_6X8);  //��ʾCH4��
  OLED_ShowString(0, 16, "CH5:0000", OLED_6X8);  //��ʾCH5��
  OLED_ShowString(0, 24, "CH6:0000", OLED_6X8);  //��ʾCH6��
  OLED_ShowString(0, 32, "CH7:0000", OLED_6X8);  //��ʾCH7��
  OLED_ShowString(0, 40, "ECD:0000", OLED_6X8);  //��ʾCH7��
  OLED_Update();

  while (1)
  {
    // CH3 = CH3_GetDuty();
    // CH4 = CH4_GetDuty();
    // CH5 = CH5_GetDuty();
    // CH6 = CH6_GetDuty();
    // CH7 = CH7_GetDuty();
    // update_oled_display(CH3, CH4, CH5, CH6,CH7);
    //����Ϊ���Գ���ʵ��ʹ��ʱɾ��
    
    if (is_open == 0) {
        check_and_toggle_relay(1, &switchFlag, &is_open);
    } else {
        check_and_toggle_relay(102, &switchFlag, &is_open);
    }
    direction_control();
    speed_control();
    spray_control();

    // ECD = Get_Encoder_Value();

    // OLED_Printf(0,40,OLED_6X8, "bfECD:%05d", ECD);
    // OLED_Update();

    Step_Motor_Control();

    // ECD = Get_Encoder_Value();
    // OLED_Printf(0,48,OLED_6X8, "atECD:%05d", ECD);
    // OLED_Update();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
