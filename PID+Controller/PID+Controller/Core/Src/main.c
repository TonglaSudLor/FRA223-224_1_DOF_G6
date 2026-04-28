/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_pid_controller.h"
#include "usart.h"
#include <string.h>
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
/* Motor control variables */
volatile uint32_t loop_counter = 0;
volatile float target_speed_rpm = 10.0f;

/* UART reception variables */
uint8_t rx_byte;              // ตัวแปรพักข้อมูลที่รับเข้าทีละ byte
char rx_buffer[4];            // Buffer ชั่วคราว (ขนาด 3 + null)
uint8_t rx_index = 0;
char controlState[4] = "O-N";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motor_Control_Demo(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Motor control demonstration function
  * @note   This example shows how to use the motor PID controller
  * @retval None
  */

static void Motor_Control_Demo(void)
{
  /* ตัวอย่าง: ส่งค่าเพื่อดูใน Live Expressions หรือ UART */
  if ((loop_counter % 10) == 0)
  {
    /* printf("Pos: %.2f deg | Speed: %.2f RPM\r\n",
           Motor_GetPosition(),
           Motor_GetSpeed());
    */
  }

  loop_counter++;
}
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
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  
  // 1. เริ่มต้นระบบ (เปิด Timer, PWM, PID)
  Motor_Init();
  Motor_SetVoltageLimit(12.0f, 12.0f);

  // 2. ตั้งค่าโปรไฟล์ S-Curve
  // (ความเร็วสูงสุด 2000 RPM, ความเร่ง 1000 RPM/s, ความสมูท 0.8)
  Motor_SetMotionProfile(2000.0f, 1000.0f, 0.8f);

  // 3. เริ่มต้นรับข้อมูลผ่าน Interrupt ทีละ 1 byte
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_matlab_tick = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Stream data to MATLAB every 20ms (50Hz)
    if (HAL_GetTick() - last_matlab_tick >= 20)
    {
      Motor_SendDataToMatlab();
      last_matlab_tick = HAL_GetTick();
    }
    
    HAL_Delay(1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Redirect printf to LPUART1 (connected to PC/USB COM Port)
  */
int _write(int file, char *ptr, int len)
{
  extern UART_HandleTypeDef hlpuart1;
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

/**
  * @brief  Callback function called by TIM6 interrupt handler
  * @note   This is called every 10ms (100Hz control loop frequency)
  * @param  htim TIM handle pointer
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Check if TIM6 generated the interrupt */
  if (htim->Instance == TIM6)
  {
    /* Call the motor PID control loop ISR */
    TIM6_Control_Loop_ISR();
  }
}

/**
  * @brief  UART reception completion callback
  * @param  huart UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    // PRIORITY CHECK: ถ้าเป็น 'P' ให้สั่งงานหยุดทันที
    if (rx_byte == 'P')
    {
      Motor_ProcessCommand('P');
    }

    // ตรวจสอบตัวจบข้อความ
    if (rx_byte == '\n' || rx_byte == '\r')
    {
      if (rx_index > 0)
      {
        rx_buffer[rx_index] = '\0';
        
        // Command (ตัวอักษรที่ 0) - ถ้าไม่ใช่ 'P' (เพราะ process ไปแล้ว)
        if (rx_buffer[0] != 'P') {
          Motor_ProcessCommand(rx_buffer[0]);
        }
        
        // Connection Status (ตัวอักษรที่ 2)
        if (rx_index >= 3) {
          Motor_SetConnectionStatus(rx_buffer[2] == 'C');
        }
        
        rx_index = 0;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      }
    }
    else
    {
      // เก็บข้อมูลลง Buffer
      if (rx_index < (sizeof(rx_buffer) - 1))
      {
        rx_buffer[rx_index++] = rx_byte;
      }
      
      // เมื่อได้รับครบ 3 ตัวอักษร ให้ประมวลผลทันทีเพื่อลด latency
      if (rx_index >= 3)
      {
        rx_buffer[rx_index] = '\0';
        
        // Update M and Y button status (for 3s hold detection)
        Motor_UpdateModeButton(rx_buffer[0] == 'M');
        Motor_UpdateSelectionButton(rx_buffer[0] == 'Y');

        // Command (ตัวอักษรที่ 0)
        Motor_ProcessCommand(rx_buffer[0]);
        
        // Connection Status (ตัวอักษรที่ 2)
        Motor_SetConnectionStatus(rx_buffer[2] == 'C');
        
        rx_index = 0;
      }
    }

    // รับค่าตัวต่อไป
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  }
}

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
#ifdef USE_FULL_ASSERT
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
