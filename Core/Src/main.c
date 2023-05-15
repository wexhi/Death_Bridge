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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "niming.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>
#include "cJSON.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SPEED 4.5
#define MIN_SPEED 0
#define PI 3.141592653589793
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

short Encoder1Count = 0;
short Encoder2Count = 0;
uint8_t str[256];

float Motor1Speed = 0.00;
float Motor2Speed = 0.00;
int Motor1Pwm = 0;
int Motor2Pwm = 0;

uint16_t Timer1Count = 0;

extern tPid pidMotor1Speed, pidMotor2Speed;
extern Car wheel1, wheel2;

extern uint8_t usart1_ReadBuf[256];
extern uint8_t usart1_ReadBufCount;

float p, i, d, a;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Show_Info()
{
		sprintf((char*)str,
		"Motor1Speed : %2f Motor2Speed : %2f\r\n Motor1Pwm : %d Motor2Pwm : %d\r\n", 
		 Motor1Speed,	   Motor2Speed,		 Motor1Pwm,		Motor1Pwm);
	HAL_UART_Transmit(&huart1, str, sizeof(str), 100);
}

void Show_Info_Car(Car *pid)
{
	sprintf((char*)str,
		"target_pos : %2f actual_pos : %2f\r\n err : \r\n", 
		 pid->target_pos,	   pid->actual_pos);
	HAL_UART_Transmit(&huart1, str, sizeof(str), 100);
}


int XianFu_PWM(int pwm)
{
	if (pwm >= 10000)
	{
		pwm = 8000;
	}
	else if (pwm <= -10000)
	{
		pwm = -8000;
	}
	return pwm;
}

void Motor_Left(int pwm)
{
	if (pwm > 0) // forward
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
	else if(pwm < 0) //back
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -pwm);
	}
	else // stop
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
}

void Motor_Right(int pwm)
{
	pwm = -pwm;	
	if (pwm > 0) // forward
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}
	else if(pwm < 0) //back
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -pwm);
	}
	else // stop
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}
}

void Motor_Control(int pwm1, int pwm2)
{
	// if (pwm1 == pwm2 ) go straight
	// if (pwm1 > pwm2) turn right
	// if (pwm1 < pwm2) turn left
	Motor_Left(pwm1);
	Motor_Right(pwm2);
	
}

//MotorPidSetSpeed(3, 4);//turn left
//MotorPidSetSpeed(4, 3);//turn right
//MotorPidSetSpeed(4, 4);//forward
//MotorPidSetSpeed(-3, -3);//backward
void MotorPidSetSpeed(float Motor1SetSpeed, float Motor2SetSpeed)
{
	//Set pid target speed
	pidMotor1Speed.target_val = Motor1SetSpeed;
	pidMotor2Speed.target_val = Motor2SetSpeed;
	
	Motor_Control(PID_realize(&pidMotor1Speed, Motor1Speed), PID_realize(&pidMotor2Speed, Motor2Speed));
}

void MotorPidSpeedUp(void)
{
	static float motor_Speed_Up = 2;
	if (motor_Speed_Up < MAX_SPEED)
	{
		motor_Speed_Up += 0.5;
		MotorPidSetSpeed(motor_Speed_Up, motor_Speed_Up);
	}
}

void MotorPidSpeedDown(void)
{
	static float motor_Speed_Down = 4;
	if (motor_Speed_Down > MIN_SPEED)
	{
		motor_Speed_Down -= 0.5;
		MotorPidSetSpeed(motor_Speed_Down, motor_Speed_Down);
	}
}

void stop(void)
{
	pidMotor1Speed.target_val = 0;
	pidMotor2Speed.target_val = 0;
	Motor_Control(0, 0);
}

//void MoveTo(int des, int act)
//{
//	if (des * 1560 )
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3) // 1000hz 10ms
	{
		static int Moto1 = 0, Moto2 = 0;
		Timer1Count++;
		if (Timer1Count % 1 == 0)
		{
			Encoder1Count = (short)__HAL_TIM_GET_COUNTER(&htim4);
			Encoder2Count = -(short)__HAL_TIM_GET_COUNTER(&htim8);
				
			Motor1Speed = (float)Encoder1Count * 100 / 30 / 13 / 4; 
			Motor2Speed = (float)Encoder2Count * 100 / 1560;
			__HAL_TIM_SET_COUNTER(&htim4, 0);
			__HAL_TIM_SET_COUNTER(&htim8, 0);
			//Show_Info();		
				
		}
		if (Timer1Count % 2 == 0) //20ms
		{
			Moto1 = PID_realize(&pidMotor1Speed, Motor1Speed);
			Moto2 = PID_realize(&pidMotor2Speed, Motor2Speed);
			Moto1 = XianFu_PWM(Moto1);
			Moto2 = XianFu_PWM(Moto2);
			Motor_Control(Moto1, Moto2);
			Timer1Count = 0;
		}
			
		//	static int Moto1 = 0, Moto2 = 0;
		//	if (htim->Instance == TIM3)
		//	{
		//		Encoder1Count = (short)__HAL_TIM_GET_COUNTER(&htim4);
		//		Encoder2Count = -(short)__HAL_TIM_GET_COUNTER(&htim8);
		//		Moto1 = Position_PID(&wheel1, 0, Encoder1Count);
		//		Moto2 = Position_PID(&wheel2, 10, Encoder2Count);
		//		Moto1 = XianFu_PWM(Moto1);
		//		Moto2 = XianFu_PWM(Moto2);
		//		Motor_Control(Moto1, Moto2);
		//	}
	}
}

	


uint8_t Usart_WaitReasFinish(void)
{
	static uint8_t Usart_LastReadCount = 0;
	
	if (usart1_ReadBufCount == 0)
	{
		Usart_LastReadCount = 0;
		return 1;
	}
	else if (Usart_LastReadCount == usart1_ReadBufCount)
	{
		Usart_LastReadCount = 0;
		usart1_ReadBufCount = 0;
		return 0;
	}
	else
	{
		Usart_LastReadCount = usart1_ReadBufCount;
		return 2;
	}
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_Base_Start_IT(&htim3);
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	
	PID_init();
	
	cJSON *cJsonData, *cJsonValue;
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  
//	  if (Motor1Speed < 3.9)
//	  {
//		  Motor1Pwm += 100;
//	  }
//	  else if (Motor1Speed > 4.1)
//	  {
//		  Motor1Pwm -= 100;
//	  }
//	   
//	  if (Motor2Speed < 3.9)
//	  {
//		  Motor2Pwm += 100;
//	  }
//	  else if (Motor2Speed > 4.1)
//	  {
//		  Motor2Pwm -= 100;
//	  }
//	  if (Motor1Pwm > MaxPwm) Motor1Pwm = MaxPwm;
//	  if (Motor2Pwm > MaxPwm) Motor2Pwm = MaxPwm;
//	  if (Motor1Pwm < MinPwm) Motor1Pwm = MinPwm;
//	  if (Motor2Pwm < MinPwm) Motor2Pwm = MinPwm;

//	  Show_Info_Car(&wheel2);
//	  Show_Info();
	  HAL_Delay(5);

	  MotorPidSetSpeed(3, 4);
	  HAL_Delay(1000);
	  stop();
	  HAL_Delay(20);
	  MotorPidSetSpeed(-3, 4);
	  HAL_Delay(1000);

//	  //MotorPidSetSpeed(3, 4);//turn left
//	  //MotorPidSetSpeed(4, 3);//turn right
//	  //MotorPidSetSpeed(4, 4);//forward
//	  //MotorPidSetSpeed(-3, -3);//backward
//	  //MotorPidSpeedUp();

	  //ANO_DT_Send_F2(wheel1.actual_pos / 1560 * 100, 3.0 * 100, wheel2.actual_pos / 1560 * 100, 3.0 * 100);
	  ANO_DT_Send_F2(Motor1Speed * 100, 3.0 * 100, Motor2Speed * 100, 3.0 * 100);
	  
	  
	  
//	  if (Usart_WaitReasFinish() == 0)
//	  {
//		  cJsonData = cJSON_Parse((const char *)usart1_ReadBuf);
//		  
//		  if (cJSON_GetObjectItem(cJsonData, "p") != NULL)
//		  {
//			  cJsonValue = cJSON_GetObjectItem(cJsonData, "p");
//			  p = cJsonValue->valuedouble;
//			  pidMotor1Speed.Kp = p;
//		  }
//		  if (cJSON_GetObjectItem(cJsonData, "i") != NULL)
//		  {
//			  cJsonValue = cJSON_GetObjectItem(cJsonData, "i");
//			  i = cJsonValue->valuedouble;
//			  pidMotor1Speed.Ki = i;
//		  }
//		  if (cJSON_GetObjectItem(cJsonData, "d") != NULL)
//		  {
//			  cJsonValue = cJSON_GetObjectItem(cJsonData, "d");
//			  d = cJsonValue->valuedouble;
//			  pidMotor1Speed.Kd = d;
//		  }
//		  if (cJSON_GetObjectItem(cJsonData, "a") != NULL)
//		  {
//			  cJsonValue = cJSON_GetObjectItem(cJsonData, "a");
//			  a = cJsonValue->valuedouble;
//			  pidMotor1Speed.target_val = a;
//		  }
//		  if (cJsonData != NULL)
//		  {
//			  cJSON_Delete(cJsonData);
//		  }
//		  memset(usart1_ReadBuf, 0, 255);
//	  }
//	  //printf("P:%3f I:%3f D:%3f A:%3f\r\n", p, i, d, a);
//	  sprintf((char*)str, "P:%3f I:%3f D:%3f A:%3f\r\n", p, i, d, a);
//	  HAL_UART_Transmit(&huart1, str, sizeof(str), 100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
