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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "i2c-lcd.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {	// enum for main FSM management
  NULL_state,
  BALANCE_state,
  ENCODER_DISPLAY_state,
  IMU_DISPLAY_state,
  HOME_state,
  EEPROM_RW_state,
  KP_state,
  KI_state,
  KD_state
} FSM_states_t;

typedef enum {	// enum for btn flag status
  BTN_not_pressed,
  BTN_pressed
} BTN_interrupt_status_t;

typedef struct {	// struct for holding all button status flags
  BTN_interrupt_status_t ok;
  BTN_interrupt_status_t no;
  BTN_interrupt_status_t left;
  BTN_interrupt_status_t right;
} BTN_flags_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_LEFT_ARROW 127	// added by Tadzik for LCD
#define LCD_RIGHT_ARROW 126
#define LCD_DEGREE_SYMBOL 223
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FSM_states_t current_state = HOME_state;
FSM_states_t previous_state = NULL_state;
BTN_flags_t volatile BTN = {		// for holding current state of btn interrupt flag
  .ok = BTN_not_pressed,	// initializing with off values
  .no = BTN_not_pressed,
  .left = BTN_not_pressed,
  .right = BTN_not_pressed
};
char MSG[20];	// string for string printing
MPU6050_t MPU6050; // MPU lib data struct instance

uint16_t old_encd_left = 0, encd_left = 0;
uint16_t old_encd_right = 0, encd_right = 0;
int duty_left = 0, duty_right = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
long map(long x, long in_min, long in_max, long out_min, long out_max);
void setMotors(int left_motor_speed, int right_motor_speed, int zero_speed, int offset_zero_speed);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void stateNULL(void);
void stateBALANCE(void);
void stateENCODER(void);
void stateIMU(void);
void stateHOME(void);
void stateEEPROM(void);
void stateKP(void);
void stateKI(void);
void stateKD(void);
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_init();
  lcd_put_cur(0, 0);
  lcd_send_string("Hello, world!");

  MPU6050_Init(&hi2c1);
  HAL_Delay(1000);

  while (1)
  {
	  switch (current_state) {
		case ENCODER_DISPLAY_state:
			stateENCODER();
			break;
		case IMU_DISPLAY_state:
			stateIMU();
			break;
		case HOME_state:
			stateHOME();
			break;
		case EEPROM_RW_state:
			stateEEPROM();
			break;
		case KP_state:
			stateKP();
			break;
		case KI_state:
			stateKI();
			break;
		case KD_state:
			stateKD();
			break;
		case BALANCE_state:
			stateBALANCE();
			break;
		default:
			stateNULL();
			break;
	  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BTN_OK_Pin)
	{
		BTN.ok = BTN_pressed;
	}
	if (GPIO_Pin == BTN_NO_Pin)
	{
		BTN.no = BTN_pressed;
	}
	if (GPIO_Pin == BTN_LE_Pin)
	{
		BTN.left = BTN_pressed;
		HAL_Delay(2); // XDDDD INTERRUPTS ALE CALLED TWICE WITHOUT THIS DUDE RIGHT HERE XD
	}
	if (GPIO_Pin == BTN_RI_Pin)
	{
		BTN.right = BTN_pressed;
		HAL_Delay(2); // XDDDD INTERRUPTS ALE CALLED TWICE WITHOUT THIS DUDE RIGHT HERE XD
	}
}
void stateNULL(void)
{
	if (current_state != previous_state)
	{
		previous_state = current_state;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("oops NULL state?");
		lcd_put_cur(1, 3);
		lcd_send_string("press ok");
	}
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	HAL_Delay(200);
	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		current_state = HOME_state;
	}
	if (BTN.no == BTN_pressed) BTN.no = BTN_not_pressed;
	if (BTN.left == BTN_pressed) BTN.left = BTN_not_pressed;
	if (BTN.right == BTN_pressed) BTN.right = BTN_not_pressed;
}
void stateBALANCE(void)
{
	if (current_state != previous_state)
	{
		previous_state = current_state;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("Balancing");
		lcd_put_cur(1, 0);
		lcd_send_string("press no to exit");
	}
//todo end this XD
	if (BTN.ok == BTN_pressed) BTN.ok = BTN_not_pressed;
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		current_state = HOME_state;
	}
	if (BTN.left == BTN_pressed) BTN.left = BTN_not_pressed;
	if (BTN.right == BTN_pressed) BTN.right = BTN_not_pressed;
}
void stateENCODER(void)
{
	if (current_state != previous_state)
	{
		previous_state = current_state;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("ENCODER");
		lcd_put_cur(1, 15);
		lcd_send_data(LCD_RIGHT_ARROW);
		HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);	// encoder timer start
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	// encoder timer start
		TIM1->CNT = 32767;	// wartosc rejestru na srodek ustawiona
		TIM2->CNT = 32767;
	}
	// todo cyclic interrupt needed 10ms like
	encd_left = TIM2->CNT;
	encd_right = TIM1->CNT;

	//encd_right = TIM2->CNT;
	if (encd_left != old_encd_left || encd_right != old_encd_right)
	{
		old_encd_left = encd_left;
		old_encd_right = encd_right;
		lcd_put_cur(1, 1);
		sprintf(MSG, "%+06d", encd_left);
		lcd_send_string(MSG);

		lcd_put_cur(1, 8);
		sprintf(MSG, "%+06d", encd_right);
		lcd_send_string(MSG);
	}

	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
		current_state = IMU_DISPLAY_state;
	}
}
void stateIMU(void)
{
	if (current_state != previous_state)
	{
		previous_state = current_state;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("IMU");
		lcd_put_cur(1, 0);
		lcd_send_data(LCD_LEFT_ARROW);
		lcd_put_cur(1, 15);
		lcd_send_data(LCD_RIGHT_ARROW);
		lcd_put_cur(0, 10);
		lcd_send_data(LCD_DEGREE_SYMBOL);
	}

	// todo cyclic interrupt needed 10ms like
	// MPU READ ANGLE
	MPU6050_Read_All(&hi2c1, &MPU6050);
	double angle = MPU6050.KalmanAngleX;
	// MPU DISPLAY ANGLE
	lcd_put_cur(0, 4);
	sprintf(MSG, "%+06.2f", angle);
	lcd_send_string(MSG);


	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = ENCODER_DISPLAY_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		current_state = HOME_state;
	}
}
void stateHOME(void)
{
	if (current_state != previous_state)
	{
		previous_state = current_state;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("Home");
		lcd_put_cur(1, 0);
		lcd_send_data(LCD_LEFT_ARROW);
		lcd_put_cur(1, 15);
		lcd_send_data(LCD_RIGHT_ARROW);
	}

	HAL_Delay(200);
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		current_state = BALANCE_state;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = IMU_DISPLAY_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		current_state = EEPROM_RW_state;
	}
}
void stateEEPROM(void)
{
	if (current_state != previous_state)
	{
		previous_state = current_state;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("EEPROM");
		lcd_put_cur(1, 0);
		lcd_send_data(LCD_LEFT_ARROW);
		lcd_put_cur(1, 15);
		lcd_send_data(LCD_RIGHT_ARROW);

//		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

	}



	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		duty_left++;
		duty_right++;
//		setMotors(duty_left, duty_right, 0, 0);
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		duty_left--;
		duty_right--;
//		setMotors(duty_left, duty_right, 0, 0);
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
//		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);	// test
//		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);	// test
		current_state = HOME_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
//		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);	//test
//		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);	//test
		current_state = KP_state;
	}
}
void stateKP(void)
{
	if (current_state != previous_state)
	{
		previous_state = current_state;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("KP");
		lcd_put_cur(1, 0);
		lcd_send_data(LCD_LEFT_ARROW);
		lcd_put_cur(1, 15);
		lcd_send_data(LCD_RIGHT_ARROW);
	}

	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = EEPROM_RW_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		current_state = KI_state;
	}
}
void stateKI(void)
{
	if (current_state != previous_state)
	{
		previous_state = current_state;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("KI");
		lcd_put_cur(1, 0);
		lcd_send_data(LCD_LEFT_ARROW);
		lcd_put_cur(1, 15);
		lcd_send_data(LCD_RIGHT_ARROW);
	}

	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = KP_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		current_state = KD_state;
	}
}
void stateKD(void)
{
	if (current_state != previous_state)
	{
		previous_state = current_state;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("KD");
		lcd_put_cur(1, 0);
		lcd_send_data(LCD_LEFT_ARROW);
	}

	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = KI_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
	}
}

// RANGE FROM -999 TO 999 	FIXME test range 20
void setMotors(int left_motor_speed, int right_motor_speed, int zero_speed, int offset_zero_speed) {
	// -------- min -------------- -offset --- zero --- +offset --------------- max ------->
	// ------ zero+offset_pwm -------------- max_pwm ------>
	long left_duty = map(abs(left_motor_speed), 0, 20, 0, 999);
	long right_duty = map(abs(right_motor_speed), 0, 20, 0, 999);

//	if (left_motor_speed > zero_speed + offset_zero_speed) {	// LEFT MOTOR
//		HAL_GPIO_WritePin(GPIOA, DIR_L_1_Pin, GPIO_PIN_SET); 	// HIGH
//		HAL_GPIO_WritePin(GPIOA, DIR_L_2_Pin, GPIO_PIN_RESET); 	// LOW
//		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, left_duty);	// Timer
//	} else if (left_motor_speed < zero_speed - offset_zero_speed) {
//		HAL_GPIO_WritePin(GPIOA, DIR_L_1_Pin, GPIO_PIN_RESET); 	// LOW
//		HAL_GPIO_WritePin(GPIOA, DIR_L_2_Pin, GPIO_PIN_SET); 	// HIGH
//		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, left_duty); 	// Timer set
//	} else {
//		HAL_GPIO_WritePin(GPIOA, DIR_L_1_Pin, GPIO_PIN_RESET); 	// LOW
//		HAL_GPIO_WritePin(GPIOA, DIR_L_2_Pin, GPIO_PIN_RESET); 	// LOW
//	}
//	if (right_motor_speed > zero_speed + offset_zero_speed) {	// RIGHT MOTOR
//		HAL_GPIO_WritePin(GPIOA, DIR_R_1_Pin, GPIO_PIN_SET); 	// HIGH
//		HAL_GPIO_WritePin(GPIOA, DIR_R_2_Pin, GPIO_PIN_RESET); 	// LOW
//		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, right_duty); 	// Timer
//	} else if (right_motor_speed < zero_speed - offset_zero_speed) {
//		HAL_GPIO_WritePin(GPIOA, DIR_R_1_Pin, GPIO_PIN_RESET); 	// LOW
//		HAL_GPIO_WritePin(GPIOA, DIR_R_2_Pin, GPIO_PIN_SET); 	// HIGH
//		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, right_duty); 	// Timer
//	} else {
//		HAL_GPIO_WritePin(GPIOA, DIR_R_1_Pin, GPIO_PIN_RESET); 	// LOW
//		HAL_GPIO_WritePin(GPIOA, DIR_R_2_Pin, GPIO_PIN_RESET); 	// LOW
//	}
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {	// y = map(x, x_min, x_max, y_min, y_max); function known from Arduino
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
