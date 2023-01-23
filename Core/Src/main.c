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
#include <string.h>
#include "i2c-lcd.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {	// enum for FSM management
  NULL_state,
  BALANCE_state,
  ENCODER_DISPLAY_state,
  IMU_DISPLAY_state,
  EEPROM_RW_state,
  HOME_state,

  REG_state,	// one function for all three states?
  KP_state,		// kp
  KI_state,		// ki
  KD_state,		// kd
  INT_state,	// integral size
  TRG_state		//target state
} FSM_states_t;

typedef enum {	// enum for PID settings choice
	NULL_choice,
	ANGLE_choice,
	POSITION_choice,
	DIRECTION_choice
} FSM_reg_choice_t;

typedef enum {	// enum for btn flag status
  BTN_not_pressed,
  BTN_pressed
} BTN_interrupt_status_t;

typedef enum {	// enum for timer flag status
	TIM_no_interrupt,
	TIM_interrupted
} TIMER_interrupt_status_t;

typedef struct {  // struct for holding all button status flags
  BTN_interrupt_status_t ok;
  BTN_interrupt_status_t no;
  BTN_interrupt_status_t left;
  BTN_interrupt_status_t right;
} BTN_flags_t;

typedef struct {  // struct for holding PID variables
	double target_value;// calc input optional
	double error;			// calc
	double previous_error;	// calc
	double integral;	// calc
	double derivative; 	// calc
	double int_limit;	// set in eeprom
	double kp;			// set in eeprom
	double ki;			// set in eeprom
	double kd;			// set in eeprom
	double delta_time;	// set in code
	char name[16 + 1];		// set in code
	uint8_t id;				// set in code
} PID_variables_t;

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
FSM_reg_choice_t current_choice = ANGLE_choice;
FSM_reg_choice_t previous_choice = NULL_choice;

TIMER_interrupt_status_t volatile tim7_20ms_flag = TIM_no_interrupt; // timer interrupts flag
BTN_flags_t volatile BTN;		// for holding current state of btn interrupt flag

char MSG[20];		// string for string printing
MPU6050_t MPU6050; 	// MPU lib data struct instance
PID_variables_t angle_PID;
PID_variables_t position_PID;
PID_variables_t direction_PID;
PID_variables_t *current_PID;
int16_t old_encd_left = 0, encd_left = 0, sum_encd_left = 0;
int16_t old_encd_right = 0, encd_right = 0, sum_encd_right = 0;
int16_t duty_left = 0, duty_right = 0;

const double pi10_over_24 = 0.654498;	// this * encoder ticks per 10 ms results in rad/s // FIXME check if true
const double delta_time = 0.02;			// 20 ms
const int16_t encd_reset = 255;			// just big enough so value doesnt overflow in negative dir

double old_value = 0;		// for updating screen and shit
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void stateNULL(void);
void stateBALANCE(void);
void stateENCODER(void);
void stateIMU(void);
void stateEEPROM(void);
void stateHOME(void);
void stateREG(PID_variables_t *hvar);
void stateKP(PID_variables_t *hvar);
void stateKI(PID_variables_t *hvar);
void stateKD(PID_variables_t *hvar);
void stateINT(PID_variables_t *hvar);
void stateTRG(PID_variables_t *hvar);

void lcd_send_arrows_to_sides(void);
void setMotors(int left_motor_speed, int right_motor_speed, int zero_speed, int offset_zero_speed);
long map(long x, long in_min, long in_max, long out_min, long out_max);
double constrain(double x, double lower_bound, double upper_bound);
double PID(double current_value, double target_value, PID_variables_t *hvar);
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
  current_PID = &angle_PID;
  angle_PID.id = 1;
  position_PID.id = 2;
  direction_PID.id = 3;
  strncpy(angle_PID.name, "angle    \0", 16);
  strncpy(position_PID.name, "position \0", 16);
  strncpy(direction_PID.name, "direction\0", 16);

  BTN.ok = BTN_not_pressed;
  BTN.no = BTN_not_pressed;
  BTN.left = BTN_not_pressed;
  BTN.right = BTN_not_pressed;
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
  MX_TIM7_Init();
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
	  switch (current_state)
	  {
		case ENCODER_DISPLAY_state:
			stateENCODER();
			break;
		case IMU_DISPLAY_state:
			stateIMU();
			break;
		case EEPROM_RW_state:
			stateEEPROM();
			break;
		case HOME_state:
			stateHOME();
			break;
		case REG_state:
			switch (current_choice)
			{
			case ANGLE_choice:
				current_PID = &angle_PID;
				stateREG(current_PID);
				break;
			case POSITION_choice:
				current_PID = &position_PID;
				stateREG(current_PID);
				break;
			case DIRECTION_choice:
				current_PID = &direction_PID;
				stateREG(current_PID);
				break;
			default:
				break;
			}
			break;
		case KP_state:
			stateKP(current_PID);
			break;
		case KI_state:
			stateKI(current_PID);
			break;
		case KD_state:
			stateKD(current_PID);
			break;
		case INT_state:
			stateINT(current_PID);
			break;
		case TRG_state:
			stateTRG(current_PID);
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
		HAL_TIM_Base_Start_IT(&htim7);		//TIMER7
	}
	if (tim7_20ms_flag)						//TIMER7
	{
		tim7_20ms_flag = TIM_no_interrupt; 	//TIMER7
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}
	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		HAL_TIM_Base_Stop_IT(&htim7);		//TIMER7
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
		HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);	// ENC TIMER1 START
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	// ENC TIMER2 START
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);	// PWM TIMER15 START
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);	// PWM TIMER15 START
		setMotors(0, 0, 0, 0);	// STOP MOTORS
		HAL_TIM_Base_Start_IT(&htim7);		// TIMER7 START
	}
	if (tim7_20ms_flag)						// TIMER7 IF FLAG
	{
		tim7_20ms_flag = TIM_no_interrupt; 		// TIMER7 FLAG ERASE

		encd_left = (TIM2->CNT) - encd_reset;	// encoders read step diff
		encd_right = (TIM1->CNT) - encd_reset;
		TIM2->CNT = encd_reset;
		TIM1->CNT = encd_reset;

		MPU6050_Read_All(&hi2c1, &MPU6050);		// MPU READ ANGLE KALMAN
		double ret = PID(MPU6050.KalmanAngleX, angle_PID.target_value, &angle_PID);	// PID

		duty_left = constrain((int16_t)ret, -999, 999);		// duty constraining
		duty_right = constrain((int16_t)ret, -999, 999);	// duty constraining
		setMotors(duty_left, duty_right, 0, 0);
	}
	if (BTN.ok == BTN_pressed) BTN.ok = BTN_not_pressed;
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		HAL_TIM_Base_Stop_IT(&htim7);		// TIMER7 STOP
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);	// PWM TIMER15 STOP
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);	// PWM TIMER15 STOP
		HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);	// ENC TIMER1 STOP
		HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);	// ENC TIMER2 STOP
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

		HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);	// ENC TIMER1 START
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	// ENC TIMER2 START
		TIM2->CNT = encd_reset;
		TIM2->CNT = encd_reset;
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);	// PWM TIMER15 START
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);	// PWM TIMER15 START
		setMotors(0, 0, 0, 0);
		HAL_TIM_Base_Start_IT(&htim7);		// TIMER7 START
	}
	if (tim7_20ms_flag)					// TIMER7 IF FLAG
	{
		tim7_20ms_flag = TIM_no_interrupt; 	// TIMER7 FLAG ERASE
		encd_left = (TIM2->CNT) - encd_reset;
		encd_right = (TIM1->CNT) - encd_reset;
		TIM2->CNT = encd_reset;
		TIM1->CNT = encd_reset;

		if (encd_left != old_encd_left || encd_right != old_encd_right)
		{
			old_encd_left = encd_left;
			old_encd_right = encd_right;
			lcd_put_cur(0, 8);
			sprintf(MSG, "%+03d", encd_left);
			lcd_send_string(MSG);
			lcd_put_cur(0, 12);
			sprintf(MSG, "%+03d", encd_right);
			lcd_send_string(MSG);
		}

		setMotors(duty_left, duty_right, 0, 0);
	}
	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		duty_left = constrain(duty_left + 333, -999, 999);
		duty_right = constrain(duty_right + 333, -999, 999);
		lcd_put_cur(1, 2);
		sprintf(MSG, "%+04d", duty_left);
		lcd_send_string(MSG);
		lcd_put_cur(1, 8);
		sprintf(MSG, "%+04d", duty_right);
		lcd_send_string(MSG);
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		duty_left = constrain(duty_left - 333, -999, 999);
		duty_right = constrain(duty_right - 333, -999, 999);
		lcd_put_cur(1, 2);
		sprintf(MSG, "%+04d", duty_left);
		lcd_send_string(MSG);
		lcd_put_cur(1, 8);
		sprintf(MSG, "%+04d", duty_right);
		lcd_send_string(MSG);
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		HAL_TIM_Base_Stop_IT(&htim7);		// TIMER7 STOP
		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);	// PWM TIMER15 STOP
		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);	// PWM TIMER15 STOP
		HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);	// ENC TIMER1 STOP
		HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);	// ENC TIMER2 STOP
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
		lcd_send_arrows_to_sides();
		lcd_put_cur(0, 10);
		lcd_send_data(LCD_DEGREE_SYMBOL);
		HAL_TIM_Base_Start_IT(&htim7);		// TIMER7 START
	}
	if (tim7_20ms_flag)						// TIMER7 IF FLAG
	{
		tim7_20ms_flag = TIM_no_interrupt; 		// TIMER7 FLAG ERASE
		MPU6050_Read_All(&hi2c1, &MPU6050);		// MPU READ ANGLE
		double angle = MPU6050.KalmanAngleX;
		lcd_put_cur(0, 4);						// MPU DISPLAY ANGLE
		sprintf(MSG, "%+06.2f", angle);
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
		HAL_TIM_Base_Stop_IT(&htim7);		// TIMER7 STOP
		current_state = ENCODER_DISPLAY_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		HAL_TIM_Base_Stop_IT(&htim7);		// TIMER7 STOP
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
		lcd_send_arrows_to_sides();
	}
	// todo finish this state and its functions
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
		current_state = IMU_DISPLAY_state;
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
		lcd_send_arrows_to_sides();
//		HAL_TIM_Base_Start_IT(&htim7);		// TIMER7 START
	}

//	if (tim7_10ms_flag)						// TIMER7 IF FLAG
//	{
//		tim7_10ms_flag = TIM_no_interrupt; 		// TIMER7 FLAG ERASE
//		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//	}
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
		current_state = EEPROM_RW_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		current_state = REG_state;
	}
}
void stateREG(PID_variables_t *hvar)
{
	if (current_state != previous_state || current_choice != previous_choice)
	{
		previous_state = current_state;
		previous_choice = current_choice;
		lcd_clear();
		lcd_send_arrows_to_sides();
		sprintf(MSG, "Regulator  %1d/3", hvar->id);
		lcd_put_cur(0, 0);
		lcd_send_string(MSG);
		lcd_put_cur(1, 2);
		lcd_send_string(hvar->name);
	}
	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		current_choice = constrain(current_choice + 1, 1, 3);
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		current_choice = constrain(current_choice - 1, 1, 3);
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = HOME_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		current_state = KP_state;
	}
}
void stateKP(PID_variables_t *hvar)
{
	if (current_state != previous_state || hvar->kp != old_value)
	{
		if (current_state != previous_state)
		{
			lcd_clear();
			lcd_send_arrows_to_sides();
		}
		previous_state = current_state;
		old_value = hvar->kp;

		lcd_put_cur(0, 0);
		sprintf(MSG,"kp = %+06.2f", hvar->kp);
		lcd_send_string(MSG);
	}
	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		hvar->kp += 0.1;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		hvar->kp -= 0.1;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = REG_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		current_state = KI_state;
	}
}
void stateKI(PID_variables_t *hvar)
{
	if (current_state != previous_state || hvar->ki != old_value)
	{
		if (current_state != previous_state)
		{
			lcd_clear();
			lcd_send_arrows_to_sides();
		}
		previous_state = current_state;
		old_value = hvar->ki;

		lcd_put_cur(0, 0);
		sprintf(MSG,"ki = %+06.2f", hvar->ki);
		lcd_send_string(MSG);
	}

	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		hvar->ki += 0.1;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		hvar->ki -= 0.1;
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
void stateKD(PID_variables_t *hvar)
{
	if (current_state != previous_state || hvar->kd != old_value)
	{
		if (current_state != previous_state)
		{
			lcd_clear();
			lcd_send_arrows_to_sides();
		}
		previous_state = current_state;
		old_value = hvar->kd;

		lcd_put_cur(0, 0);
		sprintf(MSG,"kd = %+06.2f", hvar->kd);
		lcd_send_string(MSG);
	}

	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		hvar->kd += 0.1;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		hvar->kd -= 0.1;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = KI_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		current_state = INT_state;
	}
}
void stateINT(PID_variables_t *hvar)
{
	if (current_state != previous_state || hvar->int_limit != old_value)
	{
		if (current_state != previous_state)
		{
			lcd_clear();
			lcd_send_arrows_to_sides();
		}
		previous_state = current_state;
		old_value = hvar->int_limit;

		lcd_put_cur(0, 0);
		sprintf(MSG,"int_lim = %+06.2f", hvar->int_limit);
		lcd_send_string(MSG);
	}
	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		hvar->int_limit += 0.1;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		hvar->int_limit -= 0.1;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = KD_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
		current_state = TRG_state;
	}
}
void stateTRG(PID_variables_t *hvar)
{
	if (current_state != previous_state || hvar->target_value != old_value)
	{
		if (current_state != previous_state)
		{
			lcd_clear();
			lcd_put_cur(1, 0);
			lcd_send_data(LCD_LEFT_ARROW);
		}
		previous_state = current_state;
		old_value = hvar->target_value;

		lcd_put_cur(0, 0);
		sprintf(MSG,"target = %+06.2f", hvar->target_value);
		lcd_send_string(MSG);
	}
	if (BTN.ok == BTN_pressed)
	{
		BTN.ok = BTN_not_pressed;
		hvar->target_value += 0.1;
	}
	if (BTN.no == BTN_pressed)
	{
		BTN.no = BTN_not_pressed;
		hvar->target_value -= 0.1;
	}
	if (BTN.left == BTN_pressed)
	{
		BTN.left = BTN_not_pressed;
		current_state = INT_state;
	}
	if (BTN.right == BTN_pressed)
	{
		BTN.right = BTN_not_pressed;
	}
}

/*
 * Function for displaying both arrows on LCD xd
 */
void lcd_send_arrows_to_sides(void)
{
	lcd_put_cur(1, 0);
	lcd_send_data(LCD_LEFT_ARROW);
	lcd_put_cur(1, 15);
	lcd_send_data(LCD_RIGHT_ARROW);
}
/*
 * Function for setting motors at desired speed, input must be between -999 and 999
 * @parameter left_motor_speed
 * @parameter right_motor_speed
 * @parameter
 * @parameter
 */
void setMotors(int left_motor_speed, int right_motor_speed, int zero_speed, int offset_zero_speed)
{
	// -------- min -------------- -offset --- zero --- +offset --------------- max ------->
	// ------ zero+offset_pwm -------------- max_pwm ------>
	//	long left_motor_duty = map(abs(left_motor_speed), 0, 100, 0, 999);
	//	long right_motor_duty = map(abs(right_motor_speed), 0, 100, 0, 999); for testing purposes
	long left_motor_duty = abs(left_motor_speed);
	long right_motor_duty = abs(right_motor_speed);
	if (left_motor_speed > zero_speed + offset_zero_speed) 	// LEFT MOTOR
	{
		HAL_GPIO_WritePin(GPIOB, EN_L_1_Pin, GPIO_PIN_SET); 	// HIGH
		HAL_GPIO_WritePin(GPIOA, EN_L_2_Pin, GPIO_PIN_RESET); 	// LOW
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, left_motor_duty);	// Timer
	}
	else if (left_motor_speed < zero_speed - offset_zero_speed)
	{
		HAL_GPIO_WritePin(GPIOB, EN_L_1_Pin, GPIO_PIN_RESET); 	// LOW
		HAL_GPIO_WritePin(GPIOA, EN_L_2_Pin, GPIO_PIN_SET); 	// HIGH
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, left_motor_duty); // Timer
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, EN_L_1_Pin, GPIO_PIN_RESET); 	// LOW
		HAL_GPIO_WritePin(GPIOA, EN_L_2_Pin, GPIO_PIN_RESET); 	// LOW
	}
	if (right_motor_speed > zero_speed + offset_zero_speed)	// RIGHT MOTOR
	{
		HAL_GPIO_WritePin(GPIOB, EN_R_1_Pin, GPIO_PIN_SET); 	// HIGH
		HAL_GPIO_WritePin(GPIOA, EN_R_2_Pin, GPIO_PIN_RESET); 	// LOW
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, right_motor_duty); // Timer
	}
	else if (right_motor_speed < zero_speed - offset_zero_speed)
	{
		HAL_GPIO_WritePin(GPIOB, EN_R_1_Pin, GPIO_PIN_RESET); 	// LOW
		HAL_GPIO_WritePin(GPIOA, EN_R_2_Pin, GPIO_PIN_SET); 	// HIGH
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, right_motor_duty); // Timer
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, EN_R_1_Pin, GPIO_PIN_RESET); 	// LOW
		HAL_GPIO_WritePin(GPIOA, EN_R_2_Pin, GPIO_PIN_RESET); 	// LOW
	}
}
/*
 * y = map(x, x_min, x_max, y_min, y_max) function known from Arduino
 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*
 * Function for constraining x value between bounds
 * @parameter x input value to be constrained
 * @parameter lower_bound
 * @parameter upper_bound
 *
 * @returns constrained x
 */
double constrain(double x, double lower_bound, double upper_bound)
{
	if (x > upper_bound)
	{
		return upper_bound;
	}
	else if (x < lower_bound)
	{
		return lower_bound;
	}
	else
	{
		return x;
	}
}
/*
 * Function calculates PID values according to data in PID_variables_t structure pointed by *hvar
 * error is calculated between current_value and target_value
 * @parameter current_value
 * @parameter target_value
 * @parameter *hvar
 *
 * @returns PID
 */
double PID(double current_value, double target_value, PID_variables_t *hvar)
{
	hvar->error = (target_value - current_value);	// error calc
	hvar->integral = hvar->integral + hvar->error * hvar->delta_time; 	// Euler quadrature integral
	hvar->integral = constrain(hvar->integral, -hvar->int_limit, hvar->int_limit);	// Anti windup
	hvar->derivative = (hvar->error - hvar->previous_error) / hvar->delta_time;	// derivative
	hvar->previous_error = hvar->error;		// save last error for further calculation
	return (hvar->ki * hvar->error) + (hvar->ki * hvar->integral) + (hvar->kd * hvar->derivative);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim7)
  {
	  tim7_20ms_flag = TIM_interrupted;
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BTN_OK_Pin)
	{
		BTN.ok = BTN_pressed;
		HAL_Delay(5); // XDDDD INTERRUPTS ALE CALLED TWICE WITHOUT THIS DUDE RIGHT HERE XD FIXME
	}
	if (GPIO_Pin == BTN_NO_Pin)
	{
		BTN.no = BTN_pressed;
		HAL_Delay(5); // XDDDD INTERRUPTS ALE CALLED TWICE WITHOUT THIS DUDE RIGHT HERE XD FIXME
	}
	if (GPIO_Pin == BTN_LE_Pin)
	{
		BTN.left = BTN_pressed;
		HAL_Delay(5); // XDDDD INTERRUPTS ALE CALLED TWICE WITHOUT THIS DUDE RIGHT HERE XD FIXME
	}
	if (GPIO_Pin == BTN_RI_Pin)
	{
		BTN.right = BTN_pressed;
		HAL_Delay(5); // XDDDD INTERRUPTS ALE CALLED TWICE WITHOUT THIS DUDE RIGHT HERE XD FIXME
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

	lcd_clear();
	lcd_send_string("ERROR");	// RISKY XD

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
