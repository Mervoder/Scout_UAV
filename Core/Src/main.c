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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "PID control.h"
#include "ibus.h"
#include "BNO080.h"
#include "Quaternion.h"
#include "ICM20602.h"
#include "lwgps/lwgps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUFFER_SIZE 		128
#define RX_PID_BUFFER_SIZE  73


extern PIDDouble roll;
extern PIDDouble pitch;
extern PIDSingle yaw_heading;
extern PIDSingle yaw_rate;
extern PIDSingle altitude;

lwgps_t gps;
struct FSiA6B_iBus iBus;

struct MS5611_Vars
{
  uint16_t C1, C2, C3, C4, C5, C6, C7;
  uint32_t D1, D2;
  int32_t dT, TEMP, P;
  int64_t OFF, SENS;
}MS5611;

union{
	float f;
	uint8_t b[4];

}converter;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int counter =0, counter2=0 , gyro_flag =0 , accel_flag =0, tim3_1ms_flag=0 , tim2_10ms_flag =0;
int roll_angle, pitch_angle ;
int motor1, motor2,motor3,motor4 , task;

float   bno_gyro_x,bno_gyro_y,bno_gyro_z;
float Temp , Pressure , Altitude_meas , Altitude_set_point;

uint8_t rx_buffer_gps[RX_BUFFER_SIZE];
uint8_t rx_index=0;
uint8_t rx_data_gps=0;

uint8_t rx_buffer_PID[RX_PID_BUFFER_SIZE];
uint8_t rx_data_PID=0 , rx_index_PID=0;
uint8_t PID_Config_flag=0;

uint8_t ibus_rx_buf[32];
uint8_t ibus_rx_cplt_flag=0;
uint8_t uart1_rx_data;

uint8_t motor_arm;
uint8_t test[3] = {0};
uint8_t Tx_Buff[28]={0};

unsigned char cnt =0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

void MS5611_C();
void MS5611_D_Convertion();
void MS5611_calc(void);

uint32_t SPI1_Read32(void);
uint16_t SPI1_Read16(void);
uint8_t SPI1_Read8(void);

void SPI1_Write8(uint8_t data);
void Buzzer();
void Calibrate_ESC();
void send();
void PID_Coef_Config(void);
void PID_CONFIG_LORA(void);

unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus);
int _write(int file , char*p, int len);
uint32_t map( long A , long B , long C , long D, long E);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM2) // 10ms
	{

		if(counter == 1)	task=1; //d1 conv
		if(counter == 3)	task=2; // d1 read
		if(counter == 5)	task=3; // d2 conv
		if(counter == 7)	task=4; // d2 read
		if(counter == 9)	task=5; // math1
		if(counter == 10)	task=6; // math2
		if(counter == 60)			// math3
		{
		 task = 7;
		 counter =0;
		}


		counter++;
	}

	if(htim->Instance == TIM3) // 1ms
	{
		tim3_1ms_flag =1;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//static unsigned char cnt =0;

	if(huart->Instance == USART1)
	{

		switch(cnt)
				{

				case 0:

					if(uart1_rx_data == 0x20)
					{
						ibus_rx_buf[cnt] = uart1_rx_data;
						cnt++;
					}

				break;

				case 1:

					if(uart1_rx_data == 0x40)
					{
						ibus_rx_buf[cnt] = uart1_rx_data;
						cnt++;
					}

				else cnt =0;

				break;

				case 31:
					ibus_rx_buf[cnt] = uart1_rx_data;
					cnt =0;
					ibus_rx_cplt_flag =1;

				break;

				default:
					ibus_rx_buf[cnt] = uart1_rx_data;
					cnt++;
				break;

			    }


		HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);

	}

	if(huart->Instance == USART2){
		if(rx_data_gps != '\n' && rx_index < RX_BUFFER_SIZE) {
			rx_buffer_gps[rx_index++] = rx_data_gps;
		} else {
			lwgps_process(&gps, rx_buffer_gps, rx_index+1);
			rx_index = 0;
			rx_data_gps = 0;
		}
		HAL_UART_Receive_IT(&huart2, &rx_data_gps, 1);
		}


	if(huart->Instance == USART6){

		if(rx_data_PID != '\n' && rx_index_PID < RX_PID_BUFFER_SIZE)
		{
			rx_buffer_PID[rx_index_PID++] = rx_data_PID;
		}
		else {
			rx_index_PID = 0;
			rx_data_PID = 0;
			PID_Config_flag = 1;
		}
		HAL_UART_Receive_IT(&huart6, &rx_data_PID, 1);
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	float q[4];
	float quatRadianAccuracy;
	float yaw_heading_reference;
	int motor_arming_flag=0;

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  // ** PWM **
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  HAL_UART_Receive_IT(&huart1, &uart1_rx_data, 1);
  HAL_UART_Receive_IT(&huart2, &rx_data_gps, 1);
  HAL_UART_Transmit_IT(&huart6, Tx_Buff, 28);
  HAL_UART_Receive_IT(&huart6, &rx_data_PID, 1);

  PID_Coef_Config();

  Buzzer();

  BNO080_Initialization();
  BNO080_enableRotationVector(2500); // 2500us 400 hz
  //BNO080_enableGyro(1000);

  ICM20602_Initialization();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
  MS5611_C();
  MS5611_D_Convertion();
  MS5611_calc();

  lwgps_init(&gps);

//  Calibrate_ESC();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	  if(task == 7)
	  {
		 send();

	  }


	  if(ibus_rx_cplt_flag == 1) // kumanda
	 	{
	 	 ibus_rx_cplt_flag =0;

	 		 if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
	 		 {
	 			 iBus_Parsing(&ibus_rx_buf[0], &iBus);

	 			 if(iBus_isActiveFailsafe(&iBus)== 1)
	 			 {
	 				 TIM4->CCR1 = 0;
	 				 TIM4->CCR2 = 0;
	 				 TIM4->CCR3 = 0;
	 				 TIM4->CCR4 = 0;
	 			 }
	 		 }
	 	}

	  if(BNO080_dataAvailable() == 1)
	  {
		  q[0] = BNO080_getQuatI();
		  q[1] = BNO080_getQuatJ();
		  q[2] = BNO080_getQuatK();
		  q[3] = BNO080_getQuatReal();
		  quatRadianAccuracy = BNO080_getQuatRadianAccuracy();

		  Quaternion_Update(&q[0]);

		  BNO080_Roll = BNO080_Roll-0.5;
		  BNO080_Pitch = -BNO080_Pitch-1.5;

	  }

	  if(ICM20602_DataReady() == 1)
	  {
		//  LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_4);

		  ICM20602_Get3AxisGyroRawData(&ICM20602.gyro_x_raw);

		  ICM20602.gyro_x = ICM20602.gyro_x_raw * 2000.f / 32768.f;
		  ICM20602.gyro_y = ICM20602.gyro_y_raw * 2000.f / 32768.f;
		  ICM20602.gyro_z = ICM20602.gyro_z_raw * 2000.f / 32768.f;

		  ICM20602.gyro_x = -ICM20602.gyro_x;
		  ICM20602.gyro_z = -ICM20602.gyro_z;


	  }

	  if(iBus.SwA == 1000)
	  {
		  if(iBus.LV < 1010)
	  	  {
		  motor_arming_flag = 1;
		  yaw_heading_reference = BNO080_Yaw;
	  	  }
	  }

	  if(iBus.SwA == 1000 && motor_arming_flag ==1)
	  {

	  if(tim3_1ms_flag == 1)
		  {
		  tim3_1ms_flag = 0;

	 			  Double_Roll_Pitch_PID_Calculation(&pitch, -(iBus.RV - 1500) * 0.1f, BNO080_Pitch, ICM20602.gyro_x);
	 			  Double_Roll_Pitch_PID_Calculation(&roll, -(iBus.RH - 1500) * 0.1f, BNO080_Roll, -ICM20602.gyro_y);


	 			  /* O�?UZ BURASI PID KISMI E�?ER DRONE ÇOK SAÇMALARSA BURAYI YORUMA AL VE MOTORLARDAN ÇIKART*/
//
//	 			  if(iBus.LH < 1450 || iBus.LH > 1650 )
//	 			  {
//	 				 Altitude_set_point = Altitude_meas;
//	 			  }
//
//
//	 			  if(iBus.LH < 1650 || iBus.LH > 1450 )
//	 			  {
//	 				  Single_Altitude_PID_Calculation(&altitude, Altitude_set_point, Altitude_meas);
//
//	 			  }
//	 			  else {
	 				 altitude.pid_result =0;
//	 			  }
//


	 			  if(iBus.LV < 1030 )
	 			  {
	 				  Reset_All_PID_Integrator();
	 				// yaw_heading_reference = BNO080_Yaw;
	 			  }


	 			  if(iBus.LH < 1485 || iBus.LH > 1515) // stick
	 				  {
	 				  	  yaw_heading_reference = BNO080_Yaw;
	 				      Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH - 1500), (ICM20602.gyro_z) );

	 					  motor1 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result + altitude.pid_result;
	 					  motor2 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result + altitude.pid_result;
	 					  motor3 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result + altitude.pid_result;
	 					  motor4 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result + altitude.pid_result;

	      			  }
	 				  else
	 				  {

 					     Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, BNO080_Yaw, (ICM20602.gyro_z));

       					 motor1 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result + altitude.pid_result;
						 motor2 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result + altitude.pid_result;
						 motor3 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result + altitude.pid_result;
	 					 motor4 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result + altitude.pid_result;

	 				  }

			  if(iBus.LV > 1030)
			  {
				  TIM4->CCR1 = motor1 > 21000 ? 21000 : motor1 < 11000 ? 11000 : motor1;
				  TIM4->CCR2 = motor2 > 21000 ? 21000 : motor2 < 11000 ? 11000 : motor2;
				  TIM4->CCR3 = motor3 > 21000 ? 21000 : motor3 < 11000 ? 11000 : motor3;
				  TIM4->CCR4 = motor4 > 21000 ? 21000 : motor4 < 11000 ? 11000 : motor4;
			  }
			  else
			  {
				  TIM4->CCR1 = 11000;
				  TIM4->CCR2 = 11000;
				  TIM4->CCR3 = 11000;
				  TIM4->CCR4 = 11000;
			  }


		   }

	  }
	  else {
		  TIM4->CCR1 = 10500;
		  TIM4->CCR2 = 10500;
		  TIM4->CCR3 = 10500;
		  TIM4->CCR4 = 10500;
	  }



	if(PID_Config_flag == 1)
	{

		PID_CONFIG_LORA();

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
	  HAL_Delay(2000);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
	  HAL_Delay(100);

		PID_Config_flag =0;

	}

//
//	  if(task==1)
//	  {
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);
//		  SPI1_Write8(0x48);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
//		  task=0;
//	  }
//	  else if (task==2)
//	  {
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);
//				  SPI1_Write8(0x00);
//				  test[0]=SPI1_Read8();
//				  test[1]=SPI1_Read8();
//				  test[2]=SPI1_Read8();
//				  MS5611.D1 = ((test[0]<<16) + (test[1]<<8) + ( test[2]) );
//				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
//				  task=0;
//	  }
//	  else if (task==3)
//	  {			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);
//
//	  SPI1_Write8(0x58);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
//	  task=0;
//	  }
//	  else if (task==4)
//	  {
//			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);
//				  SPI1_Write8(0x00);
//				  test[0]=SPI1_Read8();
//				  test[1]=SPI1_Read8();
//				  test[2]=SPI1_Read8();
//				  MS5611.D2 = ((test[0]<<16) + (test[1]<<8) + ( test[2]) );
//				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
//				  task=0;
//
//	  }
//	  else if(task==5)
//	  {
//	  MS5611.dT = MS5611.D2 - ((uint32_t)MS5611.C5 * 256);
//	  MS5611.TEMP = 2000 + (((int64_t)MS5611.dT * MS5611.C6) / 8388608);
//	  Temp = MS5611.TEMP/100.0;
//	  task = 0;
//	  }
//	  else if(task==6)
//	  {
//	    MS5611.OFF = (int64_t)MS5611.C2*65535 + (int64_t)MS5611.C4*MS5611.dT/128;	//5611
//		MS5611.SENS = (int64_t)MS5611.C1*32768 + (int64_t)MS5611.C3*MS5611.dT/256;	//5611
//		MS5611.P = (MS5611.D1*MS5611.SENS/2097152 - MS5611.OFF)/32768;
//		task = 0;
//	  }
//	  else if(task==7)
//	  {
//	  MS5611.P = (MS5611.D1*MS5611.SENS/2097152 - MS5611.OFF)/32768;
//	  Pressure = MS5611.P/100.0;
//	  Altitude_meas = (1 - pow((Pressure / 1013.25),0.190284)) * 44191.4;
//	  }




  } // while (1)

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 840-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 41999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LED_Pin|Buzzer_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, CS_ICM_Pin|CS_MS56_Pin|CS_BNO_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, WAKE_Pin|RST_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin|Buzzer_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = INT_ICM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(INT_ICM_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CS_ICM_Pin|CS_MS56_Pin|CS_BNO_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = WAKE_Pin|RST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MS5611_C()
{
	  uint8_t test[3] = {0};
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);

	  SPI1_Write8(0xA2);
	  test[0]=SPI1_Read8();
	  test[1]=SPI1_Read8();
	  MS5611.C1 = (test[0]<<8) + (test[1]);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);

	  SPI1_Write8(0xA4);
	  test[0]=SPI1_Read8();
	  test[1]=SPI1_Read8();
	  MS5611.C2 = (test[0]<<8) + (test[1]);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);

	  SPI1_Write8(0xA6);
	  test[0]=SPI1_Read8();
	  test[1]=SPI1_Read8();
	  MS5611.C3 = (test[0]<<8) + (test[1]);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);

	  SPI1_Write8(0xA8);
	  test[0]=SPI1_Read8();
	  test[1]=SPI1_Read8();
	  MS5611.C4 = (test[0]<<8) + (test[1]);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);

	  SPI1_Write8(0xAA);
	  test[0]=SPI1_Read8();
	  test[1]=SPI1_Read8();
	  MS5611.C5 = (test[0]<<8) + (test[1]);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);

	  SPI1_Write8(0xAC);
	  test[0]=SPI1_Read8();
	  test[1]=SPI1_Read8();
	  MS5611.C6 = (test[0]<<8) + (test[1]);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);

	  SPI1_Write8(0xAE);
	  test[0]=SPI1_Read8();
	  test[1]=SPI1_Read8();
	  MS5611.C7 = (test[0]<<8) + (test[1]);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);





}

void MS5611_D_Convertion()
{
	uint8_t test[3] = {0};
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);

	  SPI1_Write8(0x48);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
	  HAL_Delay(11);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);
	  SPI1_Write8(0x00);
	  test[0]=SPI1_Read8();
	  test[1]=SPI1_Read8();
	  test[2]=SPI1_Read8();
	  MS5611.D1 = ((test[0]<<16) + (test[1]<<8) + ( test[2]) );
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);

	  SPI1_Write8(0x58);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
	  HAL_Delay(11);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RESET);
	  SPI1_Write8(0x00);
	  test[0]=SPI1_Read8();
	  test[1]=SPI1_Read8();
	  test[2]=SPI1_Read8();
	  MS5611.D2 = ((test[0]<<16) + (test[1]<<8) + ( test[2]) );
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, SET);
}
void MS5611_calc(void)
{
	MS5611.dT = MS5611.D2 - ((uint32_t)MS5611.C5 * 256);
	MS5611.TEMP = 2000 + (((int64_t)MS5611.dT * MS5611.C6) / 8388608);
	Temp = MS5611.TEMP/100.0;

	MS5611.OFF = (int64_t)MS5611.C2*65535 + (int64_t)MS5611.C4*MS5611.dT/128;	//5611
	//MS5611.OFF = (int64_t)MS5611.C2*131072 + (int64_t)MS5611.C4*MS5611.dT/64;	//5607
	MS5611.SENS = (int64_t)MS5611.C1*32768 + (int64_t)MS5611.C3*MS5611.dT/256;	//5611
	//MS5611.SENS = (int64_t)MS5611.C1*65536 + (int64_t)MS5611.C3*MS5611.dT/128;	//5607
	MS5611.P = (MS5611.D1*MS5611.SENS/2097152 - MS5611.OFF)/32768;
	Pressure = MS5611.P/100.0;
	Altitude_meas = (1 - pow((Pressure / 1013.25),0.190284)) * 44191.4;

}
//
// 8-bit veri yazma
void SPI1_Write8(uint8_t data) {
    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
        ;
    LL_SPI_TransmitData8(SPI1, data);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
        ;
    LL_SPI_ReceiveData8(SPI1);
}

// 8-bit veri okuma
uint8_t SPI1_Read8(void) {
    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
        ;
    LL_SPI_TransmitData8(SPI1, 0xFF);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
        ;
    return LL_SPI_ReceiveData8(SPI1);
}

// 16-bit veri okuma
uint16_t SPI1_Read16(void) {
    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
        ;
    LL_SPI_TransmitData16(SPI1, 0xFFFF);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
        ;
    return LL_SPI_ReceiveData16(SPI1);
}

// 32-bit veri okuma
uint32_t SPI1_Read32(void) {
    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
        ;
    LL_SPI_TransmitData16(SPI1, 0xFFFF); // Yüksek 16 bit için dummy data
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
        ;
    uint16_t data_high = LL_SPI_ReceiveData16(SPI1);

    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
        ;
    LL_SPI_TransmitData16(SPI1, 0xFFFF); // Düşük 16 bit için dummy data
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
        ;
    uint16_t data_low = LL_SPI_ReceiveData16(SPI1);

    return ((uint32_t)data_high << 16) | data_low;
}

uint32_t map( long A , long B , long C , long D, long E)
{
	return ( A - B) * (E - D ) / (C - B )+ D ;
}

void Buzzer(){
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
	  HAL_Delay(300);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
	  HAL_Delay(150);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
	  HAL_Delay(250);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
}

int _write(int file , char*p, int len)
{
//	for(int i = 0; i <len; i++)
//	{
//		//HAL_UART_Transmit(huart, *(p+i), Size, Timeout)
//	}


	return len;
}

void Calibrate_ESC()
{
	TIM4->CCR1 = 21000;
	TIM4->CCR2 = 21000;
	TIM4->CCR3 = 21000;
	TIM4->CCR4 = 21000;
	HAL_Delay(7000);
	TIM4->CCR1 = 10500;
	TIM4->CCR2 = 10500;
	TIM4->CCR3 = 10500;
	TIM4->CCR4 = 10500;
	HAL_Delay(8000);

}


void send()
{

	 Tx_Buff[0] = 0x0;
	 Tx_Buff[1] = 0x1;
	 Tx_Buff[2] = 0x17;

	 converter.f =BNO080_Yaw ;

	 Tx_Buff[3]=converter.b[0];
	 Tx_Buff[4]=converter.b[1];
	 Tx_Buff[5]=converter.b[2];
	 Tx_Buff[6]=converter.b[3];

	 converter.f =BNO080_Roll ;

	 Tx_Buff[7]=converter.b[0];
	 Tx_Buff[8]=converter.b[1];
	 Tx_Buff[9]=converter.b[2];
	 Tx_Buff[10]=converter.b[3];

	 converter.f =BNO080_Pitch ;

	 Tx_Buff[11]=converter.b[0];
	 Tx_Buff[12]=converter.b[1];
	 Tx_Buff[13]=converter.b[2];
	 Tx_Buff[14]=converter.b[3];

	 converter.f = gps.latitude ;

	 Tx_Buff[15]=converter.b[0];
	 Tx_Buff[16]=converter.b[1];
	 Tx_Buff[17]=converter.b[2];
	 Tx_Buff[18]=converter.b[3];

	 converter.f = gps.longitude;

	 Tx_Buff[19]=converter.b[0];
	 Tx_Buff[20]=converter.b[1];
	 Tx_Buff[21]=converter.b[2];
	 Tx_Buff[22]=converter.b[3];

	 converter.f = 31.1;

	Tx_Buff[23]=converter.b[0];
	Tx_Buff[24]=converter.b[1];
	Tx_Buff[25]=converter.b[2];
	Tx_Buff[26]=converter.b[3];
	 Tx_Buff[27]='\n';



	//HAL_UART_Transmit(&huart6, &Low_Addr, 1, 100);
	//HAL_UART_Transmit(&huart6, &High_Addr, 1, 100);
	//HAL_UART_Transmit(&huart6, &CH, 1, 100);
	HAL_UART_Transmit_IT(&huart6, Tx_Buff, sizeof(Tx_Buff));

	//HAL_UART_Transmit_IT(&huart6, Tx_Buff, 11);

}

unsigned char iBus_isActiveFailsafe(FSiA6B_iBus* iBus)
{
	return iBus->FailSafe != 0;
}


void PID_Coef_Config(void)
{



	  roll.in.kd = 0.15;
	  roll.in.kp = 2;
	  roll.in.ki = 0.3;

	  roll.out.kd = 4.5;
	  roll.out.kp = 15;
	  roll.out.ki =3.5;

	  pitch.in.kd = 0.15;
	  pitch.in.kp = 2;
	  pitch.in.ki = 0.3;

	  pitch.out.kd = 4.5;
	  pitch.out.kp = 15;
	  pitch.out.ki = 3.5;

	//
	  yaw_heading.kd = 15.0;
	  yaw_heading.kp = 30;
	  yaw_heading.ki =0.0001;

	  yaw_rate.kd = 0.05;
	  yaw_rate.kp = 4;
	  yaw_rate.ki = 0.0;

	  altitude.kd = 0.01;
	  altitude.kp = 3;
	  altitude.ki = 0.001;
}

void PID_CONFIG_LORA()
{
	converter.b[0] = rx_buffer_PID[0];
	converter.b[1] = rx_buffer_PID[1];
	converter.b[2] = rx_buffer_PID[2];
	converter.b[3] = rx_buffer_PID[3];
	roll.in.kd = converter.f;

	converter.b[0] = rx_buffer_PID[4];
	converter.b[1] = rx_buffer_PID[5];
	converter.b[2] = rx_buffer_PID[6];
	converter.b[3] = rx_buffer_PID[7];
	roll.in.ki = converter.f;

	converter.b[0] = rx_buffer_PID[8];
	converter.b[1] = rx_buffer_PID[9];
	converter.b[2] = rx_buffer_PID[10];
	converter.b[3] = rx_buffer_PID[11];
	roll.in.kp = converter.f;

	converter.b[0] = rx_buffer_PID[12];
	converter.b[1] = rx_buffer_PID[13];
	converter.b[2] = rx_buffer_PID[14];
	converter.b[3] = rx_buffer_PID[15];
	roll.out.kd = converter.f;

	converter.b[0] = rx_buffer_PID[16];
	converter.b[1] = rx_buffer_PID[17];
	converter.b[2] = rx_buffer_PID[18];
	converter.b[3] = rx_buffer_PID[19];
	roll.out.ki = converter.f;

	converter.b[0] = rx_buffer_PID[20];
	converter.b[1] = rx_buffer_PID[21];
	converter.b[2] = rx_buffer_PID[22];
	converter.b[3] = rx_buffer_PID[23];
	roll.out.kp = converter.f;

	converter.b[0] = rx_buffer_PID[24];
	converter.b[1] = rx_buffer_PID[25];
	converter.b[2] = rx_buffer_PID[26];
	converter.b[3] = rx_buffer_PID[27];
	pitch.in.kd = converter.f;

	converter.b[0] = rx_buffer_PID[28];
	converter.b[1] = rx_buffer_PID[29];
	converter.b[2] = rx_buffer_PID[30];
	converter.b[3] = rx_buffer_PID[31];
	pitch.in.ki = converter.f;

	converter.b[0] = rx_buffer_PID[32];
	converter.b[1] = rx_buffer_PID[33];
	converter.b[2] = rx_buffer_PID[34];
	converter.b[3] = rx_buffer_PID[35];
	pitch.in.kp = converter.f;

	converter.b[0] = rx_buffer_PID[36];
	converter.b[1] = rx_buffer_PID[37];
	converter.b[2] = rx_buffer_PID[38];
	converter.b[3] = rx_buffer_PID[39];
	pitch.out.kd = converter.f;

	converter.b[0] = rx_buffer_PID[40];
	converter.b[1] = rx_buffer_PID[41];
	converter.b[2] = rx_buffer_PID[42];
	converter.b[3] = rx_buffer_PID[43];
	pitch.out.ki = converter.f;

	converter.b[0] = rx_buffer_PID[44];
	converter.b[1] = rx_buffer_PID[45];
	converter.b[2] = rx_buffer_PID[46];
	converter.b[3] = rx_buffer_PID[47];
	pitch.out.kp = converter.f;

	converter.b[0] = rx_buffer_PID[48];
	converter.b[1] = rx_buffer_PID[49];
	converter.b[2] = rx_buffer_PID[50];
	converter.b[3] = rx_buffer_PID[51];
	yaw_heading.kd = converter.f;

	converter.b[0] = rx_buffer_PID[52];
	converter.b[1] = rx_buffer_PID[53];
	converter.b[2] = rx_buffer_PID[54];
	converter.b[3] = rx_buffer_PID[55];
	yaw_heading.ki = converter.f;

	converter.b[0] = rx_buffer_PID[56];
	converter.b[1] = rx_buffer_PID[57];
	converter.b[2] = rx_buffer_PID[58];
	converter.b[3] = rx_buffer_PID[59];
	yaw_heading.kp = converter.f;

	converter.b[0] = rx_buffer_PID[60];
	converter.b[1] = rx_buffer_PID[61];
	converter.b[2] = rx_buffer_PID[62];
	converter.b[3] = rx_buffer_PID[63];
	yaw_rate.kd = converter.f;

	converter.b[0] = rx_buffer_PID[64];
	converter.b[1] = rx_buffer_PID[65];
	converter.b[2] = rx_buffer_PID[66];
	converter.b[3] = rx_buffer_PID[67];
	yaw_rate.ki = converter.f;

	converter.b[0] = rx_buffer_PID[68];
	converter.b[1] = rx_buffer_PID[69];
	converter.b[2] = rx_buffer_PID[70];
	converter.b[3] = rx_buffer_PID[71];
	yaw_rate.kp = converter.f;


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
