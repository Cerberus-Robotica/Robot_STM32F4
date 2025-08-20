/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#pragma pack(push, 1)
typedef struct {
    uint8_t id;
    float Vx, Vy, Vang;
    uint8_t kicker;
} Pacote;
#pragma pack(pop)

const uint8_t id = 5;
volatile float vx = 0;
volatile float vy = 0;
volatile float vang = 0;
volatile uint8_t kicker = 0;


uint8_t addr[5] = {'A', 'S', 'U', 'R', 'T'};
uint8_t channel = 123;
uint8_t pld_size = sizeof(Pacote);
volatile uint8_t radio_timeout = 0;


const uint32_t ARR = 1000;
const float R = 0.0915;
const float Rr = 0.02;
const float velocidade_maxima_motor = 29.32153; // rads/segundo de 280rmp
const float velocidade_minima_motor = 0.6*velocidade_maxima_motor;
const float epsilon = 2;
const float a1 = 0.785398;   // 45°
const float a2 = 2.35619;  // 135°
const float a3 = 3.92699; // 225°
const float a4 = 5.49779;  // 315°





/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MOTOR1_CHANNEL TIM_CHANNEL_1
#define PWM_MOTOR1_TIMER &htim1

#define PWM_MOTOR2_CHANNEL TIM_CHANNEL_2
#define PWM_MOTOR2_TIMER &htim3

#define PWM_MOTOR3_CHANNEL TIM_CHANNEL_1
#define PWM_MOTOR3_TIMER &htim3

#define PWM_MOTOR4_CHANNEL TIM_CHANNEL_1
#define PWM_MOTOR4_TIMER &htim2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void radio_setup() {
	  ce_high();
	  HAL_Delay(5);
	  ce_low();

	  nrf24_init();

	  nrf24_auto_ack_all(auto_ack);
	  nrf24_en_ack_pld(disable);
	  nrf24_dpl(disable);

	  nrf24_tx_pwr(_0dbm);
	  nrf24_data_rate(_2mbps);
	  nrf24_set_channel(channel);
	  nrf24_set_addr_width(5);

	  nrf24_pipe_pld_size(0, pld_size);
	  nrf24_set_crc(en_crc, _1byte);

	  nrf24_auto_retr_delay(0);
	  nrf24_auto_retr_limit(5);

	  nrf24_open_tx_pipe(addr);
	  nrf24_open_rx_pipe(0, addr);

	  nrf24_listen();

	  ce_high();
	  HAL_Delay(5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM11)
    {
        if (radio_timeout == 0)
        {
            vx = 0;
            vy = 0;
            vang = 0;
            kicker = 0;
        }
        else
        {
            radio_timeout = 0; // Sinal de que rádio está funcionando
        }
    }
}




void acionar_motor(int motor, float dutycycle){
	TIM_HandleTypeDef *htim = NULL;
	uint32_t channel = 0;
	GPIO_TypeDef* GPIOx0 = NULL;
	uint16_t GPIO_Pinx0 = 0;
	GPIO_TypeDef* GPIOx1 = NULL;
	uint16_t GPIO_Pinx1 = 0;


	switch(motor){
	case 1:
		htim = PWM_MOTOR1_TIMER;
		channel = PWM_MOTOR1_CHANNEL;
		GPIOx0 = SENTIDO0_MOTOR1_GPIO_Port;
		GPIO_Pinx0 = SENTIDO0_MOTOR1_Pin;
		GPIOx1 = SENTIDO1_MOTOR1_GPIO_Port;
		GPIO_Pinx1 = SENTIDO1_MOTOR1_Pin;
		break;
	case 2:
		htim = PWM_MOTOR2_TIMER;
		channel = PWM_MOTOR2_CHANNEL;
		GPIOx0 = SENTIDO0_MOTOR2_GPIO_Port;
		GPIO_Pinx0 = SENTIDO0_MOTOR2_Pin;
		GPIOx1 = SENTIDO1_MOTOR2_GPIO_Port;
		GPIO_Pinx1 = SENTIDO1_MOTOR2_Pin;
			break;
	case 3:
		htim = PWM_MOTOR3_TIMER;
		channel = PWM_MOTOR3_CHANNEL;
		GPIOx0 = SENTIDO0_MOTOR3_GPIO_Port;
		GPIO_Pinx0 = SENTIDO0_MOTOR3_Pin;
		GPIOx1 = SENTIDO1_MOTOR3_GPIO_Port;
		GPIO_Pinx1 = SENTIDO1_MOTOR3_Pin;
			break;
	case 4:
		htim = PWM_MOTOR4_TIMER;
		channel = PWM_MOTOR4_CHANNEL;
		GPIOx0 = SENTIDO0_MOTOR4_GPIO_Port;
		GPIO_Pinx0 = SENTIDO0_MOTOR4_Pin;
		GPIOx1 = SENTIDO1_MOTOR4_GPIO_Port;
		GPIO_Pinx1 = SENTIDO1_MOTOR4_Pin;
			break;
	default:
		return;
	}
	int sentido = 0;
	if (dutycycle < 0){
		HAL_GPIO_WritePin(GPIOx0, GPIO_Pinx0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOx1, GPIO_Pinx1, GPIO_PIN_RESET);
		dutycycle = -dutycycle;
		sentido = -1;
	}
	else {
		HAL_GPIO_WritePin(GPIOx0, GPIO_Pinx0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOx1, GPIO_Pinx1, GPIO_PIN_SET);
		sentido = 1;
	}
	if (dutycycle == 0){
		HAL_GPIO_WritePin(GPIOx0, GPIO_Pinx0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOx1, GPIO_Pinx1, GPIO_PIN_RESET);
	}


	if (dutycycle > 100.0f){
		dutycycle = 100.0f;
	}

	uint32_t frequencia = (uint32_t)((dutycycle / 100.0f) * (ARR));
	__HAL_TIM_SET_COMPARE(htim, channel, frequencia);
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  //radio configuration
  radio_setup();
  HAL_TIM_Base_Start_IT(&htim11);
  //UART variables
  char received_char[1];
  received_char[0] = 'a';
  char msg[64];

  //Radio variables
  uint8_t rx_buffer[pld_size];
  Pacote pacote_recebido;

  //cinematics variables

  float J[4][3] = {
  	  	    {-sin(a1), cos(a1), R},
  	  	    {-sin(a2), cos(a2), R},
  	  	    {-sin(a3), cos(a3), R},
  	  	    {-sin(a4), cos(a4), R}
  	  	  };
  HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart2, (uint8_t*)"Iniciado!", strlen("Iniciado!"), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  pacote_recebido.Vx = 0;
	  pacote_recebido.Vy = 0;
	  pacote_recebido.Vang = 0;
	  pacote_recebido.id = -1;
	  pacote_recebido.kicker = 0;

	 if (HAL_UART_Receive(&huart2, received_char, 1, 10) == HAL_OK) {
	  	  	  HAL_UART_Transmit(&huart2, (uint8_t*)"Eco: ", strlen("Eco: "), 100);
	  	  	  HAL_UART_Transmit(&huart2, received_char, 1, 100);
	  	  	  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100); // Envia nova linha
	  	  }
	  if(nrf24_data_available()) {
		  	  nrf24_receive(rx_buffer, pld_size);
		  	  memcpy(&pacote_recebido, rx_buffer, sizeof(Pacote));
		  	  if(pacote_recebido.id == id){
		  		  snprintf(msg, sizeof(msg), "Radio: %d %.2f %.2f %.2f %d\r\n", pacote_recebido.id, pacote_recebido.Vx, pacote_recebido.Vy,pacote_recebido.Vang,pacote_recebido.kicker);
		  	  	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
		  	  	  vx = pacote_recebido.Vx;
		  	  	  vy = pacote_recebido.Vy;
		  	  	  vang = pacote_recebido.Vang;
		  	  	  kicker = pacote_recebido.kicker;
		  	  	  radio_timeout = 1;
		  	  	  HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_RESET);
		  	  } else {
		  		HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_SET);
		  		radio_timeout = 0;
		  	  }
	  } else {
		  radio_timeout = 0;
		  HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_SET);
	  }
	  if(kicker  > 8 && kicker < 12){
		  HAL_Delay(4000);
		  for(int i = 0; i<4; i++){
			  acionar_motor(i+1, 1000);
			  HAL_Delay(1000);
			  acionar_motor(i+1, 0);
			  HAL_Delay(1000);
			  acionar_motor(i+1, -1000);
			  HAL_Delay(1000);
			  acionar_motor(i+1, 0);
		  }
		  continue;
	  }
	  if(kicker  > 14 && kicker < 16){
	  		  HAL_Delay(4000);
	  		  for(int i = 0; i<4; i++){
	  			  acionar_motor(1, 1000);
	  			  HAL_Delay(1000);
	  			  acionar_motor(1, 0);
	  			  acionar_motor(2, 1000);
	  			  HAL_Delay(1000);
	  			  acionar_motor(2, 0);
	  			  acionar_motor(3, 1000);
	  			  HAL_Delay(1000);
	  			  acionar_motor(3, 0);
	  			  acionar_motor(4, 1000);
	  			  HAL_Delay(1000);
	  			  acionar_motor(4, 0);
	  		  }
	  		  continue;
	  	  }
	  if(kicker > 18 && kicker < 22){
		  	for(int i = 0; i<11; i++){
	  			  acionar_motor(1, i*10);
	  			  acionar_motor(2, i*10);
	  			  acionar_motor(3, i*10);
	  			  acionar_motor(4, i*10);
	  			  HAL_Delay(2000);
	  		}
	  		for(int i = 0; i<11; i++){
	  			  acionar_motor(1, -i*10);
	  			  acionar_motor(2, -i*10);
	  			  acionar_motor(3, -i*10);
	  			  acionar_motor(4, -i*10);
	  			  HAL_Delay(2000);
	  		}
	  		continue;
	  }

	    float velocidade_angular[4];
	    for (int i = 0; i < 4; i++) {
	        velocidade_angular[i] = (1.0f / Rr) * (J[i][0] * vx + J[i][1] * vy + J[i][2] * vang);
	    }

	    // 2. Normalizar pelo máximo permitido
	    float max_val = 0.0f;
	    for (int i = 0; i < 4; i++) {
	        float abs_val = fabs(velocidade_angular[i]);
	        if (abs_val > max_val) max_val = abs_val;
	    }
	    if (max_val > velocidade_maxima_motor) {
	        float escala = velocidade_maxima_motor / max_val;
	        for (int i = 0; i < 4; i++) {
	            velocidade_angular[i] *= escala;
	        }
	    }

	    // 3. Garantir mínimo para rodas em movimento
	    float min_val = 1e9f;
	    for (int i = 0; i < 4; i++) {
	        float abs_val = fabs(velocidade_angular[i]);
	        if (abs_val > epsilon && abs_val < min_val) {
	            min_val = abs_val;
	        }
	    }
	  if (min_val < velocidade_minima_motor && min_val > epsilon) {
	        float escala = velocidade_minima_motor / min_val;
	        for (int i = 0; i < 4; i++) {
	            if (fabs(velocidade_angular[i]) > epsilon) {
	                velocidade_angular[i] *= escala;
	                if (velocidade_angular[i] > velocidade_maxima_motor) velocidade_angular[i] = velocidade_maxima_motor;
	                if (velocidade_angular[i] < -velocidade_maxima_motor) velocidade_angular[i] = -velocidade_maxima_motor;
	            }
	        }
	    }


	  acionar_motor(1, 100.0f*velocidade_angular[0]/velocidade_maxima_motor);
	  acionar_motor(2, 100.0f*velocidade_angular[1]/velocidade_maxima_motor);
	  acionar_motor(3, 100.0f*velocidade_angular[2]/velocidade_maxima_motor);
	  acionar_motor(4, 100.0f*velocidade_angular[3]/velocidade_maxima_motor);

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 8399;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SENTIDO0_MOTOR3_Pin|SENTIDO1_MOTOR3_Pin|SENTIDO1_MOTOR2_Pin|SENTIDO0_MOTOR2_Pin
                          |CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SENTIDO0_MOTOR1_Pin|SENTIDO1_MOTOR1_Pin|SENTIDO1_MOTOR4_Pin|SENTIDO0_MOTOR4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_AZUL_Pin */
  GPIO_InitStruct.Pin = LED_AZUL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_AZUL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SENTIDO0_MOTOR3_Pin SENTIDO1_MOTOR3_Pin SENTIDO1_MOTOR2_Pin SENTIDO0_MOTOR2_Pin */
  GPIO_InitStruct.Pin = SENTIDO0_MOTOR3_Pin|SENTIDO1_MOTOR3_Pin|SENTIDO1_MOTOR2_Pin|SENTIDO0_MOTOR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SENTIDO0_MOTOR1_Pin SENTIDO1_MOTOR1_Pin SENTIDO1_MOTOR4_Pin SENTIDO0_MOTOR4_Pin */
  GPIO_InitStruct.Pin = SENTIDO0_MOTOR1_Pin|SENTIDO1_MOTOR1_Pin|SENTIDO1_MOTOR4_Pin|SENTIDO0_MOTOR4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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
