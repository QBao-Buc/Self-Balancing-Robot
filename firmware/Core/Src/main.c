/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - STM32 Self Balancing Robot
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Bổ sung Module Debug */
#include "robot_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    
    float Ts;
    
    float setpoint;
    float feedback;
    float output;
    
    float error;
    float last_error;
    
    float integral;
    float derivative;

    float P_term;
    float I_term;
    float D_term;
    
    float outMax;
    float outMin;
    
    float iMax;
    float iMin;
    
    float *gyro;      // NULL nếu không dùng gyro
} PID_t;

typedef enum {
    MODE_MANUAL_PWM = 0,
    MODE_PID_SPEED  = 1
} ControlMode_t;

typedef enum {
    PRINT_OFF = 0,
    PRINT_MONITOR,   
    PRINT_PLOT,      
    PRINT_IMU,       
    PRINT_PLOTIMU    
} PrintMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONSTRAIN(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/* ===== STATE-BASED RECOVERY CONTROLLER CONFIGURATION =====
 * Recovery là một thuật toán Linear State Feedback không dùng tích lũy:
 * RecoveryAngle = K_SPEED * Speed + K_ACCEL * Acceleration
 */
#define RECOVERY_SPEED_GAIN     0.015f   // Hệ số nhân cho Vận tốc (RPM)
#define RECOVERY_ACCEL_GAIN     0.0008f  // Hệ số nhân cho Gia tốc (RPM/s)
#define RECOVERY_ANGLE_GAIN     0.08f      // mới
#define RECOVERY_GYRO_GAIN      0.015f     // mới
#define ACC_FILTER_ALPHA        0.15f    // LPF coefficient cho gia tốc
#define MAX_RECOVERY_ANGLE      6.0f     // deg - giới hạn góc bù tối đa (default = 2)

#define SPEED_PID_ENABLE        0

#define SPEED_FILTER_ENABLE     1
#define SPEED_FILTER_ALPHA      0.12f

#define SPEED_DEADBAND_ENABLE   1
#define SPEED_DEADBAND_RPM      5.0f

#define SPEED_PID_PERIOD        5

#define SPEED_PID_KP            0.025f
#define SPEED_PID_KI            0.0010f
#define SPEED_PID_KD            0.0f

#define SPEED_PID_OUT_MAX       2.0f
#define SPEED_PID_OUT_MIN      -2.0f

#define SPEED_PID_IMAX          4.0f
#define SPEED_PID_IMIN         -4.0f

#define SPEED_PID_ENABLE_ANGLE  2.0f
#define ANGLE_OFFSET_SLEW       0.05f

#define ENCODER_CPR     1320.0f
#define SAMPLE_TIME     0.01f
#define RPM_FACTOR      (60.0f / (ENCODER_CPR * SAMPLE_TIME))
#define MPU6050_ADDR    (0x68 << 1)
#define DT              0.01f
#define M_PI            3.14159265358979323846
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Control & Mode Flags
volatile ControlMode_t control_mode = MODE_MANUAL_PWM; 
volatile PrintMode_t print_mode = PRINT_OFF;
volatile uint8_t print_flag = 0; 
uint8_t Enable_SpeedPID = SPEED_PID_ENABLE;

int manual_pwm = 0;
volatile int32_t encoder_position = 0;
volatile int16_t encoder_now = 0;
volatile int16_t encoder_old = 0;
volatile int16_t encoder_delta = 0;

// Các biến lưu tốc độ riêng biệt phục vụ Debug / Serial Plotter
volatile float encoder_rpm = 0.0f;
volatile float speed_raw = 0.0f;
volatile float speed_filtered = 0.0f;
volatile float speed_feedback = 0.0f;
volatile float speed_angle = 0.0f;      // Ngõ ra Speed PID (chính là Angle Offset)
volatile float target_speed = 0.0f;      // Sẵn sàng nhận lệnh tiến/lùi Bluetooth sau này

volatile uint32_t timer_cnt = 0;
volatile uint8_t pid_flag = 0;
volatile float pid_pwm;

// UART CLI Variables
volatile uint8_t rxChar;
char rxBuffer[32];
uint8_t rxIndex = 0;
volatile uint8_t cmdReady = 0;

PID_t Speed_PID;
PID_t Angle_PID;

// MPU Variables
HAL_StatusTypeDef mpuStatus;
uint8_t who_am_i;
uint8_t data;

uint8_t mpuData[16];

int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;
int16_t Temp;

float Ax, Ay, Az;
float Gx, Gy, Gz;
float AccAngle = 0.0f;

float PitchAcc = 0.0f;
float Roll = 0.0f;
float RollAcc = 0.0f;
float RollGyro = 0.0f;
float GxOffset = 0.0f;

float Roll_Setpoint = 0.0f;
float BaseBalanceAngle = -0.25f;   // Góc cân bằng cơ khí (hằng số nền, không đổi bởi Recovery)
float BalanceAngle = 0.0f;         // = BaseBalanceAngle + RecoveryAngle, tính lại mỗi vòng lặp

float AngleOffset = 0.0f;

/* ===== STATE-BASED RECOVERY CONTROLLER VARIABLES ===== */
float LeftRPM = 0.0f;
float RightRPM = 0.0f;
float AverageRPM = 0.0f;

float Speed = 0.0f;
float LastSpeed = 0.0f;
float Acceleration = 0.0f;
float acc_filtered = 0.0f;

float RecoveryAngle = 0.0f;
int8_t RecoveryState = 0;
float AngleError = 0.0f;
float AngularVelocity = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Motor_SetPWM(int left, int right);
void Motor_Left(int pwm);
void Motor_Right(int pwm);
void Encoder_Update(void);
void PID_Init(PID_t *pid);
float PID_Update(PID_t *pid);
void Process_Command(void);
void Process_Print(void);
void MPU6050_GyroCalibration(void);
void MPU6050_Read_Angle(void);
void Control_Loop(void);

// Modules độc lập cho Speed Control
void Speed_Filter(void);
void Speed_Deadband(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Module LPF Bậc 1 lọc nhiễu vận tốc */
void Speed_Filter(void)
{
#if SPEED_FILTER_ENABLE
    speed_filtered += SPEED_FILTER_ALPHA * (speed_raw - speed_filtered);
#else
    speed_filtered = speed_raw;
#endif
}

/* Module Deadband triệt tiêu rung lắc nhỏ khi đứng yên */
void Speed_Deadband(void)
{
#if SPEED_DEADBAND_ENABLE
    if (fabsf(speed_filtered) < SPEED_DEADBAND_RPM)
    {
        speed_feedback = 0.0f;
    }
    else
    {
        speed_feedback = speed_filtered;
    }
#else
    speed_feedback = speed_filtered;
#endif
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Khởi động PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Kích hoạt TB6612

  // Khởi động Encoder TIM3
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  encoder_old = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

  // ===== Cấu hình Speed PID =====
  Speed_PID.Kp = SPEED_PID_KP;
  Speed_PID.Ki = SPEED_PID_KI;
  Speed_PID.Kd = SPEED_PID_KD;
  Speed_PID.Ts = (float)SPEED_PID_PERIOD * SAMPLE_TIME; // 5 * 0.01s = 0.05s (50ms)

  Speed_PID.outMax = SPEED_PID_OUT_MAX;
  Speed_PID.outMin = SPEED_PID_OUT_MIN;
  Speed_PID.iMax   = SPEED_PID_IMAX;
  Speed_PID.iMin   = SPEED_PID_IMIN;

  PID_Init(&Speed_PID);
  Speed_PID.gyro = NULL;

  // ===== Cấu hình Angle PID =====
  Angle_PID.Kp = 520.0f;
  Angle_PID.Ki = 20.0f;
  Angle_PID.Kd = 12.0f;
  Angle_PID.Ts = DT;
  
  Angle_PID.outMax = 3590.0f; 
  Angle_PID.outMin = -3590.0f;
  Angle_PID.iMax   = 1300.0f;
  Angle_PID.iMin   = -1300.0f;
  
  PID_Init(&Angle_PID);
  Angle_PID.gyro = &Gx;

  // Wake up MPU6050
  data = 0x01; 
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  HAL_Delay(200);

  // Calibrate Gyro
  MPU6050_GyroCalibration();

  // Đọc góc Roll ban đầu
  if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, mpuData, 6, 100) == HAL_OK)
  {
      AccX = (int16_t)(mpuData[0] << 8 | mpuData[1]);
      AccY = (int16_t)(mpuData[2] << 8 | mpuData[3]);
      AccZ = (int16_t)(mpuData[4] << 8 | mpuData[5]);
      
      float ax = (float)AccX / 16384.0f;
      float ay = (float)AccY / 16384.0f;
      float az = (float)AccZ / 16384.0f;
      
      Roll = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / M_PI;
  }

  // Khởi tạo Module Robot Debug
  RobotDebug_Init(&huart1);

  // Khởi động Ngắt Timer 10ms
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  hi2c1.Init.ClockSpeed = 400000;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Motor_Left(int pwm)
{
    if(pwm > 3599) pwm = 3599;
    if(pwm < -3599) pwm = -3599;

    if(pwm >= 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -pwm);
    }
}

void Motor_Right(int pwm)
{
    if(pwm > 3599) pwm = 3599;
    if(pwm < -3599) pwm = -3599;

    if(pwm >= 0)   
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -pwm);
    }
}

void Motor_SetPWM(int left, int right)
{
    Motor_Left(left);
    Motor_Right(right);
}

void Encoder_Update(void)
{
    encoder_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    encoder_delta = encoder_now - encoder_old;
    encoder_old = encoder_now;

    encoder_position += encoder_delta;
    encoder_rpm = (float)encoder_delta * RPM_FACTOR;

    // Gán dữ liệu thô cho Speed Filter
    speed_raw = encoder_rpm;
}

void MPU6050_Read_Angle(void)
{
    if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, mpuData, 14, 2) == HAL_OK)
    {
        AccX  = (int16_t)(mpuData[0] << 8 | mpuData[1]);
        AccY  = (int16_t)(mpuData[2] << 8 | mpuData[3]);
        AccZ  = (int16_t)(mpuData[4] << 8 | mpuData[5]);
        GyroX = (int16_t)(mpuData[8] << 8 | mpuData[9]);

        Ax = (float)AccX / 16384.0f;
        Ay = (float)AccY / 16384.0f;
        Az = (float)AccZ / 16384.0f;
        
        Gx = ((float)GyroX / 131.0f) - GxOffset;

        RollAcc = atan2f(Ay, sqrtf(Ax * Ax + Az * Az)) * 180.0f / M_PI;
        Roll = 0.98f * (Roll + Gx * DT) + 0.02f * RollAcc;
    }
    else
    {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
    }
}

void Control_Loop(void)
{
    static uint32_t last_tick = 0;
    uint32_t current_tick = HAL_GetTick(); 
    float loop_time_ms = (float)(current_tick - last_tick);
    if (last_tick == 0) loop_time_ms = 10.0f;
    last_tick = current_tick;

    static uint8_t speed_div = 0;

    //==================================================
    // 1. Đọc cảm biến
    //==================================================
    MPU6050_Read_Angle();
    Encoder_Update();

    Speed_Filter();
    Speed_Deadband();

    //==================================================
    // 2. Speed PID (50 ms)
    //==================================================
    if (Enable_SpeedPID)
    {
        if (++speed_div >= SPEED_PID_PERIOD)
        {
            speed_div = 0;

            if (fabsf(Roll - BalanceAngle) < SPEED_PID_ENABLE_ANGLE)
            {
                Speed_PID.setpoint = target_speed;
                Speed_PID.feedback = speed_feedback;

                speed_angle = PID_Update(&Speed_PID);
            }
            else
            {
                speed_angle = 0.0f;
                Speed_PID.integral = 0.0f;
                Speed_PID.last_error = 0.0f;
            }
        }
    }
    else
    {
        speed_angle = 0.0f;
        Speed_PID.integral = 0.0f;
        Speed_PID.last_error = 0.0f;
    }

    //==================================================
    // 3. Slew Rate Limit cho AngleOffset
    //==================================================
    {
        float delta = speed_angle - AngleOffset;

        if (delta > ANGLE_OFFSET_SLEW)
            delta = ANGLE_OFFSET_SLEW;
        else if (delta < -ANGLE_OFFSET_SLEW)
            delta = -ANGLE_OFFSET_SLEW;

        AngleOffset += delta;
    }

    //==================================================
    // 3.5. STATE-BASED RECOVERY CONTROLLER
    //      Linear State Feedback: RecoveryAngle = K_SPEED * Speed + K_ACCEL * Acceleration
    //==================================================
    LeftRPM = encoder_rpm;
    RightRPM = encoder_rpm;
    AverageRPM = (LeftRPM + RightRPM) * 0.5f;

    // Trạng thái vận tốc hiện tại
    Speed = encoder_rpm;

    // Tính gia tốc thô (RPM/s)
    Acceleration = (Speed - LastSpeed) / DT;
    LastSpeed = Speed;

    // Lọc gia tốc bằng Low-Pass Filter để triệt tiêu nhiễu từ encoder
    acc_filtered += ACC_FILTER_ALPHA * (Acceleration - acc_filtered);

	AngularVelocity = Gx;
	// Tính trực tiếp RecoveryAngle theo trạng thái hiện tại (không tích lũy)
    RecoveryAngle =
      RECOVERY_SPEED_GAIN * Speed
    + RECOVERY_ACCEL_GAIN * acc_filtered
    + RECOVERY_GYRO_GAIN * AngularVelocity;
	
    // Giới hạn biên độ RecoveryAngle
    RecoveryAngle = CONSTRAIN(RecoveryAngle, -MAX_RECOVERY_ANGLE, MAX_RECOVERY_ANGLE);

    // Cập nhật trạng thái cho RobotDebug
    if (RecoveryAngle > 0.05f)
        RecoveryState = 1;
    else if (RecoveryAngle < -0.05f)
        RecoveryState = -1;
    else
        RecoveryState = 0;

    // Cập nhật góc cân bằng mục tiêu cho Angle PID
    BalanceAngle = BaseBalanceAngle + RecoveryAngle;

    //==================================================
    // 4. Angle PID (10 ms)
    //==================================================
    Angle_PID.setpoint = BalanceAngle + AngleOffset;
    Angle_PID.feedback = Roll;

    pid_pwm = PID_Update(&Angle_PID);

    //==================================================
    // 5. Safety & Output
    //==================================================
    if (fabsf(Roll) > 45.0f)
    {
        Motor_SetPWM(0, 0);

        PID_Init(&Angle_PID);
        PID_Init(&Speed_PID);

        AngleOffset = 0.0f;
        speed_angle = 0.0f;
        speed_filtered = 0.0f;
        speed_feedback = 0.0f;

        // Reset trạng thái Recovery khi xe đổ
        RecoveryAngle = 0.0f;
        Speed = 0.0f;
        LastSpeed = 0.0f;
        Acceleration = 0.0f;
        acc_filtered = 0.0f;
        RecoveryState = 0;
    }
    else
    {
        Motor_SetPWM((int)pid_pwm, (int)pid_pwm);
    }

    //==================================================
    // 6. DEBUG MODULE UPDATE
    //==================================================
    RobotDebug_Update(
        HAL_GetTick(),             // Thời gian (ms)
        Roll,                      // Góc Roll
        Gx,                        // Gx đã được trừ offset
        LeftRPM,                   // LeftRPM
        RightRPM,                  // RightRPM
        pid_pwm,                   // PWM thực tế bánh trái
        pid_pwm,                   // PWM thực tế bánh phải
        Angle_PID.setpoint,        // Setpoint góc (BalanceAngle + AngleOffset)
        Angle_PID.feedback,        // Pitch/Roll feedback
        Angle_PID.error,           // Sai số góc
        Angle_PID.P_term,          // Tỉ lệ P
        Angle_PID.I_term,          // Tích phân I
        Angle_PID.D_term,          // Vi phân D
        Angle_PID.output,          // PID_Output
        AngleOffset,               // Biến AngleOffset từ Speed PID
        loop_time_ms,              // Thời gian thi hành vòng lặp (ms)
        AverageRPM,                // AverageRPM
        Acceleration,              // Gia tốc (thay thế cho DeltaSpeed cũ)
        RecoveryAngle,             // RecoveryAngle
        RecoveryState,             // Flag recovery mode
        BaseBalanceAngle,          // BaseBalanceAngle
        BalanceAngle               // BalanceAngle
    );
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
        timer_cnt++; 
        pid_flag = 1;
    }
}

void PID_Init(PID_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->P_term = 0.0f;
    pid->I_term = 0.0f;
    pid->D_term = 0.0f;
    pid->output = 0.0f;
    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
}

float PID_Update(PID_t *pid)
{
    pid->error = pid->setpoint - pid->feedback;
    
    // Tích phân
    pid->integral += pid->error * pid->Ts;
    if(pid->integral > pid->iMax) pid->integral = pid->iMax;
    if(pid->integral < pid->iMin) pid->integral = pid->iMin;

    // Vi phân
    if (pid->gyro != NULL)
    {
        // Angle PID dùng trực tiếp Gyro
        pid->derivative = -(*pid->gyro);
    }
    else
    {
        // Speed PID giữ nguyên
        pid->derivative = (pid->error - pid->last_error) / pid->Ts;
    }

    // Thành phần P, I, D
    pid->P_term = pid->Kp * pid->error;
    pid->I_term = pid->Ki * pid->integral;
    pid->D_term = pid->Kd * pid->derivative;

    // Ngõ ra
    pid->output = pid->P_term + pid->I_term + pid->D_term;

    if(pid->output > pid->outMax) pid->output = pid->outMax;
    if(pid->output < pid->outMin) pid->output = pid->outMin;

    pid->last_error = pid->error;

    return pid->output;
}

void Process_Print(void) {}
void Process_Command(void) {}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {}

void MPU6050_GyroCalibration(void)
{
    float sum = 0.0f;
    for(int i = 0; i < 500; i++)
    {
        if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, &mpuData[8], 2, 10) == HAL_OK)
        {
            int16_t raw_gx = (int16_t)(mpuData[8] << 8 | mpuData[9]);
            sum += ((float)raw_gx / 131.0f);
        }
        HAL_Delay(2);
    }
    GxOffset = sum / 500.0f;
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
