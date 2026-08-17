/**
  ******************************************************************************
  * @file    robot_debug.c
  * @brief   Implementation file for Robot Debug Module
  ******************************************************************************
  */

#include "robot_debug.h"

/* Private Variables ---------------------------------------------------------*/
static UART_HandleTypeDef *debug_huart = NULL;
static RobotDebug_Data_t debug_data;
static RingBuffer_t tx_ring_buf = { .head = 0, .tail = 0 };
static volatile uint8_t uart_tx_busy = 0;

/* Private Function Prototypes -----------------------------------------------*/
static void RingBuffer_Write(const char *data, uint16_t len);
static uint16_t RingBuffer_Read(char *data, uint16_t max_len);

/* Public Functions ----------------------------------------------------------*/

/**
  * @brief Khoi tao module debug voi UART truyen vao
  */
void RobotDebug_Init(UART_HandleTypeDef *huart)
{
    debug_huart = huart;
    tx_ring_buf.head = 0;
    tx_ring_buf.tail = 0;
    uart_tx_busy = 0;
    memset((void*)&debug_data, 0, sizeof(RobotDebug_Data_t));
}

/**
  * @brief Cap nhat tat ca cac thong so telemetry cua Robot
  */
void RobotDebug_Update(
    uint32_t tick,
    float roll,
    float gx,
    float rpm_l,
    float rpm_r,
    float pwm_l,
    float pwm_r,
    float setpoint,
    float feedback,
    float error,
    float p_term,
    float i_term,
    float d_term,
    float output,
    float angle_offset,
    float loop_time,
    float average_rpm,
    float delta_speed,
    float recovery_angle,
    int8_t recovery_state,
    float base_balance_angle,
    float balance_angle
)
{
    debug_data.timestamp       = tick;
    debug_data.roll            = roll;
    debug_data.gyro_x          = gx;
    debug_data.rpm_l           = rpm_l;
    debug_data.rpm_r           = rpm_r;
    debug_data.pwm_l           = pwm_l;
    debug_data.pwm_r           = pwm_r;
    debug_data.angle_setpoint  = setpoint;
    debug_data.angle_feedback  = feedback;
    debug_data.angle_error     = error;
    debug_data.p_term          = p_term;
    debug_data.i_term          = i_term;
    debug_data.d_term          = d_term;
    debug_data.angle_output    = output;
    debug_data.angle_offset    = angle_offset;
    debug_data.loop_time_ms    = loop_time;
    debug_data.average_rpm      = average_rpm;
    debug_data.delta_speed      = delta_speed;
    debug_data.recovery_angle   = recovery_angle;
	debug_data.recovery_state = recovery_state;
    debug_data.base_balance_angle = base_balance_angle;
    debug_data.balance_angle    = balance_angle;

    // Dong goi chuoi dinh dang Serial Plotter / CSV Debug (phan tach boi dau phay)
    // Thu tu: Roll,GyroX,Setpoint,P,I,D,PID_Output,AngleOffset,LoopTime,
    //         LeftRPM,RightRPM,AverageRPM,DeltaSpeed,RecoveryAngle,BaseBalanceAngle,BalanceAngle
    char msg_buf[256];
    int len = snprintf(msg_buf, sizeof(msg_buf),
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
		"%.2f,%.2f,%.2f,%.2f,%.3f,%d,%.2f,%.2f\r\n",
        debug_data.roll,
        debug_data.gyro_x,
        debug_data.angle_setpoint,
        debug_data.p_term,
        debug_data.i_term,
        debug_data.d_term,
        debug_data.angle_output,       // PID_Output
        debug_data.angle_offset,
        debug_data.loop_time_ms,
        debug_data.rpm_l,              // LeftRPM
        debug_data.rpm_r,              // RightRPM
        debug_data.average_rpm,        // AverageRPM
        debug_data.delta_speed,        // DeltaSpeed
        debug_data.recovery_angle,     // RecoveryAngle
		debug_data.recovery_state,     // Flag
        debug_data.base_balance_angle, // BaseBalanceAngle
        debug_data.balance_angle       // BalanceAngle
    );

    if (len > 0)
    {
        RingBuffer_Write(msg_buf, (uint16_t)len);
    }
}

/**
  * @brief Xu ly day du lieu tu Ring Buffer ra cong UART (goi trong while(1))
  */
void RobotDebug_ProcessUART(void)
{
    if (debug_huart == NULL) return;

    // Neu UART chua ban va bo dem co du lieu -> day ra UART DMA/IT hoac Polling
    if (!uart_tx_busy && (tx_ring_buf.head != tx_ring_buf.tail))
    {
        static char temp_tx[128];
        uint16_t len = RingBuffer_Read(temp_tx, sizeof(temp_tx));

        if (len > 0)
        {
            // Dung Blocking Mode an toan chong ghi de khi truyen du lieu toc do cao
            HAL_UART_Transmit(debug_huart, (uint8_t*)temp_tx, len, 10);
        }
    }
}

/* Private Functions ---------------------------------------------------------*/

static void RingBuffer_Write(const char *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t next_head = (tx_ring_buf.head + 1) % DEBUG_RING_BUF_SIZE;
        if (next_head != tx_ring_buf.tail) // Neu chua day bo dem
        {
            tx_ring_buf.buffer[tx_ring_buf.head] = data[i];
            tx_ring_buf.head = next_head;
        }
        else
        {
            break; // Tran bo dem, bo qua cac ky tu con lai
        }
    }
}

static uint16_t RingBuffer_Read(char *data, uint16_t max_len)
{
    uint16_t count = 0;
    while ((tx_ring_buf.head != tx_ring_buf.tail) && (count < max_len))
    {
        data[count++] = tx_ring_buf.buffer[tx_ring_buf.tail];
        tx_ring_buf.tail = (tx_ring_buf.tail + 1) % DEBUG_RING_BUF_SIZE;
    }
    return count;
}