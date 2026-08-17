/**
  ******************************************************************************
  * @file    robot_debug.h
  * @brief   Header file for Robot Debug Module
  ******************************************************************************
  */
#ifndef __ROBOT_DEBUG_H__
#define __ROBOT_DEBUG_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
/* Defines -------------------------------------------------------------------*/
#define DEBUG_RING_BUF_SIZE 1024
/* Struct Definition ---------------------------------------------------------*/
typedef struct {
    uint32_t timestamp;
    float roll;
    float gyro_x;
    float rpm_l;
    float rpm_r;
    float pwm_l;
    float pwm_r;

    // PID Parameters
    float angle_setpoint;
    float angle_feedback;
    float angle_error;
    float p_term;
    float i_term;
    float d_term;
    float angle_output;        // PID_Output

    // State Variables
    float angle_offset;
    float loop_time_ms;

    // Encoder Recovery Controller
    float average_rpm;
    float delta_speed;
    float recovery_angle;
	int8_t recovery_state;
    float base_balance_angle;
    float balance_angle;
} RobotDebug_Data_t;
/* Ring Buffer Structure -----------------------------------------------------*/
typedef struct {
    char buffer[DEBUG_RING_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
} RingBuffer_t;
/* Function Prototypes -------------------------------------------------------*/
void RobotDebug_Init(UART_HandleTypeDef *huart);
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
	int8_t RecoveryState,
    float base_balance_angle,
    float balance_angle
);
void RobotDebug_ProcessUART(void);
#ifdef __cplusplus
}
#endif
#endif /* __ROBOT_DEBUG_H__ */