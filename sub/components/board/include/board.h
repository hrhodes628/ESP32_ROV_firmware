#pragma once
#include "driver/gpio.h"
#include "driver/uart.h"

//IBUS pin numbers
#define IBUS_UART UART_NUM_2
#define IBUS_TXD_PIN GPIO_NUM_27
#define IBUS_RXD_PIN GPIO_NUM_14
#define IBUS_BAUD 115200

//mavlink pin numbers
#define MAVLINK_UART UART_NUM_1
#define MAVLINK_TX_PIN GPIO_NUM_17
#define MAVLINK_BUAD 57600

//sensor I2C port
#define I2C_SCL_PIN GPIO_NUM_20
#define I2C_SDA_PIN GPIO_NUM_21

#define BOARD_VOLTAGE_ADC_UNIT    ADC_UNIT_1
#define BOARD_VOLTAGE_ADC_CH      ADC_CHANNEL_6
#define BOARD_VOLTAGE_DIVIDER_RATIO 0.25

//motor pin numbers
#define MOTOR_FL_PIN GPIO_NUM_10
#define MOTOR_FR_PIN GPIO_NUM_11
#define MOTOR_RR_PIN GPIO_NUM_12
#define MOTOR_RL_PIN GPIO_NUM_13
#define MOTOR_VERT_PIN GPIO_NUM_15
#define SERVO_TILT_PIN GPIO_NUM_18
#define LED_PANEL_PIN GPIO_NUM_19