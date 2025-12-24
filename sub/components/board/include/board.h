#pragma once
#include "driver/gpio.h"
#include "driver/uart.h"

//IBUS pin numbers
#define IBUS_UART UART_NUM_2
#define IBUS_TXD_PIN GPIO_NUM_3
#define IBUS_RXD_PIN GPIO_NUM_4
#define IBUS_BAUD 115200

//mavlink pin numbers
#define MAVLINK_UART UART_NUM_1
#define MAVLINK_TX_PIN GPIO_NUM_1
#define MAVLINK_BUAD 57600

//sensor I2C port
#define I2C_SCL_PIN GPIO_NUM_7
#define I2C_SDA_PIN GPIO_NUM_6

#define BOARD_VOLTAGE_ADC_UNIT    ADC_UNIT_1
#define BOARD_VOLTAGE_ADC_CH      ADC_CHANNEL_4
#define BOARD_VOLTAGE_DIVIDER_RATIO 0.25

//motor pin numbers
#define MOTOR_FL_PIN GPIO_NUM_14
#define MOTOR_FR_PIN GPIO_NUM_13
#define MOTOR_RR_PIN GPIO_NUM_12
#define MOTOR_RL_PIN GPIO_NUM_9
#define MOTOR_VERT_PIN GPIO_NUM_10
#define SERVO_TILT_PIN GPIO_NUM_8
#define LED_PANEL_PIN GPIO_NUM_5
