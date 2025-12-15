#pragma once
#include "driver/gpio.h"

//IBUS pin numbers
#define IBUS_UART UART_NUM_2
#define IBUS_TXD_PIN GPIO_NUM_27
#define IBUS_RXD_PIN GPIO_NUM_14

//mavlink pin numbers
#define MAVLINK_UART UART_NUM_1
#define MAVLINK_TX_PIN GPIO_NUM_17

//motor pin numbers
#define M1 GPIO_NUM_10
#define M2 GPIO_NUM_11
#define M3 GPIO_NUM_12
#define M4 GPIO_NUM_13
#define M5 GPIO_NUM_15
#define SERV GPIO_NUM_18
#define LED GPIO_NUM_19