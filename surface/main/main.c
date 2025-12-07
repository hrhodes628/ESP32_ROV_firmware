/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_mac.h"

static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_16)
#define RXD_PIN (GPIO_NUM_21)
#define UART UART_NUM_2
StreamBufferHandle_t stream_1 = NULL;

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART, RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void tx_task(void* arg)
{
    uint8_t data[2] = {0};
    uint8_t pack[3] = {0};
    while (1) {
        xStreamBufferReceive(stream_1, &data, 2, 10000);
        printf("from rx task:");
        for(int i=0; i<sizeof(data);i++){
            printf("%x", data[i]);
        }
        printf("\n");
        pack[0]=0x44;

        pack[1]=data[0];
        pack[2]=data[1];
        uart_write_bytes(UART, pack , 3);
        vTaskDelay(100);
    }
    // free(data);
}

void term_read(void* arg){

    uint16_t var=0;
    uint8_t buf[2]= {0};
    while(1){
        printf("\n");
        fflush(stdout);
        vTaskDelay(1000);
        scanf("%hu", &var);
        printf("from reader %hu\n", var);

        buf[0] = var & 0xFF;
        buf[1] = var >> 8;

        xStreamBufferSend(stream_1,(void*) &buf, 2, 1000);
    }
}


void app_main(void)
{
    init();
    stream_1 = xStreamBufferCreate(16,4);
    xTaskCreate(tx_task, "uart_tx_task", 2048, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(term_read, "terminal reader", 2048, NULL, configMAX_PRIORITIES - 2, NULL);
}
