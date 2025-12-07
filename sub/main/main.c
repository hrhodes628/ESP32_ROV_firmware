#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_mac.h"

static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define UART UART_NUM_2
static StreamBufferHandle_t stream_1 = NULL;

void init(void)
{
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

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

void rx_task(void* arg){

    int* data = (int*) malloc(16);
    while (1) {
        int rxBytes = uart_read_bytes(UART, data, 4, 10000);
        printf("from uart reciever %d\n", *data);
        fflush(stdout);
        if (rxBytes > 0) {
           data[rxBytes] = 0;
           xStreamBufferSend(stream_1, data, 4, 100);
        }
    }
    free(data);

}

void led_task(void* arg){

    int* data = (int*) malloc(16);

    while(true){

        int size = xStreamBufferReceive(stream_1, data, 4, 10000);
        printf("from task2 %d\n", *data);
        if(size > 0){
            if(*data == 1){
                gpio_set_level(GPIO_NUM_2, 1);
            }
            else{
                gpio_set_level(GPIO_NUM_2, 0);
            }
        }

    }
    free(data);

}

void app_main(void)
{
    init();
    stream_1 = xStreamBufferCreate(16,4);
    xTaskCreate(rx_task, "RXTASK", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(led_task, "LEDTASK", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
}
