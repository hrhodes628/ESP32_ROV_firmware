#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "driver/mcpwm_prelude.h"


static const char *TAG = "sub";

static const int RX_BUF_SIZE = 1024;
#define streamLen 3
#define TXD_PIN (GPIO_NUM_27)
#define RXD_PIN (GPIO_NUM_14)
#define UART UART_NUM_2
#define LED_GPIO GPIO_NUM_12
StreamBufferHandle_t stream_1 = NULL;
mcpwm_cmpr_handle_t comparator = NULL;

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

    uart_driver_install(UART, RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    stream_1 = xStreamBufferCreate(16,3);

    //mcpwm block
    mcpwm_timer_handle_t pwmTimer = NULL;
    mcpwm_timer_config_t pwmTimer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks = 1000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&pwmTimer_config, &pwmTimer));
    
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, pwmTimer));

    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gen_cfg = {
        .gen_gpio_num = LED_GPIO
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &gen));

    mcpwm_comparator_config_t comparator_cfg = {
        .flags.update_cmp_on_tez = true // update at period start
    };

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_cfg, &comparator));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
    ));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)
    ));

    ESP_ERROR_CHECK(mcpwm_timer_enable(pwmTimer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwmTimer, MCPWM_TIMER_START_NO_STOP));

}

void rx_task(void* arg){

    uint8_t data[8] = {0};
    uint8_t streampack[2] = {0};
    while (1) {
        int rxBytes = uart_read_bytes(UART, &data, 3, 10000);

        if(rxBytes >= 3){
            int i = 0;
            while(data[i] != 0x44){
                i++;
                if(i >= 5){
                    i=0;
                }
            }
            streampack[0]=data[i+1];
            streampack[1]=data[i+2];
            printf("from rx task:");
            for(int i=0; i<sizeof(data);i++){
                printf("%x", data[i]);
            }
            printf("\n");
            fflush(stdout);
            xStreamBufferSend(stream_1, &streampack, streamLen, 1000);
            
        }   
    }

}

void led_task(void* arg){

    uint8_t data[2] = {0};
    uint16_t duty = 0;

    while(true){

        int size = xStreamBufferReceive(stream_1, &data, 3, 10000);
        if(size>=3){
            printf("from task2 %d\n", *data);
        
            duty = data[0] | (data[1] << 8);
            ESP_LOGI(TAG, "setting the LED to %hu", duty);
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty));
        }

    }
}

void app_main(void)
{
    init();
    xTaskCreate(rx_task, "RXTASK", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(led_task, "LEDTASK", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
}
