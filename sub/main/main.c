#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "driver/mcpwm_prelude.h"
#include "mavlink/ardupilotmega/mavlink.h"


static const char *TAG = "sub";

static const int RX_BUF_SIZE = 1024;
#define streamLen 3
#define TXD_PIN (GPIO_NUM_27)
#define RXD_PIN (GPIO_NUM_14)
#define UART UART_NUM_2

#define MAVLINK_PORT UART_NUM_1
#define MAVLINK_TX_PIN (GPIO_NUM_17)

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

static void mav_uart_init(void)
{
    uart_config_t mavcfg = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(MAVLINK_PORT, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MAVLINK_PORT, &mavcfg));
    ESP_ERROR_CHECK(uart_set_pin(MAVLINK_PORT, MAVLINK_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void mav_send_msg(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(MAVLINK_PORT, (const char *)buf, len);
}

static void mav_send_heartbeat(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        1,                    // System ID
        1,                    // Component ID
        &msg,
        MAV_TYPE_SUBMARINE,   // ROV = submarine
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_MANUAL_ARMED,
        0,
        MAV_STATE_ACTIVE
    );
    mav_send_msg(&msg);
}

static void mav_send_voltage(float voltage)
{
    mavlink_message_t msg;

    mavlink_msg_sys_status_pack(
        1, 1, &msg,
        0,0,0,                                   // sensors not used
        (uint16_t)(voltage * 1000.0f),           // millivolts
        0,                                       // current (not used)
        -1,                                      // battery remaining
        0,0,0,0,0,0,0
    );

    mav_send_msg(&msg);
}

static void mav_send_camera_tilt(float pitch_rad)
{
    mavlink_message_t msg;

    mavlink_msg_attitude_pack(
        1, 1, &msg,
        0,   // timestamp (ms)
        0.0f,                           // roll
        pitch_rad,                      // pitch (in radians)
        0.0f,                           // yaw unused here
        0,0,0
    );

    mav_send_msg(&msg);
}

static void mav_send_heading_depth(float heading_deg, float depth_m)
{
    mavlink_message_t msg;

    mavlink_msg_vfr_hud_pack(
        1, 1, &msg,
        0,                     // airspeed
        0,                     // groundspeed
        heading_deg,
        -depth_m,              // altitude = -depth
        0, 0                   // climb, throttle unused
    );

    mav_send_msg(&msg);
}

static void mav_send_osd_message(const char *text, uint8_t severity)
{
    mavlink_message_t msg;

    mavlink_msg_statustext_pack(
        1,         // system ID
        1,         // component ID
        &msg,
        severity,  // MAV_SEVERITY_INFO, WARNING, ALERT, etc.
        text       // your message (<= 50 chars)
    );

    mav_send_msg(&msg);
}

void mav_tx_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    int hb_count = 0;

    while (1)
    {
        // Replace these with your actual readings:
        float battery_voltage = 12.4;
        float heading_deg     = 87.0;
        float depth_m         = 4.5;
        float camera_pitch_deg = 10.0;

        float camera_pitch_rad = camera_pitch_deg * (M_PI / 180.0f);

        // Send heartbeat at 1 Hz
        if (++hb_count >= 10) {
            mav_send_heartbeat();
            hb_count = 0;
        }

        // Send actual telemetry
        mav_send_voltage(battery_voltage);
        mav_send_camera_tilt(camera_pitch_rad);
        mav_send_heading_depth(heading_deg, depth_m);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(100)); // 10Hz
    }
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
            for(int i=sizeof(data)-1; i>=0; i--){
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
            duty = data[0] | data[1] << 8;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty));
        }else{
            printf("dropped packet of size %d",size);
        }

    }
}

void app_main(void)
{
    init();
    xTaskCreate(rx_task, "RXTASK", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(led_task, "LEDTASK", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
}
