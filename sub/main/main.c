#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"


#include "esp_mac.h"
#include "driver/mcpwm_prelude.h"

#include "board.h"
#include "mavlink.h"
#include "ibus.h"

#define CHANNELS 8

#define IBUS_OVERHEAD 4
#define PACKLEN CHANNELS*2+IBUS_OVERHEAD
#define IBUS_STREAM_LENGTH CHANNELS*2

static const char *TAG = "sub";

static const int RX_BUF_SIZE = 1024;
StreamBufferHandle_t IBUS_STREAM = NULL;

mcpwm_cmpr_handle_t SERVcmpr = NULL;
mcpwm_cmpr_handle_t M5cmpr = NULL;
mcpwm_cmpr_handle_t LEDcmpr = NULL;

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void IBUS_init(void)
{

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(IBUS_UART, RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(IBUS_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(IBUS_UART, IBUS_TXD_PIN, IBUS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    IBUS_STREAM = xStreamBufferCreate(IBUS_STREAM_LENGTH,IBUS_STREAM_LENGTH);
}

void mcpwm_init(void){

    mcpwm_timer_handle_t LEDpwmTimer = NULL;
    mcpwm_timer_config_t LEDpwmTimer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks = 100,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&LEDpwmTimer_config, &LEDpwmTimer));

    mcpwm_timer_handle_t RCpwmTimer = NULL;
    mcpwm_timer_config_t RCpwmTimer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks = 20000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&RCpwmTimer_config,&RCpwmTimer));

    
    mcpwm_oper_handle_t RCoper = NULL;
    mcpwm_oper_handle_t LEDoper = NULL;
    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &RCoper));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &LEDoper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(RCoper, RCpwmTimer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(LEDoper, LEDpwmTimer));

    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true,   // safe update at frame start
    };

    ESP_ERROR_CHECK(mcpwm_new_comparator(RCoper, &cmp_cfg, &SERVcmpr));
    ESP_ERROR_CHECK(mcpwm_new_comparator(RCoper, &cmp_cfg, &M5cmpr));
    ESP_ERROR_CHECK(mcpwm_new_comparator(LEDoper, &cmp_cfg, &LEDcmpr));

    mcpwm_gen_handle_t SERVgen = NULL;
    mcpwm_gen_handle_t M5gen = NULL;
    mcpwm_gen_handle_t LEDgen = NULL;

    mcpwm_generator_config_t gen_cfg = {};

    gen_cfg.gen_gpio_num = SERV;
    ESP_ERROR_CHECK(mcpwm_new_generator(RCoper, &gen_cfg, &SERVgen));

    gen_cfg.gen_gpio_num = M5;
    ESP_ERROR_CHECK(mcpwm_new_generator(RCoper, &gen_cfg, &M5gen));

    gen_cfg.gen_gpio_num = LED;
    ESP_ERROR_CHECK(mcpwm_new_generator(LEDoper, &gen_cfg, &LEDgen));

    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_timer_event(
            SERVgen,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH)
        )
    );

    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_compare_event(
            SERVgen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                SERVcmpr,
                MCPWM_GEN_ACTION_LOW)
        )
    );

    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_timer_event(
            M5gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH)
        )
    );

    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_compare_event(
            M5gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                M5cmpr,
                MCPWM_GEN_ACTION_LOW)
        )
    );

    ESP_ERROR_CHECK(mcpwm_timer_enable(RCpwmTimer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(LEDpwmTimer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(RCpwmTimer, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(LEDpwmTimer, MCPWM_TIMER_START_NO_STOP));


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

    ESP_ERROR_CHECK(uart_driver_install(MAVLINK_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MAVLINK_UART, &mavcfg));
    ESP_ERROR_CHECK(uart_set_pin(MAVLINK_UART, MAVLINK_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void mav_send_msg(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(MAVLINK_UART, (const char *)buf, len);
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

void UART_rx_task(void* arg){

    uint8_t data[PACKLEN] = {0};
    uint8_t streampack[IBUS_STREAM_LENGTH] = {0};
    while (1) {
        int rxBytes = uart_read_bytes(IBUS_UART, &data, PACKLEN, 10000);

        if(rxBytes >= PACKLEN){
            int i = 0;
            while(data[i] != 0x20 && data[i+1] != 0x44){
                i++;
                if(i >= 5){
                    i=0;
                }
            }
            for(int j=0; j<IBUS_STREAM_LENGTH;j++){
                streampack[j]=data[i+j+2];
            }
            printf("from rx task:");
            for(int j=0; i < sizeof(streampack); i++){
                printf("%x", data[i]);
            }
            printf("\n");
            fflush(stdout);
            xStreamBufferSend(IBUS_STREAM, &streampack, IBUS_STREAM_LENGTH, 1000);
            
        }   
    }

}

void motor_task(void* arg){

    uint8_t data[IBUS_STREAM_LENGTH] = {0};
    uint16_t channels[CHANNELS] = {0};


    while(true){

        int size = xStreamBufferReceive(IBUS_STREAM, &data, IBUS_STREAM_LENGTH, 10000);
        if(size>=IBUS_STREAM_LENGTH){
            for(int i = 0; i < CHANNELS ; i++ ){
                channels[i] = data[2*i] | data[2*i+1] << 8;
            }

            uint16_t ledDuty = (channels[6]-1000)*100/2000;
            ledDuty = clamp_u16(ledDuty,0,99);

            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(LEDcmpr, ledDuty));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(SERVcmpr, channels[5]));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(M5cmpr, channels[4]));
        }else{
            printf("dropped packet of size %d",size);
        }

    }
}

void app_main(void)
{
    IBUS_init();
    mcpwm_init();
    xTaskCreate(UART_rx_task, "RXTASK", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(motor_task, "MOTORTASK", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
}
