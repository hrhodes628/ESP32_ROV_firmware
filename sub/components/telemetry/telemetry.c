#include "telemetry.h"
#include "board.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "mavlink.h"

#define TAG "telemetry"

/* ---------- MAVLink config ---------- */
#define MAV_SYS_ID   1
#define MAV_COMP_ID  1

/* ---------- Internal telemetry state ---------- */

static telemetry_config_t config;

typedef struct {
    float voltage;
    float depth;
    int16_t heading;
    float camera_pitch;

    bool ibus_ok;
    uint32_t ibus_dropped;

    uint32_t error_flags;

    bool armed;
} telemetry_state_t;

static telemetry_state_t state;
static SemaphoreHandle_t state_lock;

/* ---------- UART + MAVLink helpers ---------- */

static void mav_uart_init(void)
{
    uart_config_t mavlink_uart_conf = {
        .baud_rate = config.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        config.uart , 2048, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(config.uart, &mavlink_uart_conf));

    ESP_ERROR_CHECK(uart_set_pin(
        config.uart,
        config.tx_pin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE));
}

static void mav_send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(config.uart, (const char *)buf, len);
}

/* ---------- Public setters (thread-safe) ---------- */

void telemetry_set_armed(bool armed ){

    xSemaphoreTake(state_lock, portMAX_DELAY);
    state.armed = armed;
    xSemaphoreGive(state_lock);

}

void telemetry_set_voltage(float v)
{
    xSemaphoreTake(state_lock, portMAX_DELAY);
    state.voltage = v;
    xSemaphoreGive(state_lock);
}

void telemetry_set_depth(float m)
{
    xSemaphoreTake(state_lock, portMAX_DELAY);
    state.depth = m;
    xSemaphoreGive(state_lock);
}

void telemetry_set_heading(float deg)
{
    xSemaphoreTake(state_lock, portMAX_DELAY);
    state.heading = deg;
    xSemaphoreGive(state_lock);
}

void telemetry_set_camera_pitch(float rad)
{
    xSemaphoreTake(state_lock, portMAX_DELAY);
    state.camera_pitch = rad;
    xSemaphoreGive(state_lock);
}

void telemetry_report_ibus(bool ok, uint32_t dropped)
{
    xSemaphoreTake(state_lock, portMAX_DELAY);
    state.ibus_ok = ok;
    state.ibus_dropped = dropped;
    xSemaphoreGive(state_lock);
}

void telemetry_report_error(uint32_t error_mask)
{
    xSemaphoreTake(state_lock, portMAX_DELAY);
    state.error_flags |= error_mask;
    xSemaphoreGive(state_lock);
}

void telemetry_send_text(const char *text, uint8_t severity)
{
    mavlink_message_t msg;

    mavlink_msg_statustext_pack(
        MAV_SYS_ID,
        MAV_COMP_ID,
        &msg,
        severity,
        text
    );

    mav_send(&msg);
}

/* ---------- Telemetry task ---------- */

static void telemetry_task(void *arg)
{
    uint32_t last_hb_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
        telemetry_state_t snap;

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* Take snapshot */
        xSemaphoreTake(state_lock, portMAX_DELAY);
        snap = state;
        xSemaphoreGive(state_lock);

        mavlink_message_t msg;

        /* ---------- Heartbeat @ 1 Hz ---------- */
        if ((now_ms - last_hb_ms) >= config.heartbeat_period_ms) {
            
            uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

            if (snap.armed) {
                base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
            }

            mavlink_msg_heartbeat_pack(
                MAV_SYS_ID,
                MAV_COMP_ID,
                &msg,
                MAV_TYPE_SUBMARINE,
                MAV_AUTOPILOT_GENERIC,
                base_mode,
                0,
                MAV_STATE_ACTIVE
            );
            mav_send(&msg);
            last_hb_ms = now_ms;
        }

        /* ---------- System voltage ---------- */
        mavlink_msg_sys_status_pack(
            MAV_SYS_ID,
            MAV_COMP_ID,
            &msg,
            0,0,0,0,
            (uint16_t)(snap.voltage * 1000.0f),
            -1,
            -1,
            0,0,0,0,0,0
        );
        mav_send(&msg);

        /* ---------- Camera pitch ---------- */
        mavlink_msg_attitude_pack(
            MAV_SYS_ID,
            MAV_COMP_ID,
            &msg,
            0,
            0.0f,
            snap.camera_pitch,
            0.0f,
            0,0,0
        );
        mav_send(&msg);

        // mavlink_msg_global_position_int_pack(
        //     MAV_SYS_ID,
        //     MAV_COMP_ID,
        //     &msg,
        //     0,
        //     0,
        //     0,
        //     snap.depth*1000,
        //     snap.depth*1000,
        //     0,
        //     0,
        //     0,
        //     snap.heading
        // );

        /* ---------- Heading / depth ---------- */
        mavlink_msg_vfr_hud_pack(
            MAV_SYS_ID,
            MAV_COMP_ID,
            &msg,
            0, 0,
            snap.heading,
            0,
            snap.depth,   // altitude = -depth
            0
        );
        mav_send(&msg);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(config.telemetry_rate_ms));
    }
}

/* ---------- Init ---------- */

void telemetry_init(const telemetry_config_t *cfg)
{
    config = *cfg ;
    state_lock = xSemaphoreCreateMutex();

    memset(&state, 0, sizeof(state));

    mav_uart_init();

    ESP_LOGI(TAG, "Telemetry initialized");
}

void telemetry_start(){

    xTaskCreatePinnedToCore(
        telemetry_task,
        "telemetry",
        4096,
        NULL,
        5,
        NULL,
        0
    );

    ESP_LOGI(TAG, "Telemetry task started");

}