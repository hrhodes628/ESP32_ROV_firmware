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
#define MAV_BAUD     57600

/* ---------- Telemetry timing ---------- */
#define TELEMETRY_RATE_MS 100   // 10 Hz
#define HEARTBEAT_DIV     10    // 1 Hz at 10 Hz loop

/* ---------- Internal telemetry state ---------- */
typedef struct {
    float voltage;
    float depth;
    float heading;
    float camera_pitch;

    bool ibus_ok;
    uint32_t ibus_dropped;

    uint32_t error_flags;
} telemetry_state_t;

static telemetry_state_t state;
static SemaphoreHandle_t state_lock;

/* ---------- UART + MAVLink helpers ---------- */

static void mav_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = MAV_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        MAVLINK_UART, 2048, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(MAVLINK_UART, &cfg));

    ESP_ERROR_CHECK(uart_set_pin(
        MAVLINK_UART,
        MAVLINK_TX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE));
}

static void mav_send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(MAVLINK_UART, (const char *)buf, len);
}

/* ---------- Public setters (thread-safe) ---------- */

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
    TickType_t last = xTaskGetTickCount();
    uint8_t hb_div = 0;

    while (1) {
        telemetry_state_t snap;

        /* Take snapshot */
        xSemaphoreTake(state_lock, portMAX_DELAY);
        snap = state;
        xSemaphoreGive(state_lock);

        mavlink_message_t msg;

        /* ---------- Heartbeat @ 1 Hz ---------- */
        if (++hb_div >= HEARTBEAT_DIV) {
            mavlink_msg_heartbeat_pack(
                MAV_SYS_ID,
                MAV_COMP_ID,
                &msg,
                MAV_TYPE_SUBMARINE,
                MAV_AUTOPILOT_GENERIC,
                MAV_MODE_MANUAL_ARMED,
                0,
                MAV_STATE_ACTIVE
            );
            mav_send(&msg);
            hb_div = 0;
        }

        /* ---------- System voltage ---------- */
        mavlink_msg_sys_status_pack(
            MAV_SYS_ID,
            MAV_COMP_ID,
            &msg,
            0,0,0,
            (uint16_t)(snap.voltage * 1000.0f),
            0,
            -1,
            0,0,0,0,0,0,0
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

        /* ---------- Heading / depth ---------- */
        mavlink_msg_vfr_hud_pack(
            MAV_SYS_ID,
            MAV_COMP_ID,
            &msg,
            0, 0,
            snap.heading,
            -snap.depth,   // altitude = -depth
            0, 0
        );
        mav_send(&msg);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_RATE_MS));
    }
}

/* ---------- Init ---------- */

void telemetry_init(void)
{
    state_lock = xSemaphoreCreateMutex();

    mav_uart_init();

    xTaskCreatePinnedToCore(
        telemetry_task,
        "telemetry",
        4096,
        NULL,
        5,
        NULL,
        0
    );

    ESP_LOGI(TAG, "Telemetry initialized");
}