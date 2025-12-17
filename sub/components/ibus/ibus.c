#include "ibus.h"
#include "ibus_internal.h"

#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "ibus"

static const float ibus_scale_bipolar = 1.0f / (IBUS_MAX_US - IBUS_MID_US);

static SemaphoreHandle_t ibus_mutex;
static ibus_frame_t latest_frame;
static ibus_state_t status;
static ibus_config_t config;
static bool frame_valid;

static inline float norm_bipolar(uint16_t v)
{
    return (float)(v - IBUS_MID_US) * ibus_scale_bipolar;
}

static uint16_t ibus_checksum(const uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0;

    // sum all bytes except last two (CRC)
    for (int i = 0; i < len - 2; i++) {
        sum += buf[i];
    }

    return 0xFFFF - sum;
}

static void ibus_rx_task(void *arg)
{
    const int packet_len = config.channel_count * 2 + IBUS_OVERHEAD;
    uint8_t buf[32];
    uint32_t last_frame_time_ms =0;

    while (1) {
        int len = uart_read_bytes(
            config.uart,
            &buf,
            packet_len,
            pdMS_TO_TICKS(20));

        if (len < packet_len) {
            continue;
        }

        int i = 0;
        while (i < len - 1) {
            if (buf[i] == IBUS_HEADER_0 &&
                buf[i + 1] == IBUS_HEADER_1) {
                break;
            }
            i++;
        }

        if (i + packet_len > len) {
            continue;
        }

        uint8_t *payload = &buf[i + 2];

        uint8_t *frame = &buf[i];

        uint16_t rx_crc = frame[packet_len - 2] | (frame[packet_len - 1] << 8);

        uint16_t calc_crc = ibus_checksum(frame, packet_len);

        if (rx_crc != calc_crc) { // bad frame
            if (xSemaphoreTake(ibus_mutex, portMAX_DELAY)) {
                    status.dropped_frames++;
                    status.frame_ok = false;
                    xSemaphoreGive(ibus_mutex);
                }
                continue;
        }

        uint32_t now_ms = esp_timer_get_time() / 1000;

        if (xSemaphoreTake(ibus_mutex, portMAX_DELAY)) {
            for (int ch = 0; ch < config.channel_count; ch++) {
                latest_frame.channels[ch] = payload[2 * ch] | (payload[2 * ch + 1] << 8);
            }

            // Update timing
            status.last_rx_ms = now_ms - last_frame_time_ms;
            status.link_ok = true;
            status.frame_ok = true;
            last_frame_time_ms = now_ms;
        }
        xSemaphoreGive(ibus_mutex);
    }
}

// bool ibus_read(ibus_frame_t *out)
// {
//     if (!frame_valid) {
//         return false;
//     }

//     if (xSemaphoreTake(ibus_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
//         return false;
//     }

//     *out = latest_frame;

//     xSemaphoreGive(ibus_mutex);
//     return true;
// }

bool ibus_norm_read(ibus_norm_frame_t *out)
{
    uint16_t raw[IBUS_MAX_CHANNELS];
    uint8_t count;

    if (xSemaphoreTake(ibus_mutex, pdMS_TO_TICKS(1)) != pdTRUE) {
        return false;
    }

    count = latest_frame.channel_count;
    memcpy(raw, latest_frame.channels, count * sizeof(uint16_t));

    xSemaphoreGive(ibus_mutex);

    // Normalize outside critical section
    for (int i = 0; i < count; i++) {
        out->channels[i] = norm_bipolar(raw[i]);
    }

    out->channel_count = count;
    out->valid = true;

    return true;
}


bool ibus_get_state(ibus_state_t *out_state){

    if (xSemaphoreTake(ibus_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }

    *out_state = status;

    xSemaphoreGive(ibus_mutex);
    return true;
}


//init

void ibus_init(const ibus_config_t *cfg)
{
    config = *cfg;

    ibus_mutex = xSemaphoreCreateMutex();
    frame_valid = false;

    status.dropped_frames = 0;
    status.last_rx_ms = 1000;
    status.link_ok = false;
    status.frame_ok = false;

    const uart_config_t uart_cfg = {
        .baud_rate = config.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        config.uart, 1024, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(config.uart, &uart_cfg));

    ESP_ERROR_CHECK(uart_set_pin(
        config.uart,
        cfg->tx_pin,
        cfg->rx_pin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE));


    xTaskCreatePinnedToCore(
        ibus_rx_task,
        "ibus_rx",
        2048,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL,
        0);
}