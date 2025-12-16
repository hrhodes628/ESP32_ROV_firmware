#include "ibus.h"

#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"

#define TAG "ibus"

#define IBUS_BAUDRATE 115200
#define IBUS_OVERHEAD 4
#define IBUS_HEADER_0 0x20
#define IBUS_HEADER_1 0x44   // some receivers use 0x40 or 0x44

static StreamBufferHandle_t ibus_stream;
static uint8_t ibus_channels;
static uart_port_t ibus_uart;


static void ibus_rx_task(void *arg)
{
    const int packet_len = ibus_channels * 2 + IBUS_OVERHEAD;
    uint8_t buf[32];

    while (1) {
        int len = uart_read_bytes(
            ibus_uart,
            buf,
            packet_len,
            pdMS_TO_TICKS(20));

        if (len < packet_len) {
            continue;
        }

        // Find IBUS header
        int i = 0;
        while (i < len - 1) {
            if (buf[i] == IBUS_HEADER_0 &&
                (buf[i + 1] == IBUS_HEADER_1)) {
                break;
            }
            i++;
        }

        if (i + packet_len > len) {
            continue;
        }

        // Skip header (2 bytes)
        uint8_t *payload = &buf[i + 2];

        xStreamBufferSend(
            ibus_stream,
            payload,
            ibus_channels * 2,
            0);
    }
}

bool ibus_read(ibus_frame_t *out, TickType_t timeout)
{
    size_t received = xStreamBufferReceive(
        ibus_stream,
        out->channels,
        ibus_channels * 2,
        timeout);

    if (received != ibus_channels * 2) {
        return false;
    }

    out->channel_count = ibus_channels;

    // Convert little-endian bytes → uint16_t
    for (int i = 0; i < ibus_channels; i++) {
        uint8_t lo = ((uint8_t *)out->channels)[2 * i];
        uint8_t hi = ((uint8_t *)out->channels)[2 * i + 1];
        out->channels[i] = lo | (hi << 8);
    }

    return true;
}

//init

void ibus_init(const ibus_config_t *cfg)
{
    ibus_uart = cfg->uart;
    ibus_channels = cfg->channel_count;

    const uart_config_t uart_cfg = {
        .baud_rate = IBUS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        ibus_uart, 1024, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(ibus_uart, &uart_cfg));

    ESP_ERROR_CHECK(uart_set_pin(
        ibus_uart,
        cfg->tx_pin,
        cfg->rx_pin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE));

    ibus_stream = xStreamBufferCreate(
        ibus_channels * 2,
        ibus_channels * 2);

    xTaskCreatePinnedToCore(
        ibus_rx_task,
        "ibus_rx",
        2048,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL,
        0);
}