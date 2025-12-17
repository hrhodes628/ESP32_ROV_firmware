#include "board.h"
#include "ibus.h"
#include "telemetry.h"
#include "motor.h"
#include "control.h"


void app_main(void)
{
    motor_init();

    ibus_config_t ibus_cfg = {
            .uart=IBUS_UART,
            .tx_pin=IBUS_TXD_PIN,
            .rx_pin=IBUS_RXD_PIN,
            .channel_count=8,
            .baudrate = IBUS_BAUD
    };
    ibus_init(&ibus_cfg);
    control_init();

    telemetry_config_t telem_cfg = {
            .tx_pin = MAVLINK_TX_PIN,
            .uart = MAVLINK_UART,
            .baudrate = MAVLINK_BUAD,
            .heartbeat_period_ms = 1000,
            .telemetry_rate_ms = 150
    };
    telemetry_init(&telem_cfg);
}
