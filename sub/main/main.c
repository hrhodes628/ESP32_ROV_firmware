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
            .channel_count=8
    };
    ibus_init(&ibus_cfg);
    control_init();
    telemetry_init();
}
