#include "motor.h"
#include "board.h"
#include "arming.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define TAG "motor"

/* ------------------- Timing ------------------- */
#define RC_TIMER_RES_HZ     1000000   // 1 MHz → 1 tick = 1 µs
#define RC_PERIOD_US       20000      // 20 ms (50 Hz)

#define LED_TIMER_RES_HZ   1000000
#define LED_PERIOD_TICKS  100         // 0–100 = 0–100%

#define PWM_NEUTRAL_US 1500
#define PWM_RANGE_US 500

/* ------------------- PWM Channel ------------------- */
typedef struct {
    mcpwm_cmpr_handle_t cmpr;
    mcpwm_gen_handle_t  gen;
} motor_pwm_chan_t;

/* ------------------- Static Handles ------------------- */
static motor_pwm_chan_t servo_pwm;
static motor_pwm_chan_t motor_vert_pwm;
static motor_pwm_chan_t led_pwm;

/* ------------------- Helper ------------------- */

static float clamp_norm(float v)
{
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

static void pwm_channel_init(mcpwm_oper_handle_t oper, gpio_num_t gpio, motor_pwm_chan_t *chan)
{
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &chan->cmpr));

    mcpwm_generator_config_t gen_cfg = {
        .gen_gpio_num = gpio
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &chan->gen));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        chan->gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    ));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        chan->gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            chan->cmpr,
            MCPWM_GEN_ACTION_LOW
        )
    ));
}

/* ------------------- Public API ------------------- */
void motor_init(void)
{
    /* ---------- Timers ---------- */
    mcpwm_timer_handle_t rc_timer;
    mcpwm_timer_handle_t led_timer;

    mcpwm_timer_config_t rc_timer_cfg = {
        .group_id       = 0,
        .clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz  = RC_TIMER_RES_HZ,
        .period_ticks   = RC_PERIOD_US,
        .count_mode     = MCPWM_TIMER_COUNT_MODE_UP
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&rc_timer_cfg, &rc_timer));

    mcpwm_timer_config_t led_timer_cfg = {
        .group_id       = 0,
        .clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz  = LED_TIMER_RES_HZ,
        .period_ticks   = LED_PERIOD_TICKS,
        .count_mode     = MCPWM_TIMER_COUNT_MODE_UP
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&led_timer_cfg, &led_timer));

    /* ---------- Operators ---------- */
    mcpwm_oper_handle_t rc_oper;
    mcpwm_oper_handle_t led_oper;

    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0
    };

    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &rc_oper));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &led_oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(rc_oper, rc_timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(led_oper, led_timer));

    /* ---------- PWM Channels ---------- */
    pwm_channel_init(rc_oper, SERVO_TILT_PIN, &servo_pwm);
    pwm_channel_init(rc_oper, MOTOR_VERT_PIN,   &motor_vert_pwm);
    pwm_channel_init(led_oper, LED_PANEL_PIN, &led_pwm);

    /* ---------- Start Timers ---------- */
    ESP_ERROR_CHECK(mcpwm_timer_enable(rc_timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(led_timer));

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(
        rc_timer, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(
        led_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Motor MCPWM initialized");
}

void motor_set_us(motor_id_t motor, uint16_t pulse_us)
{

    if (!arming_is_armed()) {
        pulse_us = PWM_NEUTRAL_US;   // neutral / stop
    }

    switch (motor) {
    case MOTOR_SERVO:
        mcpwm_comparator_set_compare_value(
            servo_pwm.cmpr, pulse_us);
        break;

    case MOTOR_M5:
        mcpwm_comparator_set_compare_value(
            motor_vert_pwm.cmpr, pulse_us);
        break;

    default:
        break;
    }
}

void motor_set_led_duty(uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint32_t ticks = percent * LED_PERIOD_TICKS / 100;

    mcpwm_comparator_set_compare_value(
        led_pwm.cmpr, ticks);
}

void motor_set_norm(motor_id_t motor, float value)
{
    value = clamp_norm(value);

#if BUILD_DRY_RUN
    /* In dry-run, do nothing */
    return;
#endif

    uint16_t pulse_us =
        PWM_NEUTRAL_US + (int16_t)(value * PWM_RANGE_US);

    motor_set_us(motor, pulse_us);
}