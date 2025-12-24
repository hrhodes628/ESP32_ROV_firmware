#include "sensors.h"
#include "board.h"
#include "telemetry.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include <math.h>

#define SENSORS_REFRESH_MS 150

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mag_dev;
static i2c_master_dev_handle_t bmp_dev;

static adc_oneshot_unit_handle_t adc_handle;


static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), -1);
}

static esp_err_t i2c_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        data,
        len,
        -1
    );
}

#ifdef MAG_ENABLED
static float mag_read_heading(void){
    uint8_t buf[6];
    int16_t x, y, z;

    i2c_read_reg(mag_dev, 0x03, buf, 6);

    x = (buf[0] << 8) | buf[1];
    z = (buf[2] << 8) | buf[3];
    y = (buf[4] << 8) | buf[5];

    float heading = atan2f((float)y, (float)x) * (180.0f / M_PI);
    if (heading < 0.0f) heading += 360.0f;

    return heading;
}

static void hmc5883l_attach(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x1E,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(i2c_bus, &cfg, &mag_dev)
    );
}

static void hmc5883l_init(void)
{
    // Config A: 8-sample avg, 15 Hz, normal measurement
    i2c_write_reg(mag_dev, 0x00, 0x70);

    // Config B: default gain
    i2c_write_reg(mag_dev, 0x01, 0x20);

    // Mode: continuous
    i2c_write_reg(mag_dev, 0x02, 0x00);
}
#else
static int16_t mag_read_heading(void){
    return(67);
}
#endif

#ifdef BARO_ENABLED
static float baro_read_depth_m(void){

    // TODO: implement BMP180 compensated pressure
    // Return 0.0f for now so telemetry stays sane
    return 0.0f;

}

static void bmp180_attach(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x77,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(i2c_bus, &cfg, &bmp_dev)
    );
}

static void bmp180_init(void)
{
    // Nothing yet — calibration will go here
}
#else
static float baro_read_depth_m(void){
    return(10.0f);
}
#endif

#ifdef ADC_ENABLED

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BOARD_VOLTAGE_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc_handle,
        BOARD_VOLTAGE_ADC_CH,
        &chan_cfg
    ));
}

static float adc_read_voltage_v(void){

    int raw;
    adc_oneshot_read(adc_handle, BOARD_VOLTAGE_ADC_CH, &raw);

    float v_adc = (raw / 4095.0f) * 3.3f;
    return v_adc / BOARD_VOLTAGE_DIVIDER_RATIO;

}

#else
static float adc_read_voltage_v(void){
    return(12.0f);
}

#endif

static void i2c_init(){
        i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));
}

static void sensors_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while(1) {
        float heading = mag_read_heading();
        float depth   = baro_read_depth_m();
        float voltage = adc_read_voltage_v();

        telemetry_set_heading(heading);
        telemetry_set_depth(depth);
        telemetry_set_voltage(voltage);
        telemetry_set_camera_pitch(1.2f);
        telemetry_set_armed(true);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSORS_REFRESH_MS));
    }
}

void sensors_init(void){
    i2c_init();

#ifdef MAG_ENABLED
    hmc5883l_attach();
    hmc5883l_init();
#endif
#ifdef BARO_ENABLED
    bmp180_attach();
    bmp180_init();
#endif
#ifdef ADC_ENABLED
    adc_init();
#endif

}

void sensors_start(void){

    xTaskCreatePinnedToCore(
        sensors_task,
        "sensors",
        4096,
        NULL,
        5,
        NULL,
        1
    );

}