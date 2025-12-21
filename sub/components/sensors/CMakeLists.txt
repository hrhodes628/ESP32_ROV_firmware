idf_component_register(
                        INCLUDE_DIRS "include"
                        SRCS "sensors.c"
                        PRIV_REQUIRES telemetry board freertos esp_driver_i2c driver esp_adc
)