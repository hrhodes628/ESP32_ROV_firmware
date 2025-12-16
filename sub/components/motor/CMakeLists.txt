idf_component_register(
                        INCLUDE_DIRS "include"
                        SRCS "motor.c"
                        REQUIRES esp_driver_gpio
                        PRIV_REQUIRES esp_driver_mcpwm log board arming

)