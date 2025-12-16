idf_component_register(
    SRCS "ibus.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_uart
    PRIV_REQUIRES freertos esp_timer log
)