idf_component_register(SRCS "main.c" "sh2_hal_i2c.c" "imu_stream.c" "uart_sniff.c" "ble_link.c" "afe.c"
                       INCLUDE_DIRS "."
                       REQUIRES sh2 esp_driver_gpio esp_driver_i2c esp_driver_ledc
                                esp_driver_uart esp_adc esp_timer bt nvs_flash)
