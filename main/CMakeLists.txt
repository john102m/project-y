idf_component_register(SRCS "esp32_3c_ble_main.c"
                       PRIV_REQUIRES bt spi_flash nvs_flash driver esp_adc
                       INCLUDE_DIRS ".") 
