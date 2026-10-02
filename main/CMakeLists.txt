idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash esp_adc esp_wifi esp_netif esp_event nvs_flash esp_hw_support
                       INCLUDE_DIRS "")