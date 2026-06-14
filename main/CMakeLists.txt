idf_component_register(SRCS "display.c" "nus_protocol.c" "gatts_demo.c" "nus.c"
                    PRIV_REQUIRES bt nvs_flash esp_lcd driver esp_timer
                    INCLUDE_DIRS ".")
