idf_component_register(SRCS "usb_to_esp_now.c" "data_packet.h"
                    INCLUDE_DIRS "."
                    REQUIRES
                        freertos
                        esp_event
                        esp_wifi
                        esp_system
                        nvs_flash
                        driver
                        log
                        usb
                    PRIV_REQUIRES
                        esp_timer
                    )
