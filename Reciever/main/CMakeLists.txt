idf_component_register(
    SRCS "esp_now_to_usb.cpp" "data_packet.h"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb
     nvs_flash
        esp_wifi
        esp_event
        esp_netif
        lwip
        usb
        driver
        esp_timer
    )
