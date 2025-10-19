/*
 * ESP32-S3 Pengirim (USB CDC Device → ESP-NOW)
 * File: sender_main.c
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "data_packet.h"
#include "esp_timer.h"
#include "driver/gpio.h"
static const char *TAG = "CDC_SENDER";

// MAC Address ESP32-S3 Penerima (GANTI DENGAN MAC PENERIMA ANDA!)
static uint8_t receiver_mac[ESP_NOW_ETH_ALEN] = {0x98, 0xA3, 0x16, 0xE5, 0x78, 0x68}; // MAC penerima

// Queue untuk data dari CDC ke ESP-NOW
static QueueHandle_t cdc_to_espnow_queue = NULL;
#define QUEUE_SIZE 20

// Line coding saat ini
static line_coding_t current_line_coding = {
    .dwDTERate = 115200,
    .bCharFormat = 0,
    .bParityType = 0,
    .bDataBits = 8};

// Control line states
static bool current_dtr = false;
static bool current_rts = false;
static bool last_dtr = false;
static bool last_rts = false;
// Sequence number
static uint16_t sequence_num = 0;

// Semaphore untuk sinkronisasi
static SemaphoreHandle_t line_coding_mutex = NULL;

static QueueHandle_t led_queue = NULL;
#define LED_QUEUE_SIZE 50 // Cukup besar untuk traffic tinggi

// GPIO definitions (sesuaikan dengan board Anda)
#define LED_RX_GPIO GPIO_NUM_21
#define LED_TX_GPIO GPIO_NUM_47
#define LED_DTR_GPIO GPIO_NUM_35
#define LED_RTS_GPIO GPIO_NUM_36

// LED state tracking (untuk auto-off)
typedef struct
{
    bool is_on;
    int64_t turn_on_time; // microseconds
} led_state_t;

static led_state_t led_states[4] = {0};

/* ========== LED Task (EFISIEN - Single Task) ========== */

static void led_indicator_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED indicator task started");

    // Configure GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_RX_GPIO) | (1ULL << LED_TX_GPIO) |
                        (1ULL << LED_DTR_GPIO) | (1ULL << LED_RTS_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Turn off all LEDs initially
    gpio_set_level(LED_RX_GPIO, 0);
    gpio_set_level(LED_TX_GPIO, 0);
    gpio_set_level(LED_DTR_GPIO, 0);
    gpio_set_level(LED_RTS_GPIO, 0);

    led_event_t event;
    const int64_t blink_duration_us = LED_BLINK_DURATION_MS * 1000;

    while (1)
    {
        // Check queue dengan timeout kecil (non-blocking check)
        if (xQueueReceive(led_queue, &event, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            // Turn ON LED berdasarkan event
            switch (event.led_type)
            {
            case LED_RX:
                gpio_set_level(LED_RX_GPIO, 1);
                led_states[LED_RX].is_on = true;
                led_states[LED_RX].turn_on_time = esp_timer_get_time();
                break;

            case LED_TX:
                gpio_set_level(LED_TX_GPIO, 1);
                led_states[LED_TX].is_on = true;
                led_states[LED_TX].turn_on_time = esp_timer_get_time();
                break;

            case LED_DTR:
                gpio_set_level(LED_DTR_GPIO, 1);
                led_states[LED_DTR].is_on = true;
                led_states[LED_DTR].turn_on_time = esp_timer_get_time();
                break;

            case LED_RTS:
                gpio_set_level(LED_RTS_GPIO, 1);
                led_states[LED_RTS].is_on = true;
                led_states[LED_RTS].turn_on_time = esp_timer_get_time();
                break;
            }
        }

        // Check semua LED apakah sudah waktunya OFF
        int64_t now = esp_timer_get_time();

        for (int i = 0; i < 4; i++)
        {
            if (led_states[i].is_on)
            {
                if ((now - led_states[i].turn_on_time) >= blink_duration_us)
                {
                    // Turn off LED
                    switch (i)
                    {
                    case LED_RX:
                        gpio_set_level(LED_RX_GPIO, 0);
                        break;
                    case LED_TX:
                        gpio_set_level(LED_TX_GPIO, 0);
                        break;
                    case LED_DTR:
                        gpio_set_level(LED_DTR_GPIO, 0);
                        break;
                    case LED_RTS:
                        gpio_set_level(LED_RTS_GPIO, 0);
                        break;
                    }
                    led_states[i].is_on = false;
                }
            }
        }

        // Small delay untuk tidak busy-wait (penting untuk efisiensi)
        vTaskDelay(pdMS_TO_TICKS(1)); // 1ms delay, LED masih responsif
    }
}

static inline void trigger_led(led_type_t led_type)
{
    led_event_t event = {
        .led_type = led_type,
        .timestamp = esp_timer_get_time()};

    // Non-blocking send, drop jika queue penuh (tidak critical)
    xQueueSend(led_queue, &event, 0);
}
/* ========== ESP-NOW Callbacks ========== */

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "ESP-NOW: Packet sent successfully");
    }
    else
    {
        ESP_LOGW(TAG, "ESP-NOW: Packet send failed");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len != sizeof(data_packet_t))
        return;
    data_packet_t packet;
    memcpy(&packet, data, sizeof(data_packet_t));
    if (!validate_checksum(&packet))
        return;

    if (packet.packet_type == PACKET_TYPE_DATA && packet.data_len > 0)
    {
        size_t written = 0;
        tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, packet.data, packet.data_len);
        tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 10);
        ESP_LOGI(TAG, "ESPNOW->USB %d bytes sent to PC", packet.data_len);
    }
}

/* ========== ESP-NOW Init ========== */

static esp_err_t espnow_init(void)
{
    ESP_LOGI(TAG, "Initializing ESP-NOW...");

    // Init WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(01, WIFI_SECOND_CHAN_NONE));
    ESP_LOGI(TAG, "WiFi initialized in STA mode on channel 1");
    // Init ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Add peer (broadcast)
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, receiver_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 1;
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    return ESP_OK;
}

/* ========== USB CDC Callbacks ========== */

static void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t rx_size = 0;
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);

    if (ret == ESP_OK && rx_size > 0)
    {
        trigger_led(LED_RX);
        // Buat paket data
        data_packet_t packet = {0};
        packet.time_code = esp_timer_get_time();
        packet.sequence_number = sequence_num++;
        packet.packet_type = PACKET_TYPE_DATA;
        packet.data_len = (rx_size > MAX_DATA_SIZE) ? MAX_DATA_SIZE : rx_size;
        memcpy(packet.data, buf, packet.data_len);

        // Copy line coding dan control lines
        xSemaphoreTake(line_coding_mutex, portMAX_DELAY);
        packet.line_coding = current_line_coding;
        packet.dtr_state = current_dtr;
        packet.rts_state = current_rts;
        xSemaphoreGive(line_coding_mutex);

        prepare_packet(&packet);

        // Kirim ke queue
        if (xQueueSend(cdc_to_espnow_queue, &packet, pdMS_TO_TICKS(10)) != pdTRUE)
        {
            ESP_LOGW(TAG, "Queue full, packet dropped");
        }
        else
        {
            ESP_LOGI(TAG, "CDC RX: %d bytes queued (seq: %d)", rx_size, packet.sequence_number);
        }
    }
}

static void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    bool dtr = event->line_state_changed_data.dtr;
    bool rts = event->line_state_changed_data.rts;

    if (dtr != last_dtr)
    {
        trigger_led(LED_DTR); // ✅ LED DTR
    }

    if (rts != last_rts)
    {
        trigger_led(LED_RTS); // ✅ LED RTS
    }

    last_dtr = dtr;
    last_rts = rts;

    ESP_LOGI(TAG, "Line state changed: DTR=%d, RTS=%d", dtr, rts);

    xSemaphoreTake(line_coding_mutex, portMAX_DELAY);
    current_dtr = dtr;
    current_rts = rts;
    xSemaphoreGive(line_coding_mutex);

    // Kirim paket control line
    data_packet_t packet = {0};
    packet.time_code = esp_timer_get_time();
    packet.sequence_number = sequence_num++;
    packet.packet_type = PACKET_TYPE_CONTROL_LINE;
    packet.data_len = 0;
    packet.dtr_state = dtr;
    packet.rts_state = rts;
    packet.line_coding = current_line_coding;
    prepare_packet(&packet);

    xQueueSend(cdc_to_espnow_queue, &packet, pdMS_TO_TICKS(10));
}

static void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event)
{
    uint32_t bit_rate = event->line_coding_changed_data.p_line_coding->bit_rate;
    uint8_t stop_bits = event->line_coding_changed_data.p_line_coding->stop_bits;
    uint8_t parity = event->line_coding_changed_data.p_line_coding->parity;
    uint8_t data_bits = event->line_coding_changed_data.p_line_coding->data_bits;

    static line_coding_t last_line_coding = {
        .dwDTERate = 0,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 0};

    // Cek apakah sama seperti sebelumnya
    if (last_line_coding.dwDTERate == bit_rate &&
        last_line_coding.bCharFormat == stop_bits &&
        last_line_coding.bParityType == parity &&
        last_line_coding.bDataBits == data_bits)
    {
        ESP_LOGI(TAG, "Line coding unchanged, skip sending");
        return;
    }
    ESP_LOGI(TAG, "Line coding changed: Baudrate=%" PRIu32 ", Stop=%d, Parity=%d, Data=%d",
             bit_rate, stop_bits, parity, data_bits);

    xSemaphoreTake(line_coding_mutex, portMAX_DELAY);
    current_line_coding.dwDTERate = bit_rate;
    current_line_coding.bCharFormat = stop_bits;
    current_line_coding.bParityType = parity;
    current_line_coding.bDataBits = data_bits;
    xSemaphoreGive(line_coding_mutex);

    // Kirim paket line coding
    data_packet_t packet = {0};
    packet.time_code = esp_timer_get_time();
    packet.sequence_number = sequence_num++;
    packet.packet_type = PACKET_TYPE_LINE_CODING;
    packet.data_len = 0;
    packet.dtr_state = current_dtr;
    packet.rts_state = current_rts;
    packet.line_coding = current_line_coding;
    prepare_packet(&packet);

    xQueueSend(cdc_to_espnow_queue, &packet, pdMS_TO_TICKS(10));
}

/* ========== USB CDC Init ========== */

static void usb_cdc_init(void)
{
    ESP_LOGI(TAG, "USB CDC initialization");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    ESP_LOGI(TAG, "USB CDC initialized");
}

/* ========== ESP-NOW Sender Task ========== */

static void espnow_sender_task(void *pvParameters)
{
    data_packet_t packet;

    ESP_LOGI(TAG, "ESP-NOW sender task started");

    while (1)
    {
        if (xQueueReceive(cdc_to_espnow_queue, &packet, portMAX_DELAY) == pdTRUE)
        {
            // Kirim paket via ESP-NOW
            esp_err_t ret = esp_now_send(receiver_mac, (uint8_t *)&packet, sizeof(data_packet_t));

            if (ret == ESP_OK)
            {
                trigger_led(LED_TX);
                ESP_LOGI(TAG, "Sent packet seq=%d, type=%d, len=%d, time=%" PRIu32,
                         packet.sequence_number, packet.packet_type,
                         packet.data_len, packet.time_code);
            }
            else
            {
                ESP_LOGE(TAG, "ESP-NOW send failed: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(10)); // Anti spam
            }
        }
    }
}

/* ========== Heartbeat Task ========== */

static void heartbeat_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Heartbeat task started");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Every 5 seconds

        data_packet_t packet = {0};
        packet.time_code = esp_timer_get_time();
        packet.sequence_number = sequence_num++;
        packet.packet_type = PACKET_TYPE_HEARTBEAT;
        packet.data_len = 0;

        xSemaphoreTake(line_coding_mutex, portMAX_DELAY);
        packet.dtr_state = current_dtr;
        packet.rts_state = current_rts;
        packet.line_coding = current_line_coding;
        xSemaphoreGive(line_coding_mutex);

        prepare_packet(&packet);
        xQueueSend(cdc_to_espnow_queue, &packet, 0);

        ESP_LOGI(TAG, "Heartbeat sent");
        wifi_config_t conf;
        esp_wifi_get_config(WIFI_IF_STA, &conf);
        ESP_LOGI(TAG, "WiFi Channel = %d", conf.sta.channel);
    }
}

/* ========== Main ========== */

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 CDC to ESP-NOW Sender ===");

    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create mutex
    line_coding_mutex = xSemaphoreCreateMutex();
    assert(line_coding_mutex != NULL);

    // Create queue
    cdc_to_espnow_queue = xQueueCreate(QUEUE_SIZE, sizeof(data_packet_t));
    assert(cdc_to_espnow_queue != NULL);

    led_queue = xQueueCreate(LED_QUEUE_SIZE, sizeof(led_event_t));
    assert(led_queue != NULL);
    // Init USB CDC
    usb_cdc_init();

    // Init ESP-NOW
    ESP_ERROR_CHECK(espnow_init());

    // Print MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "Sender MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Start tasks
    xTaskCreate(led_indicator_task, "led_task", 4096, NULL, 2, NULL);
    xTaskCreate(espnow_sender_task, "espnow_tx", 4096, NULL, 5, NULL);
    xTaskCreate(heartbeat_task, "heartbeat", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "System initialized successfully");
}