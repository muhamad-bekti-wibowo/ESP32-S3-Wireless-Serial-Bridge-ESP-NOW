/*
 * ESP32-S3 Penerima (ESP-NOW → USB Host)
 * File: receiver_main.cpp
 */

#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_random.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"
#include "usb/vcp.hpp"

extern "C"
{
#include "data_packet.h"
}

using namespace esp_usb;

static const char *TAG = "ESPNOW_RECEIVER";

// Priority queue untuk paket berdasarkan time_code
#define MAX_PACKET_QUEUE 100
static data_packet_t packet_buffer[MAX_PACKET_QUEUE];
static int packet_count = 0;
static SemaphoreHandle_t packet_mutex = NULL;
static SemaphoreHandle_t usb_ready_sem = NULL;
static SemaphoreHandle_t device_disconnected_sem = NULL;

// USB VCP device
static std::unique_ptr<CdcAcmDevice> vcp_device = nullptr;
static bool usb_device_ready = false;

// Line coding saat ini
static line_coding_t current_line_coding = {
    .dwDTERate = 115200,
    .bCharFormat = 0,
    .bParityType = 0,
    .bDataBits = 8};

// Control states
static bool current_dtr = false;
static bool current_rts = false;

static uint8_t sender_mac[ESP_NOW_ETH_ALEN] = {0xC0, 0x4E, 0x30, 0x0A, 0xBD, 0x40}; // MAC pengirim

static QueueHandle_t usb_to_espnow_queue = NULL;
#define USB_TO_ESPNOW_QUEUE_SIZE 20

static QueueHandle_t espnow_to_usb_queue = NULL;
#define ESPNOW_TO_USB_QUEUE_SIZE 100

// Callback RX dari USB Host
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    if (data_len == 0)
        return true;

    data_packet_t packet = {0};
    packet.time_code = esp_timer_get_time();
    packet.sequence_number = esp_random() & 0xFFFF;
    packet.packet_type = PACKET_TYPE_DATA;
    packet.data_len = (data_len > MAX_DATA_SIZE) ? MAX_DATA_SIZE : data_len;
    memcpy(packet.data, data, packet.data_len);
    packet.dtr_state = current_dtr;
    packet.rts_state = current_rts;
    packet.line_coding = current_line_coding;
    prepare_packet(&packet);

    if (xQueueSend(usb_to_espnow_queue, &packet, 0) != pdTRUE)
        ESP_LOGW(TAG, "USB->ESPNOW queue full, dropping");

    return true;
}

// Task kirim data USB Host → ESP-NOW
static void usb_to_espnow_task(void *pvParameters)
{
    data_packet_t packet;
    while (1)
    {
        if (xQueueReceive(usb_to_espnow_queue, &packet, portMAX_DELAY) == pdTRUE)
        {
            esp_err_t ret = esp_now_send(sender_mac, (uint8_t *)&packet, sizeof(data_packet_t));
            if (ret == ESP_OK)
                ESP_LOGI(TAG, "Sent USB->ESPNOW seq=%d len=%d", packet.sequence_number, packet.data_len);
            else
                ESP_LOGE(TAG, "Failed send USB->ESPNOW: %s", esp_err_to_name(ret));
        }
    }
}
/* ========== Priority Queue Functions ========== */

// Insert packet dengan sorting berdasarkan time_code (ascending)
static bool insert_packet(const data_packet_t *packet)
{
    xSemaphoreTake(packet_mutex, portMAX_DELAY);

    if (packet_count >= MAX_PACKET_QUEUE)
    {
        ESP_LOGW(TAG, "Packet buffer full, dropping packet");
        xSemaphoreGive(packet_mutex);
        return false;
    }

    // Insert dan sort
    packet_buffer[packet_count] = *packet;
    packet_count++;

    // Simple insertion sort based on time_code
    for (int i = packet_count - 1; i > 0; i--)
    {
        if (packet_buffer[i].time_code < packet_buffer[i - 1].time_code)
        {
            data_packet_t temp = packet_buffer[i];
            packet_buffer[i] = packet_buffer[i - 1];
            packet_buffer[i - 1] = temp;
        }
        else
        {
            break;
        }
    }

    xSemaphoreGive(packet_mutex);

    ESP_LOGI(TAG, "Packet inserted: seq=%d, time=%" PRIu32 ", queue_size=%d",
             packet->sequence_number, packet->time_code, packet_count);

    return true;
}

// Get packet dengan time_code terkecil
static bool get_next_packet(data_packet_t *packet)
{
    xSemaphoreTake(packet_mutex, portMAX_DELAY);

    if (packet_count == 0)
    {
        xSemaphoreGive(packet_mutex);
        return false;
    }

    // Ambil paket pertama (time_code terkecil)
    *packet = packet_buffer[0];

    // Shift array
    for (int i = 1; i < packet_count; i++)
    {
        packet_buffer[i - 1] = packet_buffer[i];
    }
    packet_count--;

    xSemaphoreGive(packet_mutex);
    return true;
}

/* ========== ESP-NOW Callbacks ========== */
/*
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len != sizeof(data_packet_t))
    {
        ESP_LOGW(TAG, "Invalid packet size: %d (expected %d)", len, sizeof(data_packet_t));
        return;
    }

    data_packet_t packet;
    memcpy(&packet, data, sizeof(data_packet_t));

    // Validasi checksum
    if (!validate_checksum(&packet))
    {
        ESP_LOGE(TAG, "Checksum validation failed for packet seq=%d", packet.sequence_number);
        return;
    }

    ESP_LOGI(TAG, "ESP-NOW RX: seq=%d, type=%d, len=%d, time=%" PRIu32,
             packet.sequence_number, packet.packet_type, packet.data_len, packet.time_code);

    // Insert ke priority queue
    insert_packet(&packet);

    // Signal scheduler
    xSemaphoreGive(usb_ready_sem);
}*/

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len != sizeof(data_packet_t))
    {
        ESP_LOGW(TAG, "Invalid packet size: %d", len);
        return;
    }

    data_packet_t packet;
    memcpy(&packet, data, sizeof(data_packet_t));

    if (!validate_checksum(&packet))
    {
        ESP_LOGW(TAG, "Bad checksum seq=%d", packet.sequence_number);
        return;
    }

    if (xQueueSend(espnow_to_usb_queue, &packet, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "ESP-NOW->USB queue full, dropping packet");
    }
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    // Untuk mengirim ACK ke sender (opsional)
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

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, sender_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 1;
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    return ESP_OK;
}

/* ========== USB Host Callbacks ========== */

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type)
    {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error occurred, err_no = %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "USB Device disconnected");
        usb_device_ready = false;
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Serial state notif 0x%04X", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
        break;
    }
}

/* ========== USB Host Library Task ========== */

static void usb_lib_task(void *arg)
{
    ESP_LOGI(TAG, "USB library task started");

    while (1)
    {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(TAG, "USB: All devices freed");
        }
    }
}

/* ========== USB Host Init ========== */

static esp_err_t usb_host_init(void)
{
    ESP_LOGI(TAG, "Installing USB Host");

    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create USB library task
    BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    assert(task_created == pdTRUE);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    // Register VCP drivers
    VCP::register_driver<FT23x>();
    VCP::register_driver<CP210x>();
    VCP::register_driver<CH34x>();

    ESP_LOGI(TAG, "USB Host initialized");
    return ESP_OK;
}

/* ========== USB Device Connection Task ========== */

static void usb_connection_task(void *pvParameters)
{
    ESP_LOGI(TAG, "USB connection task started");

    while (true)
    {
        const cdc_acm_host_device_config_t dev_config = {
            .connection_timeout_ms = 10000, // 10 seconds
            .out_buffer_size = 512,
            .in_buffer_size = 512,
            .event_cb = handle_event,
            .data_cb = handle_rx,
            .user_arg = NULL,
        };

        ESP_LOGI(TAG, "Waiting for USB device connection...");
        vcp_device = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));

        if (vcp_device == nullptr)
        {
            ESP_LOGW(TAG, "No VCP device found, retrying in 2s...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "USB device connected!");
        vTaskDelay(pdMS_TO_TICKS(100));

        // Set initial line coding
        cdc_acm_line_coding_t line_coding = {
            .dwDTERate = current_line_coding.dwDTERate,
            .bCharFormat = current_line_coding.bCharFormat,
            .bParityType = current_line_coding.bParityType,
            .bDataBits = current_line_coding.bDataBits,
        };

        esp_err_t ret = vcp_device->line_coding_set(&line_coding);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set line coding: %s", esp_err_to_name(ret));
        }
        else
        {
            ESP_LOGI(TAG, "Line coding set: Baudrate=%" PRIu32 ", Stop=%d, Parity=%d, Data=%d",
                     line_coding.dwDTERate, line_coding.bCharFormat,
                     line_coding.bParityType, line_coding.bDataBits);
        }

        // Set control lines
        ret = vcp_device->set_control_line_state(0, 0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set control lines: %s", esp_err_to_name(ret));
        }
        else
        {
            ESP_LOGI(TAG, "Control lines set: DTR=%d, RTS=%d", 0, 0);
        }

        usb_device_ready = true;
        ESP_LOGI(TAG, "USB device ready for transmission");

        // Wait for disconnection
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);

        ESP_LOGI(TAG, "Device disconnected, cleaning up...");
        vcp_device.reset();
        usb_device_ready = false;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ========== Packet Scheduler Task ========== */
/*
static void packet_scheduler_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Packet scheduler task started");
    data_packet_t packet;

    while (true)
    {
        // Wait for signal (dari ESP-NOW receive atau timeout)
        xSemaphoreTake(usb_ready_sem, pdMS_TO_TICKS(100));

        // Cek apakah USB device ready
        if (!usb_device_ready || vcp_device == nullptr)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Ambil paket dengan time_code terkecil
        if (!get_next_packet(&packet))
        {
            continue; // Tidak ada paket
        }

        ESP_LOGI(TAG, "Processing packet: seq=%d, type=%d, time=%" PRIu32,
                 packet.sequence_number, packet.packet_type, packet.time_code);

        // Process berdasarkan tipe paket
        switch (packet.packet_type)
        {
        case PACKET_TYPE_DATA:
            if (packet.data_len > 0)
            {
                esp_err_t ret = vcp_device->tx_blocking(packet.data, packet.data_len);
                if (ret == ESP_OK)
                {
                    ESP_LOGI(TAG, "Sent %d bytes to USB device", packet.data_len);
                }
                else
                {
                    ESP_LOGE(TAG, "USB TX failed: %s", esp_err_to_name(ret));
                }
            }
            break;

        case PACKET_TYPE_LINE_CODING:
        {
            cdc_acm_line_coding_t line_coding = {
                .dwDTERate = packet.line_coding.dwDTERate,
                .bCharFormat = packet.line_coding.bCharFormat,
                .bParityType = packet.line_coding.bParityType,
                .bDataBits = packet.line_coding.bDataBits,
            };

            esp_err_t ret = vcp_device->line_coding_set(&line_coding);
            if (ret == ESP_OK)
            {
                current_line_coding = packet.line_coding;
                ESP_LOGI(TAG, "Line coding updated: Baudrate=%" PRIu32 ", Stop=%d, Parity=%d, Data=%d",
                         line_coding.dwDTERate, line_coding.bCharFormat,
                         line_coding.bParityType, line_coding.bDataBits);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to update line coding: %s", esp_err_to_name(ret));
            }
        }
        break;

        case PACKET_TYPE_CONTROL_LINE:
        {
            esp_err_t ret = vcp_device->set_control_line_state(
                packet.dtr_state, packet.rts_state);

            if (ret == ESP_OK)
            {
                current_dtr = packet.dtr_state;
                current_rts = packet.rts_state;
                ESP_LOGI(TAG, "Control lines updated: DTR=%d, RTS=%d",
                         packet.dtr_state, packet.rts_state);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to update control lines: %s", esp_err_to_name(ret));
            }
        }
        break;

        case PACKET_TYPE_HEARTBEAT:
            ESP_LOGI(TAG, "Heartbeat received");
            break;

        default:
            ESP_LOGW(TAG, "Unknown packet type: %d", packet.packet_type);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay untuk stabilitas
    }
}*/
static void packet_scheduler_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Optimized packet scheduler started");
    data_packet_t packet;

    while (true)
    {
        if (xQueueReceive(espnow_to_usb_queue, &packet, portMAX_DELAY) == pdTRUE)
        {
            if (!usb_device_ready || vcp_device == nullptr)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            switch (packet.packet_type)
            {
            case PACKET_TYPE_DATA:
                if (packet.data_len > 0)
                {
                    // Non-blocking TX
                    esp_err_t ret = vcp_device->tx_blocking(packet.data, packet.data_len);
                    if (ret == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Sent %d bytes to USB device", packet.data_len);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "USB TX failed: %s", esp_err_to_name(ret));
                    }
                }
                break;

            case PACKET_TYPE_LINE_CODING:
                vcp_device->line_coding_set((cdc_acm_line_coding_t *)&packet.line_coding);
                break;

            case PACKET_TYPE_CONTROL_LINE:
            {
                current_dtr = packet.dtr_state;
                current_rts = packet.rts_state;
                esp_err_t ret = vcp_device->set_control_line_state(
                    current_dtr, current_rts);
                ESP_LOGI(TAG, "Control lines updated: DTR=%d, RTS=%d",
                         packet.rts_state, packet.dtr_state);
            }
            break;
            case PACKET_TYPE_HEARTBEAT:
                ESP_LOGI(TAG, "Heartbeat received");
                break;

            default:
                ESP_LOGW(TAG, "Unknown packet type: %d", packet.packet_type);
                break;
            }
        }
    }
}
/* ========== Statistics Task ========== */

static void statistics_task(void *pvParameters)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Every 10 seconds

        xSemaphoreTake(packet_mutex, portMAX_DELAY);
        int queue_size = packet_count;
        xSemaphoreGive(packet_mutex);

        ESP_LOGI(TAG, "=== Statistics ===");
        ESP_LOGI(TAG, "Queue size: %d / %d", queue_size, MAX_PACKET_QUEUE);
        ESP_LOGI(TAG, "USB ready: %s", usb_device_ready ? "YES" : "NO");
        ESP_LOGI(TAG, "Line coding: Baudrate=%" PRIu32 ", Stop=%d, Parity=%d, Data=%d",
                 current_line_coding.dwDTERate, current_line_coding.bCharFormat,
                 current_line_coding.bParityType, current_line_coding.bDataBits);
        ESP_LOGI(TAG, "Control: DTR=%d, RTS=%d", current_dtr, current_rts);
        ESP_LOGI(TAG, "================");
        wifi_config_t conf;
        esp_wifi_get_config(WIFI_IF_STA, &conf);
        ESP_LOGI(TAG, "WiFi Channel = %d", conf.sta.channel);
    }
}

/* ========== Main ========== */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 ESP-NOW to USB Host Receiver ===");

    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create synchronization primitives
    packet_mutex = xSemaphoreCreateMutex();
    assert(packet_mutex != NULL);

    usb_ready_sem = xSemaphoreCreateBinary();
    assert(usb_ready_sem != NULL);

    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem != NULL);

    espnow_to_usb_queue = xQueueCreate(ESPNOW_TO_USB_QUEUE_SIZE, sizeof(data_packet_t));
    if (espnow_to_usb_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create espnow_to_usb_queue!");
    }
    else
    {
        ESP_LOGI(TAG, "espnow_to_usb_queue created OK");
    }

    usb_to_espnow_queue = xQueueCreate(USB_TO_ESPNOW_QUEUE_SIZE, sizeof(data_packet_t));
    assert(usb_to_espnow_queue != NULL);
    // Print MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "IMPORTANT: Configure sender with this MAC address!");

    // Init ESP-NOW
    ESP_ERROR_CHECK(espnow_init());

    // Init USB Host
    ESP_ERROR_CHECK(usb_host_init());

    // Start tasks
    xTaskCreate(packet_scheduler_task, "scheduler", 4096, NULL, 7, NULL);
    xTaskCreate(usb_to_espnow_task, "usb_to_espnow", 4096, NULL, 6, NULL);
    xTaskCreate(usb_connection_task, "usb_conn", 4096, NULL, 8, NULL);
    // xTaskCreate(packet_scheduler_task, "scheduler", 4096, NULL, 7, NULL);
    xTaskCreate(statistics_task, "statistics", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "All tasks started successfully");
    ESP_LOGI(TAG, "System is ready to receive data via ESP-NOW");
}