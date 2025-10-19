/*
 * Struktur data untuk komunikasi ESP-NOW
 * File: data_packet.h
 */

#ifndef DATA_PACKET_H
#define DATA_PACKET_H

#include <stdint.h>
#include <stdbool.h>

// Maksimal ukuran data dalam satu paket ESP-NOW (ESP-NOW max: 250 bytes)
#define MAX_DATA_SIZE 250

// Tipe paket
typedef enum {
    PACKET_TYPE_DATA = 0,
    PACKET_TYPE_LINE_CODING = 1,
    PACKET_TYPE_CONTROL_LINE = 2,
    PACKET_TYPE_HEARTBEAT = 3,
    PACKET_TYPE_ACK = 4
} packet_type_t;

// Struktur line coding (sesuai USB CDC spec)
typedef struct {
    uint32_t dwDTERate;      // Baudrate
    uint8_t  bCharFormat;    // Stop bits (0=1bit, 1=1.5bit, 2=2bit)
    uint8_t  bParityType;    // Parity (0=None, 1=Odd, 2=Even, 3=Mark, 4=Space)
    uint8_t  bDataBits;      // Data bits (5,6,7,8)
} __attribute__((packed)) line_coding_t;

// Struktur paket data utama
typedef struct {
    uint32_t time_code;           // Timestamp untuk penjadwalan (microseconds)
    uint16_t sequence_number;     // Nomor urut paket
    uint8_t  packet_type;         // Tipe paket (packet_type_t)
    uint16_t data_len;            // Panjang data aktual
    uint8_t  data[MAX_DATA_SIZE]; // Data serial (RX/TX)
    
    // Control lines
    bool     dtr_state;           // Status DTR (Data Terminal Ready)
    bool     rts_state;           // Status RTS (Request To Send)
    
    // Line coding
    line_coding_t line_coding;
    
    // Validasi
    uint16_t checksum;            // Checksum untuk validasi
} __attribute__((packed)) data_packet_t;

// Fungsi untuk menghitung checksum
static inline uint16_t calculate_checksum(const data_packet_t* packet) {
    uint32_t sum = 0;
    sum += packet->time_code;
    sum += packet->sequence_number;
    sum += packet->packet_type;
    sum += packet->data_len;
    
    for (int i = 0; i < packet->data_len && i < MAX_DATA_SIZE; i++) {
        sum += packet->data[i];
    }
    
    sum += packet->dtr_state ? 1 : 0;
    sum += packet->rts_state ? 1 : 0;
    sum += packet->line_coding.dwDTERate;
    sum += packet->line_coding.bCharFormat;
    sum += packet->line_coding.bParityType;
    sum += packet->line_coding.bDataBits;
    
    return (uint16_t)(sum & 0xFFFF);
}

// Fungsi untuk validasi checksum
static inline bool validate_checksum(const data_packet_t* packet) {
    uint16_t calculated = calculate_checksum(packet);
    return (calculated == packet->checksum);
}

// Fungsi untuk mempersiapkan paket
static inline void prepare_packet(data_packet_t* packet) {
    packet->checksum = calculate_checksum(packet);
}
typedef enum {
    LED_RX = 0,
    LED_TX = 1,
    LED_DTR = 2,
    LED_RTS = 3
} led_type_t;

typedef struct {
    led_type_t led_type;
    uint32_t timestamp;  // Untuk tracking
} led_event_t;

#define LED_BLINK_DURATION_MS 2
#endif // DATA_PACKET_H