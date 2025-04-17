#ifndef CORE1_H
#define CORE1_H

#include <Arduino.h>

// Definizioni esistenti
#define ADC_BUFFER_SIZE 16384
#define CORR_BUFFER_SIZE 256
#define PEAKS_BUFFER_SIZE 256
#define DIST_BUFFER_SIZE 256

// Struttura Bit
typedef struct {
    int value;
    int pos;
} Bit;

// Variabili globali esistenti
extern volatile bool buffer_ready;
extern volatile int32_t available_samples;
extern uint32_t i;
extern volatile uint32_t i_interrupt;
extern int statoacq;
extern volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
extern int32_t filt[CORR_BUFFER_SIZE];
extern int32_t corr[CORR_BUFFER_SIZE];
extern int32_t peaks[PEAKS_BUFFER_SIZE];
extern int32_t dist[DIST_BUFFER_SIZE];
extern Bit bits[DIST_BUFFER_SIZE];
extern uint8_t bytes[10];
extern int32_t num_picchi;
extern int32_t num_distanze;
extern int32_t num_bits;
extern uint16_t country_code;
extern uint64_t device_code;
extern bool crc_ok;

// Nuove variabili globali
extern volatile uint32_t sync_count; // Contatore sync per stampa
extern volatile uint32_t door_sync_count; // Contatore sync per gattaiola
extern volatile uint8_t last_sequence[10]; // Ultima sequenza valida
extern volatile uint64_t last_device_code; // Ultimo Device Code
extern volatile uint32_t last_sync_i; // i dell'ultimo sync
extern volatile bool door_open; // Stato gattaiola
extern volatile TickType_t door_timer_start; // Inizio timer
extern volatile uint32_t display_sync_count; // Contatore sync con CRC OK per stampa

void start_rfid_task();
void IRAM_ATTR onTimer(); // Dichiarazione di onTimer

#endif