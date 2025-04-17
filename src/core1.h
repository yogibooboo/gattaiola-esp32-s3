#ifndef CORE1_H
#define CORE1_H

#include <Arduino.h>

// Definizione della struttura Bit
struct Bit {
    int value;
    int32_t pos;
};

// Dichiarazioni variabili globali
extern volatile bool buffer_ready; // Compatibilità
extern volatile int32_t available_samples; // Contatore di campioni disponibili
extern uint32_t i; // Indice a 32 bit per il consumatore
extern int statoacq;
extern volatile uint16_t adc_buffer[16384]; // Buffer circolare, 2^14
extern int32_t filt[256]; // Ex segnale_filtrato32
extern int32_t corr[256]; // Ex correlazione32
extern int32_t peaks[256]; // Ex picchi32
extern int32_t dist[256]; // Ex distanze32
extern Bit bits[256]; // Ex bits32
extern uint8_t bytes[10]; // Ex bytes32
extern int32_t num_picchi;
extern int32_t num_distanze;
extern uint16_t country_code;
extern uint64_t device_code;
extern bool crc_ok;

// Buffer circolare per logging non bloccante
#define LOG_BUFFER_SIZE 1024
extern char log_buffer[LOG_BUFFER_SIZE];
extern volatile size_t log_head;
extern volatile size_t log_tail;
extern portMUX_TYPE log_mutex;

// Dichiarazioni funzioni
void IRAM_ATTR onTimer();
void media_correlazione_32();
void start_rfid_task();
void logNonBlocking(const char* msg);

#endif