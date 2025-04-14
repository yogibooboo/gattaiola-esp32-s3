#ifndef CORE1_H
#define CORE1_H

#include <Arduino.h>

// Dichiarazioni variabili globali
extern volatile bool buffer_ready;
extern volatile size_t buffer_index;
extern int statoacq;
extern uint16_t adc_buffer[10000];
extern int32_t segnale_filtrato32[256];
extern int32_t correlazione32[256];
extern int32_t picchi32[1000];
extern int32_t distanze32[1000];
struct Bit { int value; int32_t pos; };
extern Bit bits32[1000];
extern uint8_t bytes32[10];
extern int32_t num_picchi;
extern int32_t num_distanze;
extern int32_t num_bits;
extern uint16_t country_code;
extern uint64_t device_code;
extern bool crc_ok;

// Dichiarazioni funzioni
void IRAM_ATTR onTimer();
void media_correlazione_32(uint16_t* segnale, int32_t* filt, int32_t* corr, int32_t* peaks, int32_t* dists, Bit* bits, uint8_t* bytes,
                          int32_t& n_peaks, int32_t& n_dists, int32_t& n_bits, uint16_t& country, uint64_t& device, bool& crc_valid);
void start_rfid_task();

#endif