#include "core1.h"
#include "analog.h" // Per fadcBusy, fadcResult, fadcStart

// Definizione variabili globali
volatile bool buffer_ready = false;
volatile size_t buffer_index = 0;
int statoacq = 0;
uint16_t adc_buffer[10000];
int32_t segnale_filtrato32[256];
int32_t correlazione32[256];
int32_t picchi32[1000];
int32_t distanze32[1000];
Bit bits32[1000];
uint8_t bytes32[10];
int32_t num_picchi = 0;
int32_t num_distanze = 0;
int32_t num_bits = 0;
uint16_t country_code = 0;
uint64_t device_code = 0;
bool crc_ok = false;

extern hw_timer_t *timer; // Dichiarato in main.cpp

// Funzione di interrupt
void IRAM_ATTR onTimer() {
    digitalWrite(21, HIGH);
    if (buffer_index < 10000) {
        if (!fadcBusy()) {
            adc_buffer[buffer_index++] = fadcResult();
            fadcStart(0);
        }
    }
    if (buffer_index >= 10000) {
        buffer_ready = true;
        timerAlarmDisable(timer);
    }
    digitalWrite(21, LOW);
}

// Funzione di analisi
void media_correlazione_32(uint16_t* segnale, int32_t* filt, int32_t* corr, int32_t* peaks, int32_t* dists, Bit* bits, uint8_t* bytes,
                          int32_t& n_peaks, int32_t& n_dists, int32_t& n_bits, uint16_t& country, uint64_t& device, bool& crc_valid) {
    const int N = 10000;
    const int larghezza_finestra = 8;
    const int lunghezza_correlazione = 32;
    const int soglia_mezzo_bit = 24;
    int32_t stato_decodifica = 0, contatore_zeri = 0, contatore_bytes = 0, contatore_bits = 0, stato_decobytes = 0;
    int32_t ultima_distanza = 0, newbit = 0, numbit = 0;
    bool newpeak = false;

    uint32_t start_time = millis();

    n_peaks = n_dists = n_bits = 0;
    for (int i = 0; i < 10; i++) bytes[i] = 0;
    for (int i = 0; i < 256; i++) filt[i] = corr[i] = 0;

    int i = 32;
    if (i < N && i + 3 < N) {
        int32_t somma_media = 0;
        for (int j = i - 4; j < i + 4; j++) somma_media += segnale[j];
        filt[i & (256 - 1)] = somma_media / larghezza_finestra;
    }
    if (i >= lunghezza_correlazione) {
        corr[16 & (256 - 1)] = 0;
        for (int j = 0; j < 16; j++) corr[16 & (256 - 1)] += filt[j & (256 - 1)];
        for (int j = 16; j < 32; j++) corr[16 & (256 - 1)] -= filt[j & (256 - 1)];
    }

    int32_t max_i = corr[16 & (256 - 1)], min_i = corr[16 & (256 - 1)];
    int32_t max_i8 = corr[8 & (256 - 1)], min_i8 = corr[8 & (256 - 1)];
    int32_t stato = 1;

    for (i = 33; i < N - 4; i++) {
        filt[i & (256 - 1)] = filt[(i-1) & (256 - 1)] - (segnale[i-4] / larghezza_finestra) + (segnale[i+3] / larghezza_finestra);
        corr[(i-16) & (256 - 1)] = corr[(i-17) & (256 - 1)] - filt[(i-32) & (256 - 1)] + 2 * filt[(i-16) & (256 - 1)] - filt[i & (256 - 1)];

        newbit = 2; numbit = 0; newpeak = false;

        if (stato == 1) {
            max_i = max(corr[(i-16) & (256 - 1)], max_i);
            max_i8 = max(corr[(i-24) & (256 - 1)], max_i8);
            if (max_i == max_i8 && n_peaks < 1000) {
                peaks[n_peaks++] = i - 24;
                stato = -1;
                min_i = corr[(i-16) & (256 - 1)];
                min_i8 = corr[(i-24) & (256 - 1)];
                newpeak = true;
            }
        } else {
            min_i = min(corr[(i-16) & (256 - 1)], min_i);
            min_i8 = min(corr[(i-24) & (256 - 1)], min_i8);
            if (min_i == min_i8 && n_peaks < 1000) {
                peaks[n_peaks++] = i - 24;
                stato = 1;
                max_i = corr[(i-16) & (256 - 1)];
                max_i8 = corr[(i-24) & (256 - 1)];
                newpeak = true;
            }
        }

        if (n_peaks > 1 && newpeak) {
            int32_t nuova_distanza = peaks[n_peaks-1] - peaks[n_peaks-2];
            if (n_dists < 1000) dists[n_dists++] = nuova_distanza;

            if (stato_decodifica == 0) {
                if (nuova_distanza >= soglia_mezzo_bit) {
                    if (n_bits < 1000) { bits[n_bits].value = 1; bits[n_bits++].pos = i - 24; }
                    newbit = 1; numbit = 1;
                } else {
                    ultima_distanza = nuova_distanza;
                    stato_decodifica = 1;
                }
            } else {
                if (nuova_distanza < soglia_mezzo_bit) {
                    if (n_bits < 1000) { bits[n_bits].value = 0; bits[n_bits++].pos = i - 24; }
                    newbit = 0; numbit = 1;
                } else {
                    if (n_bits + 1 < 1000) {
                        bits[n_bits].value = 1; bits[n_bits++].pos = i - 24 - nuova_distanza;
                        bits[n_bits].value = 1; bits[n_bits++].pos = i - 24;
                    }
                    newbit = 1; numbit = 2;
                }
                stato_decodifica = 0;
            }
        }

        while (numbit > 0) {
            switch (stato_decobytes) {
                case 0:
                    if (newbit == 0) contatore_zeri++;
                    else if (contatore_zeri >= 10) {
                        stato_decobytes = 1;
                        contatore_bytes = contatore_bits = 0;
                        for (int j = 0; j < 10; j++) bytes[j] = 0;
                        Serial.print("Core 1: Sequenza sync at: ");
                        Serial.println(i);
                    } else contatore_zeri = 0;
                    break;

                case 1:
                    if (contatore_bits < 8) {
                        bytes[contatore_bytes] >>= 1;
                        if (newbit == 1) bytes[contatore_bytes] |= 0x80;
                        contatore_bits++;
                    } else if (newbit == 1) {
                        contatore_bytes++;
                        contatore_bits = 0;
                        if (contatore_bytes >= 10) {
                            Serial.print("Core 1: Byte estratti: [");
                            for (int j = 0; j < 10; j++) {
                                Serial.print(bytes[j], HEX);
                                if (j < 9) Serial.print(", ");
                                else Serial.println("]");
                            }
                            contatore_zeri = contatore_bytes = 0;
                            stato_decobytes = 0;

                            uint16_t crc = 0x0;
                            const uint16_t polynomial = 0x1021;
                            for (int b = 0; b < 10; b++) {
                                uint8_t byte = bytes[b];
                                for (int j = 0; j < 8; j++) {
                                    bool bit = ((byte >> j) & 1) == 1;
                                    bool c15 = ((crc >> 15) & 1) == 1;
                                    crc <<= 1;
                                    if (c15 ^ bit) crc ^= polynomial;
                                    crc &= 0xffff;
                                }
                            }
                            crc_valid = (crc == 0);
                            Serial.print("Core 1: CRC: ");
                            Serial.println(crc_valid ? "OK" : "KO");
                            if (crc_valid) {
                                digitalWrite(21, HIGH);
                                country = (bytes[5] << 2) | (bytes[4] >> 6);
                                device = ((uint64_t)(bytes[4] & 0x3F) << 32) | ((uint64_t)bytes[3] << 24) |
                                         ((uint64_t)bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
                                Serial.print("Core 1: Country Code: ");
                                Serial.println(country);
                                Serial.print("Core 1: Device Code: ");
                                Serial.println((uint64_t)device);
                            }
                        }
                    } else {
                        Serial.print("Core 1: Perso sync at: ");
                        Serial.println(i);
                        contatore_zeri = contatore_bits = contatore_bytes = 0;
                        stato_decobytes = 0;
                    }
                    break;
            }
            numbit--;
        }
    }

    uint32_t end_time = millis();
    Serial.print("Core 1: Durata analisi: ");
    Serial.print(end_time - start_time);
    Serial.println(" ms");
    Serial.println("Core 1: analisi completata");
}

// Task FreeRTOS per il core 1
static void rfid_task(void *pvParameters) {
    Serial.println("Core 1: task in esecuzione");
    while (1) {
        int sblue = digitalRead(39); // pblue
        if (sblue == 0) {
            Serial.println("Core 1: Pulsante premuto");
            statoacq ^= 1;
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        if (statoacq == 1) {
            Serial.println("Core 1: acquisizione iniziata");
            buffer_index = 0;
            buffer_ready = false;

            timerAlarmEnable(timer);
            timerStart(timer);
            uint32_t start_time = micros();
            while (!buffer_ready) {
                // Vuoto
            }
            uint32_t end_time = micros();
            timerAlarmDisable(timer);

            Serial.print("Core 1: ADC Value (elemento 100): ");
            Serial.println(adc_buffer[100]);
            Serial.print("Core 1: Tempo acquisizione: ");
            Serial.print(end_time - start_time);
            Serial.println(" us");
            Serial.print("Core 1: Frequenza campionamento: ");
            Serial.print((float)10000 / ((end_time - start_time) / 1000000.0));
            Serial.println(" Hz");
            Serial.println("Core 1: Primi 10 valori ADC:");
            for (int i = 0; i < 10; i++) {
                Serial.print(adc_buffer[i]);
                Serial.print(" ");
            }
            Serial.println();

            media_correlazione_32(adc_buffer, segnale_filtrato32, correlazione32, picchi32, distanze32, bits32, bytes32,
                                 num_picchi, num_distanze, num_bits, country_code, device_code, crc_ok);
            statoacq = 0;
            Serial.println("Core 1: Acquisizione completata");
        }

        digitalWrite(21, LOW); // ledverde
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Avvia il task sul core 1
void start_rfid_task() {
    xTaskCreatePinnedToCore(rfid_task, "RFID_Task", 8192, NULL, 5, NULL, 1);
}