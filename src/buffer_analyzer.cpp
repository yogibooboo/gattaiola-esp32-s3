#include "buffer_analyzer.h"
#include "common.h" // Per local_time, last_millis
#include <freertos/FreeRTOS.h> // Aggiunto per vTaskDelay
#include <freertos/task.h>     // Aggiunto per portTICK_PERIOD_MS

void analyze_buffer_32(const uint16_t* buffer, size_t length) {
    // Buffer locali per evitare conflitti con il task continuo
    int32_t filt[256] = {0};
    int32_t corr[256] = {0};
    int32_t peaks[256] = {0};
    int32_t dist[256] = {0};
    Bit bits[256];
    uint8_t bytes[10] = {0};
    int32_t num_picchi = 0;
    int32_t num_distanze = 0;
    int32_t num_bits = 0;
    uint16_t country_code = 0;
    uint64_t device_code = 0;
    bool crc_ok = false;

    // Variabili di stato
    int32_t max_i = 0, min_i = 0, max_i8 = 0, min_i8 = 0, min_i_iniziale = 0, max_i_iniziale = 0;
    int32_t stato = 1;
    int32_t stato_decodifica = 0, contatore_zeri = 0, contatore_bytes = 0, contatore_bits = 0, stato_decobytes = 0;
    int32_t ultima_distanza = 0, newbit = 0, numbit = 0;
    bool newpeak = false;

    char time_str[20];
    time_t now = local_time + (millis() - last_millis) / 1000;
    strftime(time_str, sizeof(time_str), "%H:%M:%S", localtime(&now));

    // Elaborazione del buffer
    for (uint32_t ia = 0; ia < length; ia++) {
        // Aggiungi vTaskDelay(1) ogni 100 campioni
        /*if (ia % 100 == 0 && ia > 0) {
            vTaskDelay(1 / portTICK_PERIOD_MS); // Pausa di ~1 ms
        }*/

        if (ia >= 32) {
            // Calcolo filtro mobile
            filt[ia & (256 - 1)] = filt[(ia - 1) & (256 - 1)] -
                                   (buffer[(ia - 4)] / 8) +
                                   (buffer[(ia + 3)] / 8);
            corr[(ia - 16) & (256 - 1)] = corr[(ia - 17) & (256 - 1)] -
                                         filt[(ia - 32) & (256 - 1)] +
                                         2 * filt[(ia - 16) & (256 - 1)] -
                                         filt[ia & (256 - 1)];

            newbit = 2;
            numbit = 0;
            newpeak = false;

            // Rilevamento picchi
            if (stato == 1) {
                max_i = max(corr[(ia - 16) & (256 - 1)], max_i);
                max_i8 = max(corr[(ia - 24) & (256 - 1)], max_i8);
                if ((max_i == max_i8) && (max_i != max_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = ia - 24;
                    num_picchi++;
                    stato = -1;
                    min_i = corr[(ia - 16) & (256 - 1)];
                    min_i8 = corr[(ia - 24) & (256 - 1)];
                    min_i_iniziale = min_i8;
                    newpeak = true;
                }
            } else {
                min_i = min(corr[(ia - 16) & (256 - 1)], min_i);
                min_i8 = min(corr[(ia - 24) & (256 - 1)], min_i8);
                if ((min_i == min_i8) && (min_i != min_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = ia - 24;
                    num_picchi++;
                    stato = 1;
                    max_i = corr[(ia - 16) & (256 - 1)];
                    max_i8 = corr[(ia - 24) & (256 - 1)];
                    max_i_iniziale = max_i8;
                    newpeak = true;
                }
            }

            // Calcolo distanze
            if (num_picchi > 1 && newpeak) {
                ultima_distanza = peaks[(num_picchi - 1) & 0xFF] - peaks[(num_picchi - 2) & 0xFF];
                dist[num_distanze & 0xFF] = ultima_distanza;
                num_distanze++;

                // Decodifica bit
                if (stato_decodifica == 0) {
                    if (ultima_distanza >= 24) {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos = ia - 24;
                        num_bits++;
                        newbit = 1;
                        numbit = 1;
                    } else {
                        stato_decodifica = 1;
                    }
                } else if (stato_decodifica == 1) {
                    if ((ultima_distanza + dist[(num_distanze - 2) & 0xFF]) >= 42) {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos = ia - 24 - ultima_distanza;
                        num_bits++;
                        newbit = 1;
                        numbit = 1;
                        if ((ultima_distanza + dist[(num_distanze - 2) & 0xFF]) >= 52) {
                            bits[num_bits & 0xFF].value = 1;
                            bits[num_bits & 0xFF].pos = ia - 24;
                            num_bits++;
                            newbit = 1;
                            numbit = 2;
                            stato_decodifica = 0;
                        } else {
                            stato_decodifica = 2;
                        }
                    } else {
                        bits[num_bits & 0xFF].value = 0;
                        bits[num_bits & 0xFF].pos = ia - 24;
                        num_bits++;
                        newbit = 0;
                        numbit = 1;
                        stato_decodifica = 0;
                    }
                } else if (stato_decodifica == 2) {
                    stato_decodifica = 0;
                    bits[num_bits & 0xFF].value = 0;
                    bits[num_bits & 0xFF].pos = ia - 24;
                    num_bits++;
                    newbit = 0;
                    numbit = 1;
                }
            }

            // Decodifica byte
            while (numbit > 0) {
                switch (stato_decobytes) {
                    case 0:
                        if (newbit == 0) contatore_zeri++;
                        else if (contatore_zeri >= 10) { // Modificato da == 10 a >= 10 come nel tuo codice originale
                            stato_decobytes = 1;
                            contatore_bytes = contatore_bits = 0;
                            for (int j = 0; j < 10; j++) bytes[j] = 0;
                            // Log sync con posizione, bit e inizio (352 campioni = 11 bit indietro)
                            Serial.printf("[%s] Sequenza sync at: %u (bit: %d, inizio: %u, contatore_zeri: %d)\n",
                                          time_str, ia - 24, num_bits, (ia - 24) - 352, contatore_zeri);
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
                                crc_ok = (crc == 0);
                                if (crc_ok) {
                                    country_code = (bytes[5] << 2) | (bytes[4] >> 6);
                                    device_code = ((uint64_t)(bytes[4] & 0x3F) << 32) |
                                                  ((uint64_t)bytes[3] << 24) |
                                                  ((uint64_t)bytes[2] << 16) |
                                                  (bytes[1] << 8) | bytes[0];
                                    Serial.printf("[%s] Byte estratti: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X]\n",
                                                  time_str, bytes[0], bytes[1], bytes[2], bytes[3], bytes[4],
                                                  bytes[5], bytes[6], bytes[7], bytes[8], bytes[9]);
                                } else {
                                    // Log CRC fallito con bit, byte corrente e byte acquisiti
                                    Serial.printf("[%s] CRC fallito at: %u (bit: %d, byte: %d, byte acquisiti: ",
                                                  time_str, ia - 24, num_bits, contatore_bytes);
                                    Serial.print("[");
                                    for (int j = 0; j < contatore_bytes; j++) {
                                        Serial.printf("%02X", bytes[j]);
                                        if (j < contatore_bytes - 1) Serial.print(", ");
                                    }
                                    Serial.println("])");
                                }
                                contatore_zeri = contatore_bytes = 0;
                                stato_decobytes = 0;
                            }
                        } else {
                            // Log perso sync con bit, byte corrente e byte acquisiti
                            Serial.printf("[%s] Perso Sync at: %u (bit: %d, byte: %d, byte acquisiti: ",
                                          time_str, ia - 24, num_bits, contatore_bytes);
                            Serial.print("[");
                            for (int j = 0; j < contatore_bytes; j++) {
                                Serial.printf("%02X", bytes[j]);
                                if (j < contatore_bytes - 1) Serial.print(", ");
                            }
                            Serial.println("])");
                            contatore_zeri = contatore_bits = contatore_bytes = 0;
                            stato_decobytes = 0;
                        }
                        break;
                }
                numbit--;
            }
        }
    }
}