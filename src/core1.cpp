#include "core1.h"
#include "analog.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"


// Definizione variabili globali
volatile bool buffer_ready = false; // Compatibilità
volatile int32_t available_samples = 0;
uint32_t i = -4; // Inizializzato a -4
volatile uint32_t i_interrupt = 0; // Esplicito volatile
int statoacq = 0;
volatile uint16_t adc_buffer[16384];
int32_t filt[256];
int32_t corr[256];
int32_t peaks[256];
int32_t dist[256];
Bit bits[256];
uint8_t bytes[10];
int32_t num_picchi = 0;
int32_t num_distanze = 0;
int32_t num_bits = 0;
uint16_t country_code = 0;
uint64_t device_code = 0;
bool crc_ok = false;

// Buffer circolare per logging
char log_buffer[LOG_BUFFER_SIZE];
volatile size_t log_head = 0;
volatile size_t log_tail = 0;
portMUX_TYPE log_mutex = portMUX_INITIALIZER_UNLOCKED;

// Logging non bloccante
void logNonBlocking(const char* msg) {
    size_t len = strlen(msg);
    if (len + 1 >= LOG_BUFFER_SIZE) return;
    taskENTER_CRITICAL(&log_mutex);
    size_t space = (log_tail > log_head) ? (log_tail - log_head - 1) : (LOG_BUFFER_SIZE - log_head + log_tail - 1);
    if (len + 1 <= space) {
        strncpy(log_buffer + log_head, msg, len + 1);
        log_head = (log_head + len + 1) % LOG_BUFFER_SIZE;
    }
    taskEXIT_CRITICAL(&log_mutex);
}

// Interrupt produttore
void IRAM_ATTR onTimer() {
    //digitalWrite(21, HIGH);
    if (!fadcBusy()) {
        adc_buffer[i_interrupt & 0x3FFF] = fadcResult();
        available_samples++; // Atomico
        i_interrupt++;
        fadcStart(0);
    }
    //digitalWrite(21, LOW);
}

// Funzione di analisi incrementale
void media_correlazione_32() {
    // Inizializzazione una tantum
    static bool initialized = false;
    static int32_t max_i = 0, min_i = 0, max_i8 = 0, min_i8 = 0,min_i_iniziale=0, max_i_iniziale=0;
    static int32_t stato = 1;
    static int32_t stato_decodifica = 0, contatore_zeri = 0, contatore_bytes = 0, contatore_bits = 0, stato_decobytes = 0;
    static int32_t ultima_distanza = 0, newbit = 0, numbit = 0;
    static bool newpeak = false;
    static bool last_pblues = true; // Stato precedente di pblues (attivo basso)
    pinMode(41, OUTPUT);
    pinMode(39, OUTPUT);
    GPIO.enable1_w1ts.val = (1 << (41 - 32));
    bool gpio41_state = false,gpio39_state = false;
    static int32_t loop_counter = 0;
    if (!initialized) {
        num_picchi = num_distanze = num_bits = 0;
        for (int j = 0; j < 10; j++) bytes[j] = 0;
        for (int j = 0; j < 256; j++) filt[j] = corr[j] = 0;
        initialized = true;
        logNonBlocking("Core 1: media_correlazione_32 inizializzata\n");
    }

    while (true) {

        

        
        /*  gpio41_state = !gpio41_state;
          if (gpio41_state) {
            digitalWrite(41,0);


          } else {
            digitalWrite(41,1);
          }

  */
                
          

         // Log di debug per i, i_interrupt, e available_samples ogni 10000 iterazioni
       if (++loop_counter % 100000 == 0) {
            char msg[128];
            snprintf(msg, sizeof(msg), "Core 1: i: %lu, i_interrupt: %lu, available_samples: %ld\n",
                     (unsigned long)i, (unsigned long)i_interrupt, (long)available_samples);
            logNonBlocking(msg);
        }
/* 
        // Log di debug esistente ogni 100000 iterazioni
        if (loop_counter % 100000 == 0) {
            char msg[64];
            snprintf(msg, sizeof(msg), "Core 1: Loop, samples: %ld\n", (long)available_samples);
            logNonBlocking(msg);
        } */

        // Controlla pressione di pblues (GPIO 39, attivo basso)
       /* bool pblues = digitalRead(39);
        if (last_pblues && !pblues) { // Fronte di discesa
            char msg[4096];
            int pos = snprintf(msg, sizeof(msg), "Core 1: available_samples: %ld\n", (long)available_samples);
            pos += snprintf(msg + pos, sizeof(msg) - pos, "Core 1: num_distanze: %ld\n", (long)num_distanze);
            pos += snprintf(msg + pos, sizeof(msg) - pos, "Core 1: dist: [");
            for (int j = 0; j < 256; j++) {
                pos += snprintf(msg + pos, sizeof(msg) - pos, "%ld", (long)dist[j]);
                if (j < 255) pos += snprintf(msg + pos, sizeof(msg) - pos, ", ");
            }
            pos += snprintf(msg + pos, sizeof(msg) - pos, "]\nCore 1: peaks: [");
            for (int j = 0; j < 256; j++) {
                pos += snprintf(msg + pos, sizeof(msg) - pos, "%ld", (long)peaks[j]);
                if (j < 255) pos += snprintf(msg + pos, sizeof(msg) - pos, ", ");
            }
            pos += snprintf(msg + pos, sizeof(msg) - pos, "]\n");
            logNonBlocking(msg);
        }
        last_pblues = pblues;*/

        // Controlla se ci sono dati

        

        if (available_samples <= 0) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
        }
       

        // Elabora un campione
        available_samples--; // Atomico

        // Logica di analisi
        const int larghezza_finestra = 8;
        const int lunghezza_correlazione = 32;
        const int soglia_mezzo_bit = 24;

        if (i >= 32) {
            // Calcolo filtro mobile
            filt[i & (256 - 1)] = filt[(i-1) & (256 - 1)] - 
                (adc_buffer[(i-4) & 0x3FFF] / larghezza_finestra) + 
                (adc_buffer[(i+3) & 0x3FFF] / larghezza_finestra);
            corr[(i-16) & (256 - 1)] = corr[(i-17) & (256 - 1)] - 
                filt[(i-32) & (256 - 1)] + 
                2 * filt[(i-16) & (256 - 1)] - 
                filt[i & (256 - 1)];

            newbit = 2; numbit = 0; newpeak = false;

            // Rilevamento picchi
            if (stato == 1) {
                max_i = max(corr[(i-16) & (256 - 1)], max_i);
                max_i8 = max(corr[(i-24) & (256 - 1)], max_i8);
                if ((max_i == max_i8)&(max_i != max_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = i - 24;
                    num_picchi++;
                    stato = -1;
                    min_i = corr[(i-16) & (256 - 1)];
                    min_i8 = corr[(i-24) & (256 - 1)];
                    min_i_iniziale=min_i8;
                    newpeak = true;
                }
            } else {
                min_i = min(corr[(i-16) & (256 - 1)], min_i);
                min_i8 = min(corr[(i-24) & (256 - 1)], min_i8);
                if ((min_i == min_i8)&(min_i != min_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = i - 24;
                    num_picchi++;
                    stato = 1;
                    max_i = corr[(i-16) & (256 - 1)];
                    max_i8 = corr[(i-24) & (256 - 1)];
                    max_i_iniziale=max_i8;
                    newpeak = true;
                }
            }


 


            // Calcolo distanze
            if (num_picchi > 1 && newpeak) {
                int32_t nuova_distanza = peaks[(num_picchi-1) & 0xFF] - peaks[(num_picchi-2) & 0xFF];
                dist[num_distanze & 0xFF] = nuova_distanza;
                num_distanze++;

                // Decodifica bit
                if (stato_decodifica == 0) {
                    if (nuova_distanza >= soglia_mezzo_bit) {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos = i - 24;
                        num_bits++;
                        newbit = 1; numbit = 1;
                    } else {
                        ultima_distanza = nuova_distanza;
                        stato_decodifica = 1;
                    }
                } else {
                    if (nuova_distanza < soglia_mezzo_bit) {
                        bits[num_bits & 0xFF].value = 0;
                        bits[num_bits & 0xFF].pos = i - 24;
                        num_bits++;
                        newbit = 0; numbit = 1;
                    } else {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos = i - 24 - nuova_distanza;
                        num_bits++;
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos = i - 24;
                        num_bits++;
                        newbit = 1; numbit = 2;
                    }
                    stato_decodifica = 0;
                }
            }





            // Decodifica byte
            while (numbit > 0) {
                switch (stato_decobytes) {
                    case 0:
                        if (newbit == 0) contatore_zeri++;
                        else if (contatore_zeri >= 10) {
                            stato_decobytes = 1;
                            contatore_bytes = contatore_bits = 0;
                            for (int j = 0; j < 10; j++) bytes[j] = 0;
                            char msg[64];
                            snprintf(msg, sizeof(msg), "Core 1: Sequenza sync at: %ld\n", (long)i);
                            logNonBlocking(msg);
                            // Log per dist, num_distanze, newbit, contatore_zeri, stato_decobytes
                            {
                                char dist_msg[4096];
                                int pos = snprintf(dist_msg, sizeof(dist_msg), "Core 1: num_distanze: %ld\nCore 1: dist: [", (long)num_distanze);
                                for (int j = 0; j < 256; j++) {
                                    pos += snprintf(dist_msg + pos, sizeof(dist_msg) - pos, "%ld", (long)dist[j]);
                                    if (j < 255) pos += snprintf(dist_msg + pos, sizeof(dist_msg) - pos, ", ");
                                }
                                pos += snprintf(dist_msg + pos, sizeof(dist_msg) - pos, "]\n");
                                logNonBlocking(dist_msg);
                                char state_msg[128];
                                snprintf(state_msg, sizeof(state_msg), "Core 1: newbit: %ld, contatore_zeri: %ld, stato_decobytes: %ld\n",
                                         (long)newbit, (long)contatore_zeri, (long)stato_decobytes);
                                logNonBlocking(state_msg);
                            }
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
                                char msg[128];
                                snprintf(msg, sizeof(msg), "Core 1: Byte estratti: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X]\n",
                                         bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8], bytes[9]);
                                logNonBlocking(msg);

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
                                snprintf(msg, sizeof(msg), "Core 1: CRC: %s\n", crc_ok ? "OK" : "KO");
                                logNonBlocking(msg);

                                if (crc_ok) {
                                    digitalWrite(21, HIGH);
                                    country_code = (bytes[5] << 2) | (bytes[4] >> 6);
                                    device_code = ((uint64_t)(bytes[4] & 0x3F) << 32) | ((uint64_t)bytes[3] << 24) |
                                                  ((uint64_t)bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
                                    snprintf(msg, sizeof(msg), "Core 1: Country Code: %u\n", country_code);
                                    logNonBlocking(msg);
                                    snprintf(msg, sizeof(msg), "Core 1: Device Code: %llu\n", (unsigned long long)device_code);
                                    logNonBlocking(msg);
                                } else {
                                    // Log dei bit per debug (ultimi 101 bit)
                                    char bit_msg[256];
                                    int msg_pos = snprintf(bit_msg, sizeof(bit_msg), "Core 1: Bit per CRC KO: ");
                                    for (int j = max(0, num_bits - 101); j < num_bits; j++) {
                                        int idx = j & 0xFF;
                                        msg_pos += snprintf(bit_msg + msg_pos, sizeof(bit_msg) - msg_pos, "%d", bits[idx].value);
                                    }
                                    snprintf(bit_msg + msg_pos, sizeof(bit_msg) - msg_pos, "\n");
                                    logNonBlocking(bit_msg);
                                }

                                contatore_zeri = contatore_bytes = 0;
                                stato_decobytes = 0;
                            }
                        } else {
                            char msg[64];
                            snprintf(msg, sizeof(msg), "Core 1: Perso sync at: %ld\n", (long)i);
                            logNonBlocking(msg);
                            contatore_zeri = contatore_bits = contatore_bytes = 0;
                            stato_decobytes = 0;
                        }
                        break;
                }
                numbit--;
            }
        }
        
        i++; // Avanza al prossimo campione
 /*       gpio39_state = !gpio39_state;
        if (gpio41_state) {
          digitalWrite(39,0);


        } else {
          digitalWrite(39,1);
        } */
    }
    
}

// Task FreeRTOS per il core 1
static void rfid_task(void *pvParameters) {
    logNonBlocking("Core 1: task in esecuzione\n");
    media_correlazione_32(); // Avvia analisi continua
    vTaskDelete(NULL); // Non raggiunto, ma per sicurezza
}

// Avvia il task sul core 1
void start_rfid_task() {
    xTaskCreatePinnedToCore(rfid_task, "RFID_Task", 8192, NULL, 5, NULL, 1);
}