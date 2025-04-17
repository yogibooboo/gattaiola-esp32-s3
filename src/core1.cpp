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

// Nuove variabili globali
volatile uint32_t sync_count = 0;
volatile uint32_t door_sync_count = 0;
volatile uint8_t last_sequence[10];
volatile uint64_t last_device_code = 0;
volatile uint32_t last_sync_i = 0;
volatile bool door_open = false;
volatile TickType_t door_timer_start = 0;
volatile uint32_t display_sync_count = 0; // Contatore sync con CRC OK

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
    static int32_t max_i = 0, min_i = 0, max_i8 = 0, min_i8 = 0, min_i_iniziale=0, max_i_iniziale=0;
    static int32_t stato = 1;
    static int32_t stato_decodifica = 0, contatore_zeri = 0, contatore_bytes = 0, contatore_bits = 0, stato_decobytes = 0;
    static int32_t ultima_distanza = 0, newbit = 0, numbit = 0;
    static bool newpeak = false;
    static bool last_pblues = true; // Stato precedente di pblues (attivo basso)
    pinMode(41, OUTPUT);
    pinMode(39, OUTPUT);
    GPIO.enable1_w1ts.val = (1 << (41 - 32));
    bool gpio41_state = false, gpio39_state = false;
    static int32_t loop_counter = 0;
    if (!initialized) {
        num_picchi = num_distanze = num_bits = 0;
        for (int j = 0; j < 10; j++) bytes[j] = 0;
        for (int j = 0; j < 256; j++) filt[j] = corr[j] = 0;
        initialized = true;
    }

    while (true) {
        // Log di debug per i, i_interrupt, e available_samples ogni 10000 iterazioni
        if (++loop_counter % 100000 == 0) {
            // Rimuovo logNonBlocking
        }

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
                if ((max_i == max_i8) & (max_i != max_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = i - 24;
                    num_picchi++;
                    stato = -1;
                    min_i = corr[(i-16) & (256 - 1)];
                    min_i8 = corr[(i-24) & (256 - 1)];
                    min_i_iniziale = min_i8;
                    newpeak = true;
                }
            } else {
                min_i = min(corr[(i-16) & (256 - 1)], min_i);
                min_i8 = min(corr[(i-24) & (256 - 1)], min_i8);
                if ((min_i == min_i8) & (min_i != min_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = i - 24;
                    num_picchi++;
                    stato = 1;
                    max_i = corr[(i-16) & (256 - 1)];
                    max_i8 = corr[(i-24) & (256 - 1)];
                    max_i_iniziale = max_i8;
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
                            sync_count++; // Incremento a ogni sync
                            last_sync_i = i; // Aggiunto
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
                                    digitalWrite(21, HIGH);
                                    country_code = (bytes[5] << 2) | (bytes[4] >> 6);
                                    device_code = ((uint64_t)(bytes[4] & 0x3F) << 32) | ((uint64_t)bytes[3] << 24) |
                                                  ((uint64_t)bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
                                    memcpy((void*)last_sequence, bytes, 10); // Corretto con cast
                                    last_device_code = device_code; // Aggiunto
                                    door_sync_count++; // Incremento solo con CRC OK
                                    display_sync_count++; // Incremento solo con CRC OK
                                }
                                contatore_zeri = contatore_bytes = 0;
                                stato_decobytes = 0;
                            }
                        } else {
                            contatore_zeri = contatore_bits = contatore_bytes = 0;
                            stato_decobytes = 0;
                        }
                        break;
                }
                numbit--;
            }
        }
        
        i++; // Avanza al prossimo campione
    }
}

// Task FreeRTOS per il core 1
static void rfid_task(void *pvParameters) {
    media_correlazione_32(); // Avvia analisi continua
    vTaskDelete(NULL); // Non raggiunto, ma per sicurezza
}

// Avvia il task sul core 1
void start_rfid_task() {
    xTaskCreatePinnedToCore(rfid_task, "RFID_Task", 8192, NULL, 5, NULL, 1);
}