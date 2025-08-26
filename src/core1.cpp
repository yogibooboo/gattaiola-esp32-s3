#include "core1.h"
#include "analog.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"

// Definizione variabili globali
volatile bool buffer_ready = false; // CompatibilitÃ 
volatile int32_t available_samples = 0;
uint32_t ia = -4; // Inizializzato a -4
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
volatile uint16_t last_country_code = 0;
volatile uint32_t last_sync_i = 0;
volatile bool door_open = false;
volatile TickType_t door_timer_start = 0;
volatile uint32_t display_sync_count = 0; // Contatore sync con CRC OK
uint16_t datoadc=0;

// Variabili per debug - export delle variabili statiche
int32_t debug_max_i = 0;
int32_t debug_min_i = 0; 
int32_t debug_max_i8 = 0;
int32_t debug_min_i8 = 0;
int32_t debug_stato = 1;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Interrupt produttore
void IRAM_ATTR onTimer() {
    //digitalWrite(21, HIGH);
    if (!fadcBusy()) {
        //adc_buffer[i_interrupt & 0x3FFF] = fadcResult();
        datoadc= (fadcResult()&0xFFF);
        adc_buffer[i_interrupt & 0x3FFF] = datoadc;
        //available_samples++; // Atomico
        i_interrupt++;
        adc_buffer[i_interrupt & 0x3FFF] = datoadc;
        i_interrupt++;
        fadcStart(3);  //0707 era fadcStart(0);
    }
    //digitalWrite(21, LOW);
}

// Funzione di analisi incrementale
void media_correlazione_32() {
    // Inizializzazione una tantum
    static bool initialized = false;
    static int32_t min_i_iniziale=0, max_i_iniziale=0;
    static int32_t stato_decodifica = 0, contatore_zeri = 0, contatore_bytes = 0, contatore_bits = 0, stato_decobytes = 0;
    static int32_t ultima_distanza = 0, newbit = 0, numbit = 0;
    static bool newpeak = false;
    static bool last_pblues = true; // Stato precedente di pblues (attivo basso)
    static int32_t nuova_distanza=0;
    uint8_t confro=0;   //1507
    pinMode(41, OUTPUT);
    //pinMode(39, OUTPUT);
    GPIO.enable1_w1ts.val = (1 << (41 - 32));
    //bool gpio41_state = false, gpio39_state = false;
    static int32_t loop_counter = 0;
    if (!initialized) {
        num_picchi = num_distanze = num_bits = 0;
        for (int j = 0; j < 10; j++) bytes[j] = 0;
        for (int j = 0; j < 256; j++) filt[j] = corr[j] = 0;
        initialized = true;
    }
    bool wait10000=true;
    while (true) {
        
        // Controlla se ci sono dati
        available_samples=(i_interrupt-ia); ////&0x3fff;

        /*while (available_samples <= 0) {
            available_samples = (i_interrupt - ia) & 0x3FFF;
        }*/
        
        if (available_samples <= 10) {    //10
            vTaskDelay(72 / portTICK_PERIOD_MS);     //74
            continue;
        }
        
       /* if (wait10000){
            if (available_samples <= 10000) {
                vTaskDelay(1 / portTICK_PERIOD_MS);
                continue;
            } else{
                wait10000 = false;
            }
        } else{
            if (available_samples <= 0) {
                vTaskDelay(1 / portTICK_PERIOD_MS);
                wait10000=true;
                continue;
            } 
        } */


        // Elabora un campione
        //portENTER_CRITICAL_ISR(&mux);
        //available_samples--; // sembra non essere per niente atomico
        //portEXIT_CRITICAL_ISR(&mux);

        // Logica di analisi
        const int larghezza_finestra = 8;
        const int lunghezza_correlazione = 32;
        const int soglia_mezzo_bit = 25;      //1507

        if (ia >= 32) {
            // Calcolo filtro mobile
            int32_t sum = 0;
            for(int j = 0; j < 8; j++) {
                sum += adc_buffer[(ia-4+j) & 0x3FFF];
            }
            filt[ia & 255] = sum; // 8;  // O anche sum senza divisione
            int32_t sum_corr = 0;
            for(int j = 0; j < 16; j++) {
                sum_corr += filt[(ia-31+j) & 255];  // Primi 16: somma
                sum_corr -= filt[(ia-15+j) & 255];  // Ultimi 16: sottrai  
            }
            corr[(ia-16) & 255] = sum_corr;

            newbit = 2; numbit = 0; newpeak = false;

            // Rilevamento picchi
            if (debug_stato == 1) {
                debug_max_i = max(corr[(ia-16) & (256 - 1)], debug_max_i);
                debug_max_i8 = max(corr[(ia-24) & (256 - 1)], debug_max_i8);
                if ((debug_max_i == debug_max_i8) & (debug_max_i != max_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = ia - 24;
                    num_picchi++;
                    debug_stato = -1;
                    debug_min_i = corr[(ia-16) & (256 - 1)];
                    debug_min_i8 = corr[(ia-24) & (256 - 1)];
                    min_i_iniziale = debug_min_i8;
                    newpeak = true;
                }
            } else {
                debug_min_i = min(corr[(ia-16) & (256 - 1)], debug_min_i);
                debug_min_i8 = min(corr[(ia-24) & (256 - 1)], debug_min_i8);
                if ((debug_min_i == debug_min_i8) & (debug_min_i != min_i_iniziale)) {
                    peaks[num_picchi & 0xFF] = ia - 24;
                    num_picchi++;
                    debug_stato = 1;
                    debug_max_i = corr[(ia-16) & (256 - 1)];
                    debug_max_i8 = corr[(ia-24) & (256 - 1)];
                    max_i_iniziale = debug_max_i8;
                    newpeak = true;
                }
            }

            // Calcolo distanze
            if (num_picchi > 1 && newpeak) {
                //2006 int32_t nuova_distanza = peaks[(num_picchi-1) & 0xFF] - peaks[(num_picchi-2) & 0xFF];
                ultima_distanza = peaks[(num_picchi - 1) & 0xFF] - peaks[(num_picchi - 2) & 0xFF];
                dist[num_distanze & 0xFF] = ultima_distanza;

                num_distanze++;

 
                if (stato_decodifica == 0) {
                    if (ultima_distanza >= soglia_mezzo_bit) {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos = ia - 24;
                        num_bits++;
                        newbit = 1; numbit = 1;
                    } else {
                        
                        stato_decodifica = 1;
                    }
                } else if (stato_decodifica == 1)  {
                        // PRIMA PROVA: se la somma delle ultime due distanze Ã¨ maggiore di 32+8 allora il primo era un 1 e il secondo Ã¨ l'inizio di uno zero
                    confro=42;   //1507
                    if(bits[(num_bits-1) & 0xFF].value==1) confro=48;   //test su bit orecedente
                    if((ultima_distanza + dist[(num_distanze - 2) & 0xFF]) >= confro) {
                        bits[num_bits & 0xFF].value = 1;
                        bits[num_bits & 0xFF].pos = ia - 24 - ultima_distanza;
                        num_bits++;
                        newbit = 1; numbit = 1;
                        stato_decodifica = 2;  //in buffer_analyzer non c'Ã¨
                        if ((ultima_distanza + dist[(num_distanze - 2) & 0xFF]) >= 52) {  // #2106 ma se addirittura supera 52 erano 2 uni
                            bits[num_bits & 0xFF].value = 1;
                            bits[num_bits & 0xFF].pos = ia - 24;
                            num_bits++;
                            newbit = 1; numbit = 2;
                            stato_decodifica = 0;
                        
                        } 
                    }else {
                        bits[num_bits & 0xFF].value = 0;
                        bits[num_bits & 0xFF].pos = ia - 24;
                        num_bits++;
                        newbit = 0; numbit = 1;
                        stato_decodifica = 0;
                    }
                    
                    
                }
                else if (stato_decodifica == 2)  {    
                    stato_decodifica = 0;
                    bits[num_bits & 0xFF].value = 0;
                    bits[num_bits & 0xFF].pos = ia - 24;
                    num_bits++;
                    newbit = 0; numbit = 1;
                }


            }

            // Decodifica byte
            while (numbit > 0) {
                switch (stato_decobytes) {
                    case 0:
                        if (newbit == 0) contatore_zeri++;
                        else if (contatore_zeri == 10) {   //2006 contatore_zeri >= 10
                            stato_decobytes = 1;
                            contatore_bytes = contatore_bits = 0;
                            for (int j = 0; j < 10; j++) bytes[j] = 0;
                            sync_count++; // Incremento a ogni sync
                            last_sync_i = ia; // Aggiunto
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
                                    last_country_code = country_code; // Aggiunto
                                    door_sync_count++; // Incremento solo con CRC OK
                                    display_sync_count++; // Incremento solo con CRC OK
                                    //Serial.printf(" OK ");
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
        
        ia++; // Avanza al prossimo campione
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