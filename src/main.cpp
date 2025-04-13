#include <Arduino.h>
#include <driver/ledc.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "analog.h" // ESP32-S3-FastAnalogRead

hw_timer_t *timer = NULL;
volatile bool buffer_ready = false;
volatile size_t buffer_index = 0;

const char *ssid = "VodafoneRibes";
const char *password = "scheggia2000";
#define PWM_PIN 14
#define PWM_CHANNEL 0
#define PWM_FREQ 134200
#define PWM_RESOLUTION 4
int statoacq = 0;
AsyncWebSocket ws("/ws");
AsyncWebServer server(80);
#define pblue 39
#define ledverde 21

#define BUFFER_SIZE 10000
#define CIRCULAR_SIZE 256
#define MAX_PEAKS 1000
uint16_t adc_buffer[BUFFER_SIZE];
int32_t segnale_filtrato32[CIRCULAR_SIZE];
int32_t correlazione32[CIRCULAR_SIZE];
int32_t picchi32[MAX_PEAKS];
int32_t distanze32[MAX_PEAKS];
struct Bit { int value; int32_t pos; };
Bit bits32[MAX_PEAKS];
uint8_t bytes32[10];
int32_t num_picchi = 0, num_distanze = 0, num_bits = 0;
uint16_t country_code = 0;
uint64_t device_code = 0;
bool crc_ok = false;

#define ADC_CHANNEL 0 // ADC1_CHANNEL_0 (GPIO1)

void IRAM_ATTR onTimer() {
    digitalWrite(ledverde, HIGH);
    if (buffer_index < BUFFER_SIZE) {
        if (!fadcBusy()) { // ADC non occupato
            adc_buffer[buffer_index++] = fadcResult(); // Leggi risultato
            fadcStart(ADC_CHANNEL); // Avvia nuova conversione
        }
    }
    if (buffer_index >= BUFFER_SIZE) {
        buffer_ready = true;
        timerAlarmDisable(timer); // Ferma interrupt
    }
    digitalWrite(ledverde, LOW);
}

void media_correlazione_32(uint16_t* segnale, int32_t* filt, int32_t* corr, int32_t* peaks, int32_t* dists, Bit* bits, uint8_t* bytes,
                          int32_t& n_peaks, int32_t& n_dists, int32_t& n_bits, uint16_t& country, uint64_t& device, bool& crc_valid) {
    const int N = BUFFER_SIZE;
    const int larghezza_finestra = 8;
    const int lunghezza_correlazione = 32;
    const int soglia_mezzo_bit = 24;
    int32_t stato_decodifica = 0, contatore_zeri = 0, contatore_bytes = 0, contatore_bits = 0, stato_decobytes = 0;
    int32_t ultima_distanza = 0, newbit = 0, numbit = 0;
    bool newpeak = false;

    uint32_t start_time = millis();

    n_peaks = n_dists = n_bits = 0;
    for (int i = 0; i < 10; i++) bytes[i] = 0;
    for (int i = 0; i < CIRCULAR_SIZE; i++) filt[i] = corr[i] = 0;

    int i = 32;
    if (i < N && i + 3 < N) {
        int32_t somma_media = 0;
        for (int j = i - 4; j < i + 4; j++) somma_media += segnale[j];
        filt[i & (CIRCULAR_SIZE - 1)] = somma_media / larghezza_finestra;
    }
    if (i >= lunghezza_correlazione) {
        corr[16 & (CIRCULAR_SIZE - 1)] = 0;
        for (int j = 0; j < 16; j++) corr[16 & (CIRCULAR_SIZE - 1)] += filt[j & (CIRCULAR_SIZE - 1)];
        for (int j = 16; j < 32; j++) corr[16 & (CIRCULAR_SIZE - 1)] -= filt[j & (CIRCULAR_SIZE - 1)];
    }

    int32_t max_i = corr[16 & (CIRCULAR_SIZE - 1)], min_i = corr[16 & (CIRCULAR_SIZE - 1)];
    int32_t max_i8 = corr[8 & (CIRCULAR_SIZE - 1)], min_i8 = corr[8 & (CIRCULAR_SIZE - 1)];
    int32_t stato = 1;

    for (i = 33; i < N - 4; i++) {
        filt[i & (CIRCULAR_SIZE - 1)] = filt[(i-1) & (CIRCULAR_SIZE - 1)] - (segnale[i-4] / larghezza_finestra) + (segnale[i+3] / larghezza_finestra);
        corr[(i-16) & (CIRCULAR_SIZE - 1)] = corr[(i-17) & (CIRCULAR_SIZE - 1)] - filt[(i-32) & (CIRCULAR_SIZE - 1)] + 2 * filt[(i-16) & (CIRCULAR_SIZE - 1)] - filt[i & (CIRCULAR_SIZE - 1)];

        newbit = 2; numbit = 0; newpeak = false;

        if (stato == 1) {
            max_i = max(corr[(i-16) & (CIRCULAR_SIZE - 1)], max_i);
            max_i8 = max(corr[(i-24) & (CIRCULAR_SIZE - 1)], max_i8);
            if (max_i == max_i8 && n_peaks < MAX_PEAKS) {
                peaks[n_peaks++] = i - 24;
                stato = -1;
                min_i = corr[(i-16) & (CIRCULAR_SIZE - 1)];
                min_i8 = corr[(i-24) & (CIRCULAR_SIZE - 1)];
                newpeak = true;
            }
        } else {
            min_i = min(corr[(i-16) & (CIRCULAR_SIZE - 1)], min_i);
            min_i8 = min(corr[(i-24) & (CIRCULAR_SIZE - 1)], min_i8);
            if (min_i == min_i8 && n_peaks < MAX_PEAKS) {
                peaks[n_peaks++] = i - 24;
                stato = 1;
                max_i = corr[(i-16) & (CIRCULAR_SIZE - 1)];
                max_i8 = corr[(i-24) & (CIRCULAR_SIZE - 1)];
                newpeak = true;
            }
        }

        if (n_peaks > 1 && newpeak) {
            int32_t nuova_distanza = peaks[n_peaks-1] - peaks[n_peaks-2];
            if (n_dists < MAX_PEAKS) dists[n_dists++] = nuova_distanza;

            if (stato_decodifica == 0) {
                if (nuova_distanza >= soglia_mezzo_bit) {
                    if (n_bits < MAX_PEAKS) { bits[n_bits].value = 1; bits[n_bits++].pos = i - 24; }
                    newbit = 1; numbit = 1;
                } else {
                    ultima_distanza = nuova_distanza;
                    stato_decodifica = 1;
                }
            } else {
                if (nuova_distanza < soglia_mezzo_bit) {
                    if (n_bits < MAX_PEAKS) { bits[n_bits].value = 0; bits[n_bits++].pos = i - 24; }
                    newbit = 0; numbit = 1;
                } else {
                    if (n_bits + 1 < MAX_PEAKS) {
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
                        Serial.print("Sequenza sync at: ");
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
                            Serial.print("Byte estratti: [");
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
                            Serial.print("CRC: ");
                            Serial.println(crc_valid ? "OK" : "KO");
                            if (crc_valid) {
                                digitalWrite(ledverde, HIGH);
                                country = (bytes[5] << 2) | (bytes[4] >> 6);
                                device = ((uint64_t)(bytes[4] & 0x3F) << 32) | ((uint64_t)bytes[3] << 24) |
                                         ((uint64_t)bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
                                Serial.print("Country Code: ");
                                Serial.println(country);
                                Serial.print("Device Code: ");
                                Serial.println((uint64_t)device);
                            }
                        }
                    } else {
                        Serial.print("Perso sync at: ");
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
    Serial.print("Durata analisi: ");
    Serial.print(end_time - start_time);
    Serial.println(" ms");
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.println("Client WebSocket connesso");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("Client WebSocket disconnesso");
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            Serial.printf("Ricevuto dal client: %s\n", (char*)data);
            if (strcmp((char*)data, "get_buffer") == 0) {
                Serial.println("Inizio acquisizione da WebSocket...");
                buffer_index = 0;
                buffer_ready = false;
                timerAlarmEnable(timer);
                timerStart(timer);
                while (!buffer_ready) {
                    // Vuoto
                }
                timerStop(timer);
                timerAlarmDisable(timer);
                if (buffer_index >= BUFFER_SIZE) {
                    Serial.println("Invio buffer al client...");
                    client->binary((uint8_t*)adc_buffer, BUFFER_SIZE * sizeof(uint16_t));
                    Serial.println("Buffer inviato al client (binario)");
                    media_correlazione_32(adc_buffer, segnale_filtrato32, correlazione32, picchi32, distanze32, bits32, bytes32,
                                         num_picchi, num_distanze, num_bits, country_code, device_code, crc_ok);
                } else {
                    Serial.println("Errore: buffer non pieno");
                    client->text("Errore: buffer non pieno");
                }
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Avvio ESP32-S3");

    pinMode(pblue, INPUT_PULLUP);
    pinMode(ledverde, OUTPUT);

    WiFi.begin(ssid, password);
    Serial.println("Connessione al Wi-Fi in corso...");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnesso al Wi-Fi");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nErrore: impossibile connettersi al Wi-Fi");
        return;
    }

    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    server.begin();
    Serial.println("Server WebSocket avviato");

    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 8);

    // Configurazione ADC con FastAnalogRead
    fadcInit(1, 1); // 1 pin, GPIO1 (ADC1_CHANNEL_0)
    fadcStart(ADC_CHANNEL); // Conversione iniziale per preparare l'ADC

    timer = timerBegin(0, 8, true); // Timer 0, prescaler 8
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 75, true); // ~133333 Hz
}

void loop() {
    //Serial.println("Loop in esecuzione..."); // Debug
    ws.cleanupClients();

    int sblue = digitalRead(pblue);
    if (sblue == 0) {
        Serial.println("Pulsante premuto");
        statoacq ^= 1;
        delay(50);
    }

    if (statoacq == 1) {
        Serial.println("Inizio acquisizione...");
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

        Serial.print("ADC Value (elemento 100): ");
        Serial.println(adc_buffer[100]);
        Serial.print("Tempo acquisizione: ");
        Serial.print(end_time - start_time);
        Serial.println(" us");
        Serial.print("Frequenza campionamento: ");
        Serial.print((float)BUFFER_SIZE / ((end_time - start_time) / 1000000.0));
        Serial.println(" Hz");
        Serial.println("Primi 10 valori ADC:");
        for (int i = 0; i < 10; i++) {
            Serial.print(adc_buffer[i]);
            Serial.print(" ");
        }

        media_correlazione_32(adc_buffer, segnale_filtrato32, correlazione32, picchi32, distanze32, bits32, bytes32,
                             num_picchi, num_distanze, num_bits, country_code, device_code, crc_ok);
        statoacq = 0;
        Serial.println("Acquisizione completata");
    }

    digitalWrite(ledverde, LOW);
    delay(100); // Feed WDT
}