#include <Arduino.h>
#include <driver/ledc.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <driver/gpio.h>
#include "analog.h"
#include "core1.h"
#include <driver/rmt.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

hw_timer_t *timer = NULL;

const char *ssid = "VodafoneRibes";
const char *password = "scheggia2000";
#define PWM_PIN 14
#define PWM_CHANNEL 0
#define PWM_FREQ 134200
#define PWM_RESOLUTION 4
AsyncWebSocket ws("/ws");
AsyncWebServer server(80);
#define pblue 2
#define expblue 39
#define ledverde 21
#define ADC_CHANNEL 0
#define DOOR_GPIO GPIO_NUM_20 // GPIO per gattaiola
uint32_t DOOR_TIMEOUT = 10000 / portTICK_PERIOD_MS; // 10 s, ora variabile

// Definizioni per lo stepper motor
#define STEP_A_PLUS 10
#define STEP_A_MINUS 16
#define STEP_B_PLUS 12
#define STEP_B_MINUS 13
#define ENABLE_PIN 9
uint32_t STEPS_PER_MOVEMENT = 2500; // Numero di passi configurabile
uint32_t STEP_INTERVAL_US = 500;    // Intervallo tra passi in microsecondi

// Sequenza delle fasi per full-step
const uint8_t stepSequence[4][4] = {
    {1, 0, 0, 1}, // A+ B-
    {1, 0, 1, 0}, // A+ B+
    {0, 1, 1, 0}, // A- B+
    {0, 1, 0, 1}  // A- B-
};

// Struttura per i gatti autorizzati
struct Cat {
    uint64_t device_code; // 38 bit (12 cifre)
    uint16_t country_code; // 10 bit (3 cifre)
    String name;
    bool authorized; // Flag sì/no per autorizzazione
};
#define MAX_CATS 20 // Limite massimo di gatti
Cat authorized_cats[MAX_CATS];
size_t num_cats = 0; // Numero attuale di gatti

// Variabili globali per il controllo del motore
volatile int currentStep = 0;
volatile int stepsRemaining = 0;
volatile bool motorRunning = false;
volatile bool motorDirection = true; // true = orario (apertura), false = antiorario (chiusura)
hw_timer_t *stepTimer = NULL;
portMUX_TYPE stepTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Dichiarazione del buffer statico in SRAM
uint16_t temp_buffer[10000];

// ISR per il controllo delle fasi
void IRAM_ATTR onStepTimer() {
    portENTER_CRITICAL_ISR(&stepTimerMux);
    
    if (stepsRemaining > 0) {
        digitalWrite(STEP_A_PLUS, stepSequence[currentStep][0]);
        digitalWrite(STEP_A_MINUS, stepSequence[currentStep][1]);
        digitalWrite(STEP_B_PLUS, stepSequence[currentStep][2]);
        digitalWrite(STEP_B_MINUS, stepSequence[currentStep][3]);

        if (motorDirection) {
            currentStep = (currentStep + 1) % 4;
        } else {
            currentStep = (currentStep - 1 + 4) % 4;
        }

        stepsRemaining--;
    } else {
        timerStop(stepTimer);
        digitalWrite(ENABLE_PIN, LOW);
        motorRunning = false;
    }

    portEXIT_CRITICAL_ISR(&stepTimerMux);
}

// Funzione per avviare il movimento del motore
void startMotor(bool direction) {
    portENTER_CRITICAL(&stepTimerMux);
    
    if (!motorRunning) {
        motorRunning = true;
        motorDirection = direction;
        stepsRemaining = STEPS_PER_MOVEMENT;
        currentStep = 0;
        
        digitalWrite(ENABLE_PIN, HIGH);
        timerStart(stepTimer);
    }
    
    portEXIT_CRITICAL(&stepTimerMux);
}

// Funzione per leggere config.json da SPIFFS
bool readConfig() {
    File file = SPIFFS.open("/config.json", "r");
    if (!file) {
        Serial.println("Errore: impossibile aprire config.json");
        return false;
    }

    DynamicJsonDocument doc(2048); // Dimensione per ~20 gatti
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.printf("Errore parsing JSON: %s\n", error.c_str());
        return false;
    }

    // Leggi parametri
    DOOR_TIMEOUT = doc["door_timeout"] | 10000 / portTICK_PERIOD_MS;
    STEPS_PER_MOVEMENT = doc["steps_per_movement"] | 2500;
    STEP_INTERVAL_US = doc["step_interval_us"] | 500;

    // Leggi elenco gatti
    JsonArray cats = doc["authorized_cats"];
    num_cats = min(cats.size(), (size_t)MAX_CATS);
    for (size_t i = 0; i < num_cats; i++) {
        uint64_t device_code = cats[i]["device_code"] | 0ULL;
        uint16_t country_code = cats[i]["country_code"] | 0;
        bool authorized = cats[i]["authorized"] | true; // Default true se manca

        // Validazione
        if (device_code >= (1ULL << 38)) { // Max 2^38
            Serial.printf("Errore: device_code %llu troppo grande\n", (unsigned long long)device_code);
            device_code = 0;
        }
        if (country_code >= (1U << 10)) { // Max 2^10
            Serial.printf("Errore: country_code %u troppo grande\n", country_code);
            country_code = 0;
        }

        authorized_cats[i].device_code = device_code;
        authorized_cats[i].country_code = country_code;
        authorized_cats[i].name = cats[i]["name"].as<String>();
        authorized_cats[i].authorized = authorized;
        if (authorized_cats[i].name.length() > 32) {
            authorized_cats[i].name = authorized_cats[i].name.substring(0, 32);
        }
    }

    Serial.println("Configurazione letta con successo");
    return true;
}

// Funzione per scrivere config.json di default su SPIFFS
void writeDefaultConfig() {
    File file = SPIFFS.open("/config.json", "w");
    if (!file) {
        Serial.println("Errore: impossibile creare config.json");
        return;
    }

    DynamicJsonDocument doc(2048);
    doc["door_timeout"] = 10000 / portTICK_PERIOD_MS;
    doc["steps_per_movement"] = 2500;
    doc["step_interval_us"] = 500;

    JsonArray cats = doc.createNestedArray("authorized_cats");
    // Lista vuota per default

    if (serializeJson(doc, file)) {
        Serial.println("Configurazione di default scritta su config.json");
    } else {
        Serial.println("Errore scrittura config.json");
    }
    file.close();
}

uint32_t contaporta = 0;
// Task gestione gattaiola (core 0, ogni 100 ms)
void door_task(void *pvParameters) {
    while (true) {
        if (door_sync_count > 0) {
            // Cerca il gatto corrispondente a last_device_code
            bool is_authorized = false;
            String cat_name = "Sconosciuto";
            for (size_t i = 0; i < num_cats; i++) {
                if (authorized_cats[i].device_code == last_device_code) {
                    cat_name = authorized_cats[i].name;
                    is_authorized = authorized_cats[i].authorized;
                    break;
                }
            }
            Serial.printf("Rilevato gatto: %s, Autorizzato: %s\n", cat_name.c_str(), is_authorized ? "Sì" : "No");

            if (is_authorized && !door_open) {
                gpio_set_level(DOOR_GPIO, 0);
                digitalWrite(ledverde, HIGH);
                door_open = true;
                door_timer_start = xTaskGetTickCount();
                startMotor(true);
            } else if (is_authorized) {
                door_timer_start = xTaskGetTickCount();
            }
            contaporta = 0;
            door_sync_count = 0;
        } else if (!digitalRead(pblue) && !door_open) {
            // Apertura manuale tramite pblue
            gpio_set_level(DOOR_GPIO, 0);
            digitalWrite(ledverde, HIGH);
            door_open = true;
            door_timer_start = xTaskGetTickCount();
            startMotor(true);
            contaporta = 0;
        } else if (door_open) {
            TickType_t now = xTaskGetTickCount();
            contaporta = (now - door_timer_start);
            if ((now - door_timer_start) >= DOOR_TIMEOUT) {
                contaporta = 12000;
                gpio_set_level(DOOR_GPIO, 1);
                digitalWrite(ledverde, LOW);
                door_open = false;
                startMotor(false);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Task di stampa (core 0, ogni 1 s)
void print_task(void *pvParameters) {
    while (true) {
        printf("Sync: %u, OK: %u, Last Seq: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X], "
               "Device Code: %llu, Country Code: %u, ia: %u, i_interrupt: %u, diff: %u, available_samples: %ld, contaporta: %u\n",
               sync_count, display_sync_count,
               last_sequence[0], last_sequence[1], last_sequence[2], last_sequence[3],
               last_sequence[4], last_sequence[5], last_sequence[6], last_sequence[7],
               last_sequence[8], last_sequence[9],
               (unsigned long long)last_device_code, (long)last_country_code, ia, i_interrupt, (i_interrupt-ia), (long)available_samples, contaporta);
        sync_count = 0;
        display_sync_count = 0;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Gestione eventi WebSocket
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
                // Leggi i_interrupt (senza sincronizzazione)
                uint32_t current_index = i_interrupt;
                // Calcola l'indice di partenza per le 10.000 misure precedenti
                uint32_t start_index = (current_index - 10000 + ADC_BUFFER_SIZE) % ADC_BUFFER_SIZE;
                Serial.printf("DEBUG: i_interrupt=%u, start_index=%u\n", current_index, start_index);
                // Copia le 10.000 misure in temp_buffer, gestendo il wrap-around
                if (start_index + 10000 <= ADC_BUFFER_SIZE) {
                    // Copia diretta
                    memcpy(temp_buffer, (const void*)&adc_buffer[start_index], 10000 * sizeof(uint16_t));
                } else {
                    // Copia in due blocchi per il wrap-around
                    uint32_t first_chunk_size = ADC_BUFFER_SIZE - start_index;
                    uint32_t second_chunk_size = 10000 - first_chunk_size;
                    memcpy(temp_buffer, (const void*)&adc_buffer[start_index], first_chunk_size * sizeof(uint16_t));
                    memcpy(&temp_buffer[first_chunk_size], (const void*)adc_buffer, second_chunk_size * sizeof(uint16_t));
                }
                // Trasmetti il buffer
                Serial.println("Invio buffer al client...");
                client->binary((uint8_t*)temp_buffer, 10000 * sizeof(uint16_t));
                Serial.println("Buffer inviato al client (binario)");
            }
        }
    }
}
// Configurazione RMT per generare onda quadra a 134,2 kHz
#define RMT_CHANNEL RMT_CHANNEL_0
#define RMT_CLK_DIV 1
#define RMT_TICK_1_US (80000000 / RMT_CLK_DIV / 1000000) // Tick per 1 us

void setupRMT() {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL;
    config.gpio_num = (gpio_num_t)PWM_PIN;
    config.clk_div = RMT_CLK_DIV;
    config.mem_block_num = 1;
    config.tx_config.loop_en = true;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;

    rmt_config(&config);
    rmt_driver_install(RMT_CHANNEL, 0, 0);

    float period_us = 1000000.0 / PWM_FREQ;
    uint32_t period_ticks = (uint32_t)(period_us * RMT_TICK_1_US);
    uint32_t high_ticks = period_ticks / 2;
    uint32_t low_ticks = period_ticks - high_ticks;

    rmt_item32_t items[2] = {
        {{{ high_ticks, 1, low_ticks, 0 }}},
        {{{ high_ticks, 1, low_ticks, 0 }}}
    };
    rmt_write_items(RMT_CHANNEL, items, 2, false);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Avvio ESP32-S3");

    // Inizializza SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Errore: inizializzazione SPIFFS fallita");
        return;
    }
    Serial.println("SPIFFS inizializzato");

    // Leggi config.json o crea default
    if (!readConfig()) {
        writeDefaultConfig();
        readConfig(); // Riprova dopo aver scritto i default
    }

    // Configura GPIO per gattaiola
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DOOR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(DOOR_GPIO, 1);

    pinMode(pblue, INPUT_PULLUP);
    pinMode(expblue, INPUT);
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

    // Configura il server web per servire index.html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });

    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    server.begin();
    Serial.println("Server WebSocket avviato");

    setupRMT();

    fadcInit(1, 1);
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1 << ADC_CHANNEL);
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;

    timer = timerBegin(0, 4, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 149, true);

    pinMode(STEP_A_PLUS, OUTPUT);
    pinMode(STEP_A_MINUS, OUTPUT);
    pinMode(STEP_B_PLUS, OUTPUT);
    pinMode(STEP_B_MINUS, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);

    stepTimer = timerBegin(1, 80, true);
    timerAttachInterrupt(stepTimer, &onStepTimer, true);
    timerAlarmWrite(stepTimer, STEP_INTERVAL_US, true);
    timerAlarmEnable(stepTimer);
    timerStop(stepTimer);

    start_rfid_task();
    xTaskCreatePinnedToCore(door_task, "Door_Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(print_task, "Print_Task", 4096, NULL, 1, NULL, 0);

    timerAlarmEnable(timer);
}

void loop() {
    ws.cleanupClients();
    vTaskDelay(10 / portTICK_PERIOD_MS);
}