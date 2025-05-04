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
#include <time.h>

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
#define ledverde 20   // LED contemporaneo ad apertura porta
#define detected 15   // LED istantaneo presenza RFID
#define wifi_led 6    // LED per stato Wi-Fi (logica invertita)
#define ADC_CHANNEL 0

uint32_t DOOR_TIMEOUT = 10000 / portTICK_PERIOD_MS;
uint32_t WIFI_RECONNECT_DELAY = 1000;
uint32_t UNAUTHORIZED_LOG_INTERVAL = 60000; // 1 min, configurabile
bool WIFI_VERBOSE_LOG = false;

// Definizioni per lo stepper motor
#define STEP_A_PLUS 10
#define STEP_A_MINUS 16
#define STEP_B_PLUS 12
#define STEP_B_MINUS 13
#define ENABLE_PIN 9
uint32_t STEPS_PER_MOVEMENT = 2500;
uint32_t STEP_INTERVAL_US = 500;

// Sequenza delle fasi per full-step
const uint8_t stepSequence[4][4] = {
    {1, 0, 0, 1},
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1}
};

// Struttura per i gatti autorizzati
struct Cat {
    uint64_t device_code;
    uint16_t country_code;
    String name;
    bool authorized;
};
#define MAX_CATS 20
Cat authorized_cats[MAX_CATS];
size_t num_cats = 0;

// Struttura per i log
#define LOG_BUFFER_SIZE 100
struct LogEntry {
    char timestamp[20]; // "DD/MM/YYYY HH:MM:SS"
    String type;        // Tradotto in italiano
    String name;        // Nome gatto (se applicabile)
    uint16_t country_code; // Per sconosciuti
    uint64_t device_code;  // Per sconosciuti
    bool authorized;
};
LogEntry log_buffer[LOG_BUFFER_SIZE];
size_t log_buffer_index = 0;

// Variabili globali per il controllo del motore
volatile int currentStep = 0;
volatile int stepsRemaining = 0;
volatile bool motorRunning = false;
volatile bool motorDirection = true;
hw_timer_t *stepTimer = NULL;
portMUX_TYPE stepTimerMux = portMUX_INITIALIZER_UNLOCKED;

// Variabili per Wi-Fi
volatile bool wifi_connected = false;
portMUX_TYPE wifiMux = portMUX_INITIALIZER_UNLOCKED;

// Variabili per gestione ora
unsigned long last_millis = 0;
time_t local_time = 0;

// Enum per door_mode
enum DoorMode { AUTO, ALWAYS_OPEN, ALWAYS_CLOSED };
volatile DoorMode door_mode = AUTO; // Variabile globale
portMUX_TYPE doorModeMux = portMUX_INITIALIZER_UNLOCKED;

// Dichiarazione del buffer statico in SRAM
uint16_t temp_buffer[10000];

// Dichiarazione anticipata di onWebSocketEvent
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

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

// Funzione per aggiungere un log al buffer circolare
void add_log_entry(const char* timestamp, const char* type, const String& name, uint16_t country_code, uint64_t device_code, bool authorized) {
    LogEntry& entry = log_buffer[log_buffer_index];
    strncpy(entry.timestamp, timestamp, sizeof(entry.timestamp));
    entry.type = type;
    entry.name = name;
    entry.country_code = country_code;
    entry.device_code = device_code;
    entry.authorized = authorized;

    log_buffer_index = (log_buffer_index + 1) % LOG_BUFFER_SIZE;

    if (WIFI_VERBOSE_LOG) {
        Serial.printf("Log aggiunto: [%s] Type: %s, Name: %s, CC: %u, DC: %llu, Authorized: %s\n",
                      timestamp, type, name.c_str(), country_code, (unsigned long long)device_code, authorized ? "Sì" : "No");
    }

    // Invia il log aggiornato a tutti i client WebSocket
    if (wifi_connected) {
        DynamicJsonDocument doc(8192);
        JsonArray log_array = doc.createNestedArray("log");
        for (size_t i = 0; i < LOG_BUFFER_SIZE; i++) {
            size_t idx = (log_buffer_index - 1 - i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
            if (log_buffer[idx].timestamp[0] == '\0') continue; // Entry vuota
            JsonObject entry = log_array.createNestedObject();
            entry["timestamp"] = log_buffer[idx].timestamp;
            entry["type"] = log_buffer[idx].type;
            entry["name"] = log_buffer[idx].name;
            entry["country_code"] = log_buffer[idx].country_code;
            entry["device_code"] = String((unsigned long long)log_buffer[idx].device_code);
            entry["authorized"] = log_buffer[idx].authorized;
        }
        String json;
        serializeJson(doc, json);
        ws.textAll(json);
        if (WIFI_VERBOSE_LOG) {
            Serial.printf("Log inviato a tutti i client WebSocket\n");
        }
    }
}

// Funzione per leggere config.json da SPIFFS
bool readConfig() {
    File file = SPIFFS.open("/config.json", "r");
    if (!file) {
        Serial.println("Errore: impossibile aprire config.json");
        return false;
    }

    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.printf("Errore parsing JSON: %s\n", error.c_str());
        return false;
    }

    DOOR_TIMEOUT = doc["door_timeout"] | 10000 / portTICK_PERIOD_MS;
    STEPS_PER_MOVEMENT = doc["steps_per_movement"] | 2500;
    STEP_INTERVAL_US = doc["step_interval_us"] | 500;
    WIFI_RECONNECT_DELAY = doc["wifi_reconnect_delay"] | 1000;
    UNAUTHORIZED_LOG_INTERVAL = doc["unauthorized_log_interval"] | 60000;
    WIFI_VERBOSE_LOG = doc["wifi_verbose_log"] | false;

    // Leggo door_mode
    String mode = doc["door_mode"] | "AUTO";
    portENTER_CRITICAL(&doorModeMux);
    if (mode == "ALWAYS_OPEN") door_mode = ALWAYS_OPEN;
    else if (mode == "ALWAYS_CLOSED") door_mode = ALWAYS_CLOSED;
    else door_mode = AUTO;
    portEXIT_CRITICAL(&doorModeMux);

    JsonArray cats = doc["authorized_cats"];
    num_cats = min(cats.size(), (size_t)MAX_CATS);
    for (size_t i = 0; i < num_cats; i++) {
        uint64_t device_code = cats[i]["device_code"] | 0ULL;
        uint16_t country_code = cats[i]["country_code"] | 0;
        bool authorized = cats[i]["authorized"] | true;

        if (device_code >= (1ULL << 38)) {
            Serial.printf("Errore: device_code %llu troppo grande\n", (unsigned long long)device_code);
            device_code = 0;
        }
        if (country_code >= (1U << 10)) {
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
    doc["wifi_reconnect_delay"] = 1000;
    doc["unauthorized_log_interval"] = 60000;
    doc["wifi_verbose_log"] = false;
    doc["door_mode"] = "AUTO";

    JsonArray cats = doc.createNestedArray("authorized_cats");

    if (serializeJson(doc, file)) {
        Serial.println("Configurazione di default scritta su config.json");
    } else {
        Serial.println("Errore scrittura config.json");
    }
    file.close();
}

// Funzione per salvare la configurazione aggiornata
void saveConfig() {
    File file = SPIFFS.open("/config.json", "w");
    if (!file) {
        Serial.println("Errore: impossibile creare config.json");
        return;
    }

    DynamicJsonDocument doc(2048);
    doc["door_timeout"] = DOOR_TIMEOUT;
    doc["steps_per_movement"] = STEPS_PER_MOVEMENT;
    doc["step_interval_us"] = STEP_INTERVAL_US;
    doc["wifi_reconnect_delay"] = WIFI_RECONNECT_DELAY;
    doc["unauthorized_log_interval"] = UNAUTHORIZED_LOG_INTERVAL;
    doc["wifi_verbose_log"] = WIFI_VERBOSE_LOG;

    // Salvo door_mode
    portENTER_CRITICAL(&doorModeMux);
    if (door_mode == ALWAYS_OPEN) doc["door_mode"] = "ALWAYS_OPEN";
    else if (door_mode == ALWAYS_CLOSED) doc["door_mode"] = "ALWAYS_CLOSED";
    else doc["door_mode"] = "AUTO";
    portEXIT_CRITICAL(&doorModeMux);

    JsonArray cats = doc.createNestedArray("authorized_cats");
    for (size_t i = 0; i < num_cats; i++) {
        JsonObject cat = cats.createNestedObject();
        cat["device_code"] = authorized_cats[i].device_code;
        cat["country_code"] = authorized_cats[i].country_code;
        cat["name"] = authorized_cats[i].name;
        cat["authorized"] = authorized_cats[i].authorized;
    }

    if (serializeJson(doc, file)) {
        Serial.println("Configurazione aggiornata su config.json");
    } else {
        Serial.println("Errore scrittura config.json");
    }
    file.close();
}

// Task gestione Wi-Fi
void wifi_task(void *pvParameters) {
    bool server_started = false;
    unsigned long last_ntp_attempt = 0;
    const unsigned long ntp_retry_interval = 300000;

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            portENTER_CRITICAL(&wifiMux);
            wifi_connected = false;
            digitalWrite(wifi_led, HIGH); // LED spento
            portEXIT_CRITICAL(&wifiMux);

            if (WIFI_VERBOSE_LOG) {
                Serial.println("Wi-Fi: Tentativo di connessione...");
            }

            WiFi.begin(ssid, password);
            unsigned long start = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }

            if (WiFi.status() == WL_CONNECTED) {
                portENTER_CRITICAL(&wifiMux);
                wifi_connected = true;
                digitalWrite(wifi_led, LOW); // LED acceso
                portEXIT_CRITICAL(&wifiMux);

                Serial.printf("Wi-Fi: Connesso, IP: %s\n", WiFi.localIP().toString().c_str());

                configTime(0, 0, "pool.ntp.org", "time.google.com");
                setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
                tzset();

                if (!server_started) {
                    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
                        request->send(SPIFFS, "/index.html", "text/html");
                    });
                    ws.onEvent(onWebSocketEvent);
                    server.addHandler(&ws);
                    server.begin();
                    server_started = true;
                    Serial.println("Server WebSocket avviato");
                }
            } else {
                if (WIFI_VERBOSE_LOG) {
                    Serial.println("Wi-Fi: Connessione fallita, riprovo...");
                }
                WiFi.disconnect();
            }
            vTaskDelay(WIFI_RECONNECT_DELAY / portTICK_PERIOD_MS);
        } else {
            if (millis() - last_ntp_attempt >= ntp_retry_interval || last_ntp_attempt == 0) {
                if (WIFI_VERBOSE_LOG) {
                    Serial.println("NTP: Tentativo di sincronizzazione...");
                }
                time_t now;
                time(&now);
                unsigned long start = millis();
                while (now < 946684800 && millis() - start < 10000) {
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    time(&now);
                }
                if (now > 946684800) {
                    portENTER_CRITICAL(&wifiMux);
                    local_time = now;
                    last_millis = millis();
                    portEXIT_CRITICAL(&wifiMux);
                    if (WIFI_VERBOSE_LOG) {
                        char time_str[20];
                        strftime(time_str, sizeof(time_str), "%d-%m-%Y %H:%M:%S", localtime(&now));
                        Serial.printf("NTP: Ora sincronizzata: %s\n", time_str);
                    }
                } else {
                    if (WIFI_VERBOSE_LOG) {
                        Serial.println("NTP: Sincronizzazione fallita, ritento tra 5 minuti...");
                    }
                }
                last_ntp_attempt = millis();
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

uint32_t contaporta = 0;
// Task gestione gattaiola
void door_task(void *pvParameters) {
    char time_str[20];
    bool log_emitted = false;
    unsigned long last_unauthorized_log = 0;
    DoorMode last_mode = AUTO; // Aggiunto per tracciare l'ultima modalità

    // Stato iniziale basato su door_mode
    portENTER_CRITICAL(&doorModeMux);
    DoorMode initial_mode = door_mode;
    portEXIT_CRITICAL(&doorModeMux);

    if (initial_mode == ALWAYS_OPEN && !door_open) {
        digitalWrite(ledverde, LOW);
        door_open = true;
        startMotor(true);
        time_t now = local_time + (millis() - last_millis) / 1000;
        char timestamp_full[20];
        strftime(timestamp_full, sizeof(timestamp_full), "%d/%m/%Y %H:%M:%S", localtime(&now));
        add_log_entry(timestamp_full, "Modalità sempre aperto", "Modalità Sempre Aperto", 0, 0, true);
        Serial.println("Porta aperta per modalità ALWAYS_OPEN");
    } else if (initial_mode == ALWAYS_CLOSED && door_open) {
        digitalWrite(ledverde, HIGH);
        door_open = false;
        startMotor(false);
        time_t now = local_time + (millis() - last_millis) / 1000;
        char timestamp_full[20];
        strftime(timestamp_full, sizeof(timestamp_full), "%d/%m/%Y %H:%M:%S", localtime(&now));
        add_log_entry(timestamp_full, "Modalità sempre chiuso", "Modalità Sempre Chiuso", 0, 0, false);
        Serial.println("Porta chiusa per modalità ALWAYS_CLOSED");
    } else if (initial_mode == AUTO && door_open) {
        digitalWrite(ledverde, HIGH);
        door_open = false;
        startMotor(false);
        time_t now = local_time + (millis() - last_millis) / 1000;
        char timestamp_full[20];
        strftime(timestamp_full, sizeof(timestamp_full), "%d/%m/%Y %H:%M:%S", localtime(&now));
        add_log_entry(timestamp_full, "Modalità automatica", "Modalità Automatica", 0, 0, false);
        Serial.println("Porta chiusa per modalità AUTO");
    }
    last_mode = initial_mode; // Inizializza l'ultima modalità

    while (true) {
        time_t now = local_time + (millis() - last_millis) / 1000;
        strftime(time_str, sizeof(time_str), "%H:%M:%S", localtime(&now));
        char timestamp_full[20];
        strftime(timestamp_full, sizeof(timestamp_full), "%d/%m/%Y %H:%M:%S", localtime(&now));

        portENTER_CRITICAL(&doorModeMux);
        DoorMode current_mode = door_mode;
        portEXIT_CRITICAL(&doorModeMux);

        if (current_mode != last_mode) {
            if (current_mode == ALWAYS_OPEN) {
                if (!door_open) {
                    digitalWrite(ledverde, LOW);
                    door_open = true;
                    startMotor(true);
                    add_log_entry(timestamp_full, "Modalità sempre aperto", "Modalità Sempre Aperto", 0, 0, true);
                    Serial.println("Porta aperta per modalità ALWAYS_OPEN");
                }
            } else if (current_mode == ALWAYS_CLOSED) {
                if (door_open) {
                    digitalWrite(ledverde, HIGH);
                    door_open = false;
                    startMotor(false);
                    add_log_entry(timestamp_full, "Modalità sempre chiuso", "Modalità Sempre Chiuso", 0, 0, false);
                    Serial.println("Porta chiusa per modalità ALWAYS_CLOSED");
                }
            } else if (current_mode == AUTO) {
                if (door_open) {
                    digitalWrite(ledverde, HIGH);
                    door_open = false;
                    startMotor(false);
                    add_log_entry(timestamp_full, "Modalità automatica", "Modalità Automatica", 0, 0, false);
                    Serial.println("Porta chiusa per modalità AUTO");
                } else {
                    // Log per passaggio a modalità AUTO anche se la porta è già chiusa
                    add_log_entry(timestamp_full, "Modalità automatica", "Modalità Automatica", 0, 0, false);
                    Serial.println("Passaggio a modalità AUTO");
                }
            }
            last_mode = current_mode; // Aggiorna l'ultima modalità
        }

        if (current_mode == ALWAYS_OPEN) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue; // Ignora altre logiche
        } else if (current_mode == ALWAYS_CLOSED) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue; // Ignora altre logiche
        }

        // Modalità AUTO
        if (door_sync_count > 0) {
            digitalWrite(detected, LOW);
            bool is_authorized = false;
            String cat_name = "Sconosciuto";
            uint16_t country_code = last_country_code;
            uint64_t device_code = last_device_code;

            // Cerca il gatto
            for (size_t i = 0; i < num_cats; i++) {
                if (authorized_cats[i].device_code == last_device_code) {
                    cat_name = authorized_cats[i].name;
                    is_authorized = authorized_cats[i].authorized;
                    country_code = authorized_cats[i].country_code;
                    break;
                }
            }

            if (is_authorized) {
                if (!door_open && !log_emitted) {
                    digitalWrite(ledverde, LOW);
                    door_open = true;
                    door_timer_start = xTaskGetTickCount();
                    startMotor(true);
                    Serial.printf("[%s] Rilevato gatto: %s, Autorizzato: Sì\n", time_str, cat_name.c_str());
                    add_log_entry(timestamp_full, "Autorizzato", cat_name, country_code, device_code, true);
                    log_emitted = true;
                } else if (door_open) {
                    door_timer_start = xTaskGetTickCount(); // Aggiorna timer senza log
                }
            } else {
                if (millis() - last_unauthorized_log >= UNAUTHORIZED_LOG_INTERVAL) {
                    if (cat_name != "Sconosciuto") {
                        Serial.printf("[%s] Rilevato gatto: %s, Autorizzato: No\n", time_str, cat_name.c_str());
                        add_log_entry(timestamp_full, "Non autorizzato (noto)", cat_name, country_code, device_code, false);
                    } else {
                        Serial.printf("[%s] Gatto sconosciuto, CC: %u, DC: %llu, Non autorizzato\n",
                                      time_str, country_code, (unsigned long long)device_code);
                        add_log_entry(timestamp_full, "Non autorizzato (sconosciuto)", "", country_code, device_code, false);
                    }
                    last_unauthorized_log = millis();
                }
            }
            contaporta = 0;
            door_sync_count = 0;
        } else {
            digitalWrite(detected, HIGH);
            if (!digitalRead(pblue) && !door_open) {
                digitalWrite(ledverde, LOW);
                door_open = true;
                door_timer_start = xTaskGetTickCount();
                startMotor(true);
                contaporta = 0;
                Serial.printf("[%s] Apertura manuale tramite pblue\n", time_str);
                add_log_entry(timestamp_full, "Manuale", "Manuale", 0, 0, true);
                log_emitted = true;
            } else if (door_open) {
                TickType_t now_ticks = xTaskGetTickCount();
                contaporta = (now_ticks - door_timer_start);
                if ((now_ticks - door_timer_start) >= DOOR_TIMEOUT) {
                    contaporta = 12000;
                    digitalWrite(ledverde, HIGH);
                    door_open = false;
                    startMotor(false);
                    Serial.printf("[%s] Porta chiusa dopo timeout\n", time_str);
                    add_log_entry(timestamp_full, "Timeout", "", 0, 0, false);
                    log_emitted = false; // Resetta per il prossimo accesso
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
// Task di stampa
void print_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = 1000 / portTICK_PERIOD_MS;
    uint32_t last_i_interrupt = 0;
    uint32_t freq = 0;
    char time_str[20];

    while (true) {
        time_t now = local_time + (millis() - last_millis) / 1000;
        freq = i_interrupt - last_i_interrupt;
        last_i_interrupt = i_interrupt;
        if (WIFI_VERBOSE_LOG) {
            strftime(time_str, sizeof(time_str), "%d-%m-%Y %H:%M:%S", localtime(&now));
        } else {
            strftime(time_str, sizeof(time_str), "%H:%M:%S", localtime(&now));
        }
        printf("[%s] Sync: %u, OK: %u, Last Seq: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X], "
               "DC: %llu, CC: %u, ia: %u, i_i: %u, diff: %u, freq: %u, a_s: %ld, contap: %u\n",
               time_str, sync_count, display_sync_count,
               last_sequence[0], last_sequence[1], last_sequence[2], last_sequence[3],
               last_sequence[4], last_sequence[5], last_sequence[6], last_sequence[7],
               last_sequence[8], last_sequence[9],
               (unsigned long long)last_device_code, (long)last_country_code, ia, i_interrupt, (i_interrupt-ia), freq, (long)available_samples, contaporta);
        sync_count = 0;
        display_sync_count = 0;

        vTaskDelayUntil(&last_wake_time, interval);
    }
}

// Gestione eventi WebSocket
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    char time_str[20];
    time_t now = local_time + (millis() - last_millis) / 1000;
    strftime(time_str, sizeof(time_str), "%H:%M:%S", localtime(&now));

    if (type == WS_EVT_CONNECT) {
        Serial.printf("[%s] Client WebSocket connesso\n", time_str);
        // Invia lo stato corrente di door_mode
        String mode_str;
        portENTER_CRITICAL(&doorModeMux);
        if (door_mode == ALWAYS_OPEN) mode_str = "ALWAYS_OPEN";
        else if (door_mode == ALWAYS_CLOSED) mode_str = "ALWAYS_CLOSED";
        else mode_str = "AUTO";
        portEXIT_CRITICAL(&doorModeMux);
        client->text("door_mode:" + mode_str);
        Serial.printf("[%s] Inviato stato modalità: %s\n", time_str, mode_str.c_str());
        // Invia il log corrente
        DynamicJsonDocument doc(8192);
        JsonArray log_array = doc.createNestedArray("log");
        for (size_t i = 0; i < LOG_BUFFER_SIZE; i++) {
            size_t idx = (log_buffer_index - 1 - i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
            if (log_buffer[idx].timestamp[0] == '\0') continue; // Entry vuota
            JsonObject entry = log_array.createNestedObject();
            entry["timestamp"] = log_buffer[idx].timestamp;
            entry["type"] = log_buffer[idx].type;
            entry["name"] = log_buffer[idx].name;
            entry["country_code"] = log_buffer[idx].country_code;
            entry["device_code"] = String((unsigned long long)log_buffer[idx].device_code);
            entry["authorized"] = log_buffer[idx].authorized;
        }
        String json;
        serializeJson(doc, json);
        client->text(json);
        Serial.printf("[%s] Log iniziale inviato al client\n", time_str);
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[%s] Client WebSocket disconnesso\n", time_str);
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            String message = (char*)data;
            Serial.printf("[%s] Ricevuto dal client: %s\n", time_str, message.c_str());

            if (message == "get_buffer") {
                Serial.printf("[%s] Inizio acquisizione da WebSocket...\n", time_str);
                uint32_t current_index = i_interrupt;
                uint32_t start_index = (current_index - 10000 + ADC_BUFFER_SIZE) % ADC_BUFFER_SIZE;
                Serial.printf("[%s] DEBUG: i_interrupt=%u, start_index=%u\n", time_str, current_index, start_index);
                if (start_index + 10000 <= ADC_BUFFER_SIZE) {
                    memcpy(temp_buffer, (const void*)&adc_buffer[start_index], 10000 * sizeof(uint16_t));
                } else {
                    uint32_t first_chunk_size = ADC_BUFFER_SIZE - start_index;
                    uint32_t second_chunk_size = 10000 - first_chunk_size;
                    memcpy(temp_buffer, (const void*)&adc_buffer[start_index], first_chunk_size * sizeof(uint16_t));
                    memcpy(&temp_buffer[first_chunk_size], (const void*)adc_buffer, second_chunk_size * sizeof(uint16_t));
                }
                Serial.printf("[%s] Invio buffer al client...\n", time_str);
                client->binary((uint8_t*)temp_buffer, 10000 * sizeof(uint16_t));
                Serial.printf("[%s] Buffer inviato al client (binario)\n", time_str);
            } else if (message == "get_log") {
                Serial.printf("[%s] Invio log al client...\n", time_str);
                DynamicJsonDocument doc(8192);
                JsonArray log_array = doc.createNestedArray("log");
                for (size_t i = 0; i < LOG_BUFFER_SIZE; i++) {
                    size_t idx = (log_buffer_index - 1 - i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
                    if (log_buffer[idx].timestamp[0] == '\0') continue; // Entry vuota
                    JsonObject entry = log_array.createNestedObject();
                    entry["timestamp"] = log_buffer[idx].timestamp;
                    entry["type"] = log_buffer[idx].type;
                    entry["name"] = log_buffer[idx].name;
                    entry["country_code"] = log_buffer[idx].country_code;
                    entry["device_code"] = String((unsigned long long)log_buffer[idx].device_code);
                    entry["authorized"] = log_buffer[idx].authorized;
                }
                String json;
                serializeJson(doc, json);
                client->text(json);
                Serial.printf("[%s] Log inviato al client\n", time_str);
            } else if (message.startsWith("set_door_mode:")) {
                String mode = message.substring(14);
                Serial.printf("[%s] Richiesta impostazione modalità: %s\n", time_str, mode.c_str());
                bool mode_changed = false;
                portENTER_CRITICAL(&doorModeMux);
                if (mode == "AUTO" && door_mode != AUTO) {
                    door_mode = AUTO;
                    mode_changed = true;
                } else if (mode == "ALWAYS_OPEN" && door_mode != ALWAYS_OPEN) {
                    door_mode = ALWAYS_OPEN;
                    mode_changed = true;
                } else if (mode == "ALWAYS_CLOSED" && door_mode != ALWAYS_CLOSED) {
                    door_mode = ALWAYS_CLOSED;
                    mode_changed = true;
                }
                portEXIT_CRITICAL(&doorModeMux);
                if (mode_changed) {
                    saveConfig();
                    Serial.printf("[%s] Modalità porta aggiornata: %s\n", time_str, mode.c_str());
                    ws.textAll("door_mode:" + mode); // Invia a tutti i client
                    client->text("Mode updated: " + mode);
                } else {
                    client->text("Mode unchanged");
                }
            }
        }
    }
}

// Configurazione RMT
#define RMT_CHANNEL RMT_CHANNEL_0
#define RMT_CLK_DIV 1
#define RMT_TICK_1_US (80000000 / RMT_CLK_DIV / 1000000)

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

    if (!SPIFFS.begin(true)) {
        Serial.println("Errore: inizializzazione SPIFFS fallita");
        return;
    }
    Serial.println("SPIFFS inizializzato");

    if (!readConfig()) {
        writeDefaultConfig();
        readConfig();
    }

    pinMode(pblue, INPUT_PULLUP);
    pinMode(expblue, INPUT);
    pinMode(ledverde, OUTPUT);
    digitalWrite(ledverde, HIGH);
    pinMode(detected, OUTPUT);
    digitalWrite(detected, HIGH);
    pinMode(wifi_led, OUTPUT);
    digitalWrite(wifi_led, HIGH); // LED spento

    xTaskCreatePinnedToCore(wifi_task, "WiFi_Task", 4096, NULL, 1, NULL, 0);

    setupRMT();

    fadcInit(1, 1);
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1 << ADC_CHANNEL);
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;

    timer = timerBegin(0, 4, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 298, true);

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

    last_millis = millis();
}

void loop() {
    if (wifi_connected) {
        ws.cleanupClients();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}