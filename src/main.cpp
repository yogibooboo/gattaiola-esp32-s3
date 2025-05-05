#include <Arduino.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include "analog.h"
#include "core1.h"
#include <driver/rmt.h>
#include "common.h"

// Credenziali Wi-Fi
const char *ssid = "VodafoneRibes";
const char *password = "scheggia2000";

// Definizione della sequenza delle fasi per full-step
const uint8_t stepSequence[4][4] = {
    {1, 0, 0, 1},
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1}
};

// Variabili globali
hw_timer_t *timer = NULL;
AsyncWebSocket ws("/ws");
AsyncWebServer server(80);
uint32_t DOOR_TIMEOUT = 10000 / portTICK_PERIOD_MS;
uint32_t WIFI_RECONNECT_DELAY = 1000;
uint32_t UNAUTHORIZED_LOG_INTERVAL = 60000; // 1 min, configurabile
bool WIFI_VERBOSE_LOG = false;
Cat authorized_cats[MAX_CATS];
size_t num_cats = 0;
LogEntry log_buffer[LOG_BUFFER_SIZE];
size_t log_buffer_index = 0;
volatile int currentStep = 0;
volatile int stepsRemaining = 0;
volatile bool motorRunning = false;
volatile bool motorDirection = true;
hw_timer_t *stepTimer = NULL;
portMUX_TYPE stepTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool wifi_connected = false;
portMUX_TYPE wifiMux = portMUX_INITIALIZER_UNLOCKED;
unsigned long last_millis = 0;
time_t local_time = 0;
volatile DoorMode door_mode = AUTO;
portMUX_TYPE doorModeMux = portMUX_INITIALIZER_UNLOCKED;
uint16_t temp_buffer[10000];
uint32_t contaporta = 0;
uint32_t STEPS_PER_MOVEMENT = 2500;
uint32_t STEP_INTERVAL_US = 500;

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

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("DEBUG: Avvio ESP32-S3 OTA");
    Serial.printf("Flash Size: %u bytes\n", ESP.getFlashChipSize());
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it != NULL) {
        const esp_partition_t *part = esp_partition_get(it);
        Serial.printf("Partizione: %s, Offset: 0x%X, Dimensione: %u bytes\n",
                      part->label, part->address, part->size);
        it = esp_partition_next(it);
    }
    esp_partition_iterator_release(it);

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