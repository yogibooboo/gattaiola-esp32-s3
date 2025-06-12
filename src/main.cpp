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

// Sequenza FDX-B (128 bit)
const uint8_t fdx_b_sequence[128] = {
    0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,1,
    0,0,1,0,0,1,0,0,1,1,1,1,0,0,1,1,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,1,
    1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,0,0,1,1,1,0,0,0,1,1,0,0,1,
    0,0,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0,1,1
};

// Variabili globali
hw_timer_t *timer = NULL;
AsyncWebSocket ws("/ws");
AsyncWebServer server(80);
uint32_t DOOR_TIMEOUT = 10000 / portTICK_PERIOD_MS;
uint32_t WIFI_RECONNECT_DELAY = 1000;
uint32_t UNAUTHORIZED_LOG_INTERVAL = 60000;
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
volatile MotorType motor_type = STEP;
uint32_t servo_open_us = 2000;
uint32_t servo_closed_us = 1000;
uint32_t servo_transition_ms = 500;

// Variabili per il segnale FDX-B
hw_timer_t *fdxTimer = NULL;
volatile size_t fdx_bit_index = 0;
volatile bool fdx_last_half_value = false;
portMUX_TYPE fdxTimerMux = portMUX_INITIALIZER_UNLOCKED;
#define FDX_B_PIN 18  // Pin GPIO per il segnale FDX-B
#define HALF_BIT_TICKS 298  // Conteggio per mezza bit (119.2 μs con prescaler 64)

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

// ISR per il segnale FDX-B
void IRAM_ATTR onFdxTimer() {
    portENTER_CRITICAL_ISR(&fdxTimerMux);
    
    // Determina il bit corrente
    uint8_t bit = fdx_b_sequence[fdx_bit_index / 2];
    
    // Prima metà: opposta all'ultima metà del bit precedente
    bool first_half_value = !fdx_last_half_value;
    if (fdx_bit_index % 2 == 0) {
        digitalWrite(FDX_B_PIN, first_half_value);
    } else {
        // Seconda metà: cambia livello solo se il bit è 0
        bool second_half_value = (bit == 0) ? !first_half_value : first_half_value;
        digitalWrite(FDX_B_PIN, second_half_value);
        fdx_last_half_value = second_half_value;
    }
    
    // Avanza l'indice (torna a 0 dopo 256, cioè 128 bit x 2 semionde)
    fdx_bit_index = (fdx_bit_index + 1) % 256;
    
    portEXIT_CRITICAL_ISR(&fdxTimerMux);
}

// Funzione per leggere config.json da SPIFFS
bool readConfig() {
    File file = SPIFFS.open("/config.json", "r");
    if (!file) {
        Serial.println("Errore: impossibile aprire config.json");
        return false;
    }

    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.printf("Errore parsing JSON: %s\n", error.c_str());
        return false;
    }

    num_cats = 0;
    JsonArray cats = doc["authorized_cats"];
    for (JsonObject cat : cats) {
        if (num_cats >= MAX_CATS) break;
        String device_code_str = cat["device_code"];
        uint64_t device_code;
        if (!sscanf(device_code_str.c_str(), "%llu", &device_code)) {
            Serial.printf("Errore: device_code %s non valido\n", device_code_str.c_str());
            continue;
        }
        uint16_t country_code = cat["country_code"] | 0;
        String name = cat["name"].as<String>();
        bool authorized = cat["authorized"] | true;

        if (device_code >= (1ULL << 38)) {
            Serial.printf("Errore: device_code %llu troppo grande\n", (unsigned long long)device_code);
            continue;
        }
        if (country_code >= (1U << 10)) {
            Serial.printf("Errore: country_code %u troppo grande\n", country_code);
            continue;
        }
        if (name.length() > 32) {
            name = name.substring(0, 32);
        }

        authorized_cats[num_cats] = {device_code, country_code, name, authorized};
        num_cats++;
    }

    DOOR_TIMEOUT = doc["door_timeout"] | (10000 / portTICK_PERIOD_MS);
    WIFI_RECONNECT_DELAY = doc["wifi_reconnect_delay"] | 1000;
    UNAUTHORIZED_LOG_INTERVAL = doc["unauthorized_log_interval"] | 60000;
    STEPS_PER_MOVEMENT = doc["steps_per_movement"] | 2500;
    STEP_INTERVAL_US = doc["step_interval_us"] | 500;
    WIFI_VERBOSE_LOG = doc["wifi_verbose_log"] | false;
    contaporta = doc["contaporta"] | 0;

    String mode = doc["door_mode"] | "AUTO";
    portENTER_CRITICAL(&doorModeMux);
    if (mode == "ALWAYS_OPEN") door_mode = ALWAYS_OPEN;
    else if (mode == "ALWAYS_CLOSED") door_mode = ALWAYS_CLOSED;
    else door_mode = AUTO;
    portEXIT_CRITICAL(&doorModeMux);

    String motor = doc["motor_type"] | "step";
    if (motor == "servo") motor_type = SERVO;
    else motor_type = STEP;
    servo_open_us = doc["servo_open_us"] | 2000;
    servo_closed_us = doc["servo_closed_us"] | 1000;
    servo_transition_ms = doc["servo_transition_ms"] | 500;

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

    DynamicJsonDocument doc(8192);
    doc["door_timeout"] = 10000 / portTICK_PERIOD_MS;
    doc["wifi_reconnect_delay"] = 1000;
    doc["unauthorized_log_interval"] = 60000;
    doc["steps_per_movement"] = 2500;
    doc["step_interval_us"] = 500;
    doc["wifi_verbose_log"] = false;
    doc["contaporta"] = 0;
    doc["door_mode"] = "AUTO";
    doc["motor_type"] = "step";
    doc["servo_open_us"] = 2000;
    doc["servo_closed_us"] = 1000;
    doc["servo_transition_ms"] = 500;
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

    DynamicJsonDocument doc(8192);
    JsonArray cats = doc.createNestedArray("authorized_cats");
    for (size_t i = 0; i < num_cats; i++) {
        JsonObject cat = cats.createNestedObject();
        cat["device_code"] = String((unsigned long long)authorized_cats[i].device_code);
        cat["country_code"] = authorized_cats[i].country_code;
        cat["name"] = authorized_cats[i].name;
        cat["authorized"] = authorized_cats[i].authorized;
    }
    doc["door_timeout"] = DOOR_TIMEOUT;
    doc["wifi_reconnect_delay"] = WIFI_RECONNECT_DELAY;
    doc["unauthorized_log_interval"] = UNAUTHORIZED_LOG_INTERVAL;
    doc["steps_per_movement"] = STEPS_PER_MOVEMENT;
    doc["step_interval_us"] = STEP_INTERVAL_US;
    doc["wifi_verbose_log"] = WIFI_VERBOSE_LOG;
    doc["contaporta"] = contaporta;
    doc["motor_type"] = (motor_type == SERVO) ? "servo" : "step";
    doc["servo_open_us"] = servo_open_us;
    doc["servo_closed_us"] = servo_closed_us;
    doc["servo_transition_ms"] = servo_transition_ms;
    portENTER_CRITICAL(&doorModeMux);
    if (door_mode == ALWAYS_OPEN) doc["door_mode"] = "ALWAYS_OPEN";
    else if (door_mode == ALWAYS_CLOSED) doc["door_mode"] = "ALWAYS_CLOSED";
    else doc["door_mode"] = "AUTO";
    portEXIT_CRITICAL(&doorModeMux);

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
    digitalWrite(wifi_led, HIGH);

    xTaskCreatePinnedToCore(wifi_task, "WiFi_Task", 4096, NULL, 1, NULL, 0);

    setupRMT();

    fadcInit(1, 1);
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1 << ADC_CHANNEL);
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;

    timer = timerBegin(0, 4, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 298, true);

    if (motor_type == STEP) {
        Serial.println("Configurazione Step Motor");
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
    } else {
        Serial.println("Configurazione Servomotore");
        pinMode(STEP_A_PLUS, OUTPUT);
        digitalWrite(STEP_A_PLUS, LOW); // GND
        pinMode(STEP_A_MINUS, OUTPUT);
        digitalWrite(STEP_A_MINUS, HIGH); // VCC
        pinMode(STEP_B_PLUS, OUTPUT);
        pinMode(STEP_B_MINUS, OUTPUT);
        digitalWrite(STEP_B_MINUS, LOW); // Inutilizzato
        pinMode(ENABLE_PIN, OUTPUT);
        digitalWrite(ENABLE_PIN, HIGH); // Abilita buffer

        Serial.printf("Inizializzazione LEDC: canale=%d, freq=%d Hz, risoluzione=%d bit\n", 
                      SERVO_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
        if (ledcSetup(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION) == 0) {
            Serial.println("ERRORE: ledcSetup fallito");
        } else {
            Serial.println("ledcSetup completato con successo");
        }
        ledcAttachPin(STEP_B_PLUS, SERVO_PWM_CHANNEL);
        uint32_t duty = (servo_closed_us * (1ULL << SERVO_PWM_RESOLUTION)) / 20000;
        Serial.printf("Impostazione PWM iniziale: servo_closed_us=%u us, duty=%u\n", servo_closed_us, duty);
        ledcWrite(SERVO_PWM_CHANNEL, duty); // Inizializza in posizione chiusa
    }

    // Configurazione segnale FDX-B
    pinMode(FDX_B_PIN, OUTPUT);
    digitalWrite(FDX_B_PIN, LOW);
    fdxTimer = timerBegin(2, 32, true); // Timer 2, prescaler 64 (1 tick = 0.8 μs)
    timerAttachInterrupt(fdxTimer, &onFdxTimer, true);
    timerAlarmWrite(fdxTimer, HALF_BIT_TICKS, true);
    timerAlarmEnable(fdxTimer);

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