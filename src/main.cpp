#include <Arduino.h>
#include <driver/ledc.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <driver/gpio.h>
#include "analog.h"
#include "core1.h"

hw_timer_t *timer = NULL;

const char *ssid = "VodafoneRibes";
const char *password = "scheggia2000";
#define PWM_PIN 14
#define PWM_CHANNEL 0
#define PWM_FREQ 134200
#define PWM_RESOLUTION 4
AsyncWebSocket ws("/ws");
AsyncWebServer server(80);
#define pblue 38
#define ledverde 21
#define ADC_CHANNEL 0
#define DOOR_GPIO GPIO_NUM_15 // GPIO per gattaiola
#define DOOR_TIMEOUT (10000 / portTICK_PERIOD_MS) // 10 s

// Task gestione gattaiola (core 0, ogni 100 ms)
void door_task(void *pvParameters) {
    while (true) {
        if (door_sync_count > 0) {
            if (!door_open) {
                gpio_set_level(DOOR_GPIO, 1); // Apri gattaiola
                door_open = true;
                door_timer_start = xTaskGetTickCount();
            } else {
                door_timer_start = xTaskGetTickCount(); // Azzera timer
            }
            door_sync_count = 0;
        } else if (door_open) {
            TickType_t now = xTaskGetTickCount();
            if ((now - door_timer_start) >= DOOR_TIMEOUT) {
                gpio_set_level(DOOR_GPIO, 0); // Chiudi gattaiola
                door_open = false;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // 100 ms
    }
}

// Task di stampa (core 0, ogni 1 s)
void print_task(void *pvParameters) {
    while (true) {
        printf("Sync: %u, OK: %u, Last Seq: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X], "
               "Device Code: %llu, i: %u, i_interrupt: %u, available_samples: %ld\n",
               sync_count, display_sync_count,
               last_sequence[0], last_sequence[1], last_sequence[2], last_sequence[3],
               last_sequence[4], last_sequence[5], last_sequence[6], last_sequence[7],
               last_sequence[8], last_sequence[9],
               (unsigned long long)last_device_code, i, i_interrupt, (long)available_samples);
        sync_count = 0;
        display_sync_count = 0;
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 s
    }
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
                Serial.println("Comando get_buffer temporaneamente disabilitato");
                client->text("Comando get_buffer temporaneamente disabilitato");
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Avvio ESP32-S3");

    // Configura GPIO per gattaiola
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DOOR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(DOOR_GPIO, 0); // Porta chiusa

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

    fadcInit(1, 1);
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1 << ADC_CHANNEL);
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;

    timer = timerBegin(0, 4, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 149, true);

    start_rfid_task(); // Avvia task sul core 1
    xTaskCreatePinnedToCore(door_task, "Door_Task", 4096, NULL, 2, NULL, 0); // Gattaiola su core 0
    xTaskCreatePinnedToCore(print_task, "Print_Task", 4096, NULL, 1, NULL, 0); // Stampa su core 0

    timerAlarmEnable(timer); // Avvia acquisizione continua
}

void loop() {
    ws.cleanupClients();
    vTaskDelay(10 / portTICK_PERIOD_MS);
}