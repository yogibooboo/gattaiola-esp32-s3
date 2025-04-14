#include <Arduino.h>
#include <driver/ledc.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
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
#define pblue 39
#define ledverde 21
#define ADC_CHANNEL 0

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
                if (buffer_index >= 10000) {
                    Serial.println("Invio buffer al client...");
                    client->binary((uint8_t*)adc_buffer, 10000 * sizeof(uint16_t));
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

    fadcInit(1, 1);
    SENS.sar_meas1_ctrl2.sar1_en_pad = (1 << ADC_CHANNEL);
    SENS.sar_meas1_ctrl2.meas1_start_sar = 1;

    timer = timerBegin(0, 4, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 149, true);

    start_rfid_task(); // Avvia task sul core 1
}

void loop() {
    ws.cleanupClients();
    vTaskDelay(10 / portTICK_PERIOD_MS);
}