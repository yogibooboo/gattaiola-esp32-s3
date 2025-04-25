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
#define pblue 2
#define expblue 39
#define ledverde 21
#define ADC_CHANNEL 0
#define DOOR_GPIO GPIO_NUM_20 // GPIO per gattaiola
#define DOOR_TIMEOUT (10000 / portTICK_PERIOD_MS) // 10 s

// Definizioni per lo stepper motor
#define STEP_A_PLUS 10
#define STEP_A_MINUS 16         //era 11     crretto per errore di master
#define STEP_B_PLUS 12
#define STEP_B_MINUS 13
#define ENABLE_PIN 9
#define STEPS_PER_MOVEMENT 2500 // Numero di passi configurabile
#define STEP_INTERVAL_US 1000 // Intervallo tra passi in microsecondi (0,1 ms = 100 us)

// Sequenza delle fasi per full-step
const uint8_t stepSequence[4][4] = {
    {1, 0, 0, 1}, // A+ B-
    {1, 0, 1, 0}, // A+ B+
    {0, 1, 1, 0}, // A- B+
    {0, 1, 0, 1}  // A- B-
};

// Variabili globali per il controllo del motore
volatile int currentStep = 0;
volatile int stepsRemaining = 0;
volatile bool motorRunning = false;
volatile bool motorDirection = true; // true = orario (apertura), false = antiorario (chiusura)
hw_timer_t *stepTimer = NULL;
portMUX_TYPE stepTimerMux = portMUX_INITIALIZER_UNLOCKED;

// ISR per il controllo delle fasi
void IRAM_ATTR onStepTimer() {
    portENTER_CRITICAL_ISR(&stepTimerMux);
    
    if (stepsRemaining > 0) {
        // Scrivi la sequenza corrente sui pin
        digitalWrite(STEP_A_PLUS, stepSequence[currentStep][0]);
        digitalWrite(STEP_A_MINUS, stepSequence[currentStep][1]);
        digitalWrite(STEP_B_PLUS, stepSequence[currentStep][2]);
        digitalWrite(STEP_B_MINUS, stepSequence[currentStep][3]);

        // Aggiorna il passo corrente
        if (motorDirection) {
            currentStep = (currentStep + 1) % 4; // Avanti
        } else {
            currentStep = (currentStep - 1 + 4) % 4; // Indietro
        }

        stepsRemaining--;
    } else {
        // Ferma il motore
        timerStop(stepTimer);
        digitalWrite(ENABLE_PIN, LOW); // Disattiva driver
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
        
        // Attiva il driver
        digitalWrite(ENABLE_PIN, HIGH);
        
        // Avvia il timer
        timerStart(stepTimer);
    }
    
    portEXIT_CRITICAL(&stepTimerMux);
}

uint32_t contaporta = 0;
// Task gestione gattaiola (core 0, ogni 100 ms)
void door_task(void *pvParameters) {
    while (true) {
        if ((door_sync_count > 0)||!digitalRead(pblue)) {
            if (!door_open) {
                gpio_set_level(DOOR_GPIO, 0); // Apri gattaiola
                digitalWrite(ledverde, HIGH); // Accendi LED
                door_open = true;
                door_timer_start = xTaskGetTickCount();
                startMotor(true); // Avvia motore in direzione oraria (apertura)
            } else {
                door_timer_start = xTaskGetTickCount(); // Azzera timer
            }
            contaporta = 0;
            door_sync_count = 0;
        } else if (door_open) {
            TickType_t now = xTaskGetTickCount();
            contaporta = (now - door_timer_start);
            if ((now - door_timer_start) >= DOOR_TIMEOUT) {
                contaporta = 12000;
                gpio_set_level(DOOR_GPIO, 1); // Chiudi gattaiola
                digitalWrite(ledverde, LOW); // Spegni LED
                door_open = false;
                startMotor(false); // Avvia motore in direzione antioraria (chiusura)
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // 100 ms
    }
}

// Task di stampa (core 0, ogni 1 s)
void print_task(void *pvParameters) {
    while (true) {
        printf("Sync: %u, OK: %u, Last Seq: [%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X], "
               "Device Code: %llu, ia: %u, i_interrupt: %u, diff: %u, available_samples: %ld, contaporta: %u\n",
               sync_count, display_sync_count,
               last_sequence[0], last_sequence[1], last_sequence[2], last_sequence[3],
               last_sequence[4], last_sequence[5], last_sequence[6], last_sequence[7],
               last_sequence[8], last_sequence[9],
               (unsigned long long)last_device_code, ia, i_interrupt, (i_interrupt-ia), (long)available_samples, contaporta);
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
    gpio_set_level(DOOR_GPIO, 1); // Porta chiusa

    pinMode(pblue, INPUT_PULLUP);
    pinMode(expblue, INPUT); //è rimasto in parallelo a pblue
    pinMode(ledverde, OUTPUT);
    //pinMode(15, OUTPUT);

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

    // Configura lo stepper motor
    pinMode(STEP_A_PLUS, OUTPUT);
    pinMode(STEP_A_MINUS, OUTPUT);
    pinMode(STEP_B_PLUS, OUTPUT);
    pinMode(STEP_B_MINUS, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Motore fermo inizialmente

    // Configura il timer per lo stepper (timer 1, prescaler 80 -> 1 tick = 1 us)
    stepTimer = timerBegin(1, 80, true); // Timer separato da quello dell'acquisizione
    timerAttachInterrupt(stepTimer, &onStepTimer, true);
    timerAlarmWrite(stepTimer, STEP_INTERVAL_US, true); // Intervallo configurabile
    timerAlarmEnable(stepTimer);
    timerStop(stepTimer); // Timer fermo fino all'avvio del motore

    start_rfid_task(); // Avvia task sul core 1
    xTaskCreatePinnedToCore(door_task, "Door_Task", 4096, NULL, 2, NULL, 0); // Gattaiola su core 0
    xTaskCreatePinnedToCore(print_task, "Print_Task", 4096, NULL, 1, NULL, 0); // Stampa su core 0

    timerAlarmEnable(timer); // Avvia acquisizione continua
}

void loop() {
    ws.cleanupClients();
    vTaskDelay(10 / portTICK_PERIOD_MS);
}