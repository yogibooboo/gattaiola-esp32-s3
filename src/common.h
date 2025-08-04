#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <time.h>
#include "core1.h"
#include <AS5600.h>

// Strutture dati
#define MAX_CATS 20
struct Cat {
    uint64_t device_code;
    uint16_t country_code;
    String name;
    bool authorized;
};

#define LOG_BUFFER_SIZE 100
struct LogEntry {
    char timestamp[20];
    String type;
    String name;
    uint16_t country_code;
    uint64_t device_code;
    bool authorized;
};

#define ENCODER_BUFFER_SIZE (1 << 14) // 2^14 = 16384

struct EncoderData {
    uint8_t infrared : 1;   // Bit 0
    uint8_t detect : 1;     // Bit 1
    uint8_t door_open : 1;  // Bit 2
    uint8_t padding : 1;    // Bit 3
    uint16_t rawAngle : 12; // Bit 4-15
};

// Enum per door_mode
enum DoorMode { AUTO, ALWAYS_OPEN, ALWAYS_CLOSED };

// Enum per motor_type
enum MotorType { STEP, SERVO };

// Costanti per i pin
#define PWM_PIN 14
#define PWM_FREQ 134200
#define pblue 39
#define ledrosso 7
#define APERTO LOW
#define CHIUSO HIGH
#define detected 15
#define wifi_led 6
#define STEP_A_PLUS 10
#define STEP_A_MINUS 16
#define STEP_B_PLUS 12
#define STEP_B_MINUS 13
#define ENABLE_PIN 37
#define INFRARED_PIN 1

// Costanti per ADC
#define ADC_CHANNEL 3

// Costanti per servomotore
#define SERVO_PWM_FREQ 50
#define SERVO_PWM_CHANNEL LEDC_CHANNEL_0
#define SERVO_PWM_RESOLUTION 14

// Costanti per RMT
#define RMT_CHANNEL RMT_CHANNEL_0
#define RMT_CLK_DIV 1
#define RMT_TICK_1_US (80000000 / RMT_CLK_DIV / 1000000)

// Variabili globali definite in main.cpp
extern const char *ssid;
extern const char *password;
extern hw_timer_t *timer;
extern AsyncWebSocket ws;
extern AsyncWebServer server;
extern uint32_t DOOR_TIMEOUT;
extern uint32_t WIFI_RECONNECT_DELAY;
extern uint32_t UNAUTHORIZED_LOG_INTERVAL;
extern bool WIFI_VERBOSE_LOG;
extern Cat authorized_cats[MAX_CATS];
extern size_t num_cats;
extern LogEntry log_buffer[LOG_BUFFER_SIZE];
extern size_t log_buffer_index;
extern volatile int currentStep;
extern volatile int stepsRemaining;
extern volatile bool motorRunning;
extern volatile bool motorDirection;
extern hw_timer_t *stepTimer;
extern portMUX_TYPE stepTimerMux;
extern volatile bool wifi_connected;
extern portMUX_TYPE wifiMux;
extern unsigned long last_millis;
extern time_t local_time;
extern volatile DoorMode door_mode;
extern portMUX_TYPE doorModeMux;
extern uint16_t temp_buffer[16384];
extern uint32_t contaporta;
extern volatile bool door_open;
extern volatile uint32_t door_sync_count;
extern volatile uint16_t last_country_code;
extern volatile uint64_t last_device_code;
extern volatile TickType_t door_timer_start;
extern volatile uint32_t i_interrupt;
extern volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
extern uint32_t STEPS_PER_MOVEMENT;
extern uint32_t STEP_INTERVAL_US;
extern const uint8_t stepSequence[4][4];
extern volatile MotorType motor_type;
extern uint32_t servo_open_us;
extern uint32_t servo_closed_us;
extern uint32_t servo_transition_ms;
extern EncoderData encoder_buffer[ENCODER_BUFFER_SIZE];
extern size_t encoder_buffer_index;
extern volatile uint16_t lastRawAngle;
extern volatile uint16_t lastMagnitude;
extern volatile uint32_t last_encoder_timestamp;
extern AS5600 encoder;
// Aggiunta per debug WebSocket
extern volatile bool debug_stream_enabled;
extern portMUX_TYPE debugMux;
// Nuovi parametri
extern uint32_t config_01;
extern uint32_t config_02;
extern uint32_t config_03;
extern uint32_t config_04;
extern uint32_t config_05;
extern uint32_t config_06;
extern uint32_t config_07;
extern uint32_t config_08;
extern uint32_t config_09;
extern uint32_t config_10;
extern uint32_t door_rest;
extern uint32_t door_in;
extern uint32_t door_out;
extern volatile uint16_t lastCorrectedAngle; // Nuova variabile

// Funzioni definite in wifi.cpp
void wifi_task(void *pvParameters);
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

// Funzioni definite in door.cpp
void onStepTimer();
void startMotor(bool direction);
void setServoPosition(bool open);
void add_log_entry(const char* timestamp, const char* type, const String& name, uint16_t country_code, uint64_t device_code, bool authorized);
void door_task(void *pvParameters);

// Funzioni definite in main.cpp
bool readConfig();
void writeDefaultConfig();
void saveConfig();
void print_task(void *pvParameters);

// Funzione di debug generica
void logDebug(const char* format, ...);
void logDebug(const String& message);

#endif