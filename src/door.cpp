#include "common.h"
#include <driver/ledc.h>

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

// Funzione per controllare il servomotore
void setServoPosition(bool open) {
    static uint32_t current_servo_us = 0; // Tiene traccia della posizione corrente
    if (current_servo_us == 0) current_servo_us = servo_closed_us; // Inizializza a chiuso

    uint32_t target_us = open ? servo_open_us : servo_closed_us;
    int32_t delta_us = (int32_t)target_us - (int32_t)current_servo_us;
    if (delta_us == 0) {
        return;
    }

    uint32_t steps = servo_transition_ms / 10; // Aggiornamenti ogni 10 ms
    if (steps == 0) steps = 1;

    for (uint32_t i = 0; i < steps; i++) {
        int32_t progress = (int32_t)(i + 1) * delta_us;
        uint32_t new_us = current_servo_us + (progress / (int32_t)steps);
        uint32_t duty = (new_us * (1ULL << SERVO_PWM_RESOLUTION)) / 20000;
        ledcWrite(SERVO_PWM_CHANNEL, duty);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Imposta esattamente il valore finale
    uint32_t duty = (target_us * (1ULL << SERVO_PWM_RESOLUTION)) / 20000;
    ledcWrite(SERVO_PWM_CHANNEL, duty);
    current_servo_us = target_us; // Aggiorna la posizione corrente
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

    if (wifi_connected) {
        DynamicJsonDocument doc(8192);
        JsonArray log_array = doc.createNestedArray("log");
        for (size_t i = 0; i < LOG_BUFFER_SIZE; i++) {
            size_t idx = (log_buffer_index - 1 - i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
            if (log_buffer[idx].timestamp[0] == '\0') continue;
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

// Task gestione gattaiola
void door_task(void *pvParameters) {
    char time_str[20];
    bool log_emitted = false;
    unsigned long last_unauthorized_log = 0;
    DoorMode last_mode = AUTO;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = 100 / portTICK_PERIOD_MS;
    bool detect = false;

    // Inizializzazione delle variabili per l'ultimo gatto autorizzato
    last_authorized_device_code = 0;
    last_authorized_country_code = 0;

    // Variabili statiche per l'analisi degli eventi
    static enum { IDLE, EVENTO_ATTIVO } stato = IDLE;
    static char timestamp_inizio[20] = "";
    static uint32_t indice_inizio = 0;
    static bool passaggio_confermato = false;
    static const char* direzione = NULL;
    static uint32_t conteggio_senza_trigger = 0;
    static uint32_t ultimo_trigger_idx = 0;
    static int32_t trigger_porta_idx = -1;
    static int32_t passaggio_porta_idx = -1;
    static int32_t detect_idx = -1;
    static int32_t infrared_idx = -1;
    static String cat_name_evento = "Sconosciuto";
    static uint16_t country_code_evento = 0;
    static uint64_t device_code_evento = 0;
    static bool authorized_evento = false;
    static bool codice_associato = false;

    portENTER_CRITICAL(&doorModeMux);
    DoorMode initial_mode = door_mode;
    portEXIT_CRITICAL(&doorModeMux);

    if (initial_mode == ALWAYS_OPEN && !door_open) {
        digitalWrite(ledrosso, APERTO);
        door_open = true;
        if (motor_type == STEP) startMotor(true);
        else setServoPosition(true);
        time_t now = local_time + (millis() - last_millis) / 1000;
        char timestamp_full[20];
        strftime(timestamp_full, sizeof(timestamp_full), "%d/%m/%Y %H:%M:%S", localtime(&now));
        add_log_entry(timestamp_full, "Modalità sempre aperto", "Modalità Sempre Aperto", 0, 0, true);
        Serial.println("Porta aperta per modalità ALWAYS_OPEN");
    } else if (initial_mode == ALWAYS_CLOSED && door_open) {
        digitalWrite(ledrosso, CHIUSO);
        door_open = false;
        if (motor_type == STEP) startMotor(false);
        else setServoPosition(false);
        time_t now = local_time + (millis() - last_millis) / 1000;
        char timestamp_full[20];
        strftime(timestamp_full, sizeof(timestamp_full), "%d/%m/%Y %H:%M:%S", localtime(&now));
        add_log_entry(timestamp_full, "Modalità sempre chiuso", "Modalità Sempre Chiuso", 0, 0, false);
        Serial.println("Porta chiusa per modalità ALWAYS_CLOSED");
    } else if (initial_mode == AUTO && door_open) {
        digitalWrite(ledrosso, CHIUSO);
        door_open = false;
        if (motor_type == STEP) startMotor(false);
        else setServoPosition(false);
        time_t now = local_time + (millis() - last_millis) / 1000;
        char timestamp_full[20];
        strftime(timestamp_full, sizeof(timestamp_full), "%d/%m/%Y %H:%M:%S", localtime(&now));
        add_log_entry(timestamp_full, "Modalità automatica", "Modalità Automatica", 0, 0, false);
        Serial.println("Porta chiusa per modalità AUTO");
    }
    last_mode = initial_mode;

    while (true) {
        // Dichiarazione delle variabili usate nell'analisi
        String cat_name = "Sconosciuto";
        bool is_authorized = false;
        uint16_t country_code = 0;
        uint64_t device_code = 0;

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
                    digitalWrite(ledrosso, APERTO);
                    door_open = true;
                    if (motor_type == STEP) startMotor(true);
                    else setServoPosition(true);
                    add_log_entry(timestamp_full, "Modalità sempre aperto", "Modalità Sempre Aperto", 0, 0, true);
                    Serial.println("Porta aperta per modalità ALWAYS_OPEN");
                }
            } else if (current_mode == ALWAYS_CLOSED) {
                if (door_open) {
                    digitalWrite(ledrosso, CHIUSO);
                    door_open = false;
                    if (motor_type == STEP) startMotor(false);
                    else setServoPosition(false);
                    add_log_entry(timestamp_full, "Modalità sempre chiuso", "Modalità Sempre Chiuso", 0, 0, false);
                    Serial.println("Porta chiusa per modalità ALWAYS_CLOSED");
                }
            } else if (current_mode == AUTO) {
                if (door_open) {
                    digitalWrite(ledrosso, CHIUSO);
                    door_open = false;
                    if (motor_type == STEP) startMotor(false);
                    else setServoPosition(false);
                    add_log_entry(timestamp_full, "Modalità automatica", "Modalità Automatica", 0, 0, false);
                    Serial.println("Porta chiusa per modalità AUTO");
                } else {
                    add_log_entry(timestamp_full, "Modalità automatica", "Modalità Automatica", 0, 0, false);
                    Serial.println("Passaggio a modalità AUTO");
                }
            }
            last_mode = current_mode;
        }

        detect = false;
        bool newcode = false; // Flag per newcode
        if (door_sync_count > 0) {
            digitalWrite(detected, LOW);
            detect = true;
            is_authorized = false;
            cat_name = "Sconosciuto";
            country_code = last_country_code;
            device_code = last_device_code;

            for (size_t i = 0; i < num_cats; i++) {
                if (authorized_cats[i].device_code == last_device_code) {
                    cat_name = authorized_cats[i].name;
                    is_authorized = authorized_cats[i].authorized;
                    country_code = authorized_cats[i].country_code;
                    break;
                }
            }

            // Gestione newcode: confronta con il codice dell'evento in corso
            if (stato == EVENTO_ATTIVO) {
                if (!codice_associato) {
                    codice_associato = true;
                    cat_name_evento = cat_name;
                    country_code_evento = country_code;
                    device_code_evento = device_code;
                    authorized_evento = is_authorized;
                } else if (device_code != device_code_evento || country_code != country_code_evento) {
                    newcode = true;
                }
            }

            // Gestione log e autorizzazione per gatti autorizzati
            if (is_authorized) {
                if (last_device_code != last_authorized_device_code || country_code != last_authorized_country_code) {
                    log_emitted = false;
                    last_authorized_device_code = last_device_code;
                    last_authorized_country_code = country_code;
                }
                door_timer_start = xTaskGetTickCount();
                if (!log_emitted) {
                    Serial.printf("[%s] Rilevato gatto: %s, Autorizzato: Sì\n", time_str, cat_name.c_str());
                    add_log_entry(timestamp_full, "Autorizzato", cat_name, country_code, device_code, true);
                    log_emitted = true;
                }
                if (!door_open && current_mode == AUTO) {
                    digitalWrite(ledrosso, APERTO);
                    door_open = true;
                    if (motor_type == STEP) startMotor(true);
                    else setServoPosition(true);
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
            if (!digitalRead(pblue) && !door_open && current_mode == AUTO) {
                digitalWrite(ledrosso, APERTO);
                door_open = true;
                door_timer_start = xTaskGetTickCount();
                if (motor_type == STEP) startMotor(true);
                else setServoPosition(true);
                contaporta = 0;
                Serial.printf("[%s] Apertura manuale tramite pblue\n", time_str);
                add_log_entry(timestamp_full, "Manuale", "Manuale", 0, 0, true);
                log_emitted = true;
            } else if (door_open) {
                TickType_t now_ticks = xTaskGetTickCount();
                if (contaporta != 50000) {
                    contaporta = (now_ticks - door_timer_start);
                    if ((now_ticks - door_timer_start) >= DOOR_TIMEOUT) {
                        contaporta = 50000;
                        log_emitted = false;
                        if (current_mode == AUTO) {
                            digitalWrite(ledrosso, CHIUSO);
                            door_open = false;
                            if (motor_type == STEP) startMotor(false);
                            else setServoPosition(false);
                            Serial.printf("[%s] Porta chiusa dopo timeout\n", time_str);
                        }
                    }
                }
            }
        }

        // Leggi encoder e infrared alla fine del ciclo
        uint16_t rawAngle = encoder.readAngle();
        uint16_t magnitude = encoder.readMagnitude();
        bool infrared = digitalRead(INFRARED_PIN) || interruptFlag;  // Sente sia il livello che i fronti in interrupt
        interruptFlag = false;

        if (infrared) digitalWrite(LEDBLUE, LOW); else digitalWrite(LEDBLUE, HIGH);
        if (rawAngle == 0xFFFF) {
            rawAngle = 0x3FFF; // Gestione errore
        }

        // Calcolo di correctedAngle
        uint16_t correctedAngle;
        if (rawAngle == door_rest) {
            correctedAngle = 2000;
        } else if (rawAngle >= door_rest) {
            if (door_out != door_rest) {
                int32_t temp = 2048 + (1024 * (int32_t)(rawAngle - door_rest)) / (int32_t)(door_out - door_rest);
                correctedAngle = (uint16_t)temp;
            } else {
                correctedAngle = 2048; // Evita divisione per zero
            }
        } else {
            if (door_rest != door_in) {
                int32_t temp = 1024 + (1024 * (int32_t)(rawAngle - door_in)) / (int32_t)(door_rest - door_in);
                correctedAngle = (uint16_t)temp;
            } else {
                correctedAngle = 2048; // Evita divisione per zero
            }
        }
        lastCorrectedAngle = correctedAngle;

        // ANALISI QUI
        // Calcolo della deviazione e del trigger
        int32_t angolo_deviazione = abs((int32_t)correctedAngle - 2048);
        bool trigger = (angolo_deviazione > (int32_t)config_02 || infrared || detect);

        if (stato == IDLE) {
            if (trigger) {
                stato = EVENTO_ATTIVO;
                strncpy(timestamp_inizio, timestamp_full, sizeof(timestamp_inizio));
                indice_inizio = 0;
                passaggio_confermato = false;
                direzione = NULL;
                conteggio_senza_trigger = 0;
                ultimo_trigger_idx = 0;
                trigger_porta_idx = (angolo_deviazione > (int32_t)config_02) ? 0 : -1;
                passaggio_porta_idx = -1;
                detect_idx = detect ? 0 : -1;
                infrared_idx = infrared ? 0 : -1;
                codice_associato = false;
                if (detect) {
                    cat_name_evento = cat_name;
                    country_code_evento = country_code;
                    device_code_evento = device_code;
                    authorized_evento = is_authorized;
                    codice_associato = true;
                } else {
                    cat_name_evento = "Sconosciuto";
                    country_code_evento = 0;
                    device_code_evento = 0;
                    authorized_evento = false;
                }
                Serial.printf("Evento iniziato a %s\n", timestamp_inizio);
            }
        } else if (stato == EVENTO_ATTIVO) {
            indice_inizio++;
            if (angolo_deviazione > (int32_t)config_03 && !passaggio_confermato) {
                passaggio_confermato = true;
                direzione = (correctedAngle > 2048) ? "Uscita" : "Ingresso";
                passaggio_porta_idx = indice_inizio;
                Serial.printf("Passaggio confermato (%s) al campione %u\n", direzione, indice_inizio);
            }
            if (detect && !codice_associato) {
                cat_name_evento = cat_name;
                country_code_evento = country_code;
                device_code_evento = device_code;
                authorized_evento = is_authorized;
                codice_associato = true;
            }
            if (trigger_porta_idx == -1 && angolo_deviazione > (int32_t)config_02) {
                trigger_porta_idx = indice_inizio;
            }
            if (detect_idx == -1 && detect) {
                detect_idx = indice_inizio;
            }
            if (infrared_idx == -1 && infrared) {
                infrared_idx = indice_inizio;
            }
            if (trigger) {
                conteggio_senza_trigger = 0;
                ultimo_trigger_idx = indice_inizio;
            } else {
                conteggio_senza_trigger++;
            }

            if (newcode) {
                float durata_effettiva = indice_inizio * 0.1f;
                const char* tipo = passaggio_confermato ? (correctedAngle > 2048 ? "*Uscita" : "*Ingresso") : "*Affaccio";
                float trigger_porta_t = (trigger_porta_idx >= 0) ? trigger_porta_idx * 0.1f : -1.0f;
                float passaggio_porta_t = (passaggio_porta_idx >= 0) ? passaggio_porta_idx * 0.1f : -1.0f;
                float detect_t = (detect_idx >= 0) ? detect_idx * 0.1f : -1.0f;
                float infrared_t = (infrared_idx >= 0) ? infrared_idx * 0.1f : -1.0f;
                add_log_entry(timestamp_inizio, tipo, cat_name_evento, country_code_evento, device_code_evento, authorized_evento);
                Serial.printf("Evento chiuso forzatamente per newcode a %s, tipo: %s, Nome: %s, Durata: %.2f s, TriggerPorta: %.1f s, PassaggioPorta: %.1f s, Detect: %.1f s, Infrared: %.1f s\n",
                              timestamp_full, tipo, cat_name_evento.c_str(), durata_effettiva, trigger_porta_t, passaggio_porta_t, detect_t, infrared_t);
                stato = IDLE;
                codice_associato = false;
                if (trigger) {
                    stato = EVENTO_ATTIVO;
                    strncpy(timestamp_inizio, timestamp_full, sizeof(timestamp_inizio));
                    indice_inizio = 0;
                    passaggio_confermato = false;
                    direzione = NULL;
                    conteggio_senza_trigger = 0;
                    ultimo_trigger_idx = 0;
                    trigger_porta_idx = (angolo_deviazione > (int32_t)config_02) ? 0 : -1;
                    passaggio_porta_idx = -1;
                    detect_idx = detect ? 0 : -1;
                    infrared_idx = infrared ? 0 : -1;
                    codice_associato = detect;
                    if (detect) {
                        cat_name_evento = cat_name;
                        country_code_evento = country_code;
                        device_code_evento = device_code;
                        authorized_evento = is_authorized;
                    } else {
                        cat_name_evento = "Sconosciuto";
                        country_code_evento = 0;
                        device_code_evento = 0;
                        authorized_evento = false;
                    }
                    Serial.printf("Nuovo evento iniziato per newcode a %s\n", timestamp_inizio);
                }
            } else if (conteggio_senza_trigger >= config_04) {
                float durata_effettiva = (ultimo_trigger_idx + 1) * 0.1f;
                const char* tipo = passaggio_confermato ? (correctedAngle > 2048 ? "*Uscita" : "*Ingresso") : "*Affaccio";
                float trigger_porta_t = (trigger_porta_idx >= 0) ? trigger_porta_idx * 0.1f : -1.0f;
                float passaggio_porta_t = (passaggio_porta_idx >= 0) ? passaggio_porta_idx * 0.1f : -1.0f;
                float detect_t = (detect_idx >= 0) ? detect_idx * 0.1f : -1.0f;
                float infrared_t = (infrared_idx >= 0) ? infrared_idx * 0.1f : -1.0f;
                add_log_entry(timestamp_inizio, tipo, cat_name_evento, country_code_evento, device_code_evento, authorized_evento);
                Serial.printf("Evento terminato, tipo: %s, Nome: %s, Durata: %.2f s, TriggerPorta: %.1f s, PassaggioPorta: %.1f s, Detect: %.1f s, Infrared: %.1f s\n",
                              tipo, cat_name_evento.c_str(), durata_effettiva, trigger_porta_t, passaggio_porta_t, detect_t, infrared_t);
                stato = IDLE;
                passaggio_confermato = false;
                direzione = NULL;
                conteggio_senza_trigger = 0;
                ultimo_trigger_idx = 0;
                trigger_porta_idx = -1;
                passaggio_porta_idx = -1;
                detect_idx = -1;
                infrared_idx = -1;
                codice_associato = false;
            }
        }

        encoder_buffer[encoder_buffer_index].rawAngle = correctedAngle;
        encoder_buffer[encoder_buffer_index].infrared = infrared;
        encoder_buffer[encoder_buffer_index].detect = detect;
        encoder_buffer[encoder_buffer_index].door_open = door_open;
        encoder_buffer[encoder_buffer_index].newcode = newcode; // Imposta il bit newcode
        encoder_buffer_index = (encoder_buffer_index + 1) % ENCODER_BUFFER_SIZE;
        lastRawAngle = rawAngle;
        lastMagnitude = magnitude;
        last_encoder_timestamp = local_time + (millis() - last_millis) / 1000;

        vTaskDelayUntil(&last_wake_time, interval);
    }
}