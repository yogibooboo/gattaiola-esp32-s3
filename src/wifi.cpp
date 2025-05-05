#include "common.h"
#include <ElegantOTA.h>

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
                    ElegantOTA.begin(&server);
                    Serial.println("Server WebSocket e OTA avviato. Visita http://<IP>/update per OTA.");
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

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    char time_str[20];
    time_t now = local_time + (millis() - last_millis) / 1000;
    strftime(time_str, sizeof(time_str), "%H:%M:%S", localtime(&now));

    if (type == WS_EVT_CONNECT) {
        Serial.printf("[%s] Client WebSocket connesso, ID: %u\n", time_str, client->id());
        // Invio del log all'apertura, con ordine più recente in cima
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
        Serial.printf("[%s] Log iniziale inviato al client ID %u\n", time_str, client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[%s] Client WebSocket disconnesso, ID: %u\n", time_str, client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            String message = (char*)data;
            Serial.printf("[%s] Ricevuto dal client ID %u: %s\n", time_str, client->id(), message.c_str());

            if (message == "get_buffer") {
                Serial.printf("[%s] Inizio acquisizione da WebSocket per client ID %u\n", time_str, client->id());
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
                Serial.printf("[%s] Invio buffer al client ID %u\n", time_str, client->id());
                client->binary((uint8_t*)temp_buffer, 10000 * sizeof(uint16_t));
                Serial.printf("[%s] Buffer inviato al client ID %u\n", time_str, client->id());
            } else if (message == "get_door_mode") {
                String mode_str;
                portENTER_CRITICAL(&doorModeMux);
                if (door_mode == ALWAYS_OPEN) mode_str = "ALWAYS_OPEN";
                else if (door_mode == ALWAYS_CLOSED) mode_str = "ALWAYS_CLOSED";
                else mode_str = "AUTO";
                portEXIT_CRITICAL(&doorModeMux);
                client->text("door_mode:" + mode_str);
                Serial.printf("[%s] Inviato stato modalità al client ID %u: %s\n", time_str, client->id(), mode_str.c_str());
            } else if (message == "get_log") {
                Serial.printf("[%s] Invio log al client ID %u\n", time_str, client->id());
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
                Serial.printf("[%s] Log inviato al client ID %u\n", time_str, client->id());
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