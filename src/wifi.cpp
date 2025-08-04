#include "common.h"
#include <ElegantOTA.h>
#include "buffer_analyzer.h"

void wifi_task(void *pvParameters) {
    bool server_started = false;
    unsigned long last_ntp_attempt = 0;
    unsigned long last_memory_update = 0;
    const unsigned long ntp_retry_interval = 300000; // 5 minuti
    const unsigned long memory_update_interval = 5000; // 5 secondi

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            portENTER_CRITICAL(&wifiMux);
            wifi_connected = false;
            digitalWrite(wifi_led, HIGH);
            portEXIT_CRITICAL(&wifiMux);

            if (WIFI_VERBOSE_LOG) {
                Serial.println("Wi-Fi: Tentativo di connessione...");
            }

            WiFi.begin(ssid, password);
            unsigned long start = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }

            if (WiFi.status() == WL_CONNECTED) {
                portENTER_CRITICAL(&wifiMux);
                wifi_connected = true;
                digitalWrite(wifi_led, LOW);
                portEXIT_CRITICAL(&wifiMux);

                Serial.printf("Wi-Fi: Connesso, IP: %s\n", WiFi.localIP().toString().c_str());

                configTime(0, 0, "pool.ntp.org", "time.google.com");
                setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
                tzset();

                if (!server_started) {
                    // Configurazione dei callback di ElegantOTA per versione 3.1.5
                    ElegantOTA.onStart([]() {
                        Serial.println("OTA Start");
                        Serial.println("Chiusura SPIFFS prima dell'aggiornamento");
                        SPIFFS.end(); // Chiudi SPIFFS prima di qualsiasi aggiornamento
                    });

                    ElegantOTA.onEnd([](bool success) {
                        if (success) {
                            Serial.println("OTA completato con successo");
                            Serial.println("Reinizializzazione SPIFFS");
                            if (SPIFFS.begin(true)) {
                                Serial.println("SPIFFS reinizializzato con successo");
                                // Verifica file caricati
                                File root = SPIFFS.open("/");
                                File file = root.openNextFile();
                                while (file) {
                                    Serial.printf("File SPIFFS: %s, Dimensione: %u\n", file.name(), file.size());
                                    file = root.openNextFile();
                                }
                            } else {
                                Serial.println("Errore: reinizializzazione SPIFFS fallita");
                            }
                        } else {
                            Serial.println("Errore OTA");
                        }
                    });

                    ElegantOTA.begin(&server);
                    Serial.println("Server WebSocket e OTA avviato. Visita http://<IP>/update per OTA.");

                    server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
                        request->send(SPIFFS, "/config.html", "text/html");
                    });
                    server.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request) {
                        request->send(SPIFFS, "/debug.html", "text/html");
                    });

                    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

                    ws.onEvent(onWebSocketEvent);
                    server.addHandler(&ws);
                    server.on("/config_data", HTTP_GET, [](AsyncWebServerRequest *request) {
                        DynamicJsonDocument doc(8192);
                        JsonArray cats = doc.createNestedArray("authorized_cats");
                        for (size_t i = 0; i < num_cats; i++) {
                            JsonObject cat = cats.createNestedObject();
                            cat["device_code"] = String((unsigned long long)authorized_cats[i].device_code);
                            cat["country_code"] = authorized_cats[i].country_code;
                            cat["name"] = authorized_cats[i].name;
                            cat["authorized"] = authorized_cats[i].authorized;
                        }
                        doc["DOOR_TIMEOUT"] = DOOR_TIMEOUT * portTICK_PERIOD_MS;
                        doc["WIFI_RECONNECT_DELAY"] = WIFI_RECONNECT_DELAY;
                        doc["UNAUTHORIZED_LOG_INTERVAL"] = UNAUTHORIZED_LOG_INTERVAL;
                        doc["STEPS_PER_MOVEMENT"] = STEPS_PER_MOVEMENT;
                        doc["STEP_INTERVAL_US"] = STEP_INTERVAL_US;
                        doc["WIFI_VERBOSE_LOG"] = WIFI_VERBOSE_LOG;
                        doc["contaporta"] = contaporta;
                        doc["motor_type"] = (motor_type == SERVO) ? "servo" : "step";
                        doc["servo_open_us"] = servo_open_us;
                        doc["servo_closed_us"] = servo_closed_us;
                        doc["servo_transition_ms"] = servo_transition_ms;
                        String json;
                        serializeJson(doc, json);
                        request->send(200, "application/json", json);
                    });
                    server.on("/config_data", HTTP_POST, [](AsyncWebServerRequest *request) {}, 
                        nullptr,
                        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
                            static String body;
                            if (index == 0) body = "";
                            body += String((char*)data, len);
                            if (index + len == total) {
                                DynamicJsonDocument doc(8192);
                                DeserializationError error = deserializeJson(doc, body);
                                if (error) {
                                    request->send(400, "application/json", "{\"success\":false,\"error\":\"JSON non valido\"}");
                                    return;
                                }
                                String action = doc["action"];
                                if (action == "add" || action == "update") {
                                    JsonArray cats = doc["cats"];
                                    for (JsonObject cat : cats) {
                                        String name = cat["name"];
                                        String device_code_str = cat["device_code"];
                                        uint16_t country_code = cat["country_code"];
                                        bool authorized = cat["authorized"];
                                        if (name.isEmpty() || device_code_str.isEmpty() || !cat.containsKey("country_code")) {
                                            request->send(400, "application/json", "{\"success\":false,\"error\":\"Dati gatto mancanti\"}");
                                            return;
                                        }
                                        uint64_t device_code;
                                        if (!sscanf(device_code_str.c_str(), "%llu", &device_code)) {
                                            request->send(400, "application/json", "{\"success\":false,\"error\":\"Codice dispositivo non valido\"}");
                                            return;
                                        }
                                        if (action == "add") {
                                            if (num_cats >= MAX_CATS) {
                                                request->send(400, "application/json", "{\"success\":false,\"error\":\"Limite gatti raggiunto\"}");
                                                return;
                                            }
                                            authorized_cats[num_cats] = {device_code, country_code, name, authorized};
                                            num_cats++;
                                        } else {
                                            String original_device_code = cat["original_device_code"];
                                            uint64_t orig_device_code;
                                            if (!sscanf(original_device_code.c_str(), "%llu", &orig_device_code)) {
                                                request->send(400, "application/json", "{\"success\":false,\"error\":\"Codice dispositivo originale non valido\"}");
                                                return;
                                            }
                                            for (size_t i = 0; i < num_cats; i++) {
                                                if (authorized_cats[i].device_code == orig_device_code) {
                                                    authorized_cats[i] = {device_code, country_code, name, authorized};
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    saveConfig();
                                    request->send(200, "application/json", "{\"success\":true}");
                                } else if (action == "delete") {
                                    JsonArray cats = doc["cats"];
                                    for (JsonObject cat : cats) {
                                        String device_code_str = cat["device_code"];
                                        uint64_t device_code;
                                        if (!sscanf(device_code_str.c_str(), "%llu", &device_code)) {
                                            request->send(400, "application/json", "{\"success\":false,\"error\":\"Codice dispositivo non valido\"}");
                                            return;
                                        }
                                        for (size_t i = 0; i < num_cats; i++) {
                                            if (authorized_cats[i].device_code == device_code) {
                                                for (size_t j = i; j < num_cats - 1; j++) {
                                                    authorized_cats[j] = authorized_cats[j + 1];
                                                }
                                                num_cats--;
                                                break;
                                            }
                                        }
                                    }
                                    saveConfig();
                                    request->send(200, "application/json", "{\"success\":true}");
                                } else if (action == "update_params" || action == "reset_contaporta" || action == "reset_defaults") {
                                    JsonObject params = doc["params"];
                                    if (action == "reset_defaults") {
                                        DOOR_TIMEOUT = 10000 / portTICK_PERIOD_MS;
                                        WIFI_RECONNECT_DELAY = 1000;
                                        UNAUTHORIZED_LOG_INTERVAL = 60000;
                                        STEPS_PER_MOVEMENT = 2500;
                                        STEP_INTERVAL_US = 500;
                                        WIFI_VERBOSE_LOG = false;
                                        contaporta = 0;
                                        motor_type = STEP;
                                        servo_open_us = 2000;
                                        servo_closed_us = 1000;
                                        servo_transition_ms = 500;
                                    } else {
                                        if (params.containsKey("DOOR_TIMEOUT")) DOOR_TIMEOUT = params["DOOR_TIMEOUT"].as<uint32_t>() / portTICK_PERIOD_MS;
                                        if (params.containsKey("WIFI_RECONNECT_DELAY")) WIFI_RECONNECT_DELAY = params["WIFI_RECONNECT_DELAY"].as<uint32_t>();
                                        if (params.containsKey("UNAUTHORIZED_LOG_INTERVAL")) UNAUTHORIZED_LOG_INTERVAL = params["UNAUTHORIZED_LOG_INTERVAL"].as<uint32_t>();
                                        if (params.containsKey("STEPS_PER_MOVEMENT")) STEPS_PER_MOVEMENT = params["STEPS_PER_MOVEMENT"].as<uint32_t>();
                                        if (params.containsKey("STEP_INTERVAL_US")) STEP_INTERVAL_US = params["STEP_INTERVAL_US"].as<uint32_t>();
                                        if (params.containsKey("WIFI_VERBOSE_LOG")) WIFI_VERBOSE_LOG = params["WIFI_VERBOSE_LOG"].as<bool>();
                                        if (params.containsKey("contaporta")) contaporta = params["contaporta"].as<uint32_t>();
                                        if (params.containsKey("motor_type")) {
                                            String mt = params["motor_type"].as<String>();
                                            motor_type = (mt == "servo") ? SERVO : STEP;
                                        }
                                        if (params.containsKey("servo_open_us")) servo_open_us = params["servo_open_us"].as<uint32_t>();
                                        if (params.containsKey("servo_closed_us")) servo_closed_us = params["servo_closed_us"].as<uint32_t>();
                                        if (params.containsKey("servo_transition_ms")) servo_transition_ms = params["servo_transition_ms"].as<uint32_t>();
                                        if (DOOR_TIMEOUT * portTICK_PERIOD_MS < 1000 || WIFI_RECONNECT_DELAY < 1000 || UNAUTHORIZED_LOG_INTERVAL < 1000) {
                                            request->send(400, "application/json", "{\"success\":false,\"error\":\"Tempi devono essere >= 1000 ms\"}");
                                            return;
                                        }
                                        if (STEPS_PER_MOVEMENT < 1) {
                                            request->send(400, "application/json", "{\"success\":false,\"error\":\"Passi per movimento >= 1\"}");
                                            return;
                                        }
                                        if (STEP_INTERVAL_US < 100) {
                                            request->send(400, "application/json", "{\"success\":false,\"error\":\"Intervallo passi >= 100 μs\"}");
                                            return;
                                        }
                                    }
                                    saveConfig();
                                    request->send(200, "application/json", "{\"success\":true}");
                                } else {
                                    request->send(400, "application/json", "{\"success\":false,\"error\":\"Azione non valida\"}");
                                }
                            }
                        });
                    server.on("/clear_log", HTTP_POST, [](AsyncWebServerRequest *request) {
                        log_buffer_index = 0;
                        for (size_t i = 0; i < LOG_BUFFER_SIZE; i++) {
                            log_buffer[i].timestamp[0] = '\0';
                        }
                        request->send(200, "application/json", "{\"success\":true}");
                    });
                    server.on("/reset_system", HTTP_POST, [](AsyncWebServerRequest *request) {
                        request->send(200, "application/json", "{\"success\":true}");
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                        ESP.restart();
                    });
                    // Endpoint di test per SPIFFS
                    server.on("/test_spiffs", HTTP_POST, [](AsyncWebServerRequest *request) {}, 
                        nullptr,
                        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
                            static File testFile;
                            if (index == 0) {
                                Serial.println("Inizio upload file di test su SPIFFS");
                                testFile = SPIFFS.open("/test.txt", "w");
                                if (!testFile) {
                                    Serial.println("Errore: impossibile aprire /test.txt per scrittura");
                                    request->send(500, "text/plain", "Errore apertura file");
                                    return;
                                }
                            }
                            if (testFile.write(data, len) != len) {
                                Serial.println("Errore scrittura su /test.txt");
                                testFile.close();
                                request->send(500, "text/plain", "Errore scrittura file");
                                return;
                            }
                            if (index + len == total) {
                                testFile.close();
                                Serial.println("Upload file di test completato");
                                // Verifica file
                                File verifyFile = SPIFFS.open("/test.txt", "r");
                                if (verifyFile) {
                                    Serial.printf("File /test.txt caricato, dimensione: %u bytes\n", verifyFile.size());
                                    verifyFile.close();
                                    request->send(200, "text/plain", "File caricato con successo");
                                } else {
                                    Serial.println("Errore: impossibile leggere /test.txt dopo l'upload");
                                    request->send(500, "text/plain", "Errore verifica file");
                                }
                            }
                        });
                    // Endpoint per elencare i file SPIFFS
                    server.on("/list_spiffs", HTTP_GET, [](AsyncWebServerRequest *request) {
                        String response = "File nello SPIFFS:\n";
                        File root = SPIFFS.open("/");
                        File file = root.openNextFile();
                        while (file) {
                            response += String(file.name()) + " (" + file.size() + " bytes)\n";
                            file = root.openNextFile();
                        }
                        request->send(200, "text/plain", response);
                    });
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
            // Invio periodico della memoria libera
            if (debug_stream_enabled && millis() - last_memory_update >= memory_update_interval) {
                ws.textAll("memory:" + String(ESP.getFreeHeap()));
                last_memory_update = millis();
            }

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
        client->text(json);
        Serial.printf("[%s] Log iniziale inviato al client ID %u\n", time_str, client->id());

        String mode_str;
        portENTER_CRITICAL(&doorModeMux);
        if (door_mode == ALWAYS_OPEN) mode_str = "ALWAYS_OPEN";
        else if (door_mode == ALWAYS_CLOSED) mode_str = "ALWAYS_CLOSED";
        else mode_str = "AUTO";
        portEXIT_CRITICAL(&doorModeMux);
        client->text("door_mode:" + mode_str);
        Serial.printf("[%s] Inviato stato modalità iniziale al client ID %u: %s\n", time_str, client->id(), mode_str.c_str());

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[%s] Client WebSocket disconnesso, ID: %u\n", time_str, client->id());
        debug_stream_enabled = false; // Disattiva debug
        Serial.printf("[%s] Debug streaming disattivato\n", time_str);
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            String message = (char*)data;
            Serial.printf("[%s] Ricevuto dal client ID %u: %s\n", time_str, client->id(), message.c_str());

            if (message == "start_debug") {
                debug_stream_enabled = true;
                Serial.printf("[%s] Debug streaming attivato per client ID %u\n", time_str, client->id());
                client->text("Debug streaming started");
            } else if (message == "stop_debug") {
                debug_stream_enabled = false;
                Serial.printf("[%s] Debug streaming disattivato per client ID %u\n", time_str, client->id());
                client->text("Debug streaming stopped");
            } else if (message == "get_buffer") {
                Serial.printf("[%s] Inizio acquisizione da WebSocket per client ID %u\n", time_str, client->id());
                uint32_t current_index = i_interrupt;
                uint32_t start_index = (current_index - 10000 + ADC_BUFFER_SIZE) % ADC_BUFFER_SIZE;
                if (start_index + 10000 <= ADC_BUFFER_SIZE) {
                    memcpy(temp_buffer, (const void*)&adc_buffer[start_index], 10000 * sizeof(uint16_t));
                } else {
                    uint32_t first_chunk_size = ADC_BUFFER_SIZE - start_index;
                    uint32_t second_chunk_size = 10000 - first_chunk_size;
                    memcpy(temp_buffer, (const void*)&adc_buffer[start_index], first_chunk_size * sizeof(uint16_t));
                    memcpy(&temp_buffer[first_chunk_size], (const void*)adc_buffer, second_chunk_size * sizeof(uint16_t));
                }
                client->binary((uint8_t*)temp_buffer, 10000 * sizeof(uint16_t));
                Serial.printf("[%s] Buffer inviato al client ID %u\n", time_str, client->id());
                analyze_buffer_32(temp_buffer, 10000);
            } else if (message == "get_encoder_buffer") {
                Serial.printf("[%s] Inizio acquisizione encoder_buffer per client ID %u\n", time_str, client->id());
                Serial.printf("[%s] Memoria libera prima di inviare encoder_buffer: %u bytes\n", time_str, ESP.getFreeHeap());
                uint32_t start_time = millis();
                uint32_t current_index = encoder_buffer_index;
                uint32_t start_index = (current_index - ENCODER_BUFFER_SIZE + ENCODER_BUFFER_SIZE) % ENCODER_BUFFER_SIZE;
                Serial.printf("[%s] Copia buffer: current_index=%u, start_index=%u\n", time_str, current_index, start_index);
                if (start_index + ENCODER_BUFFER_SIZE <= ENCODER_BUFFER_SIZE) {
                    memcpy(temp_buffer, &encoder_buffer[start_index], ENCODER_BUFFER_SIZE * sizeof(EncoderData));
                } else {
                    uint32_t first_chunk_size = ENCODER_BUFFER_SIZE - start_index;
                    uint32_t second_chunk_size = ENCODER_BUFFER_SIZE - first_chunk_size;
                    memcpy(temp_buffer, &encoder_buffer[start_index], first_chunk_size * sizeof(EncoderData));
                    memcpy(&temp_buffer[first_chunk_size], encoder_buffer, second_chunk_size * sizeof(EncoderData));
                }
                Serial.printf("[%s] Buffer copiato, invio binario (%u byte)\n", time_str, ENCODER_BUFFER_SIZE * sizeof(EncoderData));
                client->binary((uint8_t*)temp_buffer, ENCODER_BUFFER_SIZE * sizeof(EncoderData));
                Serial.printf("[%s] encoder_buffer inviato al client ID %u in %u ms\n", time_str, client->id(), millis() - start_time);
                DynamicJsonDocument doc(256);
                doc["encoder_timestamp"] = last_encoder_timestamp;
                doc["magnitude"] = lastMagnitude;
                String json;
                serializeJson(doc, json);
                Serial.printf("[%s] Invio JSON: %s\n", time_str, json.c_str());
                client->text(json);
                Serial.printf("[%s] Timestamp e magnitude inviati al client ID %u\n", time_str, client->id());
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
                    ws.textAll("door_mode:" + mode);
                    client->text("Mode updated: " + mode);
                } else {
                    client->text("Mode unchanged");
                }
            }
        }
    }
}