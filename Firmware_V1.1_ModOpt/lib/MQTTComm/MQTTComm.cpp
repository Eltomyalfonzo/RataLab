// ================================================================
//  MQTTComm.cpp
// ================================================================
#include "MQTTComm.h"
#include "config.h"
#include "globals.h"
#include "Motors.h"
#include "Encoders.h"
#include "Tremouse.h"

// ── Clientes ──────────────────────────────────────────────────
WiFiClientSecure espClient;
PubSubClient     mqttClient(espClient);

// =============================================================
//  CALLBACK MQTT — solo encola, no ejecuta
// =============================================================
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (length == 0 || length >= CMD_LEN) return;

    char buf[CMD_LEN];
    memcpy(buf, payload, length);
    buf[length] = '\0';

    String msg = String(buf);
    Serial.printf("📨 Recibido [%s]: %s\n", topic, msg.c_str());

    if (msg == "CONNECT") {
        isConnectedToWeb = true;
        mqttClient.publish(topicData.c_str(), "CONNECTED");
        return;
    }

    xQueueSend(queueCmds, buf, 0);
}

// =============================================================
//  INIT
// =============================================================
void mqttCommInit() {
    Serial.print("WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500); Serial.print(".");
    }
    Serial.println(WiFi.status() == WL_CONNECTED
        ? " OK " + WiFi.localIP().toString() : " FALLO");

    espClient.setInsecure();
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);

    String id = "ESP32_" + String(random(0xffff), HEX);
    if (mqttClient.connect(id.c_str(), MQTT_USER, MQTT_PASS)) {
        mqttClient.subscribe(topicCmd.c_str());
        mqttClient.publish(topicData.c_str(), "ONLINE");
        Serial.println("MQTT OK");
    } else {
        Serial.printf("MQTT FALLO rc=%d\n", mqttClient.state());
    }
}

// =============================================================
//  TAREA PROCESADORA DE COMANDOS
// =============================================================
void tareaComandos(void*) {
    char buf[CMD_LEN];
    while (true) {
        if (xQueueReceive(queueCmds, buf, portMAX_DELAY) != pdTRUE) continue;

        String msg = String(buf);
        msg.trim(); msg.toUpperCase();
        if (msg.length() == 0) continue;

        Serial.printf("🔧 CMD: [%s]\n", msg.c_str());

        // ── Parar todo ───────────────────────────────────────
        if (msg == "S") {
            modoAutonomo = false;
            modoTremouse = false;
            motorStop();
            mqttClient.publish(topicData.c_str(), "STOP");
            continue;
        }

        // ── Iniciar Tremouse ─────────────────────────────────
        if (msg == "TM") {
            modoAutonomo = false;
            modoTremouse = true;
            tremouseInit();
            if (hTremouse == NULL)
                xTaskCreatePinnedToCore(tareaTremouse, "TM", 8192, NULL, 2, &hTremouse, 1);
            mqttClient.publish(topicData.c_str(), "TREMOUSE_START");
            continue;
        }

        // ── Modo autónomo ────────────────────────────────────
        if (msg == "MV") {
            modoTremouse = false;
            modoAutonomo = true;
            continue;
        }

        // ── Reset odometría ──────────────────────────────────
        if (msg == "Z") {
            posX = posY = anguloRad = 0;
            resetPulsos();
            tremouseInit();
            mqttClient.publish(topicData.c_str(), "RESET");
            continue;
        }

        // ── Activar tracking de pasos ────────────────────────
        if (msg == "Z_STEPS") {
            stepBaseIzq = getPulsosIzq();
            stepBaseDer = getPulsosDer();
            stepTracking = true;
            continue;
        }

        // ── Bias de motores ──────────────────────────────────
        if (msg.startsWith("BIAS:")) {
            motorBias = constrain(msg.substring(5).toInt(), -50, 50);
            mqttClient.publish(topicData.c_str(), ("BIAS_OK:" + String(motorBias)).c_str());
            continue;
        }

        // ── Test de movimientos Tremouse ─────────────────────
        if (msg == "TM_TEST") {
            mqttClient.publish(topicData.c_str(), "TM_TEST_START");
            tmAvanzar(); tmGiroIzq(); tmGiroDer();
            mqttClient.publish(topicData.c_str(), "TM_TEST_DONE");
            continue;
        }

        // ── Info de configuración Tremouse ───────────────────
        if (msg == "TM_INFO") {
            String info = "TM_CFG:vel="      + String(tmVelAvance)
                        + ",gvel="           + String(tmVelGiro)
                        + ",pavance="        + String(tmPulsosAvance)
                        + ",pgiro="          + String(tmPulsosGiro)
                        + ",pared="          + String(tmDistPared, 1)
                        + ",pausa="          + String(tmPausaMs)
                        + ",muestras="       + String(tmMuestras)
                        + ",toutavance="     + String(tmTimeoutAvance)
                        + ",toutgiro="       + String(tmTimeoutGiro);
            mqttClient.publish(topicData.c_str(), info.c_str());
            continue;
        }

        // ── Ajuste de parámetros Tremouse ────────────────────
        #define TM_PARAM(prefix, len, var, lo, hi, label) \
            if (msg.startsWith(prefix)) { \
                var = constrain(msg.substring(len).toInt(), lo, hi); \
                mqttClient.publish(topicData.c_str(), (String(label) + String(var)).c_str()); \
                continue; \
            }

        TM_PARAM("TM_VEL:",        7,  tmVelAvance,    60,  255, "TM_VEL_OK:")
        TM_PARAM("TM_GVEL:",       8,  tmVelGiro,      60,  255, "TM_GVEL_OK:")
        TM_PARAM("TM_PAVANCE:",   11,  tmPulsosAvance,  5,  200, "TM_PAVANCE_OK:")
        TM_PARAM("TM_PGIRO:",      9,  tmPulsosGiro,    2,  100, "TM_PGIRO_OK:")
        TM_PARAM("TM_PAUSA:",      9,  tmPausaMs,      50, 2000, "TM_PAUSA_OK:")
        TM_PARAM("TM_MUESTRAS:",  12,  tmMuestras,      1,   20, "TM_MUESTRAS_OK:")
        TM_PARAM("TM_TOUTAVANCE:",14,  tmTimeoutAvance,500,10000,"TM_TOUTAVANCE_OK:")
        TM_PARAM("TM_TOUTGIRO:", 12,  tmTimeoutGiro,  500, 5000, "TM_TOUTGIRO_OK:")

        // TM_PARED usa float — no entra en la macro
        if (msg.startsWith("TM_PARED:")) {
            tmDistPared = constrain(msg.substring(9).toFloat(), 5.0f, 60.0f);
            mqttClient.publish(topicData.c_str(), ("TM_PARED_OK:" + String(tmDistPared, 1)).c_str());
            continue;
        }

        // ── Control manual (bloqueado si hay modo activo) ────
        if (modoAutonomo || modoTremouse) continue;

        char cmd = msg.charAt(0);

        if (cmd == 'V' && msg.length() > 1) {
            velocidadBase = constrain(msg.substring(1).toInt(), 0, 255);
            continue;
        }

        if (cmd == 'F') { mvAdelante(velocidadBase);  mqttClient.publish(topicData.c_str(), "ADELANTE"); }
        if (cmd == 'B') { mvAtras(velocidadBase);     mqttClient.publish(topicData.c_str(), "ATRAS");    }
        if (cmd == 'L') { mvIzquierda(velocidadBase); mqttClient.publish(topicData.c_str(), "IZQ");      }
        if (cmd == 'R') { mvDerecha(velocidadBase);   mqttClient.publish(topicData.c_str(), "DER");      }
    }
}

// =============================================================
//  TAREA MQTT — mantiene conexión y publica estado periódico
// =============================================================
void tareaMQTT(void*) {
    TickType_t t = xTaskGetTickCount();
    unsigned long lastRecon  = 0;
    unsigned long lastStatus = 0;
    unsigned long lastSteps  = 0;

    while (true) {
        vTaskDelayUntil(&t, pdMS_TO_TICKS(50));

        if (!mqttClient.connected()) {
            isConnectedToWeb = false;
            unsigned long now = millis();
            if (now - lastRecon > 5000) {
                lastRecon = now;
                String id = "ESP32_" + String(random(0xffff), HEX);
                if (mqttClient.connect(id.c_str(), MQTT_USER, MQTT_PASS)) {
                    mqttClient.subscribe(topicCmd.c_str());
                    mqttClient.publish(topicData.c_str(), "ONLINE");
                    Serial.println("✅ MQTT reconectado");
                } else {
                    Serial.printf("❌ MQTT rc=%d\n", mqttClient.state());
                }
            }
        } else {
            mqttClient.loop();

            unsigned long now = millis();

            if (now - lastStatus >= 500) {
                lastStatus = now;
                String st = "X:"    + String(posX, 2)
                          + "|Y:"   + String(posY, 2)
                          + "|ANG:" + String(anguloRad * 180.0f / PI, 1)
                          + "|MODO:"+ String(modoTremouse ? "TREMOUSE" : (modoAutonomo ? "AUTO" : "MANUAL"))
                          + "|BIAS:"+ String(motorBias)
                          + "|TM_COL:0|TM_ROW:0|TM_HDG:0";
                mqttClient.publish(topicStatus.c_str(), st.c_str());
            }

            if (stepTracking && now - lastSteps >= 100) {
                lastSteps = now;
                long si = getPulsosIzq() - stepBaseIzq;
                long sd = getPulsosDer() - stepBaseDer;
                mqttClient.publish(topicData.c_str(),
                    ("STEPS:" + String(si) + "," + String(sd)).c_str());
            }
        }
    }
}
