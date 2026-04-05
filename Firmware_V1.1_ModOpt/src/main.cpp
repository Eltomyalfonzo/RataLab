// ================================================================
//  main.cpp — Robot Laberinto v6
//  Programa principal: inicializa módulos y lanza tareas FreeRTOS
// ================================================================
#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "Motors.h"
#include "Encoders.h"
#include "MQTTComm.h"
#include "Navegacion.h"

// Handles locales (Ultra y Nav no necesitan ser globales)
static TaskHandle_t hComandos = NULL;
static TaskHandle_t hMQTT     = NULL;
static TaskHandle_t hUltra    = NULL;
static TaskHandle_t hNav      = NULL;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ROBOT LABERINTO v6 ===");

    // ── Colas FreeRTOS ────────────────────────────────────────
    queueCmds     = xQueueCreate(20, CMD_LEN);
    queueSensores = xQueueCreate(1, sizeof(DatosSensores));

    // ── Hardware ──────────────────────────────────────────────
    motorsInit();
    encodersInit();

    pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
    pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
    pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
    pinMode(IR_SENSOR, INPUT);

    // ── Red y MQTT ────────────────────────────────────────────
    mqttCommInit();

    // ── Tareas FreeRTOS ───────────────────────────────────────
    //                          nombre      stack   arg  prio  handle  core
    xTaskCreatePinnedToCore(tareaComandos, "Cmds",  4096, NULL, 3, &hComandos, 0);
    xTaskCreatePinnedToCore(tareaMQTT,     "MQTT",  4096, NULL, 2, &hMQTT,     1);
    xTaskCreatePinnedToCore(tareaUltra,    "Ultra", 3072, NULL, 1, &hUltra,    1);
    xTaskCreatePinnedToCore(tareaNav,      "Nav",   4096, NULL, 1, &hNav,      1);

    delay(500);
    sistemaIniciado = true;
    Serial.println("🚀 LISTO");
}

void loop() {
    // Todo corre en tareas FreeRTOS; el loop queda suspendido.
    vTaskDelay(portMAX_DELAY);
}
