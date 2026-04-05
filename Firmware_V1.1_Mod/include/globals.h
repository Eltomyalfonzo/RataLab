#pragma once
// ================================================================
//  globals.h — Estado compartido entre módulos
// ================================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ── Modo de operación ─────────────────────────────────────────
extern volatile bool modoTremouse;
extern bool          modoAutonomo;
extern bool          isConnectedToWeb;
extern bool          sistemaIniciado;

// ── Control de motores ────────────────────────────────────────
extern int  motorBias;
extern int  velocidadBase;

// ── Odometría ─────────────────────────────────────────────────
extern float posX, posY, anguloRad;

// ── Tópicos MQTT ─────────────────────────────────────────────
extern String topicCmd;
extern String topicData;
extern String topicStatus;

// ── Colas FreeRTOS ───────────────────────────────────────────
extern QueueHandle_t queueCmds;
extern QueueHandle_t queueSensores;

// ── Handles de tareas ─────────────────────────────────────────
extern TaskHandle_t hTremouse;

// ── Struct sensores ───────────────────────────────────────────
struct DatosSensores { float centro, derecha, izquierda; };
