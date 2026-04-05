// ================================================================
//  globals.cpp — Definición de variables globales compartidas
// ================================================================
#include "globals.h"
#include "config.h"

// ── Modo de operación ─────────────────────────────────────────
volatile bool modoTremouse  = false;
bool          modoAutonomo  = false;
bool          isConnectedToWeb = false;
bool          sistemaIniciado  = false;

// ── Control de motores ────────────────────────────────────────
int motorBias     = 0;
int velocidadBase = 180;

// ── Odometría ─────────────────────────────────────────────────
float posX = 0, posY = 0, anguloRad = 0;

// ── Tópicos MQTT ─────────────────────────────────────────────
String topicCmd    = String(ROBOT_NAME) + "/cmd";
String topicData   = String(ROBOT_NAME) + "/data";
String topicStatus = String(ROBOT_NAME) + "/status";

// ── Colas FreeRTOS ───────────────────────────────────────────
QueueHandle_t queueCmds;
QueueHandle_t queueSensores;

// ── Handles de tareas ─────────────────────────────────────────
TaskHandle_t hTremouse = NULL;
