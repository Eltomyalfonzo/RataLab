#pragma once
// ================================================================
//  MQTTComm.h — Conectividad WiFi + MQTT + procesado de comandos
// ================================================================
#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

extern WiFiClientSecure espClient;
extern PubSubClient     mqttClient;

void mqttCommInit();          // Conecta WiFi y MQTT
void tareaComandos(void*);    // Procesa cola de comandos
void tareaMQTT(void*);        // Mantiene conexión y publica estado
