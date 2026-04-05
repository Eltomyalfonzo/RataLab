#pragma once
// ================================================================
//  Navegacion.h — Tareas FreeRTOS de ultrasonidos y nav autónoma
// ================================================================
#include <Arduino.h>

void tareaUltra(void*);   // Lee los 3 sensores HC-SR04
void tareaNav(void*);     // Navegación autónoma reactiva
