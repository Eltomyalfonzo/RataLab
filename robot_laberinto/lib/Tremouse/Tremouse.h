#pragma once
// ================================================================
//  Tremouse.h — Algoritmo de exploración de laberinto
// ================================================================
#include <Arduino.h>

// ── Parámetros configurables ──────────────────────────────────
extern int   tmVelAvance;
extern int   tmVelGiro;
extern int   tmPulsosAvance;
extern int   tmPulsosGiro;
extern float tmDistPared;
extern int   tmPausaMs;
extern int   tmMuestras;
extern int   tmTimeoutAvance;
extern int   tmTimeoutGiro;

// ── API pública ───────────────────────────────────────────────
void    tremouseInit();          // Reinicia pila y estado
void    tareaTremouse(void*);    // Tarea FreeRTOS

// Movimientos primitivos (usados también por TM_TEST)
void    tmAvanzar();
void    tmGiroIzq();
void    tmGiroDer();
