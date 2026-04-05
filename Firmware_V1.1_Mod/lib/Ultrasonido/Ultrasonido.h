#pragma once
// ================================================================
//  Ultrasonido.h
// ================================================================
#include <Arduino.h>

// Lectura simple — rango 1-20 cm (modo autónomo)
float ultrasonido(int trig, int echo);

// Lectura extendida — rango 1-50 cm (Tremouse)
float ultrasonidoTM(int trig, int echo);

// Promedio de N muestras (Tremouse)
float tmSensor(int trig, int echo);

// Verificación frontal por mayoría simple (Tremouse, fix v6)
bool tmHayParedFrente(int trig, int echo);
