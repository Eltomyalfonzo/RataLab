#pragma once
// ================================================================
//  Encoders.h
// ================================================================
#include <Arduino.h>

void encodersInit();
void resetPulsos();
long getPulsosIzq();
long getPulsosDer();

// Tracking de pasos para calibración
extern volatile long stepBaseIzq, stepBaseDer;
extern bool          stepTracking;
