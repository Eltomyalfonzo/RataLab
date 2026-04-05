#pragma once
// ================================================================
//  Motors.h
// ================================================================
#include <Arduino.h>

void motorsInit();
void motorStop();
void mvAdelante(int vel);
void mvAtras(int vel);
void mvIzquierda(int vel);
void mvDerecha(int vel);

// Dirección actual de cada motor (1=adelante, -1=atras, 0=stop)
extern int dirIzq, dirDer;
