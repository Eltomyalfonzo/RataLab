// ================================================================
//  Encoders.cpp
// ================================================================
#include "Encoders.h"
#include "config.h"

static volatile long pulsosIzq = 0;
static volatile long pulsosDer = 0;
static portMUX_TYPE  muxEncoder = portMUX_INITIALIZER_UNLOCKED;

volatile long stepBaseIzq = 0;
volatile long stepBaseDer = 0;
bool          stepTracking = false;

static void IRAM_ATTR isrIzq() {
    portENTER_CRITICAL_ISR(&muxEncoder);
    pulsosIzq++;
    portEXIT_CRITICAL_ISR(&muxEncoder);
}

static void IRAM_ATTR isrDer() {
    portENTER_CRITICAL_ISR(&muxEncoder);
    pulsosDer++;
    portEXIT_CRITICAL_ISR(&muxEncoder);
}

void encodersInit() {
    pinMode(ENCODER_L, INPUT_PULLUP);
    pinMode(ENCODER_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_L), isrIzq, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R), isrDer, RISING);
}

void resetPulsos() {
    portENTER_CRITICAL(&muxEncoder);
    pulsosIzq = 0;
    pulsosDer = 0;
    portEXIT_CRITICAL(&muxEncoder);
}

long getPulsosIzq() {
    portENTER_CRITICAL(&muxEncoder);
    long v = pulsosIzq;
    portEXIT_CRITICAL(&muxEncoder);
    return v;
}

long getPulsosDer() {
    portENTER_CRITICAL(&muxEncoder);
    long v = pulsosDer;
    portEXIT_CRITICAL(&muxEncoder);
    return v;
}
