// ================================================================
//  Ultrasonido.cpp
// ================================================================
#include "Ultrasonido.h"
#include "config.h"
#include "globals.h"

// ── Valores externos desde Tremouse ──────────────────────────
// tmMuestras y tmDistPared se definen en Tremouse.cpp y se
// referencian aquí a través de globals.h (declarados extern).
extern int   tmMuestras;
extern float tmDistPared;

// =============================================================
float ultrasonido(int trig, int echo) {
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long d = pulseIn(echo, HIGH, 1500);
    if (d == 0) return 999.0f;
    float cm = d * 0.034f / 2.0f;
    if (cm < DIST_MIN_CM || cm > DIST_MAX_CM) return 999.0f;
    return cm;
}

// =============================================================
float ultrasonidoTM(int trig, int echo) {
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long d = pulseIn(echo, HIGH, 3000);   // ≈ 51 cm ida+vuelta
    if (d == 0) return 999.0f;
    float cm = d * 0.034f / 2.0f;
    if (cm < DIST_MIN_CM || cm > DIST_MAX_TM) return 999.0f;
    return cm;
}

// =============================================================
float tmSensor(int trig, int echo) {
    float s = 0; int n = 0;
    for (int i = 0; i < tmMuestras; i++) {
        float d = ultrasonidoTM(trig, echo);
        if (d < 999.0f) { s += d; n++; }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return n > 0 ? s / n : 999.0f;
}

// =============================================================
//  Verificación frontal por mayoría simple (fix v6)
// =============================================================
bool tmHayParedFrente(int trig, int echo) {
    int detecciones = 0;
    for (int i = 0; i < TM_VERIF_TOTAL; i++) {
        float d = ultrasonidoTM(trig, echo);
        if (d < tmDistPared) detecciones++;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    Serial.printf("  Verif frontal: %d/%d detecciones\n", detecciones, TM_VERIF_TOTAL);
    return detecciones >= TM_VERIF_MINIMAS;
}
