// ================================================================
//  Tremouse.cpp
// ================================================================
#include "Tremouse.h"
#include "config.h"
#include "globals.h"
#include "Motors.h"
#include "Encoders.h"
#include "Ultrasonido.h"
#include <PubSubClient.h>

// ── Parámetros (modificables por MQTT) ───────────────────────
int   tmVelAvance     = 200;
int   tmVelGiro       = 190;
int   tmPulsosAvance  =  45;
int   tmPulsosGiro    =  11;
float tmDistPared     = 18.0f;
int   tmPausaMs       = 300;
int   tmMuestras      =  20;
int   tmTimeoutAvance = 3000;
int   tmTimeoutGiro   = 2000;

// ── Mapa (pila de retroceso) ──────────────────────────────────
struct TmHist { int col, row, heading; };
static TmHist tmStack[512];
static int    tmStackTop = 0;

// Tabla de direcciones cardinales
static const int TM_DX[] = { 0, 1, 0, -1 };
static const int TM_DY[] = {-1, 0, 1,  0 };

// Cliente MQTT — referenciado desde el módulo MQTTComm
extern PubSubClient mqttClient;

// =============================================================
//  PRIVADO — esperar pulsos de encoder
// =============================================================
static long tmEsperarPulsos(int objetivo, int timeoutMs) {
    long baseIzq = getPulsosIzq();
    long baseDer = getPulsosDer();
    unsigned long t0 = millis();

    while (true) {
        long si = getPulsosIzq() - baseIzq;
        long sd = getPulsosDer() - baseDer;
        long promedio = (si + sd) / 2;
        if (promedio >= objetivo) return promedio;
        if ((millis() - t0) > (unsigned long)timeoutMs) {
            Serial.printf("  ⚠️ Timeout encoder (obj:%d alcanzado:%ld)\n", objetivo, promedio);
            return promedio;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =============================================================
//  PRIVADO — leer los tres sensores con verificación frontal
// =============================================================
struct TmSens { float izq, frt, der; };

static TmSens tmLeer() {
    TmSens s;
    s.izq = tmSensor(TRIG_L, ECHO_L);
    s.frt = tmSensor(TRIG_C, ECHO_C);
    s.der = tmSensor(TRIG_R, ECHO_R);

    // Verificación extra del frente (fix v6)
    if (s.frt >= tmDistPared) {
        if (tmHayParedFrente(TRIG_C, ECHO_C)) {
            s.frt = tmDistPared - 1.0f;
            Serial.println("  ⚠️ Pared frontal confirmada por mayoría");
        }
    }

    Serial.printf("  Sens IZQ:%.1f FRT:%.1f DER:%.1f (umbral:%.1f)\n",
                  s.izq, s.frt, s.der, tmDistPared);
    return s;
}

// =============================================================
//  PRIVADO — publicar celda en MQTT
// =============================================================
static void tmPublicar(int col, int row,
                        bool wN, bool wE, bool wS, bool wW) {
    if (!mqttClient.connected()) return;
    char buf[64];
    snprintf(buf, sizeof(buf), "CELL:%d,%d,%d,%d,%d,%d",
             col, row, wN?1:0, wE?1:0, wS?1:0, wW?1:0);
    mqttClient.publish(topicData.c_str(), buf);
    Serial.printf("  📡 %s\n", buf);
}

// =============================================================
//  PRIVADO — girar hacia heading destino
// =============================================================
static void tmGirarHacia(int &heading, int dest) {
    int diff = ((dest - heading) + 4) % 4;
    if      (diff == 1) { tmGiroDer(); }
    else if (diff == 3) { tmGiroIzq(); }
    else if (diff == 2) { tmGiroIzq(); tmGiroIzq(); }
    heading = dest;
}

// =============================================================
//  PÚBLICO — movimientos primitivos
// =============================================================
void tmAvanzar() {
    int vL = constrain(tmVelAvance + motorBias, 60, 255);
    int vR = constrain(tmVelAvance - motorBias, 60, 255);
    digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
    ledcWrite(CH_L, vL); ledcWrite(CH_R, vR);
    long pulsos = tmEsperarPulsos(tmPulsosAvance, tmTimeoutAvance);
    motorStop();
    Serial.printf("  Avance: %ld pulsos (obj:%d)\n", pulsos, tmPulsosAvance);
    vTaskDelay(pdMS_TO_TICKS(tmPausaMs));
}

void tmGiroIzq() {
    int v = constrain(tmVelGiro, 60, 255);
    digitalWrite(MOTOR_L_IN1, LOW);  digitalWrite(MOTOR_L_IN2, HIGH);
    digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
    ledcWrite(CH_L, v); ledcWrite(CH_R, v);
    long pulsos = tmEsperarPulsos(tmPulsosGiro, tmTimeoutGiro);
    motorStop();
    Serial.printf("  Giro IZQ: %ld pulsos (obj:%d)\n", pulsos, tmPulsosGiro);
    vTaskDelay(pdMS_TO_TICKS(tmPausaMs));
}

void tmGiroDer() {
    int v = constrain(tmVelGiro, 60, 255);
    digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW);  digitalWrite(MOTOR_R_IN2, HIGH);
    ledcWrite(CH_L, v); ledcWrite(CH_R, v);
    long pulsos = tmEsperarPulsos(tmPulsosGiro, tmTimeoutGiro);
    motorStop();
    Serial.printf("  Giro DER: %ld pulsos (obj:%d)\n", pulsos, tmPulsosGiro);
    vTaskDelay(pdMS_TO_TICKS(tmPausaMs));
}

// =============================================================
//  PÚBLICO — init
// =============================================================
void tremouseInit() {
    tmStackTop = 0;
}

// =============================================================
//  TAREA FreeRTOS
// =============================================================
void tareaTremouse(void*) {
    Serial.println("🐭 Tremouse iniciado");
    int col = 0, row = 0, heading = 0;
    tmStackTop = 0;
    vTaskDelay(pdMS_TO_TICKS(300));

    while (modoTremouse) {
        TmSens s = tmLeer();

        int hIzq   = (heading + 3) % 4;
        int hFrt   =  heading;
        int hDer   = (heading + 1) % 4;
        int hAtras = (heading + 2) % 4;

        bool pIzq   = s.izq < tmDistPared;
        bool pFrt   = s.frt < tmDistPared;
        bool pDer   = s.der < tmDistPared;
        bool pAtras = (tmStackTop == 0);

        bool walls[4] = {true, true, true, true};
        walls[hIzq]   = pIzq;
        walls[hFrt]   = pFrt;
        walls[hDer]   = pDer;
        walls[hAtras] = pAtras;
        tmPublicar(col, row, walls[0], walls[1], walls[2], walls[3]);

        // Elegir dirección libre con mayor distancia
        struct Cand { int h; float d; };
        Cand cands[3] = { {hIzq, s.izq}, {hFrt, s.frt}, {hDer, s.der} };

        for (int i = 0; i < 2; i++)
            for (int j = i+1; j < 3; j++)
                if (cands[j].d > cands[i].d) {
                    Cand t = cands[i]; cands[i] = cands[j]; cands[j] = t;
                }

        int elegido = -1;
        for (int i = 0; i < 3; i++) {
            if (cands[i].d >= tmDistPared) { elegido = cands[i].h; break; }
        }

        if (elegido >= 0) {
            if (tmStackTop < 512) tmStack[tmStackTop++] = {col, row, heading};
            tmGirarHacia(heading, elegido);
            tmAvanzar();
            col += TM_DX[heading];
            row += TM_DY[heading];
            Serial.printf("  ✅ (%d,%d) hdg=%d\n", col, row, heading);
        } else {
            Serial.println("  🔴 Dead end — backtrack");
            mqttClient.publish(topicData.c_str(), "TM_DEAD_END");
            if (tmStackTop == 0) {
                mqttClient.publish(topicData.c_str(), "TM_COMPLETE");
                modoTremouse = false;
                break;
            }
            TmHist prev = tmStack[--tmStackTop];
            tmGirarHacia(heading, hAtras);
            tmAvanzar();
            col = prev.col; row = prev.row;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    motorStop();
    mqttClient.publish(topicData.c_str(), "STOP");
    Serial.println("🐭 Tremouse terminado");
    hTremouse = NULL;
    vTaskDelete(NULL);
}
