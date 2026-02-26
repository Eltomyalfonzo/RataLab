// ================================================================
//  FIRMWARE ROBOT LABERINTO — v4
//  Fixes:
//  1. Robot NUNCA publica en /cmd (evita recibir sus propios mensajes)
//  2. Callback MQTT solo encola el comando, no lo ejecuta directamente
//  3. Sin PID en control manual ni Tremouse
//  4. Tremouse en tarea separada, no desde callback
//  5. Giro Tremouse por ENCODER (exacto) en vez de tiempo
//     Cálculo: distancia_entre_ruedas=7.5cm, radio_giro=3.75cm
//     arco_90° = (90/360)*π*7.5 = 5.89cm
//     cm_por_pulso = π*5/30 = 0.5236cm
//     pulsos_90° = 5.89/0.5236 ≈ 11 pulsos
//     pulsos_25cm = 25/0.5236 ≈ 48 pulsos
// ================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#define IR_DISABLED

const char* ssid     = "BALLARI PABLO";
const char* password = "23527257";

const char* mqtt_server   = "f9d90e4c488c4716ac1e5862b9c8a708.s1.eu.hivemq.cloud";
const int   mqtt_port     = 8883;
const char* mqtt_user     = "admin";
const char* mqtt_password = "SuperMan2233";

String robotName   = "robot01";
String topicCmd    = "";   // robot01/cmd    ← solo SUSCRIBE aquí
String topicData   = "";   // robot01/data   ← solo PUBLICA aquí
String topicStatus = "";   // robot01/status ← solo PUBLICA aquí

// ── Pines ─────────────────────────────────────────────────────
#define MOTOR_L_PWM 12
#define MOTOR_L_IN1 16
#define MOTOR_L_IN2 17
#define MOTOR_R_PWM 13
#define MOTOR_R_IN1 15
#define MOTOR_R_IN2  4

#define ENCODER_L 5
#define ENCODER_R 34

#define TRIG_C 32
#define ECHO_C 33
#define TRIG_R 23
#define ECHO_R 22
#define TRIG_L 25
#define ECHO_L 26
#define IR_SENSOR 35

// ── PWM ───────────────────────────────────────────────────────
#define PWM_FREQ       5000
#define PWM_RESOLUTION 8
#define CH_L 0
#define CH_R 1

// ── Odometría ─────────────────────────────────────────────────
const float PULSOS_POR_REV         = 30.0;
const float DIAMETRO_RUEDA         = 5.0;
const float CM_POR_PULSO           = PI * DIAMETRO_RUEDA / PULSOS_POR_REV;
const float DISTANCIA_ENTRE_RUEDAS = 15.0;

// ── Parámetros Tremouse ───────────────────────────────────────
// Velocidades (PWM) — se pueden ajustar desde la web
int   tmVelAvance    = 200;
int   tmVelGiro      = 190;

// ── Parámetros de movimiento por ENCODER (calculados geométricamente) ──
// Rueda: diámetro 5cm, 30 dientes → 1 pulso = π*5/30 = 0.5236 cm
// Avance 25cm → 25/0.5236 ≈ 48 pulsos
// Giro 90°  → arco = (90/360)*π*7.5cm = 5.89cm → 5.89/0.5236 ≈ 11 pulsos
// Estos valores son el punto de partida teórico; se pueden ajustar desde la web.
int   tmPulsosAvance = 48;   // pulsos encoder para avanzar 1 celda (25 cm)
int   tmPulsosGiro   = 11;   // pulsos encoder para girar exactamente 90°

// ── Parámetros adicionales Tremouse ──────────────────────────
float tmDistPared    = 18.0;
int   tmPausaMs      = 300;
int   tmMuestras     = 5;
// Timeout de seguridad: si el encoder no llega al objetivo en este tiempo, se para
int   tmTimeoutAvance = 3000;  // ms máximo para avanzar 1 celda
int   tmTimeoutGiro   = 2000;  // ms máximo para girar 90°

// ── Estado global ─────────────────────────────────────────────
volatile bool modoTremouse = false;
bool          modoAutonomo = false;
int           motorBias    = 0;
int           velocidadBase = 150;
bool          isConnectedToWeb = false;
bool          sistemaIniciado  = false;

// ── Encoders ──────────────────────────────────────────────────
volatile long pulsosIzq = 0;
volatile long pulsosDer = 0;
portMUX_TYPE  muxEncoder = portMUX_INITIALIZER_UNLOCKED;
volatile long stepBaseIzq = 0, stepBaseDer = 0;
bool          stepTracking = false;
int           dirIzq = 0, dirDer = 0;

// ── Odometría ─────────────────────────────────────────────────
float posX = 0, posY = 0, anguloRad = 0;

// ── Tremouse mapa ─────────────────────────────────────────────
const int TM_DX[] = { 0, 1, 0, -1 };
const int TM_DY[] = {-1, 0, 1,  0 };
struct TmHist { int col, row, heading; };
static TmHist tmStack[512];
static int    tmStackTop = 0;

// ── Modo autónomo ─────────────────────────────────────────────
const float DIST_PARED  = 15.0;
const float DIST_MINIMA =  8.0;
const float DIST_GIRO   = 20.0;
const int   VEL_AUTO    = 180;
const int   T_GIRO_AUTO = 800;

// ── MQTT ──────────────────────────────────────────────────────
WiFiClientSecure espClient;
PubSubClient     mqttClient(espClient);

// ── Cola de comandos (callback → tarea procesadora) ───────────
// El callback de MQTT NO ejecuta nada directamente.
// Solo mete el string en esta cola. La tarea de comandos lo procesa.
#define CMD_LEN 64
QueueHandle_t queueCmds;      // cola de strings de comando
QueueHandle_t queueSensores;  // datos ultrasonidos

struct DatosSensores { float centro, derecha, izquierda; };

// ── Handles de tareas ─────────────────────────────────────────
TaskHandle_t hTremouse   = NULL;
TaskHandle_t hComandos   = NULL;
TaskHandle_t hMQTT       = NULL;
TaskHandle_t hUltra      = NULL;
TaskHandle_t hNav        = NULL;

// =============================================================
//  ENCODERS
// =============================================================
void IRAM_ATTR isrIzq() { portENTER_CRITICAL_ISR(&muxEncoder); pulsosIzq++; portEXIT_CRITICAL_ISR(&muxEncoder); }
void IRAM_ATTR isrDer() { portENTER_CRITICAL_ISR(&muxEncoder); pulsosDer++; portEXIT_CRITICAL_ISR(&muxEncoder); }

long getPulsosIzq() { portENTER_CRITICAL(&muxEncoder); long v=pulsosIzq; portEXIT_CRITICAL(&muxEncoder); return v; }
long getPulsosDer() { portENTER_CRITICAL(&muxEncoder); long v=pulsosDer; portEXIT_CRITICAL(&muxEncoder); return v; }
void resetPulsos()  {
  portENTER_CRITICAL(&muxEncoder); pulsosIzq=0; pulsosDer=0; portEXIT_CRITICAL(&muxEncoder);
}

// =============================================================
//  MOTORES — control directo, igual que el código de diagnóstico
// =============================================================
void motorStop() {
  ledcWrite(CH_L, 0); ledcWrite(CH_R, 0);
  digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, LOW);
  dirIzq = 0; dirDer = 0;
  stepTracking = false;
}

void mvAdelante(int vel) {
  int vL = constrain(vel + motorBias, 60, 255);
  int vR = constrain(vel - motorBias, 60, 255);
  digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
  ledcWrite(CH_L, vL); ledcWrite(CH_R, vR);
  dirIzq = 1; dirDer = 1;
}

void mvAtras(int vel) {
  int vL = constrain(vel + motorBias, 60, 255);
  int vR = constrain(vel - motorBias, 60, 255);
  digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, HIGH);
  ledcWrite(CH_L, vL); ledcWrite(CH_R, vR);
  dirIzq = -1; dirDer = -1;
}

void mvIzquierda(int vel) {
  // Rueda izq ATRÁS, rueda der ADELANTE
  int vL = constrain(vel + motorBias, 60, 255);
  int vR = constrain(vel - motorBias, 60, 255);
  digitalWrite(MOTOR_L_IN1, LOW);  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
  ledcWrite(CH_L, vL); ledcWrite(CH_R, vR);
  dirIzq = -1; dirDer = 1;
}

void mvDerecha(int vel) {
  // Rueda izq ADELANTE, rueda der ATRÁS
  int vL = constrain(vel + motorBias, 60, 255);
  int vR = constrain(vel - motorBias, 60, 255);
  digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);  digitalWrite(MOTOR_R_IN2, HIGH);
  ledcWrite(CH_L, vL); ledcWrite(CH_R, vR);
  dirIzq = 1; dirDer = -1;
}

// =============================================================
//  ULTRASONIDO
// =============================================================
// Rango útil: 1 a 20 cm. Fuera de rango (0, muy lejos, muy cerca) → 999 (inválido)
#define DIST_MIN_CM  1.0f
#define DIST_MAX_CM 20.0f

float ultrasonido(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10); digitalWrite(trig, LOW);
  // Timeout = tiempo que tarda en ir y volver 20 cm = 20/0.034/2 ≈ 1176 µs → usamos 1500 µs con margen
  long d = pulseIn(echo, HIGH, 1500);
  if (d == 0) return 999.0f;                  // sin eco = fuera de rango (pared muy lejos o ausente)
  float cm = d * 0.034f / 2.0f;
  if (cm < DIST_MIN_CM || cm > DIST_MAX_CM) return 999.0f;  // fuera del rango útil
  return cm;
}

// =============================================================
//  TREMOUSE — movimientos (solo desde tarea Tremouse)
// =============================================================
float tmSensor(int trig, int echo) {
  float s = 0; int n = 0;
  for (int i = 0; i < tmMuestras; i++) {
    float d = ultrasonido(trig, echo);
    if (d < 999.0f) { s += d; n++; }   // solo acumula lecturas dentro de 1-20 cm
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  // Si ninguna lectura fue válida → 999 (sin pared detectada en rango útil)
  return n > 0 ? s / n : 999.0f;
}

struct TmSens { float izq, frt, der; };
TmSens tmLeer() {
  TmSens s;
  s.izq = tmSensor(TRIG_L, ECHO_L);
  s.frt = tmSensor(TRIG_C, ECHO_C);
  s.der = tmSensor(TRIG_R, ECHO_R);
  Serial.printf("  Sens IZQ:%.1f FRT:%.1f DER:%.1f (umbral:%.1f)\n",
                s.izq, s.frt, s.der, tmDistPared);
  return s;
}

// ── Helper: espera hasta acumular N pulsos en cualquier encoder (el que llegue primero) ──
// Corre motores ya iniciados y espera. Retorna pulsos reales alcanzados.
// Se llama DENTRO de una tarea FreeRTOS, nunca desde ISR.
long tmEsperarPulsos(int objetivo, int timeoutMs) {
  // Capturar base
  long baseIzq, baseDer;
  portENTER_CRITICAL(&muxEncoder);
  baseIzq = pulsosIzq;
  baseDer = pulsosDer;
  portEXIT_CRITICAL(&muxEncoder);

  unsigned long t0 = millis();
  while (true) {
    long si, sd;
    portENTER_CRITICAL(&muxEncoder);
    si = pulsosIzq - baseIzq;
    sd = pulsosDer - baseDer;
    portEXIT_CRITICAL(&muxEncoder);

    // Usar el promedio de ambos encoders para mayor precisión
    long promedio = (si + sd) / 2;
    if (promedio >= objetivo) return promedio;

    // Timeout de seguridad
    if ((millis() - t0) > (unsigned long)timeoutMs) {
      Serial.printf("  ⚠️ Timeout encoder (obj:%d alcanzado:%ld)\n", objetivo, promedio);
      return promedio;
    }
    vTaskDelay(pdMS_TO_TICKS(2));  // check cada 2ms
  }
}

void tmAvanzar() {
  int vL = constrain(tmVelAvance + motorBias, 60, 255);
  int vR = constrain(tmVelAvance - motorBias, 60, 255);
  // Activar motores
  digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
  ledcWrite(CH_L, vL); ledcWrite(CH_R, vR);
  dirIzq = 1; dirDer = 1;
  // Esperar hasta alcanzar el objetivo de pulsos
  long pulsos = tmEsperarPulsos(tmPulsosAvance, tmTimeoutAvance);
  motorStop();
  Serial.printf("  Avance: %ld pulsos (obj:%d)\n", pulsos, tmPulsosAvance);
  vTaskDelay(pdMS_TO_TICKS(tmPausaMs));
}

void tmGiroIzq() {
  // Rueda izq ATRÁS, rueda der ADELANTE
  int v = constrain(tmVelGiro, 60, 255);
  digitalWrite(MOTOR_L_IN1, LOW);  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
  ledcWrite(CH_L, v); ledcWrite(CH_R, v);
  dirIzq = -1; dirDer = 1;
  long pulsos = tmEsperarPulsos(tmPulsosGiro, tmTimeoutGiro);
  motorStop();
  Serial.printf("  Giro IZQ: %ld pulsos (obj:%d)\n", pulsos, tmPulsosGiro);
  vTaskDelay(pdMS_TO_TICKS(tmPausaMs));
}

void tmGiroDer() {
  // Rueda izq ADELANTE, rueda der ATRÁS
  int v = constrain(tmVelGiro, 60, 255);
  digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);  digitalWrite(MOTOR_R_IN2, HIGH);
  ledcWrite(CH_L, v); ledcWrite(CH_R, v);
  dirIzq = 1; dirDer = -1;
  long pulsos = tmEsperarPulsos(tmPulsosGiro, tmTimeoutGiro);
  motorStop();
  Serial.printf("  Giro DER: %ld pulsos (obj:%d)\n", pulsos, tmPulsosGiro);
  vTaskDelay(pdMS_TO_TICKS(tmPausaMs));
}

void tmGirarHacia(int &heading, int dest) {
  int diff = ((dest - heading) + 4) % 4;
  if      (diff == 1) { tmGiroDer(); }
  else if (diff == 3) { tmGiroIzq(); }
  else if (diff == 2) { tmGiroIzq(); tmGiroIzq(); }
  heading = dest;
}

void tmPublicar(int col, int row, bool wN, bool wE, bool wS, bool wW) {
  if (!mqttClient.connected()) return;
  char buf[64];
  snprintf(buf, sizeof(buf), "CELL:%d,%d,%d,%d,%d,%d",
           col, row, wN?1:0, wE?1:0, wS?1:0, wW?1:0);
  mqttClient.publish(topicData.c_str(), buf);
  Serial.printf("  📡 %s\n", buf);
}

// =============================================================
//  TAREA TREMOUSE
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

    // Elegir mejor dirección libre (más distancia)
    struct Cand { int h; float d; };
    Cand cands[3] = { {hIzq, s.izq}, {hFrt, s.frt}, {hDer, s.der} };
    for (int i=0; i<2; i++)
      for (int j=i+1; j<3; j++)
        if (cands[j].d > cands[i].d) { Cand t=cands[i]; cands[i]=cands[j]; cands[j]=t; }

    int elegido = -1;
    for (int i=0; i<3; i++) {
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

// =============================================================
//  TAREA PROCESADORA DE COMANDOS
//  Lee de la cola y ejecuta. Nunca bloquea el callback MQTT.
// =============================================================
void tareaComandos(void*) {
  char buf[CMD_LEN];
  while (true) {
    // Espera hasta que haya un comando en la cola
    if (xQueueReceive(queueCmds, buf, portMAX_DELAY) != pdTRUE) continue;

    String msg = String(buf);
    msg.trim();
    msg.toUpperCase();
    if (msg.length() == 0) continue;

    Serial.printf("🔧 CMD: [%s]\n", msg.c_str());

    // ── STOP — siempre primero ───────────────────────────────
    if (msg == "S") {
      modoAutonomo = false;
      modoTremouse = false;
      motorStop();
      mqttClient.publish(topicData.c_str(), "STOP");
      continue;
    }

    // ── TREMOUSE ─────────────────────────────────────────────
    if (msg == "TM") {
      modoAutonomo = false;
      modoTremouse = true;
      tmStackTop   = 0;
      if (hTremouse == NULL)
        xTaskCreatePinnedToCore(tareaTremouse, "TM", 8192, NULL, 2, &hTremouse, 1);
      mqttClient.publish(topicData.c_str(), "TREMOUSE_START");
      continue;
    }

    // ── MODO AUTÓNOMO ─────────────────────────────────────────
    if (msg == "MV") {
      modoTremouse = false;
      modoAutonomo = true;
      continue;
    }

    // ── RESET ─────────────────────────────────────────────────
    if (msg == "Z") {
      posX = posY = anguloRad = 0;
      resetPulsos();
      tmStackTop = 0;
      mqttClient.publish(topicData.c_str(), "RESET");
      continue;
    }

    if (msg == "Z_STEPS") {
      portENTER_CRITICAL(&muxEncoder);
      stepBaseIzq = pulsosIzq;
      stepBaseDer = pulsosDer;
      portEXIT_CRITICAL(&muxEncoder);
      stepTracking = true;
      continue;
    }

    // ── BIAS ──────────────────────────────────────────────────
    if (msg.startsWith("BIAS:")) {
      motorBias = constrain(msg.substring(5).toInt(), -50, 50);
      mqttClient.publish(topicData.c_str(), ("BIAS_OK:" + String(motorBias)).c_str());
      continue;
    }

    // ── TEST TREMOUSE ─────────────────────────────────────────
    if (msg == "TM_TEST") {
      mqttClient.publish(topicData.c_str(), "TM_TEST_START");
      // Ejecutar test en esta tarea (tiene stack y puede usar delay)
      tmAvanzar();
      tmGiroIzq();
      tmGiroDer();
      mqttClient.publish(topicData.c_str(), "TM_TEST_DONE");
      continue;
    }

    // ── CONFIG TREMOUSE ───────────────────────────────────────
    if (msg == "TM_INFO") {
      String info = "TM_CFG:vel="      + String(tmVelAvance)
                  + ",gvel="           + String(tmVelGiro)
                  + ",pavance="        + String(tmPulsosAvance)
                  + ",pgiro="          + String(tmPulsosGiro)
                  + ",pared="          + String(tmDistPared, 1)
                  + ",pausa="          + String(tmPausaMs)
                  + ",muestras="       + String(tmMuestras)
                  + ",toutavance="     + String(tmTimeoutAvance)
                  + ",toutgiro="       + String(tmTimeoutGiro);
      mqttClient.publish(topicData.c_str(), info.c_str());
      continue;
    }
    if (msg.startsWith("TM_VEL:"))      { tmVelAvance     = constrain(msg.substring(7).toInt(),   60,255);    mqttClient.publish(topicData.c_str(), ("TM_VEL_OK:"      +String(tmVelAvance)).c_str());     continue; }
    if (msg.startsWith("TM_GVEL:"))     { tmVelGiro       = constrain(msg.substring(8).toInt(),   60,255);    mqttClient.publish(topicData.c_str(), ("TM_GVEL_OK:"     +String(tmVelGiro)).c_str());       continue; }
    if (msg.startsWith("TM_PAVANCE:"))  { tmPulsosAvance  = constrain(msg.substring(11).toInt(),   5,200);    mqttClient.publish(topicData.c_str(), ("TM_PAVANCE_OK:"  +String(tmPulsosAvance)).c_str());  continue; }
    if (msg.startsWith("TM_PGIRO:"))    { tmPulsosGiro    = constrain(msg.substring(9).toInt(),    2,100);    mqttClient.publish(topicData.c_str(), ("TM_PGIRO_OK:"    +String(tmPulsosGiro)).c_str());    continue; }
    if (msg.startsWith("TM_PARED:"))    { tmDistPared     = constrain(msg.substring(9).toFloat(),5.0f,60.0f); mqttClient.publish(topicData.c_str(), ("TM_PARED_OK:"    +String(tmDistPared,1)).c_str());   continue; }
    if (msg.startsWith("TM_PAUSA:"))    { tmPausaMs       = constrain(msg.substring(9).toInt(),   50,2000);   mqttClient.publish(topicData.c_str(), ("TM_PAUSA_OK:"    +String(tmPausaMs)).c_str());        continue; }
    if (msg.startsWith("TM_MUESTRAS:")){ tmMuestras       = constrain(msg.substring(12).toInt(),   1,20);     mqttClient.publish(topicData.c_str(), ("TM_MUESTRAS_OK:" +String(tmMuestras)).c_str());     continue; }
    if (msg.startsWith("TM_TOUTAVANCE:")){ tmTimeoutAvance= constrain(msg.substring(14).toInt(), 500,10000);  mqttClient.publish(topicData.c_str(), ("TM_TOUTAVANCE_OK:"+String(tmTimeoutAvance)).c_str());continue; }
    if (msg.startsWith("TM_TOUTGIRO:")){ tmTimeoutGiro    = constrain(msg.substring(12).toInt(), 500,5000);   mqttClient.publish(topicData.c_str(), ("TM_TOUTGIRO_OK:" +String(tmTimeoutGiro)).c_str());   continue; }

    // ── CONTROL MANUAL (bloqueado si hay modo activo) ─────────
    if (modoAutonomo || modoTremouse) continue;

    char cmd = msg.charAt(0);

    if (cmd == 'V' && msg.length() > 1) {
      velocidadBase = constrain(msg.substring(1).toInt(), 0, 255);
      continue;
    }

    if (cmd == 'F') { mvAdelante(velocidadBase);  mqttClient.publish(topicData.c_str(), "ADELANTE"); }
    if (cmd == 'B') { mvAtras(velocidadBase);      mqttClient.publish(topicData.c_str(), "ATRAS");    }
    if (cmd == 'L') { mvIzquierda(velocidadBase);  mqttClient.publish(topicData.c_str(), "IZQ");      }
    if (cmd == 'R') { mvDerecha(velocidadBase);    mqttClient.publish(topicData.c_str(), "DER");      }
  }
}

// =============================================================
//  CALLBACK MQTT — SOLO ENCOLA, NO EJECUTA NADA
// =============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (length == 0 || length >= CMD_LEN) return;

  char buf[CMD_LEN];
  memcpy(buf, payload, length);
  buf[length] = '\0';

  String msg = String(buf);
  Serial.printf("📨 Recibido [%s]: %s\n", topic, msg.c_str());

  // CONNECT: confirmar conexión → publicar en /data, NO en /cmd
  if (msg == "CONNECT") {
    isConnectedToWeb = true;
    // Publicamos en /data, nunca en /cmd
    mqttClient.publish(topicData.c_str(), "CONNECTED");
    return;
  }

  // Todo lo demás va a la cola para que la tarea de comandos lo procese
  xQueueSend(queueCmds, buf, 0);  // no bloquea si la cola está llena
}

// =============================================================
//  TAREA MQTT
// =============================================================
void tareaMQTT(void*) {
  TickType_t t = xTaskGetTickCount();
  unsigned long lastRecon  = 0;
  unsigned long lastStatus = 0;
  unsigned long lastSteps  = 0;

  while (true) {
    vTaskDelayUntil(&t, pdMS_TO_TICKS(50));

    if (!mqttClient.connected()) {
      isConnectedToWeb = false;
      unsigned long now = millis();
      if (now - lastRecon > 5000) {
        lastRecon = now;
        String id = "ESP32_" + String(random(0xffff), HEX);
        if (mqttClient.connect(id.c_str(), mqtt_user, mqtt_password)) {
          // Suscribir SOLO al topic de comandos
          mqttClient.subscribe(topicCmd.c_str());
          // Publicar en /data que estamos online
          mqttClient.publish(topicData.c_str(), "ONLINE");
          Serial.println("✅ MQTT reconectado");
        } else {
          Serial.printf("❌ MQTT rc=%d\n", mqttClient.state());
        }
      }
    } else {
      mqttClient.loop();

      unsigned long now = millis();

      // Status periódico cada 500ms
      if (now - lastStatus >= 500) {
        lastStatus = now;
        String st = "X:"    + String(posX, 2)
                  + "|Y:"   + String(posY, 2)
                  + "|ANG:" + String(anguloRad * 180.0 / PI, 1)
                  + "|MODO:"+ String(modoTremouse ? "TREMOUSE" : (modoAutonomo ? "AUTO" : "MANUAL"))
                  + "|BIAS:"+ String(motorBias)
                  + "|TM_COL:0|TM_ROW:0|TM_HDG:0";
        mqttClient.publish(topicStatus.c_str(), st.c_str());
      }

      // Steps periódico cada 100ms
      if (stepTracking && now - lastSteps >= 100) {
        lastSteps = now;
        long si, sd;
        portENTER_CRITICAL(&muxEncoder);
        si = pulsosIzq - stepBaseIzq;
        sd = pulsosDer - stepBaseDer;
        portEXIT_CRITICAL(&muxEncoder);
        mqttClient.publish(topicData.c_str(),
          ("STEPS:" + String(si) + "," + String(sd)).c_str());
      }
    }
  }
}

// =============================================================
//  TAREA ULTRASONIDOS
// =============================================================
void tareaUltra(void*) {
  TickType_t t = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&t, pdMS_TO_TICKS(80));
    if (!sistemaIniciado || modoTremouse) continue;
    DatosSensores d;
    d.centro  = ultrasonido(TRIG_C, ECHO_C); vTaskDelay(pdMS_TO_TICKS(20));
    d.derecha  = ultrasonido(TRIG_R, ECHO_R); vTaskDelay(pdMS_TO_TICKS(20));
    d.izquierda= ultrasonido(TRIG_L, ECHO_L);
    xQueueOverwrite(queueSensores, &d);
  }
}

// =============================================================
//  TAREA NAVEGACIÓN AUTÓNOMA
// =============================================================
void tareaNav(void*) {
  TickType_t t = xTaskGetTickCount();
  DatosSensores s;
  while (true) {
    vTaskDelayUntil(&t, pdMS_TO_TICKS(150));
    if (!sistemaIniciado || !modoAutonomo || modoTremouse) continue;
    if (xQueuePeek(queueSensores, &s, 0) != pdTRUE) continue;

    if (s.centro < DIST_GIRO) {
      motorStop(); vTaskDelay(pdMS_TO_TICKS(150));
      if      (s.izquierda > DIST_GIRO) { mvIzquierda(VEL_AUTO); vTaskDelay(pdMS_TO_TICKS(T_GIRO_AUTO)); }
      else if (s.derecha   > DIST_GIRO) { mvDerecha(VEL_AUTO);   vTaskDelay(pdMS_TO_TICKS(T_GIRO_AUTO)); }
      else                               { mvIzquierda(VEL_AUTO); vTaskDelay(pdMS_TO_TICKS(T_GIRO_AUTO*2)); }
      motorStop(); vTaskDelay(pdMS_TO_TICKS(200));
    } else if (s.derecha > DIST_GIRO) {
      mvAdelante(VEL_AUTO); vTaskDelay(pdMS_TO_TICKS(400));
      motorStop(); vTaskDelay(pdMS_TO_TICKS(100));
      mvDerecha(VEL_AUTO); vTaskDelay(pdMS_TO_TICKS(T_GIRO_AUTO));
      motorStop(); vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      if      (s.derecha < DIST_MINIMA)    mvAdelante(VEL_AUTO - 20);
      else if (s.derecha > DIST_PARED + 5) mvAdelante(VEL_AUTO + 10);
      else                                 mvAdelante(VEL_AUTO);
    }
  }
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ROBOT LABERINTO v3 ===");

  // Colas
  queueCmds    = xQueueCreate(20, CMD_LEN);          // hasta 20 comandos pendientes
  queueSensores = xQueueCreate(1, sizeof(DatosSensores));

  // Motores
  pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT); pinMode(MOTOR_R_IN2, OUTPUT);
  ledcSetup(CH_L, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MOTOR_L_PWM, CH_L);
  ledcSetup(CH_R, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MOTOR_R_PWM, CH_R);
  motorStop();

  // Encoders
  pinMode(ENCODER_L, INPUT_PULLUP); pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), isrIzq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), isrDer, RISING);

  // Sensores
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(IR_SENSOR, INPUT);

  // Topics
  topicCmd    = robotName + "/cmd";
  topicData   = robotName + "/data";
  topicStatus = robotName + "/status";

  // WiFi
  Serial.print("WiFi...");
  WiFi.begin(ssid, password);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " OK " + WiFi.localIP().toString() : " FALLO");

  // MQTT
  espClient.setInsecure();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);

  {
    String id = "ESP32_" + String(random(0xffff), HEX);
    if (mqttClient.connect(id.c_str(), mqtt_user, mqtt_password)) {
      mqttClient.subscribe(topicCmd.c_str());   // solo /cmd
      mqttClient.publish(topicData.c_str(), "ONLINE");  // publica en /data
      Serial.println("MQTT OK");
    } else {
      Serial.printf("MQTT FALLO rc=%d\n", mqttClient.state());
    }
  }

  // Tareas
  // Comandos en core 0, prioridad alta para respuesta rápida
  xTaskCreatePinnedToCore(tareaComandos, "Cmds",  4096, NULL, 3, &hComandos, 0);
  // MQTT en core 1
  xTaskCreatePinnedToCore(tareaMQTT,     "MQTT",  4096, NULL, 2, &hMQTT,     1);
  // Ultrasonidos en core 1
  xTaskCreatePinnedToCore(tareaUltra,    "Ultra", 3072, NULL, 1, &hUltra,    1);
  // Navegación en core 1
  xTaskCreatePinnedToCore(tareaNav,      "Nav",   4096, NULL, 1, &hNav,      1);

  delay(500);
  sistemaIniciado = true;
  Serial.println("🚀 LISTO");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
