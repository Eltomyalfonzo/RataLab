#pragma once
// ================================================================
//  config.h — Parámetros globales del robot
// ================================================================

// ── WiFi / MQTT ──────────────────────────────────────────────
#define WIFI_SSID       "Galaxy A24 B789"
#define WIFI_PASS       "superman"
#define MQTT_SERVER     "f9d90e4c488c4716ac1e5862b9c8a708.s1.eu.hivemq.cloud"
#define MQTT_PORT       8883
#define MQTT_USER       "admin"
#define MQTT_PASS       "SuperMan2233"
#define ROBOT_NAME      "robot01"

// ── Pines motores ────────────────────────────────────────────
#define MOTOR_L_PWM     12
#define MOTOR_L_IN1     16
#define MOTOR_L_IN2     17
#define MOTOR_R_PWM     13
#define MOTOR_R_IN1     15
#define MOTOR_R_IN2      4

// ── Pines encoders ───────────────────────────────────────────
#define ENCODER_L        5
#define ENCODER_R       34

// ── Pines ultrasonidos ───────────────────────────────────────
#define TRIG_C          32
#define ECHO_C          33
#define TRIG_R          23
#define ECHO_R          22
#define TRIG_L          25
#define ECHO_L          26
#define IR_SENSOR       35

// ── PWM ──────────────────────────────────────────────────────
#define PWM_FREQ        5000
#define PWM_RESOLUTION     8
#define CH_L               0
#define CH_R               1

// ── Odometría ────────────────────────────────────────────────
#define PULSOS_POR_REV     30.0f
#define DIAMETRO_RUEDA      5.0f
#define DISTANCIA_ENTRE_RUEDAS 15.0f

// ── Modo autónomo ────────────────────────────────────────────
#define DIST_PARED      15.0f
#define DIST_MINIMA      8.0f
#define DIST_GIRO       20.0f
#define VEL_AUTO          190
#define T_GIRO_AUTO       400

// ── Ultrasonido rango ────────────────────────────────────────
#define DIST_MIN_CM      1.0f
#define DIST_MAX_CM     20.0f
#define DIST_MAX_TM     50.0f

// ── Cola de comandos ─────────────────────────────────────────
#define CMD_LEN           64

// ── Verificación frontal Tremouse ────────────────────────────
#define TM_VERIF_TOTAL     5
#define TM_VERIF_MINIMAS   3
