// ================================================================
//  Motors.cpp
// ================================================================
#include "Motors.h"
#include "config.h"
#include "globals.h"

int dirIzq = 0;
int dirDer = 0;

void motorsInit() {
    pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT); pinMode(MOTOR_R_IN2, OUTPUT);
    ledcSetup(CH_L, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_L_PWM, CH_L);
    ledcSetup(CH_R, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_R_PWM, CH_R);
    motorStop();
}

void motorStop() {
    ledcWrite(CH_L, 0); ledcWrite(CH_R, 0);
    digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, LOW);
    dirIzq = 0; dirDer = 0;
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
    digitalWrite(MOTOR_L_IN1, LOW);  digitalWrite(MOTOR_L_IN2, HIGH);
    digitalWrite(MOTOR_R_IN1, LOW);  digitalWrite(MOTOR_R_IN2, HIGH);
    ledcWrite(CH_L, vL); ledcWrite(CH_R, vR);
    dirIzq = -1; dirDer = -1;
}

void mvIzquierda(int vel) {
    int vL = constrain(vel + motorBias, 60, 255);
    int vR = constrain(vel - motorBias, 60, 255);
    digitalWrite(MOTOR_L_IN1, LOW);  digitalWrite(MOTOR_L_IN2, HIGH);
    digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
    ledcWrite(CH_L, vL); ledcWrite(CH_R, vR);
    dirIzq = -1; dirDer = 1;
}

void mvDerecha(int vel) {
    int vL = constrain(vel + motorBias, 60, 255);
    int vR = constrain(vel - motorBias, 60, 255);
    digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW);  digitalWrite(MOTOR_R_IN2, HIGH);
    ledcWrite(CH_L, vL); ledcWrite(CH_R, vR);
    dirIzq = 1; dirDer = -1;
}
