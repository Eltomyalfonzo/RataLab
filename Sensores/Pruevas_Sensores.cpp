#include <Arduino.h>
// ===== CONFIGURACIÓN SENSORES ULTRASÓNICOS =====
// Sensor Centro
#define TRIG_C 32
#define ECHO_C 33

// Sensor Derecho
#define TRIG_R 23
#define ECHO_R 22

// Sensor Izquierdo 
#define TRIG_L 25
#define ECHO_L 26

// ===== SENSOR INFRARROJO =====
#define IR_SENSOR 35  // Sensor infrarrojo para detectar línea negra


// ===== FUNCIÓN PARA MEDIR DISTANCIA =====
float medirDistancia(int trigPin, int echoPin) {
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH, 30000); // timeout 30ms

  float distancia = duracion * 0.034 / 2; // cm
  return distancia;
}


// ===== SETUP =====
void setup() {

  Serial.begin(115200);

  // Ultrasonicos
  pinMode(TRIG_C, OUTPUT);
  pinMode(ECHO_C, INPUT);

  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  // Infrarrojo
  pinMode(IR_SENSOR, INPUT);

  Serial.println("Test de sensores iniciado...");
}


// ===== LOOP =====
void loop() {

  float distC = medirDistancia(TRIG_C, ECHO_C);
  float distR = medirDistancia(TRIG_R, ECHO_R);
  float distL = medirDistancia(TRIG_L, ECHO_L);

  int estadoIR = digitalRead(IR_SENSOR);

  Serial.println("===== LECTURAS =====");

  Serial.print("Centro: ");
  Serial.print(distC);
  Serial.println(" cm");

  Serial.print("Derecha: ");
  Serial.print(distR);
  Serial.println(" cm");

  Serial.print("Izquierda: ");
  Serial.print(distL);
  Serial.println(" cm");

  Serial.print("IR Linea: ");
  Serial.println(estadoIR); // 0 o 1

  Serial.println("--------------------");

  delay(500);
}
