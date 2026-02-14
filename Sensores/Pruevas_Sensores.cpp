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

/*
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ===== CONFIGURACIÓN WiFi =====
const char* ssid = "BALLARI PABLO";
const char* password = "23527257";

// ===== CONFIGURACIÓN MQTT - HiveMQ Cloud =====
const char* mqtt_server = "f9d90e4c488c4716ac1e5862b9c8a708.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "admin";
const char* mqtt_password = "SuperMan2233";

// ===== TOPICS DEL ROBOT =====
String robotName = "robot01";
String topicCmd = "";
String topicData = "";
String topicStatus = "";

// ===== CONFIGURACIÓN ENCODERS ÓPTICOS =====
#define ENCODER_L 5
#define ENCODER_R 34

// ===== CONFIGURACIÓN MOTOR IZQUIERDO (L298N) =====
#define MOTOR_L_PWM 12
#define MOTOR_L_IN1 16
#define MOTOR_L_IN2 17

// ===== CONFIGURACIÓN MOTOR DERECHO (L298N) =====
#define MOTOR_R_PWM 13
#define MOTOR_R_IN1 15
#define MOTOR_R_IN2 4

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

// ===== PARÁMETROS DE CONVERSIÓN PULSOS → DISTANCIA =====
const float PULSOS_POR_REVOLUCION = 30.0;
const float DIAMETRO_RUEDA = 5.0;
const float CIRCUNFERENCIA = PI * DIAMETRO_RUEDA;
const float CM_POR_PULSO = CIRCUNFERENCIA / PULSOS_POR_REVOLUCION;
const float DISTANCIA_ENTRE_RUEDAS = 15.0;

// ===== PARÁMETROS PID =====
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;
bool pidEnabled = true;

// ===== PARÁMETROS DE NAVEGACIÓN AUTÓNOMA =====
const float DISTANCIA_PARED = 15.0;        // Distancia ideal a la pared (cm)
const float DISTANCIA_MINIMA = 8.0;        // Distancia mínima para evitar colisión (cm)
const float DISTANCIA_GIRO = 20.0;         // Distancia para decidir girar (cm)
const int VELOCIDAD_AUTONOMA = 120;        // Velocidad en modo autónomo
const int TIEMPO_GIRO = 800;               // Tiempo de giro en ms (aprox 90°)
const int UMBRAL_NEGRO = 2000;             // Umbral ADC para detectar negro (ajustar según sensor)

bool modoAutonomo = false;                 // Estado del modo autónomo
bool lineaNegraDetectada = false;          // Flag de línea negra

// ===== ESTRUCTURA DE CONTROL PID =====
struct ControladorPID {
  float errorAnterior;
  float errorIntegral;
  float setpoint;
  float velocidadActual;
  int pwmBase;
  int pwmActual;
  unsigned long ultimaActualizacion;
};

ControladorPID pidIzq = {0, 0, 0, 0, 0, 0, 0};
ControladorPID pidDer = {0, 0, 0, 0, 0, 0, 0};

// ===== VARIABLES DE ESTADO DEL MOVIMIENTO =====
int direccionMotorIzq = 0;
int direccionMotorDer = 0;

// ===== CONTADORES DE PULSOS =====
volatile long pulsosIzq = 0;
volatile long pulsosDer = 0;

// ===== POSICIÓN EN EL PLANO CARTESIANO =====
float posX = 0.0;
float posY = 0.0;
float anguloRad = 0.0;

// ===== VARIABLES DE CONTROL =====
int velocidadBase = 150;
bool isConnectedToWeb = false;
unsigned long lastDataMsg = 0;
const long dataInterval = 200;

// ===== VARIABLES PARA TRACKING DE PULSOS (ODOMETRÍA) =====
long ultimoPulsosIzqOdom = 0;
long ultimoPulsosDerOdom = 0;

// ===== VARIABLES PARA TRACKING DE PULSOS (PID) =====
long ultimoPulsosIzqPID = 0;
long ultimoPulsosDerPID = 0;

// ===== VARIABLES PARA CÁLCULO DE VELOCIDAD =====
unsigned long ultimoTiempoPID = 0;
const long intervaloPID = 100;

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ===== DECLARACIONES FORWARD (PROTOTIPOS) =====
void motorIzquierdoAdelante(int pwm);
void motorIzquierdoAtras(int pwm);
void motorIzquierdoStop();
void motorDerechoAdelante(int pwm);
void motorDerechoAtras(int pwm);
void motorDerechoStop();
void todosLosMotoresStop();
void adelante(int pwm);
void atras(int pwm);
void izquierda(int pwm);
void derecha(int pwm);
void girarDerecha90();
void girarIzquierda90();
void avanzarPoco();
void iniciarModoAutonomo();
void detenerModoAutonomo();
void navegacionAutonoma();

// ===== INTERRUPCIONES DE ENCODERS =====
void IRAM_ATTR contarIzquierda() {
  pulsosIzq++;
}

void IRAM_ATTR contarDerecha() {
  pulsosDer++;
}

// ===== LEER SENSOR ULTRASONICO =====
float leerUltrasonico(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duracion = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
  
  if (duracion == 0) {
    return 999.0; // Sin eco = muy lejos o sin obstáculo
  }
  
  float distancia = duracion * 0.034 / 2.0; // Velocidad del sonido
  return distancia;
}

// ===== LEER SENSOR INFRARROJO =====
bool detectarLineaNegra() {
  int valorIR = analogRead(IR_SENSOR);
  
  // Debug cada 500ms
  static unsigned long lastDebugIR = 0;
  if (millis() - lastDebugIR > 500) {
    Serial.print("IR: ");
    Serial.print(valorIR);
    Serial.println(valorIR > UMBRAL_NEGRO ? " [NEGRO]" : " [BLANCO]");
    lastDebugIR = millis();
  }
  
  return valorIR > UMBRAL_NEGRO;
}

// ===== ALGORITMO DE NAVEGACIÓN: SEGUIR PARED DERECHA =====
void navegacionAutonoma() {
  // 1. Verificar línea negra (salida del laberinto)
  if (detectarLineaNegra()) {
    lineaNegraDetectada = true;
    detenerModoAutonomo();
    Serial.println("🏁 ¡LÍNEA NEGRA DETECTADA! Laberinto completado");
    client.publish(topicData.c_str(), "LINEA_NEGRA_DETECTADA");
    return;
  }
  
  // 2. Leer sensores ultrasónicos
  float distCentro = leerUltrasonico(TRIG_C, ECHO_C);
  float distDerecha = leerUltrasonico(TRIG_R, ECHO_R);
  float distIzquierda = leerUltrasonico(TRIG_L, ECHO_L);
  
  // Debug sensores
  static unsigned long lastDebugUlt = 0;
  if (millis() - lastDebugUlt > 500) {
    Serial.print("Sensores → C:");
    Serial.print(distCentro, 1);
    Serial.print("cm  R:");
    Serial.print(distDerecha, 1);
    Serial.print("cm  L:");
    Serial.print(distIzquierda, 1);
    Serial.println("cm");
    lastDebugUlt = millis();
  }
  
  // 3. LÓGICA DE DECISIÓN (Algoritmo seguir pared derecha)
  
  // Debug odometría cada 1 segundo
  static unsigned long lastDebugOdom = 0;
  if (millis() - lastDebugOdom > 1000) {
    Serial.print("📍 Posición: (");
    Serial.print(posX, 1);
    Serial.print(", ");
    Serial.print(posY, 1);
    Serial.print(") Ang: ");
    Serial.print(anguloRad * 180.0 / PI, 0);
    Serial.println("°");
    lastDebugOdom = millis();
  }
  
  // CASO 1: Pared adelante - necesita girar
  if (distCentro < DISTANCIA_GIRO) {
    Serial.println("🚧 Pared adelante - Decidiendo giro...");
    
    // Si puede girar a la izquierda
    if (distIzquierda > DISTANCIA_GIRO) {
      Serial.println("↰ Girando IZQUIERDA");
      girarIzquierda90();
    }
    // Si puede girar a la derecha
    else if (distDerecha > DISTANCIA_GIRO) {
      Serial.println("↱ Girando DERECHA");
      girarDerecha90();
    }
    // Callejón sin salida - giro 180°
    else {
      Serial.println("🔄 Callejón sin salida - Giro 180°");
      girarIzquierda90();
      delay(100);
      girarIzquierda90();
    }
  }
  
  // CASO 2: Camino libre a la derecha - girar derecha (seguir pared)
  else if (distDerecha > DISTANCIA_GIRO) {
    Serial.println("➡️ Camino libre a la derecha - Girando");
    avanzarPoco();
    delay(300);
    girarDerecha90();
  }
  
  // CASO 3: Camino recto - avanzar siguiendo pared derecha
  else {
    // Avanzar con corrección de pared
    if (distDerecha < DISTANCIA_MINIMA) {
      // Muy cerca de pared derecha - corregir a la izquierda
      Serial.println("⬅ Ajustando - Muy cerca pared derecha");
      motorIzquierdoAdelante(VELOCIDAD_AUTONOMA - 20);
      motorDerechoAdelante(VELOCIDAD_AUTONOMA + 20);
    }
    else if (distDerecha > DISTANCIA_PARED + 5) {
      // Muy lejos de pared derecha - corregir a la derecha
      Serial.println("➡ Ajustando - Muy lejos pared derecha");
      motorIzquierdoAdelante(VELOCIDAD_AUTONOMA + 20);
      motorDerechoAdelante(VELOCIDAD_AUTONOMA - 20);
    }
    else {
      // Distancia ideal - avanzar recto
      adelante(VELOCIDAD_AUTONOMA);
    }
  }
  
  delay(100); // Pequeña pausa entre decisiones
}

// ===== FUNCIONES AUXILIARES DE NAVEGACIÓN =====
void girarDerecha90() {
  todosLosMotoresStop();
  delay(100);
  derecha(VELOCIDAD_AUTONOMA);
  delay(TIEMPO_GIRO);
  todosLosMotoresStop();
  delay(200);
}

void girarIzquierda90() {
  todosLosMotoresStop();
  delay(100);
  izquierda(VELOCIDAD_AUTONOMA);
  delay(TIEMPO_GIRO);
  todosLosMotoresStop();
  delay(200);
}

void avanzarPoco() {
  adelante(VELOCIDAD_AUTONOMA);
  delay(300);
  todosLosMotoresStop();
}

// ===== INICIAR MODO AUTÓNOMO =====
void iniciarModoAutonomo() {
  modoAutonomo = true;
  lineaNegraDetectada = false;
  
  Serial.println("\n🤖 ========================================");
  Serial.println("🤖 MODO AUTÓNOMO ACTIVADO");
  Serial.println("🤖 Algoritmo: Seguir pared derecha");
  Serial.println("🤖 ========================================\n");
  
  client.publish(topicData.c_str(), "MODO_AUTONOMO_INICIADO");
  
  // Pequeña pausa antes de empezar
  delay(500);
}

// ===== DETENER MODO AUTÓNOMO =====
void detenerModoAutonomo() {
  modoAutonomo = false;
  todosLosMotoresStop();
  
  Serial.println("\n⏹️ ========================================");
  Serial.println("⏹️ MODO AUTÓNOMO DETENIDO");
  
  if (lineaNegraDetectada) {
    Serial.println("⏹️ Razón: Línea negra detectada (FIN)");
  } else {
    Serial.println("⏹️ Razón: Comando manual STOP");
  }
  
  Serial.println("⏹️ ========================================\n");
  
  client.publish(topicData.c_str(), "MODO_AUTONOMO_DETENIDO");
}

// ===== CALCULAR PID =====
float calcularPID(ControladorPID &pid, float medicion, float dt) {
  float error = pid.setpoint - medicion;
  float P = Kp * error;
  
  pid.errorIntegral += error * dt;
  pid.errorIntegral = constrain(pid.errorIntegral, -50, 50);
  float I = Ki * pid.errorIntegral;
  
  float D = Kd * (error - pid.errorAnterior) / dt;
  pid.errorAnterior = error;
  
  float correccion = P + I + D;
  return correccion;
}

// ===== ACTUALIZAR CONTROL PID =====
void actualizarPID() {
  unsigned long tiempoActual = millis();
  float dt = (tiempoActual - ultimoTiempoPID) / 1000.0;
  
  if (dt < (intervaloPID / 1000.0)) return;
  
  long deltaPulsosIzq = pulsosIzq - ultimoPulsosIzqPID;
  long deltaPulsosDer = pulsosDer - ultimoPulsosDerPID;
  
  pidIzq.velocidadActual = deltaPulsosIzq / dt;
  pidDer.velocidadActual = deltaPulsosDer / dt;
  
  if (pidEnabled && !modoAutonomo && (direccionMotorIzq != 0 || direccionMotorDer != 0)) {
    float correccionIzq = calcularPID(pidIzq, pidIzq.velocidadActual, dt);
    float correccionDer = calcularPID(pidDer, pidDer.velocidadActual, dt);
    
    pidIzq.pwmActual = constrain(pidIzq.pwmBase + (int)correccionIzq, 0, 255);
    pidDer.pwmActual = constrain(pidDer.pwmBase + (int)correccionDer, 0, 255);
    
    if (direccionMotorIzq != 0) {
      analogWrite(MOTOR_L_PWM, abs(pidIzq.pwmActual));
    }
    if (direccionMotorDer != 0) {
      analogWrite(MOTOR_R_PWM, abs(pidDer.pwmActual));
    }
  }
  
  ultimoPulsosIzqPID = pulsosIzq;
  ultimoPulsosDerPID = pulsosDer;
  ultimoTiempoPID = tiempoActual;
}

// ===== CONFIGURAR SETPOINT PID =====
void configurarSetpointPID(int pwmBase) {
  float factorPulsosPorPWM = 0.5;
  
  pidIzq.setpoint = pwmBase * factorPulsosPorPWM;
  pidDer.setpoint = pwmBase * factorPulsosPorPWM;
  pidIzq.pwmBase = pwmBase;
  pidDer.pwmBase = pwmBase;
  
  pidIzq.errorIntegral = 0;
  pidDer.errorIntegral = 0;
  pidIzq.errorAnterior = 0;
  pidDer.errorAnterior = 0;
}

// ===== TRANSFORMACIÓN VECTORIAL: PULSOS → COORDENADAS X,Y =====
void actualizarPosicion() {
  long deltaPulsosIzq = pulsosIzq - ultimoPulsosIzqOdom;
  long deltaPulsosDer = pulsosDer - ultimoPulsosDerOdom;
  
  ultimoPulsosIzqOdom = pulsosIzq;
  ultimoPulsosDerOdom = pulsosDer;
  
  float distanciaIzq = deltaPulsosIzq * CM_POR_PULSO * direccionMotorIzq;
  float distanciaDer = deltaPulsosDer * CM_POR_PULSO * direccionMotorDer;
  
  float distanciaCentro = (distanciaIzq + distanciaDer) / 2.0;
  float deltaAngulo = (distanciaDer - distanciaIzq) / DISTANCIA_ENTRE_RUEDAS;
  
  anguloRad += deltaAngulo;
  
  while (anguloRad > PI) anguloRad -= 2.0 * PI;
  while (anguloRad < -PI) anguloRad += 2.0 * PI;
  
  float deltaX = distanciaCentro * cos(anguloRad);
  float deltaY = distanciaCentro * sin(anguloRad);
  
  posX += deltaX;
  posY += deltaY;
}

// ===== FUNCIONES DE CONTROL DE MOTORES =====
void motorIzquierdoAdelante(int pwm) {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, pwm);
  direccionMotorIzq = 1;
}

void motorIzquierdoAtras(int pwm) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  analogWrite(MOTOR_L_PWM, pwm);
  direccionMotorIzq = -1;
}

void motorIzquierdoStop() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, 0);
  direccionMotorIzq = 0;
  pidIzq.pwmActual = 0;
  pidIzq.errorIntegral = 0;
}

void motorDerechoAdelante(int pwm) {
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, pwm);
  direccionMotorDer = 1;
}

void motorDerechoAtras(int pwm) {
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  analogWrite(MOTOR_R_PWM, pwm);
  direccionMotorDer = -1;
}

void motorDerechoStop() {
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, 0);
  direccionMotorDer = 0;
  pidDer.pwmActual = 0;
  pidDer.errorIntegral = 0;
}

void todosLosMotoresStop() {
  motorIzquierdoStop();
  motorDerechoStop();
}

void adelante(int pwm) {
  configurarSetpointPID(pwm);
  motorIzquierdoAdelante(pwm);
  motorDerechoAdelante(pwm);
}

void atras(int pwm) {
  configurarSetpointPID(pwm);
  motorIzquierdoAtras(pwm);
  motorDerechoAtras(pwm);
}

void izquierda(int pwm) {
  configurarSetpointPID(pwm);
  motorIzquierdoAtras(pwm);
  motorDerechoAdelante(pwm);
}

void derecha(int pwm) {
  configurarSetpointPID(pwm);
  motorIzquierdoAdelante(pwm);
  motorDerechoAtras(pwm);
}

// ===== PROCESAR COMANDOS MQTT =====
void processCommand(String message) {
  message.trim();
  message.toUpperCase();
  
  Serial.print("🔧 Comando: ");
  Serial.println(message);
  
  if (message.length() < 1) return;
  
  // COMANDO START - INICIAR NAVEGACIÓN AUTÓNOMA
  if (message == "START") {
    iniciarModoAutonomo();
    return;
  }
  
  // STOP
  if (message.charAt(0) == 'S') {
    if (modoAutonomo) {
      detenerModoAutonomo();
    } else {
      todosLosMotoresStop();
      Serial.println("⏹️ STOP");
    }
    client.publish(topicData.c_str(), "STOP");
    return;
  }
  
  // RESET POSICIÓN
  if (message.charAt(0) == 'Z') {
    posX = 0.0;
    posY = 0.0;
    anguloRad = 0.0;
    pulsosIzq = 0;
    pulsosDer = 0;
    ultimoPulsosIzqOdom = 0;
    ultimoPulsosDerOdom = 0;
    ultimoPulsosIzqPID = 0;
    ultimoPulsosDerPID = 0;
    Serial.println("🔄 Posición reseteada a (0, 0)");
    client.publish(topicData.c_str(), "RESET");
    return;
  }
  
  // Si está en modo autónomo, ignorar otros comandos manuales
  if (modoAutonomo) {
    Serial.println("⚠️ Modo autónomo activo - Comando ignorado (usa S para detener)");
    return;
  }
  
  char comando = message.charAt(0);
  
  // VELOCIDAD
  if (comando == 'V' && message.length() > 1) {
    String velStr = message.substring(1);
    int vel = velStr.toInt();
    
    if (vel >= 0 && vel <= 255) {
      velocidadBase = vel;
      Serial.print("🎚️ Velocidad: ");
      Serial.println(velocidadBase);
      
      String resp = "VELOCIDAD:" + String(velocidadBase);
      client.publish(topicData.c_str(), resp.c_str());
    }
    return;
  }
  
  // ACTIVAR/DESACTIVAR PID
  if (comando == 'P') {
    pidEnabled = !pidEnabled;
    Serial.print("🎛️ PID: ");
    Serial.println(pidEnabled ? "ACTIVADO" : "DESACTIVADO");
    
    String resp = "PID:" + String(pidEnabled ? "ON" : "OFF");
    client.publish(topicData.c_str(), resp.c_str());
    return;
  }
  
  // CONFIGURAR PARÁMETROS PID
  if (comando == 'K' && message.indexOf(':') > 0) {
    int separador = message.indexOf(':');
    String param = message.substring(1, separador);
    float valor = message.substring(separador + 1).toFloat();
    
    if (param == "P") {
      Kp = valor;
      Serial.print("Kp = ");
      Serial.println(Kp);
    } else if (param == "I") {
      Ki = valor;
      Serial.print("Ki = ");
      Serial.println(Ki);
    } else if (param == "D") {
      Kd = valor;
      Serial.print("Kd = ");
      Serial.println(Kd);
    }
    
    String resp = "PID_" + param + ":" + String(valor);
    client.publish(topicData.c_str(), resp.c_str());
    return;
  }
  
  // MOVIMIENTOS BÁSICOS (solo si NO está en modo autónomo)
  if (comando == 'F') {
    adelante(velocidadBase);
    client.publish(topicData.c_str(), "ADELANTE");
    return;
  }
  
  if (comando == 'B') {
    atras(velocidadBase);
    client.publish(topicData.c_str(), "ATRAS");
    return;
  }
  
  if (comando == 'L') {
    izquierda(velocidadBase);
    client.publish(topicData.c_str(), "IZQUIERDA");
    return;
  }
  
  if (comando == 'R') {
    derecha(velocidadBase);
    client.publish(topicData.c_str(), "DERECHA");
    return;
  }
  
  Serial.println("❌ Comando desconocido");
}

// ===== CALLBACK MQTT =====
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("📩 [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == topicCmd) {
    if (message == "CONNECT") {
      Serial.println("✅ Conectando web");
      isConnectedToWeb = true;
      client.publish(topicCmd.c_str(), "CONNECTED");
    } else {
      processCommand(message);
    }
  }
}

// ===== CONFIGURAR WiFi =====
void setup_wifi() {
  Serial.print("📡 Conectando WiFi...");
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" ✅");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" ❌");
  }
}

// ===== RECONECTAR MQTT =====
void reconnect() {
  while (!client.connected()) {
    Serial.print("🔌 MQTT...");
    
    String clientId = "ESP32_" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println(" ✅");
      client.subscribe(topicCmd.c_str());
      client.publish(topicData.c_str(), "ONLINE");
    } else {
      Serial.print(" ❌ rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n🤖 ROBOT AUTÓNOMO - NAVEGACIÓN DE LABERINTO");
  Serial.println("=============================================");
  
  // Parámetros físicos
  Serial.print("Pulsos por revolución: ");
  Serial.println(PULSOS_POR_REVOLUCION);
  Serial.print("Diámetro rueda: ");
  Serial.print(DIAMETRO_RUEDA);
  Serial.println(" cm");
  Serial.print("cm por pulso: ");
  Serial.println(CM_POR_PULSO, 4);
  Serial.print("Distancia entre ruedas: ");
  Serial.print(DISTANCIA_ENTRE_RUEDAS);
  Serial.println(" cm");
  Serial.println();
  
  // Parámetros PID
  Serial.println("PARÁMETROS PID:");
  Serial.print("  Kp = ");
  Serial.println(Kp);
  Serial.print("  Ki = ");
  Serial.println(Ki);
  Serial.print("  Kd = ");
  Serial.println(Kd);
  Serial.println();
  
  // Parámetros navegación
  Serial.println("PARÁMETROS NAVEGACIÓN:");
  Serial.print("  Distancia pared ideal: ");
  Serial.print(DISTANCIA_PARED);
  Serial.println(" cm");
  Serial.print("  Distancia mínima: ");
  Serial.print(DISTANCIA_MINIMA);
  Serial.println(" cm");
  Serial.print("  Umbral negro (IR): ");
  Serial.println(UMBRAL_NEGRO);
  Serial.println();
  
  // Comandos
  Serial.println("COMANDOS:");
  Serial.println("  START  → Iniciar navegación autónoma");
  Serial.println("  S      → Detener (manual o autónomo)");
  Serial.println("  F/B/L/R → Control manual");
  Serial.println("  P      → Toggle PID");
  Serial.println("  Z      → Reset posición");
  Serial.println();
  
  // Topics MQTT
  topicCmd = robotName + "/cmd";
  topicData = robotName + "/data";
  topicStatus = robotName + "/status";
  
  Serial.println("Topics MQTT:");
  Serial.println("  CMD:  " + topicCmd);
  Serial.println("  DATA: " + topicData);
  Serial.println();
  
  // Configurar encoders
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), contarIzquierda, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), contarDerecha, RISING);
  Serial.println("✅ Encoders configurados");
  
  // Configurar motores
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  todosLosMotoresStop();
  Serial.println("✅ Motores configurados");
  
  // Configurar sensores ultrasónicos
  pinMode(TRIG_C, OUTPUT);
  pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  Serial.println("✅ Sensores ultrasónicos configurados");
  
  // Configurar sensor infrarrojo
  pinMode(IR_SENSOR, INPUT);
  Serial.println("✅ Sensor infrarrojo configurado");
  
  // WiFi y MQTT
  setup_wifi();
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  ultimoTiempoPID = millis();
  
  Serial.println("\n✅ Sistema listo");
  Serial.println("Esperando comando START para navegación autónoma...\n");
}

// ===== LOOP =====
void loop() {
  // MQTT
  if (!client.connected()) {
    isConnectedToWeb = false;
    reconnect();
  }
  client.loop();
  
  // MODO AUTÓNOMO
  if (modoAutonomo) {
    navegacionAutonoma();
  } else {
    // Modo manual - actualizar PID
    actualizarPID();
  }
  
  // ACTUALIZAR POSICIÓN
  actualizarPosicion();
  
  // ENVIAR COORDENADAS A LA WEB
  unsigned long now = millis();
  if (now - lastDataMsg >= dataInterval && isConnectedToWeb) {
    lastDataMsg = now;
    
    String coordenadas = String(posX, 2) + "," + String(posY, 2);
    client.publish(topicData.c_str(), coordenadas.c_str());
    
    String status = "X:" + String(posX, 2) + 
                    "|Y:" + String(posY, 2) +
                    "|ANGULO:" + String(anguloRad * 180.0 / PI, 1) +
                    "|MODO:" + String(modoAutonomo ? "AUTO" : "MANUAL") +
                    "|PID:" + String(pidEnabled ? "ON" : "OFF");
    client.publish(topicStatus.c_str(), status.c_str());
  }
}
*/