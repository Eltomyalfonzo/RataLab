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
#define ENCODER_L 5  // Encoder Izquierdo
#define ENCODER_R 34 // Encoder Derecho

// ===== CONFIGURACIÓN MOTOR IZQUIERDO (L298N) =====
#define MOTOR_L_PWM 12
#define MOTOR_L_IN1 16
#define MOTOR_L_IN2 17

// ===== CONFIGURACIÓN MOTOR DERECHO (L298N) =====
#define MOTOR_R_PWM 13
#define MOTOR_R_IN1 15
#define MOTOR_R_IN2 4

// ===== PARÁMETROS FÍSICOS DEL ROBOT =====
const float DIENTES_ENCODER = 30.0;           // Dientes de la rueda dentada
const float DIAMETRO_RUEDA = 5.0;             // Diámetro de rueda en cm
const float PERIMETRO_RUEDA = PI * DIAMETRO_RUEDA;  // Perímetro = π * d
const float CM_POR_PULSO = PERIMETRO_RUEDA / DIENTES_ENCODER;  // ~0.524 cm/pulso
const float DISTANCIA_ENTRE_RUEDAS = 15.0;    // Distancia entre ruedas en cm (ajustar según tu robot)

// Variables para contar pulsos (volatile porque se usan en interrupciones)
volatile long pulsosIzquierda = 0;
volatile long pulsosDerecha = 0;

// Variables de posición y orientación
float posX = 0.0;  // Posición X en cm
float posY = 0.0;  // Posición Y en cm
float angulo = 0.0;  // Orientación en radianes

// Variables para calcular RPM
unsigned long lastRpmTime = 0;
long lastPulsosIzq = 0;
long lastPulsosDer = 0;
float rpmIzq = 0.0;
float rpmDer = 0.0;

// Variables de control
int velocidadBase = 150;
bool isConnectedToWeb = false;
unsigned long lastDataMsg = 0;
const long dataInterval = 200;  // Enviar datos cada 200ms

// Variables para comandos I y O (movimiento con encoder)
bool modoEncoderIzq = false;
bool modoEncoderDer = false;
long targetPulsosIzq = 0;
long targetPulsosDer = 0;

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ===== FUNCIONES DE INTERRUPCIÓN PARA ENCODERS =====
void IRAM_ATTR contarIzquierda() {
  pulsosIzquierda++;
}

void IRAM_ATTR contarDerecha() {
  pulsosDerecha++;
}

// ===== CALCULAR POSICIÓN (ODOMETRÍA) =====
void calcularPosicion() {
  static long ultimoPulsosIzq = 0;
  static long ultimoPulsosDer = 0;
  
  // Calcular diferencia de pulsos desde última actualización
  long deltaPulsosIzq = pulsosIzquierda - ultimoPulsosIzq;
  long deltaPulsosDer = pulsosDerecha - ultimoPulsosDer;
  
  // Actualizar últimos pulsos
  ultimoPulsosIzq = pulsosIzquierda;
  ultimoPulsosDer = pulsosDerecha;
  
  // Calcular distancia recorrida por cada rueda
  float distIzq = deltaPulsosIzq * CM_POR_PULSO;
  float distDer = deltaPulsosDer * CM_POR_PULSO;
  
  // Calcular distancia y cambio de ángulo
  float distCentro = (distIzq + distDer) / 2.0;
  float deltaAngulo = (distDer - distIzq) / DISTANCIA_ENTRE_RUEDAS;
  
  // Actualizar orientación
  angulo += deltaAngulo;
  
  // Actualizar posición
  posX += distCentro * cos(angulo);
  posY += distCentro * sin(angulo);
}

// ===== CALCULAR RPM =====
void calcularRPM() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastRpmTime) / 1000.0; // Tiempo en segundos
  
  if (deltaTime >= 0.2) {  // Calcular cada 200ms
    // Calcular pulsos en este intervalo
    long deltaPulsosIzq = pulsosIzquierda - lastPulsosIzq;
    long deltaPulsosDer = pulsosDerecha - lastPulsosDer;
    
    // Calcular revoluciones
    float revIzq = deltaPulsosIzq / DIENTES_ENCODER;
    float revDer = deltaPulsosDer / DIENTES_ENCODER;
    
    // Calcular RPM (revoluciones por minuto)
    rpmIzq = (revIzq / deltaTime) * 60.0;
    rpmDer = (revDer / deltaTime) * 60.0;
    
    // Actualizar para próximo cálculo
    lastPulsosIzq = pulsosIzquierda;
    lastPulsosDer = pulsosDerecha;
    lastRpmTime = currentTime;
  }
}

// ===== FUNCIONES DE CONTROL DE MOTORES =====
void motorIzquierdoAdelante(int pwm) {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, pwm);
}

void motorIzquierdoAtras(int pwm) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  analogWrite(MOTOR_L_PWM, pwm);
}

void motorIzquierdoStop() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, 0);
}

void motorDerechoAdelante(int pwm) {
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, pwm);
}

void motorDerechoAtras(int pwm) {
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  analogWrite(MOTOR_R_PWM, pwm);
}

void motorDerechoStop() {
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, 0);
}

void todosLosMotoresStop() {
  motorIzquierdoStop();
  motorDerechoStop();
  modoEncoderIzq = false;
  modoEncoderDer = false;
  Serial.println("⏹️ STOP");
}

void adelante(int pwm) {
  motorIzquierdoAdelante(pwm);
  motorDerechoAdelante(pwm);
  Serial.print("⬆️ ADELANTE PWM:");
  Serial.println(pwm);
}

void atras(int pwm) {
  motorIzquierdoAtras(pwm);
  motorDerechoAtras(pwm);
  Serial.print("⬇️ ATRAS PWM:");
  Serial.println(pwm);
}

void izquierda(int pwm) {
  motorIzquierdoAtras(pwm);
  motorDerechoAdelante(pwm);
  Serial.print("⬅️ IZQUIERDA PWM:");
  Serial.println(pwm);
}

void derecha(int pwm) {
  motorIzquierdoAdelante(pwm);
  motorDerechoAtras(pwm);
  Serial.print("➡️ DERECHA PWM:");
  Serial.println(pwm);
}

// ===== PROCESAR COMANDOS MQTT =====
void processCommand(String message) {
  message.trim();
  message.toUpperCase();
  
  Serial.print("🔧 Comando: ");
  Serial.println(message);
  
  if (message.length() < 1) return;
  
  char comando = message.charAt(0);
  
  // ===== STOP =====
  if (comando == 'S') {
    todosLosMotoresStop();
    client.publish(topicData.c_str(), "STOP");
    return;
  }
  
  // ===== RESET POSICIÓN =====
  if (comando == 'Z') {
    posX = 0.0;
    posY = 0.0;
    angulo = 0.0;
    pulsosIzquierda = 0;
    pulsosDerecha = 0;
    Serial.println("🔄 Posición reseteada");
    client.publish(topicData.c_str(), "RESET_POSICION");
    return;
  }
  
  // ===== VELOCIDAD =====
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
  
  // ===== MOVIMIENTOS BÁSICOS =====
  if (comando == 'F') {
    modoEncoderIzq = false;
    modoEncoderDer = false;
    adelante(velocidadBase);
    client.publish(topicData.c_str(), "ADELANTE");
    return;
  }
  
  if (comando == 'B') {
    modoEncoderIzq = false;
    modoEncoderDer = false;
    atras(velocidadBase);
    client.publish(topicData.c_str(), "ATRAS");
    return;
  }
  
  if (comando == 'L') {
    modoEncoderIzq = false;
    modoEncoderDer = false;
    izquierda(velocidadBase);
    client.publish(topicData.c_str(), "IZQUIERDA");
    return;
  }
  
  if (comando == 'R') {
    modoEncoderIzq = false;
    modoEncoderDer = false;
    derecha(velocidadBase);
    client.publish(topicData.c_str(), "DERECHA");
    return;
  }
  
  // ===== COMANDO I: Motor Izquierdo =====
  if (comando == 'I' && message.indexOf(':') > 0) {
    int sep = message.indexOf(':');
    String pwmStr = message.substring(1, sep);
    String pulsosStr = message.substring(sep + 1);
    
    int pwmVal = pwmStr.toInt();
    targetPulsosIzq = pulsosStr.toInt();
    
    if (targetPulsosIzq > 0) {
      pulsosIzquierda = 0;
      modoEncoderIzq = true;
      
      if (pwmVal > 0) {
        motorIzquierdoAdelante(abs(pwmVal));
      } else if (pwmVal < 0) {
        motorIzquierdoAtras(abs(pwmVal));
      }
      
      Serial.print("🔵 IZQ PWM:");
      Serial.print(pwmVal);
      Serial.print(" Target:");
      Serial.println(targetPulsosIzq);
      
      String resp = "I_INICIO:PWM=" + String(pwmVal) + ",TARGET=" + String(targetPulsosIzq);
      client.publish(topicData.c_str(), resp.c_str());
    }
    return;
  }
  
  // ===== COMANDO O: Motor Derecho =====
  if (comando == 'O' && message.indexOf(':') > 0) {
    int sep = message.indexOf(':');
    String pwmStr = message.substring(1, sep);
    String pulsosStr = message.substring(sep + 1);
    
    int pwmVal = pwmStr.toInt();
    targetPulsosDer = pulsosStr.toInt();
    
    if (targetPulsosDer > 0) {
      pulsosDerecha = 0;
      modoEncoderDer = true;
      
      if (pwmVal > 0) {
        motorDerechoAdelante(abs(pwmVal));
      } else if (pwmVal < 0) {
        motorDerechoAtras(abs(pwmVal));
      }
      
      Serial.print("🔴 DER PWM:");
      Serial.print(pwmVal);
      Serial.print(" Target:");
      Serial.println(targetPulsosDer);
      
      String resp = "O_INICIO:PWM=" + String(pwmVal) + ",TARGET=" + String(targetPulsosDer);
      client.publish(topicData.c_str(), resp.c_str());
    }
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
  
  Serial.println("\n🤖 ROBOT CON ODOMETRÍA Y RPM");
  Serial.println("========================================");
  
  // Mostrar parámetros físicos
  Serial.print("Dientes encoder: ");
  Serial.println(DIENTES_ENCODER);
  Serial.print("Diámetro rueda: ");
  Serial.print(DIAMETRO_RUEDA);
  Serial.println(" cm");
  Serial.print("cm por pulso: ");
  Serial.println(CM_POR_PULSO, 4);
  Serial.println();
  
  // Topics
  topicCmd = robotName + "/cmd";
  topicData = robotName + "/data";
  topicStatus = robotName + "/status";
  
  Serial.println("CMD: " + topicCmd);
  Serial.println("DATA: " + topicData);
  
  // Encoders
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), contarIzquierda, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), contarDerecha, RISING);
  Serial.println("✅ Encoders");
  
  // Motores
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  todosLosMotoresStop();
  Serial.println("✅ Motores");
  
  // WiFi y MQTT
  setup_wifi();
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  // Inicializar tiempo para RPM
  lastRpmTime = millis();
  
  Serial.println("✅ Listo\n");
}

// ===== LOOP =====
void loop() {
  // MQTT
  if (!client.connected()) {
    isConnectedToWeb = false;
    reconnect();
  }
  client.loop();
  
  // Actualizar odometría
  calcularPosicion();
  
  // Calcular RPM
  calcularRPM();
  
  // ===== VERIFICAR ENCODERS EN MODO I/O =====
  if (modoEncoderIzq && pulsosIzquierda >= targetPulsosIzq) {
    motorIzquierdoStop();
    Serial.print("✅ IZQ COMPLETO: ");
    Serial.println(pulsosIzquierda);
    
    String msg = "I_COMPLETO:" + String(pulsosIzquierda);
    client.publish(topicData.c_str(), msg.c_str());
    
    modoEncoderIzq = false;
    targetPulsosIzq = 0;
  }
  
  if (modoEncoderDer && pulsosDerecha >= targetPulsosDer) {
    motorDerechoStop();
    Serial.print("✅ DER COMPLETO: ");
    Serial.println(pulsosDerecha);
    
    String msg = "O_COMPLETO:" + String(pulsosDerecha);
    client.publish(topicData.c_str(), msg.c_str());
    
    modoEncoderDer = false;
    targetPulsosDer = 0;
  }
  
  // ===== ENVIAR COORDENADAS Y RPM CADA 200ms =====
  unsigned long now = millis();
  if (now - lastDataMsg >= dataInterval && isConnectedToWeb) {
    lastDataMsg = now;
    
    // FORMATO PARA VISUALIZACIÓN DE TRAYECTORIA: X,Y
    String coordenadas = String(posX, 2) + "," + String(posY, 2);
    client.publish(topicData.c_str(), coordenadas.c_str());
    
    // ENVIAR RPM Y DATOS ADICIONALES AL TOPIC STATUS
    String status = "RPM_IZQ:" + String(rpmIzq, 1) + 
                    "|RPM_DER:" + String(rpmDer, 1) +
                    "|X:" + String(posX, 2) + 
                    "|Y:" + String(posY, 2) +
                    "|ANGULO:" + String(angulo * 180.0 / PI, 1);
    client.publish(topicStatus.c_str(), status.c_str());
    
    // También en Serial para debugging
    Serial.print("Pos: (");
    Serial.print(posX, 2);
    Serial.print(", ");
    Serial.print(posY, 2);
    Serial.print(") | RPM: IZQ=");
    Serial.print(rpmIzq, 1);
    Serial.print(" DER=");
    Serial.println(rpmDer, 1);
  }
}
