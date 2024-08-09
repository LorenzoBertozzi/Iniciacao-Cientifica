//definição pinos ponte H L298N
#define ENA 5
#define IN1 4
#define IN2 9
#define IN3 8
#define IN4 7
#define ENB 6

// Definições dos pinos dos encoders do Motor 1
#define encoderA1_pin 2
#define encoderA2_pin 3

// Definições dos pinos dos encoders do Motor 2
#define encoderB1_pin 13
#define encoderB2_pin 12

#define MAX_PWM 255 // Valor máximo de PWM para os motores
#define WHEEL_DIAMETER 0.065 // Diâmetro da roda em metros (65 mm)
#define WHEEL_BASE 0.15 // Distância entre os eixos das rodas em metros (ajuste conforme necessário)

// Definições dos pinos do sensor ultrassônico
#define trigPin 10
#define echoPin 11

// Variáveis de contagem dos encoders
volatile long encoderA1_count = 0;
volatile long encoderA2_count = 0;
volatile long encoderB1_count = 0;
volatile long encoderB2_count = 0;

// Variáveis de tempo para cálculo de RPM
unsigned long previousMillisA = 0;
unsigned long previousMillisB = 0;

// Funções de interrupção para os encoders do Motor 1
void encoderA1_ISR() {
  encoderA1_count++;
}

void encoderA2_ISR() {
  encoderA2_count++;
}

// Funções de interrupção para os encoders do Motor 2
void encoderB_ISR() {
  // Use a função para ambos os pinos 12 e 13, pois estamos usando PCINT
  if (digitalRead(encoderB1_pin) == HIGH) {
    encoderB1_count++;
  }
  if (digitalRead(encoderB2_pin) == HIGH) {
    encoderB2_count++;
  }
}

// Função para calcular RPM
float RPM(int motor) {
  unsigned long currentMillis = millis();
  float rpm = 0;

  if (motor == 1) {
    // Cálculo da RPM para o Motor A
    unsigned long elapsedMillisA = currentMillis - previousMillisA;
    previousMillisA = currentMillis;

    // Considera o encoder do canal 1 (encoderA1_count) para cálculo
    rpm = (encoderA1_count / 11.0) * (600.0  / elapsedMillisA); // 8 pulsos por rotação (ajuste conforme necessário)
    
    // Resetar a contagem para o próximo cálculo
    encoderA1_count = 0;
  } else if (motor == 2) {
    // Cálculo da RPM para o Motor B
    unsigned long elapsedMillisB = currentMillis - previousMillisB;
    previousMillisB = currentMillis;

    // Considera o encoder do canal 1 (encoderB1_count) para cálculo
    rpm = (encoderB1_count / 11.0) * (600.0 / elapsedMillisB); // 8 pulsos por rotação (ajuste conforme necessário)
    
    // Resetar a contagem para o próximo cálculo
    encoderB1_count = 0;
  }

  return rpm;
}

float RPM_to_kmph(float rpm) {
  // Calcular a circunferência da roda
  float wheelCircumference = PI * WHEEL_DIAMETER; // Circunferência em metros

  // Calcular a velocidade em km/h
  float speedKmph = (rpm * wheelCircumference * 60) / 1000; // Convertendo para km/h

  return speedKmph;
}

// Função para converter a velocidade desejada em km/h para RPM
float speedToRPM(float speedKmph) {
  // Calcular a circunferência da roda
  float wheelCircumference = PI * WHEEL_DIAMETER; // Circunferência em metros

  // Calcular a RPM correspondente
  float rpm = (speedKmph * 1000) / (wheelCircumference * 60); // Convertendo para RPM

  return rpm;
}

// Função para converter RPM em um valor PWM
int rpmToPWM(float rpm) {
  // Definir a RPM máxima que corresponde ao valor máximo de PWM
  float maxRPM = 1000; // Ajuste conforme necessário
  int pwm = map(rpm, 0, maxRPM, 0, MAX_PWM);
  pwm = constrain(pwm, 0, MAX_PWM); // Garantir que o valor de PWM esteja dentro do intervalo

  return pwm;
}

// Função para definir a velocidade dos motores
void setMotorSpeed(float desiredSpeedKmph) {
  float rpm = speedToRPM(desiredSpeedKmph);
  int pwm = rpmToPWM(rpm);

  // Aplicar PWM aos motores
  analogWrite(ENA, pwm); // Ajuste para o Motor A
  analogWrite(ENB, pwm); // Ajuste para o Motor B

  // Definir a direção dos motores se necessário
  digitalWrite(IN1, HIGH); // Para frente
  digitalWrite(IN2, LOW);  // Para frente
  digitalWrite(IN3, HIGH); // Para frente
  digitalWrite(IN4, LOW);  // Para frente
}

void setMotorSpeed(float linearSpeed, float angularSpeed) {
  // Calcular a velocidade linear das rodas internas e externas
  float radius = WHEEL_BASE / 2.0;
  float V_inner = linearSpeed - (angularSpeed * radius);
  float V_outer = linearSpeed + (angularSpeed * radius);

  // Converter a velocidade linear das rodas para PWM
  int pwm_inner = rpmToPWM(speedToRPM(V_inner * 3.6)); // Convertendo km/h para RPM
  int pwm_outer = rpmToPWM(speedToRPM(V_outer * 3.6)); // Convertendo km/h para RPM

  // Aplicar PWM aos motores
  analogWrite(ENA, pwm_inner); // Ajuste para o Motor A (interna)
  analogWrite(ENB, pwm_outer); // Ajuste para o Motor B (externa)

  // Definir a direção dos motores se necessário
  digitalWrite(IN1, HIGH); // Para frente
  digitalWrite(IN2, LOW);  // Para frente
  digitalWrite(IN3, HIGH); // Para frente
  digitalWrite(IN4, LOW);  // Para frente
}

void setup() {
  // Configuração dos pinos da ponte H
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Configuração dos pinos dos encoders do Motor 1
  pinMode(encoderA1_pin, INPUT);
  pinMode(encoderA2_pin, INPUT);
  
  // Configuração dos pinos dos encoders do Motor 2
  pinMode(encoderB1_pin, INPUT);
  pinMode(encoderB2_pin, INPUT);

  // Ativando interrupções para os encoders do Motor 1
  attachInterrupt(digitalPinToInterrupt(encoderA1_pin), encoderA1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA2_pin), encoderA2_ISR, RISING);

  // Ativando Pin Change Interrupt para os encoders do Motor 2 (pinos 12 e 13)
  PCICR |= (1 << PCIE0); // Ativar interrupções para PCINT[7:0]
  PCMSK0 |= (1 << PCINT4) | (1 << PCINT5); // Ativar interrupção para pinos 12 e 13

  // Configuração dos pinos do sensor ultrassônico
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Iniciar comunicação serial
  Serial.begin(9600);
}

// Função de interrupção para PCINT0_vect (pinos 12 e 13)
ISR(PCINT0_vect) {
  encoderB_ISR();
}

void loop() {
  // Medir a distância usando o sensor ultrassônico
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Imprimir a distância medida
  Serial.print("Distância: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Imprimir contagem dos encoders do Motor 1
  Serial.print("Motor A - Canal 1: ");
  Serial.print(encoderA1_count);
  Serial.print(" | Canal 2: ");
  Serial.println(encoderA2_count);

  // Imprimir contagem dos encoders do Motor 2
  Serial.print("Motor B - Canal 1: ");
  Serial.print(encoderB1_count);
  Serial.print(" | Canal 2: ");
  Serial.println(encoderB2_count);

  // Calcular e imprimir a RPM de cada motor
  Serial.print("RPM Motor A: ");
  Serial.println(RPM(1));
  Serial.print("RPM Motor B: ");
  Serial.println(RPM(2));

  // Imprimir a velocidade dos motores em km/h
  Serial.print("Velocidade Motor A: ");
  Serial.print(RPM_to_kmph(rpmA));
  Serial.println(" km/h");

  Serial.print("Velocidade Motor B: ");
  Serial.print(RPM_to_kmph(rpmB));
  Serial.println(" km/h");

  // Movimentar os motores se a distância for menor que 100 cm
  if (distance > 30) {
    // Mover motor A para frente
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255); // velocidade do motor A
    
    // Mover motor B para frente
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255); // velocidade do motor B
  } else {
    // Parar os motores
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  //setMotorSpeed(1.0);

  delay(100); // Pequeno atraso para estabilizar a leitura
}
