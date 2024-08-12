#define FILTER_SIZE 10 // Tamanho do filtro de média móvel

float rpmA_history[FILTER_SIZE]; // Histórico de leituras do motor A
float rpmB_history[FILTER_SIZE]; // Histórico de leituras do motor B
int rpm_index = 0; // Índice para o filtro de média móvel

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

  // Inicializar os valores do histórico com zero
  for (int i = 0; i < FILTER_SIZE; i++) {
    rpmA_history[i] = 0.0;
    rpmB_history[i] = 0.0;
  }
}

float calculateFilteredRPM(float rpm, float* history) {
  // Atualizar o histórico de leituras
  history[rpm_index] = rpm;

  // Calcular a média das leituras
  float sum = 0.0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += history[i];
  }

  // Retornar a média como o valor filtrado
  return sum / FILTER_SIZE;
}

void loop() {
  // Calcular o RPM de cada motor
  float rpmA = RPM(1);
  float rpmB = RPM(2);

  // Aplicar o filtro de média móvel
  float filteredRPM_A = calculateFilteredRPM(rpmA, rpmA_history);
  float filteredRPM_B = calculateFilteredRPM(rpmB, rpmB_history);

  // Atualizar o índice para o próximo valor
  rpm_index = (rpm_index + 1) % FILTER_SIZE;

  // Imprimir o RPM filtrado de cada motor
  Serial.print("RPM Motor A (filtrado): ");
  Serial.println(filteredRPM_A);
  Serial.print("RPM Motor B (filtrado): ");
  Serial.println(filteredRPM_B);

  // Movimentar os motores se a distância for menor que 100 cm
  if (distance > 10) {
    // Mover motor A para frente
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255); // Velocidade do motor A
    
    // Mover motor B para frente
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 255); // Velocidade do motor B
  } else {
    // Parar os motores
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  delay(200); // Pequeno atraso para estabilizar a leitura
}
