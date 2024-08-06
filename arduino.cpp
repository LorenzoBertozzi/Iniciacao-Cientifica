int i = 0; 

int trigPin = 10;    // Disparador (Trigger)
int echoPin = 11;    // Eco (Echo)
long duration, cm, inches;

//declaracao dos pinos utilizados para controlar a velocidade de rotacao
const int PINO_ENA = 4; 
const int PINO_ENB = 9;

//declaracao dos pinos utilizados para controlar o sentido do motor
const int PINO_IN1 = 5; 
const int PINO_IN2 = 6;
const int PINO_IN3 = 7;
const int PINO_IN4 = 8;

int encoder_pin = 2;                        
float rpm = 0;                          
float velocity = 0;                     
volatile float pulses = 0;              
unsigned long timeold = 0;              
unsigned int pulsesperturn = 8;         
unsigned int wheel_diameter = 66;       
int ratio = 120;                           
 
void counter(){
  pulses++; 
}  

void setup() {
  //Inicia el Serial Port
  Serial.begin (9600);
  //Define entradas y salidas
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(encoder_pin, INPUT);
  attachInterrupt(0, counter, RISING);

  pulses = 0;                          
  rpm = 0;
  timeold = 0;
  Serial.println("");
  Serial.print("            MOTOR A       ");
  Serial.print("MOTOR       ");
  Serial.print("WHEEL       "); 
  Serial.println("WHEEL");
  Serial.print("Seconds     ");
  Serial.print("Pulses      ");
  Serial.print("RPM         ");
  Serial.print("RPM         ");
  Serial.println("Velocity[Km/h]");

  pinMode(PINO_ENA, OUTPUT); 
  pinMode(PINO_ENB, OUTPUT);
  pinMode(PINO_IN1, OUTPUT);
  pinMode(PINO_IN2, OUTPUT);
  pinMode(PINO_IN3, OUTPUT);
  pinMode(PINO_IN4, OUTPUT);

  digitalWrite(PINO_IN1, LOW); 
  digitalWrite(PINO_IN2, LOW);
  digitalWrite(PINO_IN3, LOW);
  digitalWrite(PINO_IN4, LOW);
  digitalWrite(PINO_ENA, LOW);
  digitalWrite(PINO_ENB, LOW);
}
 
void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  digitalWrite(PINO_IN1, LOW); 
  digitalWrite(PINO_IN2, HIGH);
  digitalWrite(PINO_IN3, HIGH);
  digitalWrite(PINO_IN4, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = (duration/2) / 29.1;

  noInterrupts();                // Desconectamos la interrupción para que no actué en esta parte del programa.
  rpm = 60 * pulses / pulsesperturn * 1000 / (millis() - timeold) ; // Calculamos las revoluciones por minuto
  //Ojo con la fórmula de arriba, la variable rpm tiene que ser tipo float porque salen decimales en medio de la operación.
  velocity = rpm/ratio * 3.1416 * wheel_diameter * 60 / 1000000; // Cálculo de la velocidad de la rueda en [Km/h]
  Serial.print(millis()/1000); Serial.print("             "); // Se envia al puerto serie el valor de tiempo, de las rpm y los pulsos.
  Serial.print(pulses,0); Serial.print("          ");
  Serial.print(rpm,0); Serial.print("         ");
  Serial.print(rpm/ratio,1); Serial.print("           ");
  Serial.print(velocity,2);Serial.print("           ");
  Serial.println(cm);
  pulses = 0;                    // Inicializamos los pulsos.
  timeold = millis();            // Almacenamos el tiempo actual.
  interrupts();                  // Reiniciamos la interrupción
  
  
  if (i < 256 && cm > 100){ 
    analogWrite(PINO_ENA, i);
    analogWrite(PINO_ENB, i);
    delay(30); //intervalo para incrementar a variavel i
    i=i+10;
    
  }
  
  if(cm < 100){ 
    i=0;
    analogWrite(PINO_ENA, LOW);
    analogWrite(PINO_ENB, LOW);
        
    //tenta desviar
  }
  

}
