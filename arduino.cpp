int i = 0; 
float tacografoA, tacografoB = 0.0;

int trigPin = 10;    // Disparador (Trigger)
int echoPin = 11;    // Eco (Echo)
long duration, cm, inches;

//declaracao dos pinos utilizados para controlar a velocidade de rotacao
const int PINO_ENA = 6;
const int PINO_ENB = 5;


//declaracao dos pinos utilizados para controlar o sentido do motor
const int PINO_IN1 = 4;
const int PINO_IN2 = 9;
const int PINO_IN3 = 8;
const int PINO_IN4 = 7;

int encoderA_pin = 2;                        
float rpmA = 0;                          
float velocityA = 0;                     
volatile float pulsesA = 0;     
unsigned long timeoldA = 0;           
  
int encoderB_pin = 13;                        
float rpmB = 0;                          
float velocityB = 0;                     
volatile float pulsesB = 0; 
unsigned long timeoldB = 0;   

unsigned int pulsesperturn = 8;         
unsigned int wheel_diameter = 66;       
int ratio = 120;                           
 
void counter(){
  pulsesA++; pulsesB++; 
}  

void tittle(){
  Serial.println("");
  Serial.println("            MOTOR A                                                                              MOTOR B");
  Serial.print("Seconds     ");
  Serial.print("Pulses      ");
  Serial.print("RPM         ");
  Serial.print("Velocity[Km/h]         ");
  Serial.print("distancia S      ");
  Serial.print("Pulses      ");
  Serial.print("RPM         ");
  Serial.println("Velocity[Km/h]         ");
}

void setup() {
  Serial.begin (9600);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(encoderA_pin, INPUT);
  pinMode(encoderB_pin, INPUT);
  attachInterrupt(0, counter, RISING);

  pulsesA, pulsesB = 0;                          
  rpmA = 0;
  rpmB = 0;
  timeoldA = 0;
  timeoldB = 0;

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

  noInterrupts();               
  rpmA = 60 * pulsesA / pulsesperturn * 1000 / (millis() - timeoldA) ;
  velocityA = rpmA/ratio * 3.1416 * wheel_diameter * 60 / 1000000; 
  tacografoA = tacografoA + (rpmA/ratio * 3.1416 * wheel_diameter /1000000);

  rpmB = 60 * pulsesB / pulsesperturn * 1000 / (millis() - timeoldB) ;
  velocityB = rpmB/ratio * 3.1416 * wheel_diameter * 60 / 1000000; 
  tacografoB = tacografoB + (rpmB/ratio * 3.1416 * wheel_diameter /1000000);

  tittle();
  Serial.print(millis()/1000); Serial.print("           "); 
  Serial.print(pulsesA,0); Serial.print("        ");
  Serial.print(rpmA/ratio,1); Serial.print("           ");
  Serial.print(velocityA,2);Serial.print("                       ");
  Serial.print(cm);Serial.print("                       ");
  Serial.print(pulsesB,0); Serial.print("        ");
  Serial.print(rpmB/ratio,1); Serial.print("           ");
  Serial.println(velocityB,2);

  Serial.print("distancia (A e B) : ");
  Serial.print(tacografoA/ratio);
  Serial.print(" : ");
  Serial.println(tacografoB/ratio);

  pulsesA = 0;                   
  timeoldA = millis();   
  pulsesB = 0;                   
  timeoldB = millis();         
  interrupts();                
  
  if (cm > 50){ 
    analogWrite(PINO_ENA, i);
    analogWrite(PINO_ENB, i);
    if(i < 256){
      i=i+20;
    }
  }
  if(cm < 50){ 
    i = 0;
    analogWrite(PINO_ENA, LOW);
    analogWrite(PINO_ENB, 100);
        
    //tenta desviar
    /*
    se encontrarmor um obstaculo
    desviamos para a esquerda
    roda direita = high
      viramos ate poder seguir em frente 
      quardamos quanto o carrinho virou
        seguimos em frente 
          viramos para a direita mesma quantidade que para a esquerda
          seguimos em frente 
          viramos para a direita mesma quantidade que para a esquerda
        seguimos em frente
      viramos a mesma quantidade para a direita

    */
  }
  

}
