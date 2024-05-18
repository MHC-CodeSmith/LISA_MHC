#include <Arduino.h>
#include <AccelStepper.h>

// Definições de pinos para o motor
#define STEP_PIN1 14
#define DIR_PIN1 12

#define IN21 5
#define IN22 18
#define IN23 19
#define IN24 21

//para esse motor, a ordem dos fios no driver fica cruzado (os dois do meio) ao definir os INX2 e INX3
#define IN31 26
#define IN32 33
#define IN33 25
#define IN34 32

#define IN41 23
#define IN42 22
#define IN43 4
#define IN44 2

//#define IN51 13
//#define IN52 15
//#define IN53 34
//#define IN54 35

#define LED_BUILTIN 2

// Criação do objeto stepper

AccelStepper stepper1(1, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::FULL4WIRE, IN21, IN22, IN23, IN24);
AccelStepper stepper3(8, IN31, IN32, IN33, IN34);
AccelStepper stepper4(8, IN41, IN43, IN42, IN44);
//AccelStepper stepper5(AccelStepper::FULL4WIRE, IN51, IN52, IN53, IN54);

void setup() {
  // Inicia a comunicação serial com 115200 baud rate.
  Serial.begin(115200);
  
  // Configura o pino do LED como saída
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Configurações iniciais do motor
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper1.moveTo(0);

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper2.moveTo(0);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);
  stepper3.moveTo(0);

  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(500);
  stepper4.moveTo(0);

 // stepper5.setMaxSpeed(1000);
 // stepper5.setAcceleration(500);
 // stepper5.moveTo(0);

  // 500 passos por segundo
  stepper1.setSpeed(500); 
  stepper2.setSpeed(500);
  stepper3.setSpeed(1000);
  stepper4.setSpeed(1000);
  //stepper5.setSpeed(500);
}

void loop() {
  // Movimenta o motor a uma velocidade constante
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepper4.runSpeed();
 // stepper5.runSpeed();

  static unsigned long lastMillis = 0;
  if (millis() - lastMillis > 5000) {
    stepper1.setSpeed(-stepper1.speed()); // Inverte a direção
    stepper2.setSpeed(-stepper2.speed());
    stepper3.setSpeed(-stepper3.speed());
    stepper4.setSpeed(-stepper4.speed());
   // stepper5.setSpeed(-stepper5.speed());
    lastMillis = millis();
  }
  /*
  Serial.println("comecando...");

  stepper1.setSpeed(500); 
  stepper1.runSpeed();
  static unsigned long lastMillis = 0;
  while(millis() - lastMillis > 2000) {
    lastMillis = millis();
    Serial.println(millis() - lastMillis);
  }
  lastMillis = 0;
  stepper1.setSpeed(0);
  while(millis() - lastMillis < 2000) {
    lastMillis = millis();
  }*/
}
