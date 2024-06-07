#include <ros.h>
#include <std_msgs/Float32.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

// Definições de pinos para o motor da cabeça
#define STEP_PIN1 12
#define DIR_PIN1 14
#define SLEEP_PIN1 13

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);

float distancia_x = 0;
float distancia_y = 0;
bool shouldRunStepper = false;

void centerXCallback(const std_msgs::Float32& msg) {
  distancia_x = msg.data;
  
  if (distancia_x > 100) {
    digitalWrite(2, HIGH);  // Acende o LED no pino 2
    digitalWrite(SLEEP_PIN1, HIGH);
    stepper1.setSpeed(30);
    shouldRunStepper = true;  // Permite que o motor rode no loop
  } else if (distancia_x < -100) {
    digitalWrite(2, HIGH);  // Acende o LED no pino 2
    digitalWrite(SLEEP_PIN1, HIGH);
    stepper1.setSpeed(-30); // Velocidade negativa para inverter o sentido
    shouldRunStepper = true;  // Permite que o motor rode no loop
  } else {
    digitalWrite(2, LOW);   // Apaga o LED no pino 2
    digitalWrite(SLEEP_PIN1, LOW);
    shouldRunStepper = false; // Impede que o motor rode no loop
  }
}

void centerYCallback(const std_msgs::Float32& msg) {
  distancia_y = msg.data;
  Serial.print("Distância Y: ");
  Serial.println(distancia_y);
}

ros::Subscriber<std_msgs::Float32> sub_x("/face_center_x", &centerXCallback);
ros::Subscriber<std_msgs::Float32> sub_y("/face_center_y", &centerYCallback);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_x);
  nh.subscribe(sub_y);
  
  // Configura os pinos de sono como saída e os define como 1 (desativados)
  pinMode(SLEEP_PIN1, OUTPUT);
  digitalWrite(SLEEP_PIN1, LOW);
  
  // Configurações iniciais do motor
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(200);
  stepper1.setSpeed(0); // Começa parado

  pinMode(2, OUTPUT);  // Configura o pino 2 como saída para controlar o LED
}

void loop() {
  nh.spinOnce();
  if (shouldRunStepper) {
    stepper1.runSpeed(); // Esta função deve ser chamada para manter o motor em movimento constante
  }
  delay(1);
}
