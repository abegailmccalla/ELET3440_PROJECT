// The University of the West Indies
// ELET3440
// Abegail McCalla (620157646)
// Chassis Assignment 1

#include <avr/wdt.h>
#include <arduino.h>
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN_1 8
#define PIN_Motor_BIN_1 7
#define PIN_Motor_Enable 3  // Define the enable pin
#define speed_Max 255
#define speed_Reg 125
#define trigPin 13
#define echoPin 12
int mode = 0;
float distance = 0.0;

void setup() {
  // Initialize motor control pins
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_Enable, OUTPUT); // Set the enable pin as output
  Serial.begin(9600);
  // Enable the motors
  digitalWrite(PIN_Motor_Enable, HIGH);

  //Radar 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

// Mode 5: Stop
void motorStop() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0); 
}

void stop_10cm (){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 80000);
  distance = duration * 0.0343 / 2;
  if (distance <= 10)
  {
    motorStop();
  	//delay(5000);
  }
}

// Mode 1: Move forward
void moveForward(int speed) {
  for (speed = speed; speed >= 0; speed--) {
    digitalWrite(PIN_Motor_AIN_1, HIGH);
    analogWrite(PIN_Motor_PWMA, speed); // Adjust left motor speed
    digitalWrite(PIN_Motor_BIN_1, HIGH);
    analogWrite(PIN_Motor_PWMB, speed); // Adjust right motor speed
    //stop_10cm();
    delay(50); // Pause briefly to make the reduction gradual
  }

  // digitalWrite(PIN_Motor_AIN_1, HIGH);  
  // analogWrite(PIN_Motor_PWMA, speed); // Set left motor speed
  // digitalWrite(PIN_Motor_BIN_1, HIGH); 
  // analogWrite(PIN_Motor_PWMB, speed); // Set right motor speed
  // delay(3000);
}

// Mode 2: Move back 
void moveBack(int speed) {
  for (speed = speed; speed >= 0; speed--) {
    digitalWrite(PIN_Motor_AIN_1, LOW);
    analogWrite(PIN_Motor_PWMA, speed); // Adjust left motor speed
    digitalWrite(PIN_Motor_BIN_1, LOW);
    analogWrite(PIN_Motor_PWMB, speed); // Adjust right motor speed
    delay(50); // Pause briefly to make the reduction gradual
  }

  // digitalWrite(PIN_Motor_AIN_1, LOW);  
  // analogWrite(PIN_Motor_PWMA, speed); 
  // digitalWrite(PIN_Motor_BIN_1, LOW); 
  // analogWrite(PIN_Motor_PWMB, speed); 
  // delay(3000);
}

// Mode 3: Move left
void turnLeft(int speed) {
  digitalWrite(PIN_Motor_AIN_1, LOW);  
  analogWrite(PIN_Motor_PWMA, speed); 
  digitalWrite(PIN_Motor_BIN_1, HIGH); 
  analogWrite(PIN_Motor_PWMB, speed); 
  delay(500);
}

// Mode 4: Move right
void turnRight(int speed) {
  digitalWrite(PIN_Motor_AIN_1, HIGH);  
  analogWrite(PIN_Motor_PWMA, speed); 
  digitalWrite(PIN_Motor_BIN_1, LOW); 
  analogWrite(PIN_Motor_PWMB, speed); 
  delay(500);  
}

void loop() {
  // A == LEFT, B == RIGHT, HIGH == Forward, LOW == Back

  // // Move right forward
  // digitalWrite(PIN_Motor_AIN_1, HIGH);  
  // analogWrite(PIN_Motor_PWMA, speed_Reg/2);
  // digitalWrite(PIN_Motor_BIN_1, HIGH); 
  // analogWrite(PIN_Motor_PWMB, speed_Reg); 
  // delay(3000); 

  // // Move left back
  // digitalWrite(PIN_Motor_AIN_1, LOW);  
  // analogWrite(PIN_Motor_PWMA, speed_Reg); 
  // digitalWrite(PIN_Motor_BIN_1, LOW); 
  // analogWrite(PIN_Motor_PWMB, speed_Reg/2); 
  // delay(3000);

  // // Move left forward
  // digitalWrite(PIN_Motor_AIN_1, HIGH);  
  // analogWrite(PIN_Motor_PWMA, speed_Reg);
  // digitalWrite(PIN_Motor_BIN_1, HIGH); 
  // analogWrite(PIN_Motor_PWMB, speed_Reg/2); 
  // delay(3000); 

  // // Move right back
  // digitalWrite(PIN_Motor_AIN_1, LOW);  
  // analogWrite(PIN_Motor_PWMA, speed_Reg/2); 
  // digitalWrite(PIN_Motor_BIN_1, LOW); 
  // analogWrite(PIN_Motor_PWMB, speed_Reg); 
  // delay(3000);

  // mode = random(0, 5); // Simulating button states (0 to 4)
  if (mode < 5){
    mode++;
  }
  else{
    mode = 0;
  }
  

  switch (mode) {
    case 1:
      moveForward(speed_Reg);
      break;
    case 2:
      moveBack(speed_Reg);
      break;
    case 3:
      turnLeft(speed_Reg);
      break;
    case 4:
      turnRight(speed_Reg);
      break;
    case 5:
      motorStop();
      delay(1000); 
      break;
  }
}

