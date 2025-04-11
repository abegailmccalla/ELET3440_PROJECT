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
float duration = 0.0;

// #################################### SET UP #########################################

void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_Enable, OUTPUT); // Set the enable pin as output
  Serial.begin(9600);
  // Enable the motors
  digitalWrite(PIN_Motor_Enable, HIGH);
  delay(100);
  Serial.println("setup ready"); 
}

// #################################### LOOP #########################################

void loop() {
    // Check for IR signal
    getFwd();
}

void getFwd() {
  if (mode == 0){
    moveForward(100);
  }
}

// ###################################### GET DISTANCE ################################################
int get_distance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 80000);
  // Calculating the distance
  distance = (duration * 0.034 / 2);
  // Serial.print("Loop Distance: ");
  // Serial.print(distance);
  // Serial.println(" cm");
  // delay(30);
  delay(10);
  return distance;
}

// ##################################### MOVE DIRECTION #############################

void moveForward(int speed) //Open Straight-line Paths 
{
  distance = get_distance();
  while (distance > 10)  
  {
    digitalWrite(PIN_Motor_AIN_1, HIGH);
    digitalWrite(PIN_Motor_BIN_1, HIGH);
    analogWrite(PIN_Motor_PWMA, speed);
    analogWrite(PIN_Motor_PWMB, speed);
    distance = get_distance();
    Serial.print("Distance: ");
    Serial.println(distance);
  }
  motorStop();  // Stop if output is close to 0
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(30); // Prevent sensor overload
}

void motorStop() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0); 
}
