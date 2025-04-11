#include <Servo.h>

#define DELAY_MS 10

// Servo objects
Servo baseServo, lowerServo, upperServo, gripperServo;

// Servo pins
const int BASE_SERVO_PIN = 3;
const int LOWER_SERVO_PIN = 5;
const int UPPER_SERVO_PIN = 6;
const int GRIPPER_SERVO_PIN = 9;

// Joystick pins
const int CTRL_LEFT_X = A2;
const int CTRL_LEFT_Y = A1; 
const int CTRL_RIGHT_X = A5;
const int CTRL_RIGHT_Y = A4;
const int CTRL_LEFT_SW = 11;  // Left joystick button
const int CTRL_RIGHT_SW = 2; // Right joystick button

// Servo angle limits
const int BASE_MIN = 0, BASE_MAX = 180;
const int LOWER_MIN = 45, LOWER_MAX = 175;
const int UPPER_MIN = 50, UPPER_MAX = 180; //90;
const int GRIPPER_MIN = 60, GRIPPER_MAX = 33;

// Store current angles
int baseAngle = 90;
int lowerAngle = 50; //50;
int upperAngle = 100;
int gripperAngle = 30; //3

// Function to move servo smoothly

void setup() {
  Serial.begin(9600);

  // Attach servos
  baseServo.attach(BASE_SERVO_PIN);
  lowerServo.attach(LOWER_SERVO_PIN);
  upperServo.attach(UPPER_SERVO_PIN);
  gripperServo.attach(GRIPPER_SERVO_PIN);

  // Set initial positions
  baseServo.write(baseAngle);
  lowerServo.write(lowerAngle);
  upperServo.write(upperAngle);
  gripperServo.write(gripperAngle);

  // Setup joystick buttons
  pinMode(CTRL_LEFT_SW, INPUT_PULLUP);
  //digitalWrite(CTRL_LEFT_SW, HIGH);
  pinMode(CTRL_RIGHT_SW, INPUT_PULLUP);
  //digitalWrite(CTRL_RIGHT_SW, HIGH);
}

void loop() {
  // Read joystick values
  int baseInput = analogRead(CTRL_LEFT_X);
  int upperInput = analogRead(CTRL_LEFT_Y);
  int lowerInput = analogRead(CTRL_RIGHT_Y);
  int gripperInput = analogRead(CTRL_RIGHT_X);
  int rightButton = digitalRead(CTRL_RIGHT_SW);
  int leftButton = digitalRead(CTRL_LEFT_SW);

 if(rightButton == LOW){
  Serial.println("RIGHT BUTTON");
 }
 else if (leftButton == LOW){
  Serial.println("LEFT BUTTON");
 }
  
  if(abs(baseInput - 495) > 15) {
    int change = (baseInput < 495) ? -3 : 3;
    baseAngle += change;
    if (baseAngle < BASE_MIN) {
      baseAngle = BASE_MIN;
    } else if (baseAngle > BASE_MAX) {
      baseAngle = BASE_MAX;
    }
    baseServo.write(baseAngle);
    Serial.print("Base angle: ");
    Serial.println(baseAngle);
    Serial.print("Base input: ");
    Serial.println(baseInput);

  }

  if(abs(lowerInput - 520) > 15) {
    int change = (lowerInput < 520) ? -3 : 3;
    
    lowerAngle += change;
    if (lowerAngle < LOWER_MIN) {
      lowerAngle = LOWER_MIN;
    } else if (lowerAngle > LOWER_MAX) {
      lowerAngle = LOWER_MAX;
    }

    lowerServo.write(lowerAngle);
    Serial.print("Lower angle: ");
    Serial.println(lowerAngle);
    Serial.print("Lower input: ");
    Serial.println(lowerInput);
   
  }

  if(abs(upperInput - 500) > 15) {
    int change = (upperInput < 500) ? -3 : 3;
    upperAngle += change;
    if (upperAngle < UPPER_MIN) {
      upperAngle = UPPER_MIN;
    } else if (upperAngle > UPPER_MAX) {
      upperAngle = UPPER_MAX;
    }
    upperServo.write(upperAngle);
    Serial.print("Upper angle: ");
    Serial.println(upperAngle);
    Serial.print("Upper input: ");
    Serial.println(upperInput);
    
  }

  if (rightButton == LOW || leftButton == LOW) {
    delay(300);
    if (gripperServo.read() == GRIPPER_MIN) {
      gripperServo.write(GRIPPER_MAX);
      Serial.println("Grippers: OPEN");
    } else {
      gripperServo.write(GRIPPER_MIN);
      Serial.println("Grippers: CLOSED");
    }
  }

  delay(10);
}
