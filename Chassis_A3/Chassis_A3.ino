#include <avr/wdt.h>
// #include "ApplicationFunctionSet_xxx0.h"
#include "IRremote.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "DeviceDriverSet_xxx0.h"


MPU6050 mpu;
Adafruit_MPU6050 mpu1;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define M_PI		3.14159265358979323846	/* pi */

#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

#define Forward_CODE 16736925  //"↑"
#define Backward_CODE 16754775 //"↓"
#define Left_CODE_90 16720605 // "←"
#define Right_CODE_90 16761405 // "→"
#define Stop_CODE 16712445 //"ok"
#define Left_About_180 16738455 //"1"
#define Right_About_180 16750695 // "2"

int RECEIVER_PIN = 9;
IRrecv irrecv(RECEIVER_PIN);
decode_results results;

// defines pins numbers
const int trigPin = 13;
const int echoPin = 12;
// defines variables
long duration;
int distance;
int16_t gz;
bool  angle = false;
static unsigned long MotorRL_time = 0;

unsigned long now, lastTime = 0;
float agz = 0; //Angle variable
long gzo = 0;  //Gyro offset

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, yaw2, initialYaw = 0,targetYaw ;
uint8_t speed = 150, leftSpeed,rightSpeed;
float correctionFactor = 14;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// #################################### SET UP #########################################

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  delay(100);
  irrecv.enableIRIn(); // Start the IR receiver
  delay(1000);
  Serial.begin(9600);
  mpu_setup();
  // mpu1.begin();
  // mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  // mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  // mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // delay(100);
  targetYaw = round(get_yaw());
  initialYaw = round(get_yaw());
  Serial.println("setup end");
}

void mpu_setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.begin(9600);
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // Serial.print(devStatus);
  // supply your own gyro offsets here, scaled for min sensitivity

  mpu.setXGyroOffset(14);
  mpu.setYGyroOffset(23);
  mpu.setZGyroOffset(-76);
  mpu.setZAccelOffset(1472); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

// #################################### LOOP #########################################

void loop() {
    // Check for IR signal
    yaw = round(get_yaw());
    //Serial.println(round(yaw));
    if (irrecv.decode(&results)) 
    {
        Serial.println(results.value);
        handleRemoteCommand(results.value);  // Handle the remote command
        irrecv.resume();  // Prepare the receiver for the next signal
    }
    digitalWrite(PIN_Motor_STBY, LOW);
}

// Function to handle remote commands
void handleRemoteCommand(unsigned long command) {
    switch (command)
    {
      case Forward_CODE:
        moveForwardGyro(100); //drive_straight();
        break;

      case Backward_CODE:
        reverseGyro(100); //drive_backward(500);
        break;

      case Left_CODE_90:
        Serial.println("Moving Left 90");
        turnleftGyro(100); //turn_counter_clockwise(-82);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      
      case Right_CODE_90:
        Serial.println("Moving Right 90");
        yaw = get_yaw();
        turnRightGyro(100); //turn_clockwise(77);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;

      case Left_About_180:
        Serial.println("Moving Left 180");
        yaw = get_yaw();
        turnLAroundGyro(100); //turn_counter_clockwise(-170);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;

      case Right_About_180:
        Serial.println("Moving Right 180");
        turnRAroundGyro(100); //turn_clockwise(166);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;

      case Stop_CODE:
        Serial.println("Stopping the car.");
        motorStop();
        break;
      
      default:
        Serial.println("Unknown command.");
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
    }
}

// #################################### GET ANGLES #########################################

float get_angles()
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { 
    Serial.print("\nYAW ANGLE (Str8 → 0°, right turn → +90°, left turn → -90): ");
    Serial.println(get_yaw());
    delay(5);
    Serial.print("\nPITCH ANGLE (level → 0°, climbing → +30°, descending → -30°): ");
    Serial.println(get_pitch());
    delay(5);
    Serial.print("\nROLL ANGLE (level → 0°, tilt right → +45°, tilt left → -45°): ");
    Serial.println(get_roll());
    delay(5);  
    Serial.println("-----------------------------------------");
  }
}

float get_yaw()
{
  int16_t gx, gy, gz; // Gyroscope data for yaw
  float currentYaw;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.getRotation(&gx, &gy, &gz);
    currentYaw = gx / 131.0; // Update current yaw

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    delay(5);
    return (ypr[0] * 180/M_PI); // Absolute Angle
    //return "Y1: " + String(ypr[0] * 180/M_PI) + ", Y2: " + String(currentYaw);
    //return (currentYaw); // Angular Velocity
  }
}

float get_pitch()
{
  int16_t ax, ay, az; // Acceleration data for pitch
  float pitch;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.getAcceleration(&ax, &ay, &az);
    pitch = atan(ax / sqrt(ay * ay + az * az)) * 180 / M_PI; // Calculate pitch

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return (ypr[1] * 180/M_PI); // DMP Filtered Pitch
    //return "Y1: " + String(ypr[1] * 180/M_PI) + ", Y2: " + String(pitch);
    //return (pitch); // Accelerometer-Based Pitch
  }
}

float get_roll()
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return (ypr[2] * 180/M_PI);
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

void moveForwardGyro(int speed) //Open Straight-line Paths 
{
  float currentAngle = round(get_yaw());
  float error =   targetYaw - currentAngle ; // Calculate error (positive = turn right, negative = turn left)
  int motorSpeed = speed; // Set a base motor speed (adjust as needed)
  // Drive forward with correction based on yaw error
  digitalWrite(PIN_Motor_STBY, HIGH);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  rightSpeed = constrain(motorSpeed - (error * correctionFactor),50,220); 
  leftSpeed = constrain(motorSpeed + (error * correctionFactor),50,220); 
  analogWrite(PIN_Motor_PWMA, rightSpeed);
  analogWrite(PIN_Motor_PWMB, leftSpeed);
  distance = get_distance();
  while (distance > 10)  
  {
    currentAngle = round(get_yaw());
    error =   targetYaw - currentAngle;
    Serial.print("Error: ");
    Serial.println(error);
    // Adjust motor speeds based on error (optional, refine control strategy here)
    digitalWrite(PIN_Motor_STBY, HIGH);
    digitalWrite(PIN_Motor_AIN_1, HIGH);
    digitalWrite(PIN_Motor_BIN_1, HIGH);
    rightSpeed = constrain(motorSpeed - (error * correctionFactor),50,220);
    leftSpeed = constrain(motorSpeed + (error * correctionFactor),50,220);
    analogWrite(PIN_Motor_PWMA, rightSpeed);
    analogWrite(PIN_Motor_PWMB, leftSpeed);
    distance = get_distance();
    if (irrecv.decode(&results)) {
      Serial.println(results.value); 
      if (results.value == Stop_CODE) {
        motorStop();  // Stop the motors
        break;  // Exit the loop
      }
      irrecv.resume();  // Reset the IR receiver
    }
  }
  digitalWrite(PIN_Motor_STBY, LOW);
  delay(30); // Prevent sensor overload
}

void reverseGyro(int speed) //Open Straight-line Paths 
{
  float currentAngle = round(get_yaw());
  float error =   targetYaw - currentAngle ; // Calculate error (positive = turn right, negative = turn left)
  int motorSpeed = speed; // Set a base motor speed (adjust as needed)
  // Drive forward with correction based on yaw error
  digitalWrite(PIN_Motor_STBY, HIGH);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  rightSpeed = constrain(motorSpeed + (error * correctionFactor),50,220); 
  leftSpeed = constrain(motorSpeed - (error * correctionFactor),50,220); 
  analogWrite(PIN_Motor_PWMA, rightSpeed);
  analogWrite(PIN_Motor_PWMB, leftSpeed);
  distance = get_distance();
  while (distance > 10)  
  {
    currentAngle = round(get_yaw());
    error =   targetYaw - currentAngle  ;
    Serial.print("Error: ");
    Serial.println(error);
    // Adjust motor speeds based on error (optional, refine control strategy here)
    digitalWrite(PIN_Motor_STBY, HIGH);
    digitalWrite(PIN_Motor_AIN_1, LOW);
    digitalWrite(PIN_Motor_BIN_1, LOW);
    rightSpeed = constrain(motorSpeed + (error * correctionFactor),50,220);
    leftSpeed = constrain(motorSpeed - (error * correctionFactor),50,220);
    analogWrite(PIN_Motor_PWMA, rightSpeed);
    analogWrite(PIN_Motor_PWMB, leftSpeed);
    distance = get_distance();
    if (irrecv.decode(&results)) {
      Serial.println(results.value); 
      if (results.value == Stop_CODE) {
        motorStop();  // Stop the motors
        break;  // Exit the loop
      }
      irrecv.resume();  // Reset the IR receiver
    }
  }
  digitalWrite(PIN_Motor_STBY, LOW);
  delay(30); // Prevent sensor overload
}

void turnleftGyro(int speed)
{
  angle = false;
  initialYaw = round(get_yaw());
  Serial.print("Initial is: ");
  Serial.println(initialYaw);
  targetYaw = initialYaw + -82;
  if (targetYaw < -179)
  {targetYaw += 360;}
  Serial.print("Target is: ");
  Serial.println(targetYaw);
  while (!angle)
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    digitalWrite(PIN_Motor_AIN_1, HIGH);
    analogWrite(PIN_Motor_PWMA, speed);
    digitalWrite(PIN_Motor_BIN_1, LOW);
    analogWrite(PIN_Motor_PWMB, speed);
    yaw = round(get_yaw());
    if ( (yaw == targetYaw - 1) || (yaw == targetYaw - 2) || (yaw == targetYaw - 3) || (yaw == targetYaw - 4)|| (yaw == targetYaw) || (yaw == targetYaw + 1) || (yaw == targetYaw + 2) || (yaw == targetYaw + 3) || (yaw == targetYaw + 4))
    {
      angle = true;
      digitalWrite(PIN_Motor_STBY, LOW);
    }
    //Serial.println(yaw);
  }
  digitalWrite(PIN_Motor_STBY, LOW);
}

void turnRightGyro(int speed)
{
  angle = false;
  initialYaw = round(get_yaw());
  Serial.print("Initial is: ");
  Serial.println(initialYaw);
  targetYaw = initialYaw + 77;
  if (targetYaw > 179)
  {targetYaw -= 360;}
  Serial.print("Target is: ");
  Serial.println(targetYaw);
  while (!angle)
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    digitalWrite(PIN_Motor_AIN_1, LOW);
    analogWrite(PIN_Motor_PWMA, speed);
    digitalWrite(PIN_Motor_BIN_1, HIGH);
    analogWrite(PIN_Motor_PWMB, speed);
    yaw = round(get_yaw());
    if ( (yaw == targetYaw - 1) || (yaw == targetYaw - 2) || (yaw == targetYaw - 3) || (yaw == targetYaw - 4)|| (yaw == targetYaw) || (yaw == targetYaw + 1) || (yaw == targetYaw + 2) || (yaw == targetYaw + 3) || (yaw == targetYaw + 4))
    {
      angle = true;
      digitalWrite(PIN_Motor_STBY, LOW);
    }
    // Serial.println(yaw);
  }
  digitalWrite(PIN_Motor_STBY, LOW);
}

void turnLAroundGyro(int speed)
{
  angle = false;
  initialYaw = round(get_yaw());
  Serial.print("Initial is: ");
  Serial.println(initialYaw);
  targetYaw = initialYaw + -170;
  if (targetYaw < -179)
  {targetYaw += 360;}
  Serial.print("Target is: ");
  Serial.println(targetYaw);
  while (!angle)
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    digitalWrite(PIN_Motor_AIN_1, HIGH);
    analogWrite(PIN_Motor_PWMA, speed);
    digitalWrite(PIN_Motor_BIN_1, LOW);
    analogWrite(PIN_Motor_PWMB, speed);
    yaw = round(get_yaw());
    if ( (yaw == targetYaw - 1) || (yaw == targetYaw - 2) || (yaw == targetYaw - 3) || (yaw == targetYaw - 4)|| (yaw == targetYaw) || (yaw == targetYaw + 1) || (yaw == targetYaw + 2) || (yaw == targetYaw + 3) || (yaw == targetYaw + 4))
    {
      angle = true;
      digitalWrite(PIN_Motor_STBY, LOW);
    }
    //Serial.println(yaw);
  }
  digitalWrite(PIN_Motor_STBY, LOW);
}

void turnRAroundGyro(int speed)
{
  angle = false;
  initialYaw = round(get_yaw());
  Serial.print("Initial is: ");
  Serial.println(initialYaw);
  targetYaw = initialYaw + 166;
  if (targetYaw > 179)
  {targetYaw -= 360;}
  Serial.print("Target is: ");
  Serial.println(targetYaw);
  while (!angle)
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    digitalWrite(PIN_Motor_AIN_1, LOW);
    analogWrite(PIN_Motor_PWMA, speed);
    digitalWrite(PIN_Motor_BIN_1, HIGH);
    analogWrite(PIN_Motor_PWMB, speed);
    yaw = round(get_yaw());
    if ( (yaw == targetYaw - 1) || (yaw == targetYaw - 2) || (yaw == targetYaw - 3) || (yaw == targetYaw - 4)|| (yaw == targetYaw) || (yaw == targetYaw + 1) || (yaw == targetYaw + 2) || (yaw == targetYaw + 3) || (yaw == targetYaw + 4))
    {
      angle = true;
      digitalWrite(PIN_Motor_STBY, LOW);
    }
    // Serial.println(yaw);
  }
  digitalWrite(PIN_Motor_STBY, LOW);
}

void motorStop() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0); 
  digitalWrite(PIN_Motor_STBY, LOW);
}