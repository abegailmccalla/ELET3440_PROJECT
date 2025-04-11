#include <Servo.h>
#include <EEPROM.h>


#define DELAY_MS 100

// Servo objects
Servo baseServo, lowerServo, upperServo, gripperServo;

// EEPROM storage start address
const int eepromStartAddress = 0;


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
const int UPPER_MIN = 50, UPPER_MAX = 180;
const int GRIPPER_MIN = 60, GRIPPER_MAX = 32;

// Store current angles
int baseAngle = 90;
int lowerAngle = 50; //50;
int upperAngle = 100;
int gripperAngle = 30; //3

// Storage for recorded positions
const int maxSteps = 100;
int recordedPositions[maxSteps][4];  // Stores [base, lower, upper, gripper] positions
int stepCount = 0;  // Number of stored positions
int lastRecordTime = 0;

// State variables
bool recordMode = false;
bool playMode = false;


void setup() {
  // put your setup code here, to run once:

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

void saveToEEPROM() {
  int addr = eepromStartAddress;
  EEPROM.put(addr, stepCount);  // Store step count
  addr += sizeof(stepCount);

  for (int i = 0; i < stepCount; i++) {
    for (int j = 0; j < 4; j++) {
      EEPROM.put(addr, recordedPositions[i][j]);
      addr += sizeof(recordedPositions[i][j]);
    }
  }
  // Clear remaining space if a previous session had more steps
  for (int i = stepCount; i < maxSteps; i++) {
    for (int j = 0; j < 4; j++) {
      EEPROM.put(addr, 0);  // Overwrite old values with 0
      addr += sizeof(int);
    }
  }
  Serial.println("Recorded positions saved to EEPROM.");
}

void loadFromEEPROM() {
  int addr = eepromStartAddress;
  EEPROM.get(addr, stepCount);
  addr += sizeof(stepCount);

  Serial.print("Loaded Step Count: ");
  Serial.println(stepCount);

  for (int i = 0; i < stepCount; i++) {
    for (int j = 0; j < 4; j++) {
      EEPROM.get(addr, recordedPositions[i][j]);
      addr += sizeof(recordedPositions[i][j]);
    }
  }
  Serial.println("Data loaded from EEPROM:");
}

void Record() {
  // Record current servo positions only if the change in angle is greater than 2 for at least one servo
  static int lastBaseAngle = baseServo.read();
  static int lastLowerAngle = lowerServo.read();
  static int lastUpperAngle = upperServo.read();
  static int lastGripperAngle = gripperServo.read();

  if (abs(baseServo.read() - lastBaseAngle) > 2 || 
      abs(lowerServo.read() - lastLowerAngle) > 2 || 
      abs(upperServo.read() - lastUpperAngle) > 2 || 
      abs(gripperServo.read() - lastGripperAngle) > 20) {
    if (stepCount < maxSteps) {
      recordedPositions[stepCount][0] = baseServo.read();
      recordedPositions[stepCount][1] = lowerServo.read();
      recordedPositions[stepCount][2] = upperServo.read();
      recordedPositions[stepCount][3] = gripperServo.read();
      stepCount++;
      Serial.println("Position Recorded");

      // Update last recorded angles
      lastBaseAngle = baseServo.read();
      lastLowerAngle = lowerServo.read();
      lastUpperAngle = upperServo.read();
      lastGripperAngle = gripperServo.read();

      
    } else {
      Serial.println("Max steps reached");
    }
  }
}



void Control() {
   // Read joystick values
   int baseInput = analogRead(CTRL_LEFT_X);
   int upperInput = analogRead(CTRL_LEFT_Y);
   int lowerInput = analogRead(CTRL_RIGHT_Y);
   int gripperInput = analogRead(CTRL_RIGHT_X);
   int leftButton = digitalRead(CTRL_LEFT_SW);
 
   
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
 
   if (leftButton == LOW) {
     if (gripperServo.read() == GRIPPER_MIN) {
       gripperServo.write(GRIPPER_MAX);
     } else {
       gripperServo.write(GRIPPER_MIN);
     }
     while(digitalRead(CTRL_LEFT_SW) == LOW) {
       delay(10); // Small delay to avoid excessive CPU usage
     }
   }
 
   delay(10);
}
  
void Play() {
  // Play back recorded positions
  static int playStep = 0;
  if (playStep < stepCount) {
  Serial.println("To be achieved.. ");
  Serial.print("Base: ");
  Serial.print(baseServo.read());
  Serial.print(" Lower: ");
  Serial.print(lowerServo.read());
  Serial.print(" Upper: ");
  Serial.print(upperServo.read());
  Serial.print(" Gripper: ");
  Serial.println(gripperServo.read());
    while (baseServo.read() != recordedPositions[playStep][0] || 
           lowerServo.read() != recordedPositions[playStep][1] || 
           upperServo.read() != recordedPositions[playStep][2]) {
      // Move servos to recorded positions
    
    if(baseServo.read() < recordedPositions[playStep][0]){
      baseServo.write(baseServo.read() + 1);
      
    }
    else if(baseServo.read() > recordedPositions[playStep][0]){
      baseServo.write(baseServo.read() - 1);
      
    }
    if(lowerServo.read() < recordedPositions[playStep][1]){
      lowerServo.write(lowerServo.read() + 1);
      
    }
    else if(lowerServo.read() > recordedPositions[playStep][1]){
      lowerServo.write(lowerServo.read() - 1);
      
    }
    if(upperServo.read() < recordedPositions[playStep][2]){
      upperServo.write(upperServo.read() + 1);
      
    }
    else if(upperServo.read() > recordedPositions[playStep][2]){
      upperServo.write(upperServo.read() - 1);
      
    }
    delay(10);
  }
  if (recordedPositions[playStep][3] != gripperServo.read()){
    gripperServo.write(recordedPositions[playStep][3]);
    delay(250);
  }
    playStep++;
  }
  else {
    playMode = false;
    playStep = 0;
    Serial.println("Playback complete");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  if (digitalRead(CTRL_RIGHT_SW) == LOW && digitalRead(CTRL_LEFT_SW) == LOW) {
    recordMode = !recordMode;
    if (recordMode) {
      Serial.println("Record Mode ON");
      stepCount = 0;

    }
    else {
      Serial.println("Record Mode OFF");
      saveToEEPROM();
    }
    while (digitalRead(CTRL_RIGHT_SW) == LOW && digitalRead(CTRL_LEFT_SW) == LOW) {
      // Wait for both buttons to be released
    delay(100);
    }
  }
  if (digitalRead(CTRL_LEFT_SW)== LOW && digitalRead(CTRL_RIGHT_SW) == HIGH && recordMode == false) {
    playMode = !playMode;
    if (playMode) {
      loadFromEEPROM();
      Serial.print("Number of steps: ");
      Serial.println(stepCount);
      if (stepCount > 0){

      Serial.println("Play Mode ON");

      delay(250);
      }else {
      playMode = false;
      }
    }
    else {
      Serial.println("Play Mode OFF");
      delay(250);
    }
    while (digitalRead(CTRL_LEFT_SW) == LOW) {
      // Wait for button to be released
      delay(100);
    }
  }


  int grippy = gripperServo.read();
  if (recordMode) {
    
    Control();
    if ((stepCount < maxSteps) && (millis() - lastRecordTime > 500) || grippy != gripperServo.read()) {
      lastRecordTime = millis();
      grippy = gripperServo.read();
      Record();
   }
  }
  if (playMode) {
    Play();
  }
  

}
