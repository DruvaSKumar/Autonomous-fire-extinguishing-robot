#include <Servo.h>

// ===========================
// Motor Pins
// ===========================
#define ENA 3     // Motor A Speed Control
#define ENB 11    // Motor B Speed Control
#define IN1 8     // Motor A Direction 1
#define IN2 9     // Motor A Direction 2
#define IN3 10    // Motor B Direction 1
#define IN4 12    // Motor B Direction 2

// ===========================
// Ultrasonic Sensor Pins
// ===========================
#define TRIG 6
#define ECHO 7

// ===========================
// Servo
// ===========================
#define SERVO_PIN 5
Servo myServo;

// ===========================
// Analog Flame Sensors
// ===========================
const int flameSensor1 = A1; // Left Flame Sensor
const int flameSensor2 = A0; // Front Flame Sensor
const int flameSensor3 = A2; // Right Flame Sensor
const int flameThreshold = 400; // Flame detection threshold (lower value = closer/hotter flame)

bool obstacleDetected = false; // Flag to track if obstacle was detected
String lastTurn = "";          // Store the direction of last turn for realignment

void setup() {
  Serial.begin(9600);

  // Motor setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic setup
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Servo initialization
  myServo.attach(SERVO_PIN);
  myServo.write(60);  // Center the servo (60 degrees)
  delay(500);
}

// Function to read ultrasonic distance in cm
long readDistance() {
  digitalWrite(TRIG, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  return duration * 0.034 / 2; // Convert time to distance
}

// ===========================
// Motor Control Functions
// ===========================
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 103);
  analogWrite(ENB, 100);
}

void stopMoving() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  // Reverse left motor
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Forward right motor
  analogWrite(ENA, 53);
  analogWrite(ENB, 50);
  delay(500); // Duration of turn
  stopMoving();
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Forward left motor
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  // Reverse right motor
  analogWrite(ENA, 53);
  analogWrite(ENB, 50);
  delay(500); // Duration of turn
  stopMoving();
}

void loop() {
  // ===========================
  // Flame Sensor Readings
  // ===========================
  int value1 = analogRead(flameSensor1); // Left
  int value2 = analogRead(flameSensor2); // Front
  int value3 = analogRead(flameSensor3); // Right

  // Print Flame Sensor Data
  Serial.println("---- Flame Sensor Readings ----");
  Serial.print("Sensor LEFT  (A1): "); Serial.println(value1);
  Serial.print("Sensor FRONT (A0): "); Serial.println(value2);
  Serial.print("Sensor RIGHT (A2): "); Serial.println(value3);

  // Check for flames and react
  if (value1 < flameThreshold || value2 < flameThreshold || value3 < flameThreshold) {
    stopMoving(); // Stop if flame is detected

    if (value1 < flameThreshold) {
      Serial.println("🔥 Flame detected on LEFT");
      myServo.write(120); // Aim water/actuator to left
    } else if (value3 < flameThreshold) {
      Serial.println("🔥 Flame detected on RIGHT");
      myServo.write(0); // Aim water/actuator to right
    } else {
      Serial.println("🔥 Flame detected in FRONT");
      myServo.write(60); // Aim water/actuator to front
    }

    delay(500); // Allow time for servo to move
    return;     // Skip the rest of the loop when flame is detected
  }

  // ===========================
  // Obstacle Avoidance
  // ===========================
  long frontDist, leftDist, rightDist;

  // Measure left distance
  myServo.write(120); 
  delay(400); 
  leftDist = readDistance();

  // Measure front distance
  myServo.write(60); 
  delay(400); 
  frontDist = readDistance();

  // Measure right distance
  myServo.write(0); 
  delay(400); 
  rightDist = readDistance();

  myServo.write(60); // Re-center servo after measuring

  // Print ultrasonic distances
  Serial.print("L:"); Serial.print(leftDist);
  Serial.print(" F:"); Serial.print(frontDist);
  Serial.print(" R:"); Serial.println(rightDist);

  // If obstacle is detected in front
  if (frontDist < 20) {
    stopMoving();            // Stop bot
    obstacleDetected = true; // Set obstacle flag

    // Decide direction based on open space
    if (leftDist > rightDist) {
      turnLeft(); 
      lastTurn = "left";
    } else {
      turnRight(); 
      lastTurn = "right";
    }

    moveForward(); // Move ahead after avoiding
    delay(1000);   // Move forward to bypass the obstacle
  }

  // After bypassing, realign to original path
  else if (obstacleDetected) {
    stopMoving();

    // Undo the previous turn to realign
    if (lastTurn == "left") {
      turnRight();
    } else if (lastTurn == "right") {
      turnLeft();
    }

    delay(500);     // Time for realignment
    moveForward();  // Resume forward movement

    // Reset obstacle tracking
    obstacleDetected = false;
    lastTurn = "";
  }

  // If no flame or obstacle, just move forward
  else {
    moveForward();
  }
}