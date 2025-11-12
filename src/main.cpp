#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

// === Pin Motor Driver TB6612FNG ===
#define AIN1 9
#define AIN2 10
#define PWMA 11
#define BIN1 8
#define BIN2 7
#define PWMB 6

// === Pin Encoder ===
#define ENCODER_LEFT_A 2  // Interrupt 0
#define ENCODER_LEFT_B 4
#define ENCODER_RIGHT_A 3 // Interrupt 1
#define ENCODER_RIGHT_B 5

// === Pin Ultrasonik ===
#define TRIG_LEFT 44
#define ECHO_LEFT 42
#define TRIG_MIDDLE 40
#define ECHO_MIDDLE 38
#define TRIG_RIGHT 36
#define ECHO_RIGHT 34

// === Robot Geometry ===
const float wheelDiameter = 6.75;          
const float wheelCircumference = 3.1416 * wheelDiameter;  // 21.205 cm
const float gearRatio = 30.0;
const int encoderCPR = 11;
const int encoderCountsPerRev = encoderCPR * 2 * gearRatio;  // quadrature ×2
const float countPerCm = encoderCountsPerRev / wheelCircumference;  // ≈ 62.26
const float wheelBase = 18.0;  // cm, jarak antar roda

// === PID Parameters ===
float Kp = 0.6, Ki = 0.02, Kd = 0.3;
float errorSum = 0, lastError = 0;
unsigned long lastPIDTime = 0;

// === Variabel encoder ===
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long prevEncoderLeftCount = 0;
long prevEncoderRightCount = 0;
unsigned long lastEncoderTime = 0;
float leftSpeed = 0;
float rightSpeed = 0;

// === Variabel Odometry (Posisi Robot) ===
float posX = 0.0;  // posisi X dalam cm
float posY = 0.0;  // posisi Y dalam cm
float theta = 0.0; // orientasi robot dalam radian

// === Target Gerak Lurus ===
float targetDistance = 0;
long targetCount = 0;
bool moveActive = false;
int moveDirection = 1; // 1 = maju, -1 = mundur

// === Target Rotasi (Pivot Turn) ===
bool turnActive = false;
int turnDir = 0; // -1 = kiri, +1 = kanan
long targetCountTurn = 0;

// === Variabel ultrasonik ===
long duration;
int distanceLeft, distanceMiddle, distanceRight;

// === MPU6050 DMP ===
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3]; // yaw, pitch, roll

// Raw sensor
int16_t ax, ay, az;
int16_t gx, gy, gz;

// === Variabel kontrol motor ===
int motorSpeed = 100;

void setup() {
  Serial.begin(115200);

  // Setup motor driver
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  // Setup encoder
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), isrLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), isrRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), isrRightB, CHANGE);

  // Setup ultrasonik
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_MIDDLE, OUTPUT); pinMode(ECHO_MIDDLE, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Inisialisasi MPU6050 DMP
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // (opsional) kalibrasi offset
  mpu.setXAccelOffset(-1684);
  mpu.setYAccelOffset(-1929);
  mpu.setZAccelOffset(1131);
  mpu.setXGyroOffset(43);
  mpu.setYGyroOffset(-29);
  mpu.setZGyroOffset(12);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 DMP ready!");
  } else {
    Serial.print("DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  stopMotors();
  Serial.println("DDMR Ready! Waiting commands from Raspberry Pi...");
  Serial.println("Format: maju XX, mundur XX, kiri XX, kanan XX");
}

void loop() {
  // Baca sensor ultrasonik
  distanceLeft   = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  distanceMiddle = readUltrasonic(TRIG_MIDDLE, ECHO_MIDDLE);
  distanceRight  = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  // Hitung kecepatan encoder
  calculateSpeed();

  // Baca IMU raw
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Hitung pitch & roll dari DMP
  float pitch = 0, roll = 0;
  if (dmpReady) {
    fifoCount = mpu.getFIFOCount();
    if (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      pitch = ypr[1] * 180 / M_PI;
      roll  = ypr[2] * 180 / M_PI;
    }
  }

  // Terima perintah dari Raspberry Pi
  readPiCommands();

  // Proses pergerakan robot dengan PID
  processMovement();

  // Kirim data sensor ke Raspberry Pi
  sendSensorData(pitch, roll);
}

// === Baca sensor ultrasonik ===
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return 200;
  return duration * 0.034 / 2;
}

// === Quadrature ISR Left ===
void isrLeftA() {
  bool A = digitalRead(ENCODER_LEFT_A);
  bool B = digitalRead(ENCODER_LEFT_B);
  if (A == B) encoderLeftCount++; else encoderLeftCount--;
}
void isrLeftB() {
  bool A = digitalRead(ENCODER_LEFT_A);
  bool B = digitalRead(ENCODER_LEFT_B);
  if (A != B) encoderLeftCount++; else encoderLeftCount--;
}

// === Quadrature ISR Right ===
void isrRightA() {
  bool A = digitalRead(ENCODER_RIGHT_A);
  bool B = digitalRead(ENCODER_RIGHT_B);
  if (A == B) encoderRightCount++; else encoderRightCount--;
}
void isrRightB() {
  bool A = digitalRead(ENCODER_RIGHT_A);
  bool B = digitalRead(ENCODER_RIGHT_B);
  if (A != B) encoderRightCount++; else encoderRightCount--;
}

// === Hitung kecepatan motor ===
void calculateSpeed() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastEncoderTime;

  if (deltaTime >= 100) {
    long deltaLeft  = encoderLeftCount  - prevEncoderLeftCount;
    long deltaRight = encoderRightCount - prevEncoderRightCount;

    leftSpeed  = (deltaLeft  / (encoderCountsPerRev / 1.0)) / (deltaTime / 1000.0) * 60.0;
    rightSpeed = (deltaRight / (encoderCountsPerRev / 1.0)) / (deltaTime / 1000.0) * 60.0;

    prevEncoderLeftCount  = encoderLeftCount;
    prevEncoderRightCount = encoderRightCount;
    lastEncoderTime = currentTime;
  }
}

// === Fungsi Motor Control dengan PWM ===
void motorControl(float pwmLeft, float pwmRight) {
  pwmLeft = constrain(pwmLeft, -255, 255);
  pwmRight = constrain(pwmRight, -255, 255);

  // Left motor
  if (pwmLeft >= 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, pwmLeft);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, -pwmLeft);
  }

  // Right motor
  if (pwmRight >= 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, pwmRight);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, -pwmRight);
  }
}

// === Update Odometry ===
void updateOdometry() {
  static long lastLeftCount = 0;
  static long lastRightCount = 0;
  
  long deltaLeft = encoderLeftCount - lastLeftCount;
  long deltaRight = encoderRightCount - lastRightCount;
  
  lastLeftCount = encoderLeftCount;
  lastRightCount = encoderRightCount;
  
  float distLeft = deltaLeft / countPerCm;
  float distRight = deltaRight / countPerCm;
  float distCenter = (distLeft + distRight) / 2.0;
  float deltaTheta = (distRight - distLeft) / wheelBase;
  
  posX += distCenter * cos(theta);
  posY += distCenter * sin(theta);
  theta += deltaTheta;
  
  // Normalize theta to -PI to PI
  while (theta > 3.1416) theta -= 2 * 3.1416;
  while (theta < -3.1416) theta += 2 * 3.1416;
}

// === Proses Pergerakan dengan PID ===
void processMovement() {
  updateOdometry();
  
  // === Gerak Lurus ===
  if (moveActive) {
    long avgCount = (abs(encoderLeftCount) + abs(encoderRightCount)) / 2;
    float distanceNow = avgCount / countPerCm;
    float error = targetCount - avgCount;

    unsigned long now = millis();
    float dt = (now - lastPIDTime) / 1000.0;
    if (dt > 0) {
      lastPIDTime = now;

      errorSum += error * dt;
      float dError = (error - lastError) / dt;
      lastError = error;

      float output = Kp * error + Ki * errorSum + Kd * dError;
      output = constrain(output, -255, 255);

      motorControl(output * moveDirection, output * moveDirection);

      // Stop condition
      if (abs(error) < 5) {
        motorControl(0, 0);
        moveActive = false;
        Serial.print("Movement complete! Distance: ");
        Serial.print(distanceNow, 2);
        Serial.print(" cm | Position X: ");
        Serial.print(posX, 2);
        Serial.print(" Y: ");
        Serial.println(posY, 2);
      }
    }
  }

  // === Rotasi Pivot ===
  if (turnActive) {
    long leftAbs = abs(encoderLeftCount);
    long rightAbs = abs(encoderRightCount);
    long avgCount = (leftAbs + rightAbs) / 2;
    float error = targetCountTurn - avgCount;

    unsigned long now = millis();
    float dt = (now - lastPIDTime) / 1000.0;
    if (dt > 0) {
      lastPIDTime = now;

      errorSum += error * dt;
      float dError = (error - lastError) / dt;
      lastError = error;

      float output = Kp * error + Ki * errorSum + Kd * dError;
      output = constrain(output, -200, 200);

      motorControl(-output * turnDir, output * turnDir);

      if (abs(error) < 5) {
        motorControl(0, 0);
        turnActive = false;
        Serial.print("Rotation complete! Theta: ");
        Serial.println(theta * 180 / 3.1416, 2);
      }
    }
  }
}

// === Fungsi Rotasi Pivot ===
void rotatePivot(int dir, float angleDeg) {
  encoderLeftCount = encoderRightCount = 0;
  float arcLength = 3.1416 * wheelBase * (angleDeg / 360.0);
  targetCountTurn = arcLength * countPerCm;
  turnActive = true;
  turnDir = dir;
  errorSum = 0;
  lastError = 0;
  lastPIDTime = millis();
  Serial.print("Rotating ");
  Serial.print(angleDeg);
  Serial.print(" degrees ");
  Serial.println(dir > 0 ? "(right)" : "(left)");
}

// === Fungsi Gerak Lurus ===
void moveLinear(float distance, int direction) {
  targetDistance = abs(distance);
  targetCount = targetDistance * countPerCm;
  encoderLeftCount = encoderRightCount = 0;
  moveActive = true;
  moveDirection = direction;
  errorSum = 0;
  lastError = 0;
  lastPIDTime = millis();
  Serial.print("Moving ");
  Serial.print(direction > 0 ? "forward" : "backward");
  Serial.print(" ");
  Serial.print(targetDistance);
  Serial.println(" cm");
}

// === Kirim data sensor ke Raspberry Pi (CSV sesuai format) ===
void sendSensorData(float pitch, float roll) {
  Serial.print(distanceLeft); Serial.print(",");
  Serial.print(distanceMiddle); Serial.print(",");
  Serial.print(distanceRight); Serial.print(",");
  Serial.print((moveActive || turnActive) ? 1 : 0); Serial.print(",");
  Serial.print(motorSpeed); Serial.print(",");
  Serial.print(ax/16384.0, 2); Serial.print(",");
  Serial.print(ay/16384.0, 2); Serial.print(",");
  Serial.print(az/16384.0, 2); Serial.print(",");
  Serial.print(gx/131.0, 2); Serial.print(",");
  Serial.print(gy/131.0, 2); Serial.print(",");
  Serial.print(gz/131.0, 2); Serial.print(",");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.println(roll, 2);
}

// === Terima perintah dari Raspberry Pi ===
void readPiCommands() {
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();
    incoming.toLowerCase();

    // Parse command: "maju 50", "kiri 30", etc.
    int spaceIndex = incoming.indexOf(' ');
    String command = "";
    float value = 0;

    if (spaceIndex > 0) {
      command = incoming.substring(0, spaceIndex);
      value = incoming.substring(spaceIndex + 1).toFloat();
    } else {
      command = incoming;
    }

    if (command == "maju" && value > 0) {
      moveLinear(value, 1);
      Serial.print("Reply: ok moving forward ");
      Serial.print(value);
      Serial.println(" cm");
    } 
    else if (command == "mundur" && value > 0) {
      moveLinear(value, -1);
      Serial.print("Reply: ok moving backward ");
      Serial.print(value);
      Serial.println(" cm");
    } 
    else if (command == "kiri" && value > 0) {
      rotatePivot(-1, 90);
      delay(100);
      while(turnActive) { processMovement(); }
      moveLinear(value, 1);
      Serial.print("Reply: ok turn left and move ");
      Serial.print(value);
      Serial.println(" cm");
    } 
    else if (command == "kanan" && value > 0) {
      rotatePivot(1, 90);
      delay(100);
      while(turnActive) { processMovement(); }
      moveLinear(value, 1);
      Serial.print("Reply: ok turn right and move ");
      Serial.print(value);
      Serial.println(" cm");
    } 
    else if (command == "stop") {
      stopMotors();
      moveActive = false;
      turnActive = false;
      Serial.println("Reply: ok stopping");
    }
    else {
      Serial.println("Reply: unknown command");
    }
  }
}

// === Stop Motors ===
void stopMotors() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}