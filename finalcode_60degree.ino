const int dirPin = 9;                   // Direction pin for A4988
const int stepPin = 8;                  // Step pin for A4988
const int stepsPerRevolution = 200;     // Full revolution steps (1.8° per step)
const int targetDegrees = 120;           
const float stepAngle = 1.8;            // Each step moves 1.8 degrees
const int stepsToMove = targetDegrees / stepAngle;  // ~33 steps

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  Serial.begin(9600);                   // Start Serial Monitor
  Serial.println("Stepper control started...");
}

void loop() {
  Serial.println("Rotating motor 60 degrees...");

  digitalWrite(dirPin, HIGH);  // Set direction (HIGH or LOW)

  for (int i = 0; i < stepsToMove; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

  Serial.println("Rotation complete.");
  delay(2000);  // Wait before next move
}