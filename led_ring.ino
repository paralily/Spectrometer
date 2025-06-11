const int clockPin = 2;   // CD4017 Clock (Pin 14)
const int resetPin = 3;   // CD4017 Reset (Pin 15)
const int enablePin = 4;  // CD4017 Enable (Pin 13, active LOW)

const int maxCount = 6;   // Number of LEDs/steps

void setup() {
  pinMode(clockPin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  digitalWrite(clockPin, LOW);
  digitalWrite(resetPin, LOW);
  digitalWrite(enablePin, LOW); // Enable counting

  // Reset to Q0 at start
  digitalWrite(resetPin, HIGH);
  delay(20); // Longer reset pulse
  digitalWrite(resetPin, LOW);
}

void loop() {
  for (int i = 0; i < maxCount; i++) {
    // Send one clock pulse
    digitalWrite(clockPin, HIGH);
    delay(10); // Longer HIGH pulse
    digitalWrite(clockPin, LOW);

    delay(500); // LED on-time
  }

  // After maxCount pulses, reset to Q0
  digitalWrite(resetPin, HIGH);
  delay(20); // Longer reset pulse
  digitalWrite(resetPin, LOW);

  delay(500); // Pause before next cycle
}