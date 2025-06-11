#include <Adafruit_AS7341.h>  // AS7341 spectral sensor library

#define N 4  // Number of linear equations

Adafruit_AS7341 as7341;
template<typename T> void SerialPrintArray(T arr[], int len); // Declaration only

const int dirPin = 9;                   // Direction pin for A4988
const int stepPin = 8;                  // Step pin for A4988
const int stepsPerRevolution = 200;     // Full revolution steps (1.8° per step)
const int targetDegrees = 60;
const float stepAngle = 1.8;            // Each step moves 1.8 degrees
const int stepsToMove = targetDegrees / stepAngle;  // ~33 steps

/* Limit switch and decade counter pins */
#define lim 4
#define rst 6
#define clk 5
#define enable 7

int currentPos = 0;
int requiredPos;
int wavelengthIndex;
int lambda;

float pmBuffer = 0.0;

const float max_390 = 3610.0;
const float max_470 = 9234.0;
const float max_570 = 860.0;
const float max_588 = 3450.0;
const float max_600 = 3648.0;
const float max_628 = 3648.0;

float M[6] = {0};
float h = 1.0;

float y_0;
float y_1;
float y_2;
float y_3;
float y_4;
float y_5;

float ybuff_0;
float ybuff_1;
float ybuff_2;
float ybuff_3;
float ybuff_4;
float ybuff_5;
float ybuff_max = 0.0;

int mode = 1;
bool dataAvailable = false;

const float x_0 = 87;
const float x_1 = 128;
const float x_2 = 180;
const float x_3 = 190;
const float x_4 = 196;
const float x_5 = 211;

int pulseCount = 0;

/* --- Function Declarations --- */
void wavelengthSelect(int k);
void Lagrange();
void Linear();
void Cubic();
void LESsolver();
void gaussianElimination(double mat[N][N+1]);
int forwardElim(double mat[N][N+1]);
void backSub(double mat[N][N+1]);
void swap_row(double mat[N][N+1], int i, int j);
void rotateMotor60Degrees();
void resetCounter();

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(lim, INPUT);
  pinMode(rst, OUTPUT);
  pinMode(clk, OUTPUT);
  pinMode(enable, OUTPUT);

  digitalWrite(rst, LOW);
  digitalWrite(clk, LOW);
  digitalWrite(dirPin, HIGH);
  digitalWrite(enable, LOW);

  /*while (digitalRead(lim) == 1) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(700);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(700);
  }*/
  Serial.println("Stepper control started...");

  //as7341.setATIME(100);
 // as7341.setASTEP(499);
  //as7341.setGain(AS7341_GAIN_16X);
}

void loop() {
  Serial.println("\nSelect Mode (1-5):");
  Serial.println("1: Photometer Mode");
  Serial.println("2: Spectrometer Mode");
  Serial.println("3: Lagrange Interpolation Mode");
  Serial.println("4: Linear Interpolation Mode");
  Serial.println("5: Cubic Spline Interpolation Mode");

  while (Serial.available() == 0){} // Wait for user input
  mode = Serial.parseInt();  // Get mode from user
  Serial.read(); // Clear Serial buffer

  switch(mode){
    case 1: {
      Serial.println("Photometer Mode Selected.");
      Serial.println("Enter Wavelength Index (1-6):");
      while (Serial.available() == 0);
      int photoLambda = Serial.parseInt();
      photoLambda = constrain(photoLambda, 1, 6);

      wavelengthSelect(photoLambda);
      //as7341.readAllChannels();
      //float t = as7341.getChannel(AS7341_CHANNEL_CLEAR);
      float t = 1000.0 + random(0, 1000);
      float s = 0.0, pmAbsorbance = 0.0;

      switch (photoLambda) {
        case 1: s = constrain(t, 0, max_390); pmAbsorbance = 100.0 * (1.0 - s / max_390); break;
        case 2: s = constrain(t, 0, max_470); pmAbsorbance = 100.0 * (1.0 - s / max_470); break;
        case 3: s = constrain(t, 0, max_570); pmAbsorbance = 100.0 * (1.0 - s / max_570); break;
        case 4: s = constrain(t, 0, max_588); pmAbsorbance = 100.0 * (1.0 - s / max_588); break;
        case 5: s = constrain(t, 0, max_600); pmAbsorbance = 100.0 * (1.0 - s / max_600); break;
        case 6: s = constrain(t, 0, max_628); pmAbsorbance = 100.0 * (1.0 - s / max_628); break;
      }

      Serial.print("Absorbance at selected wavelength: ");
      Serial.println(pmAbsorbance, 2);
      break;
    }
    case 2: {
      Serial.println("Spectrometer Mode Selected. Measuring...");
      float y[6];
      for (int i = 1; i <= 6; i++) {
        wavelengthSelect(i);
        delay(100);
        //as7341.readAllChannels();
        //float t = as7341.getChannel(AS7341_CHANNEL_CLEAR);
        // Replace with a simulated value
        float t = 1000.0 + random(0, 1000);
        float maxVal = 0;

        switch (i) {
          case 1: maxVal = max_390; break;
          case 2: maxVal = max_470; break;
          case 3: maxVal = max_570; break;
          case 4: maxVal = max_588; break;
          case 5: maxVal = max_600; break;
          case 6: maxVal = max_628; break;
        }
        float s = constrain(t, 0, maxVal);
        y[i - 1] = 100.0 * (1.0 - s / maxVal);
        Serial.print("Absorbance ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(y[i - 1], 2);
      }

      // Find max
      float maxAbs = 0.0;
      for (int i = 0; i < 6; i++) {
        if (y[i] > maxAbs) maxAbs = y[i];
      }
      Serial.print("Max Absorbance: ");
      Serial.println(maxAbs, 2);

      // Store for interpolation modes
      y_0 = y[0];
      y_1 = y[1];
      y_2 = y[2];
      y_3 = y[3];
      y_4 = y[4];
      y_5 = y[5];

      ybuff_0 = y_0;
      ybuff_1 = y_1;
      ybuff_2 = y_2;
      ybuff_3 = y_3;
      ybuff_4 = y_4;
      ybuff_5 = y_5;
      ybuff_max = maxAbs;
      dataAvailable = true;
      break;
    }
    case 3:
      Serial.println("Lagrange Interpolation:");
      if (dataAvailable) {
        Lagrange();
      } else {
        Serial.println("No data available. Run Spectrometer mode first.");
      }
      break;
    case 4:
      Serial.println("Linear Interpolation:");
      if (dataAvailable) {
        Linear();
      } else {
        Serial.println("No data available. Run Spectrometer mode first.");
      }
      break;
    case 5:
      Serial.println("Cubic Spline Interpolation:");
      if (dataAvailable) {
        Cubic();
      } else {
        Serial.println("No data available. Run Spectrometer mode first.");
      }
      break;
    default:
      Serial.println("Invalid mode. Enter a number between 1 and 5.");
      break;
  }
  
  // Move the motor 60 degrees each loop (if desired)
  rotateMotor60Degrees();

  delay(500); // Short delay before next loop
}

/* --- Motor Control and Utility Functions --- */
void rotateMotor60Degrees() {
  digitalWrite(enable, LOW);
  digitalWrite(clk, HIGH);
  delay(100);
  digitalWrite(clk, LOW);
  delay(100);
  pulseCount++;
  // Reset after 6 pulses
  if (pulseCount >= 6) {
    digitalWrite(rst, HIGH);
    delay(50);
    digitalWrite(rst, LOW);
    pulseCount = 0;
  }
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

void resetCounter() {
  digitalWrite(rst, HIGH);
  delayMicroseconds(700);
  digitalWrite(rst, LOW);
  delayMicroseconds(700);
}

void wavelengthSelect(int k) {
  // Translate index to requiredPos and sort it from low to high wavelengths
  if (k == 1) requiredPos = 1340; // UV
  if (k == 2) requiredPos = 800;  // BLUE
  if (k == 3) requiredPos = 250;  // GREEN
  if (k == 4) requiredPos = 290;  // YELLOW
  if (k == 5) requiredPos = 2370; // ORANGE
  if (k == 6) requiredPos = 1860; // RED
  if (requiredPos > currentPos) digitalWrite(dirPin, HIGH);
  if (requiredPos < currentPos) digitalWrite(dirPin, LOW);
  int posDiff = requiredPos - currentPos;
  int absPosDiff = abs(posDiff);
  for (int i = 0; i < absPosDiff; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(700);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(700);
    if (requiredPos > currentPos) currentPos++;
    if (requiredPos < currentPos) currentPos--;
  }
  if (requiredPos == 250) {
    wavelengthIndex = 1;
    resetCounter();
  }
  if (requiredPos == 800) {
    wavelengthIndex = 5;
    resetCounter();
  }
  if (requiredPos == 1340) {
    wavelengthIndex = 4;
    resetCounter();
  }
  if (requiredPos == 1860) {
    wavelengthIndex = 3;
    resetCounter();
  }
  if (requiredPos == 2370) {
    wavelengthIndex = 2;
    resetCounter();
  }
  if (requiredPos == 2900) {
    wavelengthIndex = 6;
    resetCounter();
  }
  for (int i = 0; i < wavelengthIndex; i++) {
    digitalWrite(clk, HIGH);
    delayMicroseconds(700);
    digitalWrite(clk, LOW);
    delayMicroseconds(700);
  }
}

void Lagrange() {
  Serial.println("Performing Lagrange interpolation...");
  Serial.println("Lagrange Interpolation Result:");
  float x_values[] = {x_0, x_1, x_2, x_3, x_4, x_5};
  float y_values[] = {y_0, y_1, y_2, y_3, y_4, y_5};
  for (int x = x_0; x <= x_5; x++) {
    float y_interp = 0;
    for (int i = 0; i < 6; i++) {
      float term = y_values[i];
      for (int j = 0; j < 6; j++) {
        if (i != j) {
          term *= (float)(x - x_values[j]) / (x_values[i] - x_values[j]);
        }
      }
      y_interp += term;
    }
    Serial.print("x = "); Serial.print(x); Serial.print(", y = "); Serial.println(y_interp);
  }
}

void Linear() {
  Serial.println("Performing Linear interpolation...");
  Serial.println("Linear Interpolation Result:");
  float x_values[] = {x_0, x_1, x_2, x_3, x_4, x_5};
  float y_values[] = {y_0, y_1, y_2, y_3, y_4, y_5};
  for (int i = 0; i < 5; i++) {
    for (int x = x_values[i]; x <= x_values[i + 1]; x++) {
      float m = (y_values[i + 1] - y_values[i]) / (x_values[i + 1] - x_values[i]);
      float y = y_values[i] + m * (x - x_values[i]);
      Serial.print("x = "); Serial.print(x); Serial.print(", y = "); Serial.println(y);
    }
  }
}

void Cubic() {
  Serial.println("Performing Cubic Spline interpolation...");
  Serial.println("Cubic Spline Interpolation Result:");
  float x_values[] = {x_0, x_1, x_2, x_3, x_4, x_5};
  float y_values[] = {y_0, y_1, y_2, y_3, y_4, y_5};

  LESsolver();

  for (int i = 0; i < 5; i++) {
    for (float x = x_values[i]; x <= x_values[i + 1]; x += 1.0) {
      float xi = x_values[i];
      float xi1 = x_values[i + 1];
      float hi = xi1 - xi;

      float A = (xi1 - x) / hi;
      float B = (x - xi) / hi;

      float y = A * y_values[i] + B * y_values[i + 1]
                + ((A * A * A - A) * M[i] + (B * B * B - B) * M[i + 1]) * (hi * hi) / 6.0;

      Serial.print("x = "); Serial.print(x); Serial.print(", y = "); Serial.println(y);
    }
  }
}

void LESsolver() {
  // Solve the linear equation system
  float z_0 = (6.0 / sq(h)) * (y_0 - 2 * y_1 + y_2);
  float z_1 = (6.0 / sq(h)) * (y_1 - 2 * y_2 + y_3);
  float z_2 = (6.0 / sq(h)) * (y_2 - 2 * y_3 + y_4);
  float z_3 = (6.0 / sq(h)) * (y_3 - 2 * y_4 + y_5);
  // Define matrix
  double mat[N][N + 1] = {
    {4.0, 1.0, 0.0, 0.0, z_0},
    {1.0, 4.0, 1.0, 0.0, z_1},
    {0.0, 1.0, 4.0, 1.0, z_2},
    {0.0, 0.0, 1.0, 4.0, z_3}
  };
  gaussianElimination(mat);
  for (int i = 0; i < N; i++) {
    M[i + 1] = mat[i][N];
  }
}

void gaussianElimination(double mat[N][N + 1]) {
  int singular_flag = forwardElim(mat);
  if (singular_flag != -1) {
    Serial.println("Singular matrix detected.");
    return;
  }
  backSub(mat);
}

int forwardElim(double mat[N][N + 1]) {
  for (int k = 0; k < N; k++) {
    int i_max = k;
    double v_max = abs(mat[i_max][k]);

    for (int i = k + 1; i < N; i++) {
      if (abs(mat[i][k]) > v_max) {
        v_max = abs(mat[i][k]);
        i_max = i;
      }
    }

    if (mat[i_max][k] == 0.0)
      return k;

    if (i_max != k)
      swap_row(mat, k, i_max);
    for (int i = k + 1; i < N; i++) {
      double f = mat[i][k] / mat[k][k];
      for (int j = k + 1; j <= N; j++) {
        mat[i][j] -= mat[k][j] * f;
      }
      mat[i][k] = 0;
    }
  }
  return -1;
}

void backSub(double mat[N][N + 1]) {
  double w[N];
  for (int i = N - 1; i >= 0; i--) {
    w[i] = mat[i][N];
    for (int j = i + 1; j < N; j++) {
      w[i] -= mat[i][j] * w[j];
    }
    w[i] /= mat[i][i];
  }
  for (int i = 0; i < N; i++) {
    mat[i][N] = w[i];
  }
}

void swap_row(double mat[N][N + 1], int i, int j) {
  for (int k = 0; k <= N; k++) {
    double temp = mat[i][k];
    mat[i][k] = mat[j][k];
    mat[j][k] = temp;
  }
}
