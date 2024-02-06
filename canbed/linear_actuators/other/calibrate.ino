// Define pin numbers
const int pwmPin = 6;
const int directionPin = 7;
const int potPin = A0;

// Calibration parameters
const float strokeLength = 250.0; // Stroke length in mm
const float speed = 5.0; // Approximate speed in mm/s at 200 PWM
const float bufferPercentage = 0.1; // 10% time buffer

void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, OUTPUT);
  pinMode(directionPin, OUTPUT);

  // Calibrate in one direction (true for extend, false for retract)
  float averageValue = calibrate(true); // Calibrating by extending
  Serial.print("Calibrated forward Value: ");
  Serial.println(averageValue);

  averageValue = calibrate(false); // Calibrating by extending
  Serial.print("Calibrated backward Value: ");
  Serial.println(averageValue);
}

void loop() {
  // Main loop code
}

float calibrate(bool direction) {
  const int numSamples = 10;
  int samples[numSamples];
  float totalTime = (strokeLength / speed) * (1 + bufferPercentage) * 1000; // Total time in milliseconds, including buffer

  // Set direction and move
  digitalWrite(directionPin, direction ? HIGH : LOW);
  analogWrite(pwmPin, 200); // PWM value mapped to ~5 mm/s
  unsigned long startTime = millis();
  while (millis() - startTime < totalTime) {
    // Keep moving
  }
  analogWrite(pwmPin, 0); // Stop the actuator

  delay(500); // Wait for any residual movement to stop

  // Sampling potentiometer values
  for (int i = 0; i < numSamples; i++) {
    samples[i] = analogRead(potPin);
    delay(100); // Delay between samples
  }

  // Calculate mean
  float sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += samples[i];
  }
  float mean = sum / numSamples;

  // Calculate standard deviation to identify outliers
  float variance = 0;
  for (int i = 0; i < numSamples; i++) {
    variance += pow(samples[i] - mean, 2);
  }
  float stdDev = sqrt(variance / numSamples);

  // Filter outliers and re-calculate mean
  float filteredSum = 0;
  int countFiltered = 0;
  for (int i = 0; i < numSamples; i++) {
    if (abs(samples[i] - mean) <= stdDev) { // Keeping within 1 standard deviation
      filteredSum += samples[i];
      countFiltered++;
    }
  }

  float meanFiltered = countFiltered > 0 ? filteredSum / countFiltered : mean;

  return meanFiltered;
}
