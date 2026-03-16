// PWM Input to Output Pass-Through for Arduino Uno
// Reads PWM signal on input pin and reproduces it on output pin

#define INPUT_PIN   6    // Must be pin 2 or 3 on Uno (interrupt pins only)
#define OUTPUT_PIN  12    // Must be a PWM pin (3, 5, 6, 9, 10, 11)

volatile unsigned long riseTime = 0;
volatile unsigned long pulseWidth = 0;
volatile bool newPulse = false;

void pwmISR() {
  if (digitalRead(INPUT_PIN) == HIGH) {
    riseTime = micros();
  } else {
    if (riseTime > 0) {
      pulseWidth = micros() - riseTime;
      newPulse = true;
    }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), pwmISR, CHANGE);

  pinMode(OUTPUT_PIN, OUTPUT);

  Serial.println("PWM Pass-Through Ready");
}

void loop() {
  if (newPulse) {
    noInterrupts();
    unsigned long pw = pulseWidth;
    noInterrupts();
    newPulse = false;
    interrupts();

    // Map pulse width to 0-255 duty cycle for analogWrite
    // Default Uno PWM period is ~4000us (490Hz) on pin 9
    uint8_t duty = (uint8_t)constrain(map(pw, 0, 2000, 0, 255), 0, 255);

    analogWrite(OUTPUT_PIN, duty);

    Serial.print("Pulse Width (us): ");
    Serial.print(pw);
    Serial.print(" | Duty: ");
    Serial.print(duty / 255.0 * 100.0, 1);
    Serial.println("%");
  }
}