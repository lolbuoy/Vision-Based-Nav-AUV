const int pwmPin = 9;  // Replace with the actual PWM pin you are using
int pwmValue = 1000;   // Initial PWM value

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(pwmPin, OUTPUT);  // Set the PWM pin as an output
}

void loop() {
  // Set the PWM value
  analogWrite(pwmPin, pwmValue);

  // Check if a character is received through serial communication
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();

    // Set PWM value to 1000 when 'r' is pressed
    if (receivedChar == 'r') {
      pwmValue = 1000;
    }

    // Set PWM value to 2000 when 'c' is pressed
    else if (receivedChar == 'c') {
      pwmValue = 2000;
    }
  }
}
