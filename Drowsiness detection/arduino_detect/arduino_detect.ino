#define BUZZER_PIN 8  // Replace 8 with the actual pin connected to the buzzer

bool buzzerActive = false;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    char data = Serial.read();  // Read a single character from the serial port

    if (data == '1' && !buzzerActive) {
      // Closed eyes detected, activate the buzzer
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerActive = true;
    } else if (data == '0' && buzzerActive) {
      // Open eyes detected, stop the buzzer
      digitalWrite(BUZZER_PIN, LOW);
      buzzerActive = false;
    }
  }
}
