const int trigPin = 9;  // Pin connected to the trigger pin on the ultrasonic sensor
const int echoPin = 10; // Pin connected to the echo pin on the ultrasonic sensor
const int buzzerPin = 8; // Pin connected to the control pin of the buzzer

const int thresholdDistance = 10; // Set your threshold distance in centimeters

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

void loop() { 
  // Triggering the ultrasonic sensor to send a pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(1);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(trigPin, LOW);

  // Reading the echo pulse duration and calculating distance
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration / 58.2; // Convert the duration to centimeters

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check if the distance is below the threshold
  if (distance > thresholdDistance) {
    // Sound the buzzer
    digitalWrite(buzzerPin, HIGH);
  } else {
    // Silence the buzzer
    digitalWrite(buzzerPin, LOW);
  }

  delay(1000); // Adjust the delay as needed for your application
}
