#define trigPin1 A0
#define echoPin1 A1
#define trigPin2 A2
#define echoPin2 A3
#define trigPin3 A4
#define echoPin3 A5
#define trigPin4 A6
#define echoPin4 A7
int ALARM = 7;
long duration1, distance1, duration2, distance2, duration3, distance3, duration4, distance4;

void setup()
{
  Serial.begin(9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(ALARM, OUTPUT);
  digitalWrite(ALARM, LOW);
}

void loop()
{
  SonarSensor(trigPin1, echoPin1, distance1);
  SonarSensor(trigPin2, echoPin2, distance2);
  SonarSensor(trigPin3, echoPin3, distance3);
  SonarSensor(trigPin4, echoPin4, distance4);

  // Print distances to Serial Monitor
  Serial.print("Distance 1: ");
  Serial.println(distance1);
  Serial.print("Distance 2: ");
  Serial.println(distance2);
  Serial.print("Distance 3: ");
  Serial.println(distance3);
  Serial.print("Distance 4: ");
  Serial.println(distance4);

  // Check for collision (adjust the distance threshold as needed)
  if ((distance1 >= 10 && distance1 <= 50) ||
      (distance2 >= 10 && distance2 <= 50) ||
      (distance3 >= 10 && distance3 <= 50) ||
      (distance4 >= 10 && distance4 <= 50))
  {
    digitalWrite(ALARM, HIGH);
    delay(500);
    digitalWrite(ALARM, LOW);
    delay(500);
  }

  delay(1000);
}

void SonarSensor(int trigPin, int echoPin, long &distance)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration1 = pulseIn(echoPin, HIGH);
  distance = (duration1 / 2) / 29.1;
}
