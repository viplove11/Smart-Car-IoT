#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address 0x27 for a 16x2 I2C LCD
const int MQ3_PIN = A0;              // Analog pin connected to the MQ-3 sensor

void setup() {
  Serial.begin(9600);
  // lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);

  lcd.clear();
}

void loop() {
  int sensorValue = analogRead(MQ3_PIN);
  float voltage = sensorValue * (5.0 / 1024.0);  // Convert ADC value to voltage

  // Calculate alcohol concentration in ppm
  float alcoholValue = (voltage - 0.1) * 100.0 / (0.8);
  // checking the condition
  Serial.print("Alcohol Concentration: ");
  Serial.print(alcoholValue);
  Serial.println(" ppm");
  if (alcoholValue > 290) {
    displayAlcoholConcentration(alcoholValue);
    delay(2000);
  } else {
    lcd.clear();
    lcd.print("Alcohol Detector!!");
    delay(9000);
  }
}
void displayAlcoholConcentration(float concentration) {
  lcd.clear();
  lcd.print("Alcohol Detected ");
  lcd.setCursor(0, 1);
  lcd.print("= ");
  lcd.print(concentration);
  lcd.print(" ppm");
}
