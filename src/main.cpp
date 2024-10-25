#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Define pin for the button
const int buttonPin = D3; // Pin for the button

// Define variables for the zero-offset error
double offsetX, offsetY, offsetZ;

void calibrateGyro()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  offsetX = g.gyro.x;
  offsetY = g.gyro.y;
  offsetZ = g.gyro.z;

  Serial.println("Gyro calibrated");
}

void setup()
{
  Serial.begin(115200); // Set baud rate
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 sensor");
    while (1)
      ;
  }
  Serial.println("MPU6050 initialized");

  // Perform sensor calibration
  calibrateGyro();

  // Set up button pin
  pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Adjust gyro data based on zero-offset error
  double x = g.gyro.x - offsetX;
  double y = g.gyro.y - offsetY;
  double z = g.gyro.z - offsetZ;

  // Read the button state
  int buttonState = digitalRead(buttonPin);

  // Send acceleration, gyro data, and button state as ASCII text
  // Serial.print("DATAL,");
  // Serial.print(a.acceleration.x);
  // Serial.print(",");
  // Serial.print(a.acceleration.y);
  // Serial.print(",");
  // Serial.println(a.acceleration.z);

  Serial.print("DATAG,");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);

  Serial.print("BUTTON,");
  Serial.println(buttonState == LOW ? "PRESSED" : "RELEASED");

  // delay(10);
}
