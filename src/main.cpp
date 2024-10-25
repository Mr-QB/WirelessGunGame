#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Định nghĩa chân cho các nút
const int buttonPin = D3; // Chân của nút

// Định nghĩa các biến cho sai số không bằng không
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
  Serial.begin(115200); // Đặt baudrate
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 sensor");
    while (1)
      ;
  }
  Serial.println("MPU6050 initialized");

  // Thực hiện hiệu chỉnh cảm biến
  calibrateGyro();

  // Cài đặt chân nút
  pinMode(buttonPin, INPUT_PULLUP); // Sử dụng pull-up nội bộ
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Điều chỉnh dữ liệu con quay hồi chuyển dựa trên sai số không bằng không
  double x = g.gyro.x - offsetX;
  double y = g.gyro.y - offsetY;
  double z = g.gyro.z - offsetZ;

  // Đọc trạng thái của nút
  int buttonState = digitalRead(buttonPin);

  // Gửi dữ liệu gia tốc, con quay hồi chuyển và trạng thái nút dưới dạng văn bản ASCII
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
