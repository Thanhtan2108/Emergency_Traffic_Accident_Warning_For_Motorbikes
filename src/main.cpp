#include <Arduino.h>
#include <Wire.h>
#include <ReadData.h>

MPU mpu;
ReadData readData(mpu);

void setup() {
  Serial.begin(115200);
  while(!Serial) {
    delay(10);
  }

  Serial.println("MPU6050 Test");

  Wire.begin();
  mpu.begin();

  Serial.println("Initialize MPU6050 successful");
}

void loop() {
  readData.update();

  MPUData data = readData.getData();

  Serial.println("------------ RAW DATA ------------");

  Serial.print("ACC  (g)  : ");
  Serial.print(data.accX, 3); Serial.print(" , ");
  Serial.print(data.accY, 3); Serial.print(" , ");
  Serial.println(data.accZ, 3);

  Serial.print("GYRO (d/s): ");
  Serial.print(data.gyroX, 3); Serial.print(" , ");
  Serial.print(data.gyroY, 3); Serial.print(" , ");
  Serial.println(data.gyroZ, 3);

  Serial.print("ANGLE(deg): ");
  Serial.print(data.angleX, 2); Serial.print(" , ");
  Serial.print(data.angleY, 2); Serial.print(" , ");
  Serial.println(data.angleZ, 2);

  Serial.print("ACC |a|   : ");
  Serial.println(readData.getTotalAcceleration(), 3);

  Serial.print("GYRO |w|  : ");
  Serial.println(readData.getTotalGyroscope(), 3);

  Serial.println("----------------------------------\n");

  delay(200);
}
