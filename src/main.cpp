#include <Arduino.h>
#include <ReadDataFromSensor.h>

ReadData ReadDataSensor;

void setup() {
    Serial.begin(115200);

    ReadDataSensor.begin();
}

void loop()
{
    ReadDataSensor.update();

    float A = ReadDataSensor.getAccelTotal();
    float angleX = ReadDataSensor.getAngleX();
    float angleY = ReadDataSensor.getAngleY();

    Serial.print("AccelTotal: ");
    Serial.println(A);

    Serial.print("AngleX: ");
    Serial.println(angleX);

    Serial.print("AngleY: ");
    Serial.println(angleY);
    Serial.println("========================================");

    delay(1000);
}
