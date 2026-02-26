#include <Arduino.h>
#include <Wire.h>

#include "SensorReader.h"
#include "Signalprocessor.h"

SensorReader sensor;
SignalProcessor processor;

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("Starting sensor reader...");

    Wire.begin();

    if (!sensor.begin(Wire, true)) {
        Serial.print("Sensor init failed: ");
        Serial.println(sensor.getLastError());
    } else {
        Serial.println("Sensor initialized.");
    }

    processor.begin();

    Serial.println("ts,ax,ay,az,gx,gy,gz,roll,pitch,magnitude_a,magnitude_a_raw,magnitude_omega,baseline,baseline_ready");
}

void loop() {
    RawSensorData raw = sensor.read();

    if (!raw.valid) {
        Serial.print("Sensor invalid: ");
        Serial.println(sensor.getLastError());
    } else {
        ProcessedData p = processor.process(raw);

        // CSV output
        Serial.print(raw.timestamp); Serial.print(",");
        Serial.print(raw.ax, 6); Serial.print(",");
        Serial.print(raw.ay, 6); Serial.print(",");
        Serial.print(raw.az, 6); Serial.print(",");
        Serial.print(raw.gx, 6); Serial.print(",");
        Serial.print(raw.gy, 6); Serial.print(",");
        Serial.print(raw.gz, 6); Serial.print(",");
        Serial.print(raw.roll, 2); Serial.print(",");
        Serial.print(raw.pitch, 2); Serial.print(",");
        Serial.print(p.magnitude_a, 6); Serial.print(",");
        Serial.print(p.magnitude_a_raw, 6); Serial.print(",");
        Serial.print(p.magnitude_omega, 6); Serial.print(",");
        Serial.print(p.baseline, 6); Serial.print(",");
        Serial.println(p.baseline_ready ? "1" : "0");
    }

    delay(SENSOR_PERIOD_MS);
}
