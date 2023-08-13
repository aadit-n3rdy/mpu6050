#include "MPU6050.h"

MPU6050::MPU6050 mpu;
float accel[3];
float gyro[3];

void setup() {
	Serial.begin(115200);
	delay(50);
	mpu.begin();
	mpu.setAccelRange(MPU6050::AccelRange::_8G);
	mpu.setGyroRange(MPU6050::GyroRange::_2000dS);
}

void loop() {
	int i = mpu.getAccel(accel);
	if (i < 0) {
		Serial.println("Error!");
	}
	mpu.getGyro(gyro);
	for (int i= 0; i < 3; i++) {
		Serial.print(accel[i]);
		Serial.print(" ");
	}
	for (int i= 0; i < 3; i++) {
		Serial.print(gyro[i]);
		Serial.print(" ");
	}
	Serial.println("");
	delay(50);
}
