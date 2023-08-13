
#include "MPU6050.h"
#include "Wire.h"

#define ACCEL_CONFIG_ADDR	0x1C
#define GYRO_CONFIG_ADDR	0x1B

#define ACCEL_VALUE_ADDR	0x3B
#define GYRO_VALUE_ADDR		0x43

#define G_F 9.80665f
#define G_D 9.80665

namespace MPU6050 {

	void MPU6050::setDefault() {
		accelRange = AccelRange::_2G;
		gyroRange = GyroRange::_250dS;
	}

	void MPU6050::begin() {
		wire = &Wire;
		wire->begin();
		addr = MPU6050_DEFAULT_ADDR;
		setDefault();
	}

	void MPU6050::begin(uint8_t mpuAddr) {
		wire = &Wire;
		wire->begin();
		addr = mpuAddr;
		setDefault();
	}

	void MPU6050::begin(TwoWire& twoWire) {
		wire = &twoWire;
		addr = MPU6050_DEFAULT_ADDR;
		setDefault();
	}

	void MPU6050::begin(TwoWire& twoWire, uint8_t mpuAddr) {
		wire = &twoWire;
		addr = mpuAddr;
		setDefault();
	}

	
	void MPU6050::setAccelRange(AccelRange range) {
		accelRange = range;
		wire->beginTransmission(addr);
		wire->write(ACCEL_CONFIG_ADDR);
		wire->write((uint8_t)range);
		wire->endTransmission();
	}

	void MPU6050::setGyroRange(GyroRange range) {
		gyroRange = range;
		wire->beginTransmission(addr);
		wire->write(GYRO_CONFIG_ADDR);
		wire->write((uint8_t)range);
		wire->endTransmission();
	}

	int MPU6050::getRaw(char *buf, int n, uint8_t from) {
		wire->beginTransmission(addr);
		wire->write(from);
		wire->endTransmission();

		wire->requestFrom(addr, n);
		if (wire->available() < n) {
			return -1;
		}

		for (int i = 0; i < n; i++) {
			buf[i] = wire->read();
		}

		return 0;
	}


	int MPU6050::getAccel(double* out) {
		char buf[6];
		int16_t raw;
		double factor;

		if (getRaw(buf, 6, ACCEL_VALUE_ADDR)) {
			return -1;
		}

		char x = (int)accelRange >> 3;
		factor = 16384 >> x;

		for (int i = 0; i < 3; i++) {
			raw = ((int16_t)buf[2*i] << 8) + (int16_t)buf[2*i + 1];
			out[i] = (double)raw * G_D / factor;
		}

		return 0;
	}

	int MPU6050::getAccel(float* out) {
		char buf[6];
		int16_t raw;
		float factor;

		if (getRaw(buf, 6, ACCEL_VALUE_ADDR)) {
			return -1;
		}

		char x = (int)accelRange >> 3;
		factor = 16384 >> x;

		for (int i = 0; i < 3; i++) {
			raw = ((int16_t)buf[2*i] << 8) + (int16_t)buf[2*i + 1];
			out[i] = (float)raw * G_F / factor;
		}

		return 0;
	}

	int MPU6050::getGyro(double* out) {
		char buf[6];
		int16_t raw;
		double factor;

		if (getRaw(buf, 6, GYRO_VALUE_ADDR)) {
			return -1;
		}

		char x = (int)gyroRange >> 3;
		factor = 131.0 / (double)(1<<x);

		for (int i = 0; i < 3; i++) {
			raw = ((int16_t)buf[2*i] << 8) + (int16_t)buf[2*i + 1];
			out[i] = (double)raw / factor;
		}

		return 0;
	}

	int MPU6050::getGyro(float* out) {
		char buf[6];
		int16_t raw;
		float factor;

		if (getRaw(buf, 6, GYRO_VALUE_ADDR)) {
			return -1;
		}

		char x = (int)gyroRange >> 3;
		factor = 131.0f / (float)(1<<x);

		for (int i = 0; i < 3; i++) {
			raw = ((int16_t)buf[2*i] << 8) + (int16_t)buf[2*i + 1];
			out[i] = (float)raw / factor;
		}

		return 0;
	}

}
