
#ifndef MPU6050_H
#define MPU6050_H

#include "Arduino.h"
#include <stdint.h>
#include "Wire.h"

namespace MPU6050 {

	const uint8_t MPU6050_DEFAULT_ADDR = 0x68;

	enum class AccelRange {
		_2G  = 0,
		_4G  = 1<<3,
		_8G  = 2<<3,
		_16G = 3<<3
	};

	enum class GyroRange {
		_250dS  = 0,
		_500dS  = 1<<3,
		_1000dS = 2<<3,
		_2000dS = 3<<3
	};

	class MPU6050 {
		private:
			AccelRange accelRange;
			GyroRange gyroRange;
			TwoWire *wire;
			uint8_t addr;
			void setDefault();
			int getRaw(char *buf, int n, uint8_t from);
		public:
			void begin();
			void begin(uint8_t mpuAddr);
			void begin(TwoWire& twoWire);
			void begin(TwoWire& twoWire, uint8_t mpuAddr);
			void reset();

			void setAccelRange(AccelRange range);
			int getAccel(double* out); 
			int getAccel(float* out);

			void setGyroRange(GyroRange range);
			int getGyro(double *out);
			int getGyro(float* out);
	};

}

#endif
