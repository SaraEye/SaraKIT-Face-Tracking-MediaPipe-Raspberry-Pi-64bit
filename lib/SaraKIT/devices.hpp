#include <numeric>

typedef unsigned char uchar;

struct InfoAccGyro{
	float accX;
	float accY;
	float accZ;
	float gyroX;
	float gyroY;
	float gyroZ;
	float temp;
	int16_t rawAccX;
	int16_t rawAccY;
	int16_t rawAccZ;
	int16_t rawGyroX;
	int16_t rawGyroY;
	int16_t rawGyroZ;
};

struct InfoEncoders{
	float angle;
	float velocity;
	float angleDeg;
	float velocityDeg;
	int16_t direction;
	float zeroAngle;
};

struct initFOC{
	int16_t ready;
	float angleZero;
	int16_t direction;
};

struct GetInfoResult {
	uint16_t idata1;
	uint16_t idata2;
	uint16_t idata3;
	float fdata1;
	float fdata2;
	float fdata3;
	bool bdata1;
	bool bdata2;
	bool bdata3;
};

#define _PI	    3.14159265358979323846	/* pi */
#define _2PI    6.28318530717958647693  /* 2pi */
#define DEG_TO_RAD  (_PI / 180)
#define RAD_TO_DEG  (180.0 / _PI)

extern "C" {
void sleepms(int ms);
void sendCommandSet(uchar *buffer);
void sendCommandGet(uchar *buffer, int len);

bool _SPICheck();
void BLDCMotor_Reset();
//void BLDCMotor_Reboot();

//acc+gyro+temp
void Sensors_On(uchar enable);
InfoAccGyro getAccGyro();
float getTemperature();
uint8_t LSM6DS3TR_readRegisters(uint8_t address, uint8_t* data, uint8_t length);
void LSM6DS3TR_writeRegisters(uint8_t address, uint8_t data);
void GyroCalib();
void AccCalib();

//encoders
void Encoder_On(uchar EncoderId, uchar enable);
void Encoder_Param(uchar EncoderId, uint16_t minpwm, uint16_t maxpwm);
void Encoder_LowPassFilter(uchar EncoderId, float angle, float velocity);
InfoEncoders Encoder_Get(uchar EncoderId);
InfoEncoders Encoder_GetParam(uchar EncoderId);

//gimbal motors
void BLDCMotor_PolePairs(uchar motorId, uchar polePairs);
void BLDCMotor_On(uchar motorId, bool enable);
void BLDCMotor_IdleTorque(uchar motorId, uchar torque, unsigned short torqueMs);
void BLDCMotor_PIDAngle(uchar motorId, float P, float I, float D, float ramp);
void BLDCMotor_PIDVelocity(uchar motorId, float P, float I, float D, float ramp);
initFOC BLDCMotor_InitFOC(uchar motorId, uchar encoderId, int direction, float angle);
void BLDCMotor_MoveToAngle(uchar motorId, float angle, float speed, uchar torque, bool degrees);
void BLDCMotor_MoveByAngle(uchar motorId, float angle, float speed, uchar torque, bool degrees);
void BLDCMotor_DriveMeters(uchar motorId, float centimeters, float speed, uchar torque, float WhellDiameter);
void BLDCMotor_MoveContinuousTorque(uchar motorId, int direction, uchar torque);
void BLDCMotor_MoveContinuousVelocity(uchar motorId, int direction, uchar torque, float speed, bool degrees);
void BLDCMotor_MoveStop(uchar motorId);
GetInfoResult BLDCMotor_GetInfo(uchar motorId, uchar info, bool zprint);
}
