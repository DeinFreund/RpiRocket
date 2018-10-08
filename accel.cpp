#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <thread>
#include <chrono>
#include <ctime>
#include "mtQuaternions.h"
#include <pigpio.h>
#include <bits/stdc++.h>

#define LOG_RAW "sensors_raw.log"
#define LOG_PROCESSED "sensors_pro.log"
#define DEBUG 0


// COMPILE USING -std=c++11
using namespace std;

const float MY_PI = 3.14159265358979323f;

const int MIN_SERVO = 144;
const int MAX_SERVO = 85;

const float G = 9.81;
const float INIT_BEEP_PERIOD = 1.5;
const float CALIBRATION_BEEP_PERIOD = 0.5;
const float PRE_LAUNCH_BEEP_PERIOD = 3;
const float POST_LAUNCH_BEEP_PERIOD = 0.1;
const float CALIBRATION_TIME = 60;
const float INIT_TIME = 150;
const int INIT = -1;
const int CALIBRATION = 0;
const int PRE_LAUNCH = 1;
const int POST_LAUNCH = 2;
const int HMC5883L_I2C_ADDR = 0x1E;
const int DEFAULT_PITCH = 5;
const int DEFAULT_YAW = 0;
const int DEFAULT_ENGINE = -50;
const int DEFAULT_ROLL = 0;
const float DEFAULT_PITCH_SENSITIVITY = 0.9;
const float DEFAULT_ROLL_SENSITIVITY = 1.1;
const float DEFAULT_YAW_SENSITIVITY = 0.0;
const int MPU6050_I2C_ADDR = 0x68;
const int BMP280_I2C_ADDR = 0x77;//0b11101100;
const int BMP280_I2C_ADDR_2 = 0b11101110;
bool isBMP280 = false;
const int TIMOUT_CLICKS = 10;
const float GYRO_WEIGHT = 5; // used for correcting inertial measurement
const float ALTI_WEIGHT_VEL = 0.002;
const float ALTI_WEIGHT_POS = 0.05;

float AUTO_PITCH_SENSITIVITY = DEFAULT_PITCH_SENSITIVITY;
float AUTO_ROLL_SENSITIVITY = DEFAULT_ROLL_SENSITIVITY;
float AUTO_YAW_SENSITIVITY = DEFAULT_YAW_SENSITIVITY;
float ZROT_OFFSET = 0;
float YROT_OFFSET = 0;
float XROT_OFFSET = 0;

int msgPitch = DEFAULT_PITCH;
int msgRoll = DEFAULT_ROLL;
int msgYaw = DEFAULT_YAW;
int msgEngine = DEFAULT_ENGINE;

float deltaTime;

int flightMode;
std::chrono::high_resolution_clock::time_point initTime;
std::chrono::high_resolution_clock::time_point calibrationTime;
float lastBeepSwitch;
float launchTime;

//calibration data
float xaccsum = 0;
float yaccsum = 0;
float zaccsum = 0;
float xgyrosum = 0;
float ygyrosum = 0;
float zgyrosum = 0;
float xgyrodrift = 0;
float ygyrodrift = 0;
float zgyrodrift = 0;
float accmult = 1;
float startAlti = -1;
//rotation matrix
float Czx, Czy, Czz, Cyx, Cyy, Cyz, Cxx, Cxy, Cxz;

//sensor queue;
struct SensorData {
	short compassX, compassY, compassZ;
	short accelX, accelY, accelZ;
	short gyroX, gyroY, gyroZ;
	short gyroT;
	int pressP, pressT;
	float time;
};
int sensorQueueIndex = 0;
const int maxSensorQueueLength = 32;
int sensorQueueLength = maxSensorQueueLength;
struct SensorData sensorQueue[maxSensorQueueLength] = { {0} };

const int integrationBufferLength = 5;
float positionBuffer[4][integrationBufferLength];
float velocityBuffer[4][integrationBufferLength];
float accelerationBuffer[4][integrationBufferLength];
float timeBuffer[8][integrationBufferLength];


//current sensor data
std::chrono::high_resolution_clock::time_point lastReadTime;
std::chrono::high_resolution_clock::time_point lastPrint;
std::chrono::high_resolution_clock::time_point lastMPURead;
MTQuaternion rot;
float xpos = 0;
float ypos = 0;
float zpos = 0;
float zpos2 = 0;
float xvel = 0;
float yvel = 0;
float zvel = 0;
float zvel2 = 0;
float xacc = 0;
float yacc = 0;
float zacc = 0;
float acceleration = 0;
int errorCount = 0;
bool beepState = 0;

//old sensor data
float oldgyrox, oldgyroy, oldgyroz;
float oldaccx, oldaccy, oldaccz;
float oldalti = -1;
int tempCountdown = 10;
float lastAlti = 0;
float lastAltiTime = 0;
float lastAltiDeltaTime = 0;
float lastAltiSpeed = 0;

int fd;

std::vector<char> controlTable;
vector<float> consts;
vector<float> minAltitude;
vector<float> bandHeight;
uint16_t altiSize;
uint16_t width;
uint16_t oversampling;
float VELOCITY_RESOLUTION;

std::vector<char> ReadAllBytes(char const* filename)
{
    ifstream ifs(filename, ios::binary|ios::ate);
    ifstream::pos_type pos = ifs.tellg();

    std::vector<char>  result(pos);

    ifs.seekg(0, ios::beg);
    ifs.read(&result[0], pos);

    return result;
}



float sign(float num){
	return num >= 0 ? 1 : -1;
}

void selectDevice(int fd, int addr, char * name) {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        fprintf(stderr, "%s not present\n", name);
    }
}

void writeToDevice(int fd, int reg, int val) {
    char buf[2];
    buf[0] = reg;
    buf[1] = val;

    if (write(fd, buf, 2) != 2) {
        fprintf(stderr, "Can't write to device\n");
    }
}

int onerr=0;
short readShort(int fd, int reg){

	unsigned char buf[16];
	buf[0]=reg;
	if (write(fd,buf,1) != 1){
		cout << "MPU6050: can't tell device to read from buffer" << endl;
		onerr = 10;
	}
	if (read(fd,buf,2) != 2){
		cout << "MPU6050: can't read from device" << endl;
		onerr = 10;
	}
	short res = (buf[0] << 8) + buf[1];
	if (res >= 32768){
		return -1*((65536 - res) + 1);
	}
	return res;
}
short readShortInv(int fd, int reg){

	unsigned char buf[16];
	buf[0]=reg;
	if (write(fd,buf,1) != 1){
		cout << "MPU6050: can't tell device to read from buffer" << endl;
		onerr = 10;
	}
	if (read(fd,buf,2) != 2){
		cout << "MPU6050: can't read from device" << endl;
		onerr = 10;
	}
	short res = (buf[1] << 8) + buf[0];
	if (res >= 32768){
		return -1*((65536 - res) + 1);
	}
	return res;
}
long long readLong(int fd, int reg, int bytes){

	unsigned char buf[16];
	buf[0]=reg;
	if (write(fd,buf,1) != 1){
		cout << "MPU6050: can't tell device to read from buffer" << endl;
		onerr = 10;
	}
	if (read(fd,buf,bytes) != bytes){
		cout << "MPU6050: can't read from device" << endl;
		onerr = 10;
	}
	long long res = 0;
	for (int i = 0; i < bytes; i++){
		res <<= 8;
		res += buf[i];
	}
	return res;
}
short readByte(int fd, int reg){

	unsigned char buf[16];
	buf[0]=reg;
	if (write(fd,buf,1) != 1){
		cout << "MPU6050: can't tell device to read from buffer" << endl;
		onerr = 10;
	}
	if (read(fd,buf,1) != 1){
		cout << "MPU6050: can't read from device" << endl;
		onerr = 10;
	}
	short res = buf[0];
	return res;
}


bool sendping = false;
char pingval;


float avgangle(float a, float b, float fac){
	a = a * MY_PI / 180.0;
	b = b * MY_PI / 180.0;
	float x = cos(a) + fac * cos(b);
	float y = sin(a) + fac * sin(b);
	return 180/MY_PI * atan2(y,x);
}

float mod(float a, float b){
	return fmod(fmod(a,b) + b, b);
}
/*
float abs(float a){
	return a ? (a > 0) : (-1*a);
}*/

std::chrono::high_resolution_clock::time_point getTime(){
	return chrono::high_resolution_clock::now();
}

unsigned short dig_T1;
short dig_T2;
short dig_T3;
unsigned short dig_P1;
short dig_P2;
short dig_P3;
short dig_P4;
short dig_P5;
short dig_P6;
short dig_P7;
short dig_P8;
short dig_P9;

short dig_AC1;
short dig_AC2;
short dig_AC3;
unsigned short dig_AC4;
unsigned short dig_AC5;
unsigned short dig_AC6;
short dig_B1;
short dig_B2;
short dig_MB;
short dig_MC;
short dig_MD;
long dig_B5;


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of -5123- equals 51.23 DegC.
// t_fine carries fine temperature as global value
int t_fine;
float bmp280_temp(int adc_T)
{
	int var1, var2, T;
	var1 = ((((adc_T>>3) - ((int)dig_T1<<1))) * ((int)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int)dig_T1)) * ((adc_T>>4) - ((int)dig_T1))) >> 12) *	((int)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T / 100.0f;
}
float bmp280_press(int adc_P)
{
	long long var1, var2, p;
	var1 = ((long long)t_fine) - 128000;
	var2 = var1 * var1 * (long long)dig_P6;
	var2 = var2 + ((var1*(long long)dig_P5)<<17);
	var2 = var2 + (((long long)dig_P4)<<35);
	var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
	var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((long long)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
	return ((unsigned int)p) / 256.0f;
}
float bmp085_temp(unsigned int adc_T)
{
	int ut = adc_T;
    if(ut == 0) return NAN;
    int x1 = ((ut - (int)dig_AC6) * (int)dig_AC5) >> 15;
    int x2 = ((int)dig_MC << 11) / (x1 + dig_MD);
    dig_B5 = x1 + x2;
    return (float)((dig_B5 + 8) >> 4) / 10.0f;
}
float bmp085_press(unsigned int adc_P)
{
    unsigned int up = adc_P;
    if(up == 0) return NAN;
    unsigned char oss = (0xF4 & 0xC0) >> 6;
    int p;
    int b6 = dig_B5 - 4000;
    int x1 = ((int)dig_B2 * ((b6 * b6) >> 12)) >> 11;
    int x2 = ((int)dig_AC2 * b6) >> 11;
    int x3 = x1 + x2;
    int b3 = ((((int)dig_AC1 * 4 + x3) << oss) + 2) >> 2;
    x1 = ((int)dig_AC3 * b6) >> 13;
    x2 = ((int)dig_B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    unsigned int b4 = ((unsigned int)dig_AC4 * (unsigned int)(x3 + 32768)) >> 15;
    unsigned int b7 = ((unsigned int)up - b3) * (unsigned int)(50000UL >> oss);
    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    return p + ((x1 + x2 + (int)3791) / 16.0f);
}
float altitude(float press){
	return 44330 * (1 - pow(press/101325, 1/5.255f));
}

float integrate(float (& values)[integrationBufferLength], float (& derivative)[integrationBufferLength], float (& times)[integrationBufferLength], float measurement, float time){
	float y0 = values[0];
	float y1 = values[1];
	float y2 = values[2];
	float y3 = values[3];
	float t0 = times[0];
	float t1 = times[1];
	float t2 = times[2];
	float t3 = times[3];
	float t4 = time;
	float f0 = derivative[0];
	float f1 = derivative[1];
	float f2 = derivative[2];
	float f3 = derivative[3];
	float f4 = measurement;

	//float y5 = y4 + t4 * f4 * 1901 / 720.0 - t3 * f3 * 1387 / 360.0 + t2 * f2 * 109 / 30.0 - t1 * f1 * 637 / 360.0 + t0 * f0 * 251 / 720.0;//adams bashforth
	float y4 = y3 + t4 * f4 * 251.0f / 720.f + t3 * f3 * 646.f / 720.0f - t2 * f2 * 264.f / 720.0f + t1 * f1 * 106.f / 720.0f - t0 * f0 * 19.f / 720.0f; //adams moulton

	values[0] = y1;
	values[1] = y2;
	values[2] = y3;
	values[3] = y4;
	times[0] = t1;
	times[1] = t2;
	times[2] = t3;
	times[3] = t4;
	derivative[0] = f1;
	derivative[1] = f2;
	derivative[2] = f3;
	derivative[3] = f4;

	return y4;
}

FILE * rawFile;
FILE * processedFile;

void initLog(){
	processedFile = fopen(LOG_PROCESSED, "a");
	rawFile = fopen(LOG_RAW, "a");
}


void initSensors(){

	if (gpioInitialise() < 0){ // initialise the pins of leds and buzzers (noise one)
		cerr << "failed initialising gpio" << endl;
	}
	gpioWrite(4,0);
    unsigned char buf[16];

    if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
        // Open port for reading and writing
        fprintf(stderr, "Failed to open i2c bus\n");

        //return 1;
    }

    // initialize HMC5883L => magnetometer

    selectDevice(fd, HMC5883L_I2C_ADDR, "HMC5883L");

    writeToDevice(fd, 0x00, 0x70);
    writeToDevice(fd, 0x01, 0xA0);
    writeToDevice(fd, 0x02, 0x00);


	//initialize MPU6050 => gyroscope and accelerometer

  selectDevice(fd, MPU6050_I2C_ADDR, "MPU6050");

	writeToDevice(fd,0x6b,0x00); // wake up
	writeToDevice(fd,0x1c,0b00011000); // set accel to +- 16g
	writeToDevice(fd,0x1b,0b00011000); // set gyro to +- 2000deg
	writeToDevice(fd,0x1a,0b00000110); // set lowpass to 5hz
	writeToDevice(fd,0x38,0b00000001); // interrupt data

	//initialise BMP280 and bmp085 barometers
	// detect which sensor of the two is connected
  selectDevice(fd, BMP280_I2C_ADDR, "BMP280");
	isBMP280 = readByte(fd, 0xD0) == 0x58;
	cout << "is bmp280: " << isBMP280 << endl;

	if (isBMP280){
		writeToDevice(fd,0xf4, 0b10110111); // configure measurements
		writeToDevice(fd,0xf5, 0b00000000); // configure filters

		dig_T1 = readShortInv(fd, 0x88);//calibration values
		dig_T2 = readShortInv(fd, 0x8A);
		dig_T3 = readShortInv(fd, 0x8C);
		dig_P1 = readShortInv(fd, 0x8E);
		dig_P2 = readShortInv(fd, 0x90);
		dig_P3 = readShortInv(fd, 0x92);
		dig_P4 = readShortInv(fd, 0x94);
		dig_P5 = readShortInv(fd, 0x96);
		dig_P6 = readShortInv(fd, 0x98);
		dig_P7 = readShortInv(fd, 0x9A);
		dig_P8 = readShortInv(fd, 0x9C);
		dig_P9 = readShortInv(fd, 0x9E);
		cout << dig_T1 << ", " << dig_T2 << ", " << dig_T3 << ", " << dig_P1 << ", "  << dig_P2 << ", "  << dig_P3 << ", "
				<< dig_P4 << ", "  << dig_P5 << ", "  << dig_P6 << ", "  << dig_P7 << ", "  << dig_P8 << ", "  << dig_P9 << endl;
	}else{
		// initialising bmp085 => barometer

		// use calibration constants from the factory
		dig_AC1 = readShort(fd, 0xAA);
		dig_AC2 = readShort(fd, 0xAC);
		dig_AC3 = readShort(fd, 0xAE);
		dig_AC4 = readShort(fd, 0xB0);
		dig_AC5 = readShort(fd, 0xB2);
		dig_AC6 = readShort(fd, 0xB4);
		dig_B1 = readShort(fd, 0xB6);
		dig_B2 = readShort(fd, 0xB8);
		dig_MB = readShort(fd, 0xBA);
		dig_MC = readShort(fd, 0xBC);
		dig_MD = readShort(fd, 0xBE);

		writeToDevice(fd,0xf4, 0x2e); // measure temp
		usleep(10000);
		unsigned int rawtemp = readLong(fd, 0xF6, 2) ;
		float temp = bmp085_temp(rawtemp);
		cout <<  "bmp085 calibration temperature: " <<  temp << endl;
		writeToDevice(fd,0xf4, 0xf4); // configure maximum oversampling
	}

	lastReadTime = getTime();
	initTime = lastReadTime;
	lastPrint = lastReadTime;
	lastMPURead = lastReadTime;

}


void finishCalibration(){
	float dt = ((chrono::duration<float>)(lastReadTime - calibrationTime)).count();
	xaccsum /= dt;
	yaccsum /= dt;
	zaccsum /= dt;
	xgyrosum /= dt;
	ygyrosum /= dt;
	zgyrosum /= dt;
	xgyrodrift = xgyrosum;
	ygyrodrift = ygyrosum;
	zgyrodrift = zgyrosum;

	fprintf(processedFile, "-42 GYRO_CALIB %.7g %.7g %.7g\n", xgyrodrift, ygyrodrift, zgyrodrift);
	cout << xaccsum << "|" << yaccsum << "|" << zaccsum  << endl;

	//float z = -atan2(yaccsum, xaccsum);
	//float y = atan2(zaccsum, sqrt(xaccsum * xaccsum + yaccsum * yaccsum));
	//cout << y << "degs, z: " << z << "degs "  << endl;

	accmult = 1.0 / sqrt(xaccsum * xaccsum + yaccsum * yaccsum + zaccsum * zaccsum);
	MTVec3D target = mtNormVector3D({0,0,1});
	MTVec3D measured = mtNormVector3D({xaccsum, yaccsum, zaccsum});
	rot = mtCreateMTQuaternion(mtNormVector3D(mtCrossProduct3D(measured, target)), mtAngleVectorVector(measured, target));
	MTVec3D target2 = mtNormVector3D({1,0,0});
	MTVec3D measured2 = mtRotatePointWithMTQuaternion(rot, target);
	measured2.z = 0;
	measured2 = mtNormVector3D(measured2);
	MTQuaternion xcorrect = mtCreateMTQuaternion(mtNormVector3D(mtCrossProduct3D(measured2, target2)), mtAngleVectorVector(measured2, target2));
	rot = mtMultMTQuaternionMTQuaternion(&xcorrect, &rot);

	startAlti = -1;
	zpos = 0;
	/*
	Cxx = cos(y) * cos(z);
	Cxy = -cos(y) * sin(z);
	Cxz = sin(y);
	Cyx = sin(z);
	Cyy = cos(z);
	Cyz = 0;
	Czx = -sin(y) * cos(z);
	Czy = sin(y) * sin(z);
	Czz = cos(y);
	/*
	Cxx = cos(y) * cos(z);
	Cxy = -sin(z);
	Cxz = cos(z) * sin(y);
	Cyx = cos(y) * sin(z);
	Cyy = cos(z);
	Cyz = sin(y) * sin(z);
	Czx = -sin(y);
	Czy = 0;
	Czz = cos(y);
	*/
}

void updateFlightMode(SensorData newReadings, SensorData bufferedReadings){

	if (bufferedReadings.gyroT != 0 && flightMode == INIT && ((chrono::duration<float>)(getTime() - initTime)).count() > INIT_TIME){
		flightMode = CALIBRATION;
		calibrationTime = lastReadTime;
		cout << "Entering flight mode: CALIBRATION" << endl;
	}
	if (((chrono::duration<float>)(getTime() - initTime)).count() > CALIBRATION_TIME + INIT_TIME && flightMode == CALIBRATION){
		flightMode = PRE_LAUNCH;
		finishCalibration();
		cout << "Entering flight mode: PRE_LAUNCH" << endl;
		fprintf(processedFile, "-42 CALIBRATED\n");
		fflush(processedFile);
	}
	float accelx, accely, accelz;
	accelx = newReadings.accelX / 2048.0 * accmult;
	accely = newReadings.accelY / 2048.0 * accmult;
	accelz = newReadings.accelZ / 2048.0 * accmult;
	float totAccel = sqrt(accelx * accelx + accely * accely + accelz * accelz);
	cout << "totAccel is " << totAccel << endl;;
	if (totAccel > 2 && flightMode < POST_LAUNCH){
		if (flightMode == CALIBRATION){
			finishCalibration();
			cout << "ABORTED CALIBRATION" << endl;
			fprintf(processedFile, "-42 CALIBRATION ABORTED\n");
			fflush(processedFile);
		}
		launchTime = ((chrono::duration<float>)(lastReadTime - initTime)).count();
		flightMode = POST_LAUNCH;
		cout << "Entering flight mode: POST_LAUNCH" << endl;
	}
}

float getBraking(float velocity_float, float altitude_float){
	float targetBraking;
	uint16_t velIndex = lround(velocity_float / VELOCITY_RESOLUTION);
	uint16_t velIndexLimits = lround(oversampling * velocity_float / VELOCITY_RESOLUTION);
	if (altitude_float < 0 || velocity_float < 0){
		targetBraking = 0x00;
	}else if (velIndex >= width){
		targetBraking = 0xFF;
	}else{
		//if (velIndex >= lastVelocityIndex) continue; //make sure control is only updated near known velocities to reduce errors.
		
		int32_t altIndex = lround(width * (altitude_float - minAltitude[velIndexLimits]) / bandHeight[velIndexLimits]);
		float err1 = altitude_float - altIndex * bandHeight[velIndexLimits] / width - minAltitude[velIndexLimits];
		float err2 = -altitude_float + (altIndex + 1) * bandHeight[velIndexLimits] / width + minAltitude[velIndexLimits];
		if (altIndex + 1 >= width) targetBraking = 0xFF;
		else if (altIndex < 0) targetBraking = 0x00;
		//not within critical zone -> linearly interpolate optimal control for current velocity
		else {
			uint8_t data1, data2;
			data1 = controlTable[width * altIndex + velIndex];
			data2 = controlTable[width * (altIndex + 1) + velIndex]; 
			targetBraking = lround(err2 * data1 / (err1 + err2) + err1 * data2 / (err1 + err2)); 
		}
	}		
	return targetBraking;
}


SensorData newReadings;
float lastProcessedTime = -1;
bool readSwap = 0;

bool firstPrint = 1;

void updateSensors(){

  readSwap = !readSwap;
  //cerr << sensorQueueLength << endl;
	if (flightMode < POST_LAUNCH || flightMode == POST_LAUNCH && sensorQueueLength == 1 || readSwap){
		newReadings.pressT = -1;
		newReadings.pressP = -1;

		// Process MPU 6050 => gyroscope and accelerometer
		selectDevice(fd, MPU6050_I2C_ADDR, "MPU6050");

		short interrupt = readByte(fd,0x3A); // is device ready to measure

		// make sure that sensros are initialised all the time
		// TODO test if each sensor is initialised all the time during the rocket flight
		bool deviceReady = interrupt & 0b00000001 > 0;
		if (!deviceReady) {
			cout << "not ready" << endl;
			errorCount ++;
			if (errorCount > 5){
			  cout << "Reinitializing sensors" << endl;
			  errorCount = 0;
			  initSensors();
			}
			return;
		}

		errorCount = 0;

		auto now = getTime();
		float time = ((chrono::duration<float>)(getTime() - initTime)).count();
		deltaTime = ((chrono::duration<float>)(now - lastReadTime)).count();
		lastReadTime = now;

		newReadings.gyroX = readShort(fd,0x43);
		newReadings.gyroY = readShort(fd,0x45);
		newReadings.gyroZ = readShort(fd,0x47);
		newReadings.accelX = readShort(fd,0x3b);
		newReadings.accelY = readShort(fd,0x3d);
		newReadings.accelZ = readShort(fd,0x3f);
		newReadings.gyroT = readShort(fd,0x41);
		newReadings.time = time;

		// Process BMP 280
		selectDevice(fd, BMP280_I2C_ADDR, "BMP280");

		if (isBMP280){
			newReadings.pressT = readLong(fd, 0xFA, 3);
			newReadings.pressP = readLong(fd, 0xF7, 3);
		}else{
			bool readed = false;
			float mdt = ((chrono::duration<float>)(now - lastMPURead)).count();
			if (tempCountdown > 0 && mdt > 0.0255){
				tempCountdown--;
				newReadings.pressP = readLong(fd, 0xF6, 3);
				lastMPURead = now;
				readed = 1;
			}else if (tempCountdown <= 0 && mdt > 0.005){
				tempCountdown = 50;
				newReadings.pressT = readLong(fd, 0xF6, 2);
				lastMPURead = now;
				readed = 1;
			}

			if (readed){
				if (tempCountdown == 0){
					writeToDevice(fd,0xf4, 0x2e); // measure temp
					//cout << "requesting temp" << endl;
				}else if (tempCountdown > 0){
					//cout << "requesting press" << endl;
					writeToDevice(fd,0xf4, 0xf4); // configure maximum oversampling
				}
			}
		}

		
		sensorQueue[sensorQueueIndex++] = newReadings;
		sensorQueueIndex %= maxSensorQueueLength;
	}else{
		sensorQueueLength--;
	}
	
	SensorData lastReadings;
	lastReadings = sensorQueue[(sensorQueueIndex + maxSensorQueueLength - sensorQueueLength) % maxSensorQueueLength];

	//cerr << "last read time " << lastReadings.time << endl;
	// Process MPU 6050 => gyroscope and accelerometer

	float gyrox, gyroy, gyroz;
	const float magic_number_from_docs = 16.4;
	gyrox = lastReadings.gyroX / magic_number_from_docs;
	gyroy = lastReadings.gyroY / magic_number_from_docs;
	gyroz = lastReadings.gyroZ / magic_number_from_docs;

	//fprintf(rawFile, "%.6g g %.7g %.7g %.7g\n", lastReadings.time, gyrox, gyroy, gyroz);
	gyrox -= xgyrodrift;
	gyroy -= ygyrodrift;
	gyroz -= zgyrodrift;


	float accelx, accely, accelz;
	// TODO fix magic numbers
	accelx = lastReadings.accelX / 2048.0;
	accely = lastReadings.accelY / 2048.0;
	accelz = lastReadings.accelZ / 2048.0;
	//fprintf(rawFile, "%.6g a %.7g %.7g %.7g\n", lastReadings.time, accelx, accely, accelz);
	accelx *= accmult;
	accely *= accmult;
	accelz *= accmult;

	float temp;
	// TODO fix magic numbers
	temp = lastReadings.gyroT / 340.0 + 36.53;

	acceleration = sqrt(accelx*accelx + accely*accely + accelz* accelz);




	// Process BMP 280

	float temp2 = -1;
	float press = -1;


	if (isBMP280){
		int rawtemp = lastReadings.pressT >> 4;
		int rawpress = lastReadings.pressP >> 4;
		temp2 = bmp280_temp(rawtemp);
		press = bmp280_press(rawpress);
	}else{

		if (lastReadings.pressP != -1){
			unsigned int rawpress = lastReadings.pressP >> 5;
			press = bmp085_press(rawpress);
			//cout << rawpress << " -> " << press << endl;
		}
		if (lastReadings.pressT != -1){
			unsigned int rawtemp = lastReadings.pressT;
			temp2 = bmp085_temp(rawtemp);
			//cout <<  "bmp085 calibration temperature: " <<  temp2 << endl;
		}
	}

	//if (press > 0) cout << press << " - > " << altitude(press) << endl;
	//cout << readShort(fd, 0xFA) << " | " << readByte(fd, 0xFC) << endl;
	//cout << rawtemp << " -> " << (temp2 / 100.0)   << " vs " << temp << endl;
	if (press > 0) {
		lastAltiSpeed = altitude(press) - lastAlti;
		lastAlti = altitude(press);
		lastAltiDeltaTime = lastReadings.time - lastAltiTime;
		lastAltiSpeed /= lastAltiDeltaTime;
		lastAltiTime = lastReadings.time;
		//fprintf(rawFile, "%.6g t %.7g %.7g\n", lastReadings.time, temp, temp2);
		//fprintf(rawFile, "%.6g p %.7g %.7g\n", lastReadings.time, press, altitude(press));
	}
	//fflush(rawFile); // dump data to the operating system

	

	float beepPeriod = 1;

	switch (flightMode){
		case INIT:
			beepPeriod = INIT_BEEP_PERIOD;
			{
			float targetBraking = 0;
			if (((chrono::duration<float>)(getTime() - initTime)).count() < 5) targetBraking = 0xFF;
			else if (((chrono::duration<float>)(getTime() - initTime)).count() < 10) targetBraking = 0x00;
			else if (((chrono::duration<float>)(getTime() - initTime)).count() < 15) targetBraking = 255 * (((chrono::duration<float>)(getTime() - initTime)).count() - 10) / 5.f;
			gpioServo(17, lround(MIN_SERVO + (targetBraking / 255.0 * (MAX_SERVO - MIN_SERVO))) * 10);
			}
			
			break;
		case CALIBRATION:
			xaccsum += accelx * deltaTime;
			yaccsum += accely * deltaTime;
			zaccsum += accelz * deltaTime;
			xgyrosum += gyrox * deltaTime;
			ygyrosum += gyroy * deltaTime;
			zgyrosum += gyroz * deltaTime;
			beepPeriod = CALIBRATION_BEEP_PERIOD;
			break;

		case PRE_LAUNCH:

			beepPeriod = PRE_LAUNCH_BEEP_PERIOD;
			break;

		case POST_LAUNCH:

			float deltaTime = (lastProcessedTime < 0) ? 0 : (lastReadings.time - lastProcessedTime);
			lastProcessedTime = lastReadings.time;
		
			//apply rotation
			MTVec3D curX = mtRotatePointWithMTQuaternion(rot, {1,0,0});
			MTVec3D curY = mtRotatePointWithMTQuaternion(rot, {0,1,0});
			MTVec3D curZ = mtRotatePointWithMTQuaternion(rot, {0,0,1});
			MTQuaternion rotX = mtCreateMTQuaternion(curX, 0.5f * (gyrox + oldgyrox) * MY_PI / 180 * deltaTime);
			MTQuaternion rotY = mtCreateMTQuaternion(curY, 0.5f * (gyroy + oldgyroy) * MY_PI / 180 * deltaTime);
			MTQuaternion rotZ = mtCreateMTQuaternion(curZ, 0.5f * (gyroz + oldgyroz) * MY_PI / 180 * deltaTime);
			rot = mtMultMTQuaternionMTQuaternion(&rotX, &rot);
			rot = mtMultMTQuaternionMTQuaternion(&rotY, &rot);
			rot = mtMultMTQuaternionMTQuaternion(&rotZ, &rot);
			mtNormMTQuaternion(&rot);

			float mtime = ((chrono::duration<float>)(lastReadings.time - launchTime)).count();
			//use gyro and acc estimations to get better result
			MTVec3D acc = mtRotatePointWithMTQuaternion(rot, {accelx, accely, accelz});
			xacc = acc.x;
			yacc = acc.y;
			zacc = acc.z - 1;
			/*
			xacc = Cxx * accelx + Cxy * accely + Cxz * accelz;
			yacc = Cyx * accelx + Cyy * accely + Cyz * accelz;
			zacc = Czx * accelx + Czy * accely + Czz * accelz;
			*/

			if (startAlti < 0) startAlti = lastAlti;

			float oxvel = xvel;
			float oyvel = yvel;
			float ozvel = zvel;
			float alti = lastAlti - startAlti;

			if (oldalti < 0) oldalti = alti;

			float altispeed = lastAltiSpeed;

			xvel = integrate(velocityBuffer[0], accelerationBuffer[0], timeBuffer[0], xacc * G, deltaTime);
			yvel = integrate(velocityBuffer[1], accelerationBuffer[1], timeBuffer[1], yacc * G, deltaTime);
			zvel = integrate(velocityBuffer[2], accelerationBuffer[2], timeBuffer[2], zacc * G, deltaTime);
			zvel2 = integrate(velocityBuffer[3], accelerationBuffer[3], timeBuffer[3], zacc * G, deltaTime);

			zvel = zvel * (1 - ALTI_WEIGHT_VEL) + altispeed * (ALTI_WEIGHT_VEL);

			xpos = integrate(positionBuffer[0], velocityBuffer[0], timeBuffer[4], xvel, deltaTime);
			ypos = integrate(positionBuffer[1], velocityBuffer[1], timeBuffer[5], yvel, deltaTime);
			zpos = integrate(positionBuffer[2], velocityBuffer[2], timeBuffer[6], zvel, deltaTime);
			zpos2 = integrate(positionBuffer[3], velocityBuffer[3], timeBuffer[7], zvel2, deltaTime);

			zpos = zpos * (1 - ALTI_WEIGHT_POS) + alti * (ALTI_WEIGHT_POS);
			/*
			xvel += 0.5 * (xacc + oldaccx) * G * deltaTime;
			yvel += 0.5 * (yacc + oldaccy) * G * deltaTime;
			zvel += 0.5 * (zacc + oldaccz - 2) * G * deltaTime;
			xpos += 0.5 * (oxvel + xvel) * deltaTime;
			ypos += 0.5 * (oyvel + yvel) * deltaTime;
			zpos += 0.5 * (ozvel + zvel) * deltaTime;*/
			
			
			//Calculate control output
			float velocity_float = zvel2;
			float altitude_float = zpos;

			/*
   velocity_float = rand() % 300;
   altitude_float = rand() % 3000;*/
			uint16_t targetBraking = getBraking(velocity_float, altitude_float);

			fprintf(processedFile, "%.5g b %.5g %.5g %.5g\n", mtime, targetBraking / 255.0f, velocity_float, altitude_float);

			gpioServo(17, lround(MIN_SERVO + (targetBraking / 255.0 * (MAX_SERVO - MIN_SERVO))) * 10);


			if (press > 0) {
				fprintf(processedFile, "%.5g t %.5g %.5g\n", mtime, temp, temp2);
				fprintf(processedFile, "%.5g h %.5g %.5g %.5g\n", mtime, press, alti, altispeed);
			}
			//printf("%.5g gyro %.5g %.5g %.5g\n", mtime, gyrox, gyroy, gyroz);
			fprintf(processedFile, "%.5g g %.5g %.5g %.5g\n", mtime, gyrox, gyroy, gyroz);
			fprintf(processedFile, "%.5g a %.5g %.5g %.5g\n", mtime, xacc, yacc, zacc);
			fprintf(processedFile, "%.5g v %.5g %.5g %.5g %.5g\n", mtime, xvel, yvel, zvel, zvel2);
			fprintf(processedFile, "%.5g p %.5g %.5g %.5g %.5g\n", mtime, xpos, ypos, zpos, zpos2);
			fprintf(processedFile, "%.5g r %.5g %.5g %.5g %.5g\n", mtime, rot.v.x, rot.v.y, rot.v.z, rot.s);

			fflush(processedFile);

			if (((chrono::duration<float>)(getTime() - lastPrint)).count() > 10){
			  if (!firstPrint){
				fsync(fileno(processedFile)); // os linux/unix level function that dumps buffer from os memory to hardware
				//fsync(fileno(rawFile));
				if (DEBUG){
				MTVec3D euler = mtQuaternionToEuler(&rot);
				euler = mtMultiplyVectorScalar(euler, 180 / MY_PI);
				euler.x = mod(euler.x + 180, 360) - 180;
				euler.y = mod(euler.y + 180, 360) - 180;
				euler.z = mod(euler.z + 180, 360) - 180;
				cout << "rotated: " << euler.x << "|" << euler.y << "|" << euler.z << endl;
				//cout << xpos << "|" << ypos << "|" << zpos << " -> " << xacc << "|" << yacc << "|" << zacc << endl;
				cout << xpos << "|" << ypos << "|" << zpos << " <- " << xvel << "|" << yvel << "|" << zvel << endl;
				cout << "dt: " << deltaTime << endl;
				}
			  }
			  firstPrint = false;
				lastPrint = getTime();
			}
			beepPeriod = POST_LAUNCH_BEEP_PERIOD;
			//usleep(1000000);
			oldalti = alti;
			break;
	}

	if (lastReadings.time - lastBeepSwitch > beepPeriod){
		lastBeepSwitch = lastReadings.time;
		gpioWrite(4, beepState = !beepState);
		if (flightMode == INIT){

		  gpioServo(17, 10 * (beepState ? MAX_SERVO : MIN_SERVO));

		}else
		if (flightMode < POST_LAUNCH){

		  gpioServo(17, 10 * ( MIN_SERVO));

		}
	}

	oldgyrox = gyrox;
	oldgyroy = gyroy;
	oldgyroz = gyroz;
	oldaccx = xacc;
	oldaccy = yacc;
	oldaccz = zacc;

	updateFlightMode(newReadings, lastReadings);
}

vector<float> readFile(const char * name){
	
	ifstream fin (name);
	int n;
	fin >> n;
	vector<float> ret(n);
	for (int i = 0; i < n; i++){
		fin >> ret[i];
	}
	return ret;
}

void initVars(){
	
	controlTable = ReadAllBytes("control.bin");
	consts = readFile("consts.txt");
	minAltitude = readFile("limitsLower.txt");
	bandHeight = readFile("limitsUpper.txt");
	for (int i = 0; i < bandHeight.size(); i++){
		bandHeight[i] -= minAltitude[i];
	}
	
	altiSize = lround(consts[1] * consts[2]) + 5;
	width = lround(consts[2]);
	oversampling = lround(consts[1]);
	VELOCITY_RESOLUTION = consts[0];
}

int main(int argc, char *argv[]) {

	initVars();
	initLog(); // creates and open files for the storing the data obtained and writing
	initSensors(); // initiate the sensors => set low pass filter, turn them on in such a way that no more set up is needed afterwards
	flightMode = INIT; // initiate flight

    while (true) {
		updateSensors();
    }
    return 0;
}
