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

// COMPILE USING -std=c++11
using namespace std;

const double G = 9.81;
const int CALIBRATION = 0;
const int PRE_LAUNCH = 1;
const int POST_LAUNCH = 2;
const double CALIBRATION_TIME = 10;
const int HMC5883L_I2C_ADDR = 0x1E;
const int DEFAULT_PITCH = 5;
const int DEFAULT_YAW = 0;
const int DEFAULT_ENGINE = -50;
const int DEFAULT_ROLL = 0;
const double DEFAULT_PITCH_SENSITIVITY = 0.9;
const double DEFAULT_ROLL_SENSITIVITY = 1.1;
const double DEFAULT_YAW_SENSITIVITY = 0.0;
const int MPU6050_I2C_ADDR = 0x68;
const int BMP280_I2C_ADDR = 0x77;//0b11101100;
const int BMP280_I2C_ADDR_2 = 0b11101110;
const int TIMOUT_CLICKS = 10;
const double GYRO_WEIGHT = 5; // used for correcting inertial measurement

double AUTO_PITCH_SENSITIVITY = DEFAULT_PITCH_SENSITIVITY;
double AUTO_ROLL_SENSITIVITY = DEFAULT_ROLL_SENSITIVITY;
double AUTO_YAW_SENSITIVITY = DEFAULT_YAW_SENSITIVITY;
double ZROT_OFFSET = 0;
double YROT_OFFSET = 0;
double XROT_OFFSET = 0;

int msgPitch = DEFAULT_PITCH;
int msgRoll = DEFAULT_ROLL;
int msgYaw = DEFAULT_YAW;
int msgEngine = DEFAULT_ENGINE;

double deltaTime;

int flightMode = PRE_LAUNCH;
std::chrono::high_resolution_clock::time_point initTime;

//calibration data
double xaccsum = 0;
double yaccsum = 0;
double zaccsum = 0;
double xgyrosum = 0;
double ygyrosum = 0;
double zgyrosum = 0;
double xgyrodrift = 0;
double ygyrodrift = 0;
double zgyrodrift = 0;
double accmult = 1;
//rotation matrix
double Czx, Czy, Czz, Cyx, Cyy, Cyz, Cxx, Cxy, Cxz;

//current sensor data
std::chrono::high_resolution_clock::time_point lastReadTime;
std::chrono::high_resolution_clock::time_point lastPrint;
MTQuaternion rot;
double xpos = 0;
double ypos = 0;
double zpos = 0;
double xvel = 0;
double yvel = 0;
double zvel = 0;
double xacc = 0;
double yacc = 0;
double zacc = 0;
double acceleration = 0;

//old sensor data
double oldgyrox, oldgyroy, oldgyroz;
double oldaccx, oldaccy, oldaccz;

int fd;



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

	
double avgangle(double a, double b, double fac){
	a = a * M_PI / 180.0;
	b = b * M_PI / 180.0;
	double x = cos(a) + fac * cos(b);
	double y = sin(a) + fac * sin(b);
	return 180/M_PI * atan2(y,x);
}

double mod(double a, double b){
	return fmod(fmod(a,b) + b, b);
}

double abs(double a){
	return a ? (a > 0) : (-1*a);
}

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

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of -5123- equals 51.23 DegC.
// t_fine carries fine temperature as global value
int t_fine;
int bmp280_temp(int adc_T)
{
	int var1, var2, T;
	var1 = ((((adc_T>>3) - ((int)dig_T1<<1))) * ((int)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int)dig_T1)) * ((adc_T>>4) - ((int)dig_T1))) >> 12) *	((int)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}
double bmp280_press(int adc_P)
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
	return ((unsigned int)p) / 256.0;
}

void initSensors(){
	
    unsigned char buf[16];

    if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
        // Open port for reading and writing
        fprintf(stderr, "Failed to open i2c bus\n");

        //return 1;
    }

    /* initialize HMC5883L */

    /*   selectDevice(fd, HMC5883L_I2C_ADDR, "HMC5883L");

    writeToDevice(fd, 0x00, 0x70);
    writeToDevice(fd, 0x01, 0xA0);
    writeToDevice(fd, 0x02, 0x00);
    */	
	
	//initialize MPU6050
	
    selectDevice(fd, MPU6050_I2C_ADDR, "MPU6050");
	
	writeToDevice(fd,0x6b,0x00); // wake up
	writeToDevice(fd,0x1c,0b00011000); // set accel to +- 16g
	writeToDevice(fd,0x1a,0b00000110); // set lowpass to 5hz
	writeToDevice(fd,0x38,0b00000001); // interrupt data
	
    selectDevice(fd, BMP280_I2C_ADDR_2, "BMP280_2");
    selectDevice(fd, BMP280_I2C_ADDR, "BMP280");
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
	
	
	lastReadTime = getTime();
	initTime = getTime();
	lastPrint = getTime();
	
}

void updateSensors(){
	
	// MPU 6050
	selectDevice(fd, MPU6050_I2C_ADDR, "MPU6050");
	short interrupt = readByte(fd,0x3A);
	bool deviceReady = interrupt & 0b00000001 > 0;
	if (!deviceReady) {
		cout << "not ready" << endl;
		return;
	}
	double gyrox, gyroy, gyroz;
	gyrox = readShort(fd,0x43) / 131.0 - xgyrodrift;
	gyroy = readShort(fd,0x45) / 131.0 - ygyrodrift;
	gyroz = readShort(fd,0x47) / 131.0 - zgyrodrift;
	
	double accelx, accely, accelz;
	accelx= readShort(fd,0x3b) / 2048.0 * accmult;
	accely= readShort(fd,0x3d) / 2048.0 * accmult;
	accelz= readShort(fd,0x3f) / 2048.0 * accmult;
	
	double temp;
	temp = readShort(fd,0x41) / 340.0 + 36.53;
	
	acceleration = sqrt(accelx*accelx + accely*accely + accelz* accelz);
	
	auto now = getTime();
	deltaTime = ((chrono::duration<double>)(now - lastReadTime)).count();
	lastReadTime = now;
	
	MTVec3D curX = mtRotatePointWithMTQuaternion(rot, {1,0,0});
	MTVec3D curY = mtRotatePointWithMTQuaternion(rot, {0,1,0});
	MTVec3D curZ = mtRotatePointWithMTQuaternion(rot, {0,0,1});
	MTQuaternion rotX = mtCreateMTQuaternion(curX, 0.5 * (gyrox + oldgyrox) * M_PI / 180 * deltaTime);
	MTQuaternion rotY = mtCreateMTQuaternion(curY, 0.5 * (gyroy + oldgyroy) * M_PI / 180 * deltaTime);
	MTQuaternion rotZ = mtCreateMTQuaternion(curZ, 0.5 * (gyroz + oldgyroz) * M_PI / 180 * deltaTime);
	rot = mtMultMTQuaternionMTQuaternion(&rotX, &rot);
	rot = mtMultMTQuaternionMTQuaternion(&rotY, &rot);
	rot = mtMultMTQuaternionMTQuaternion(&rotZ, &rot);
	mtNormMTQuaternion(&rot);
	
	
	// BMP 280
    selectDevice(fd, BMP280_I2C_ADDR, "BMP280");
	int rawtemp = readLong(fd, 0xFA, 3) >> 4;
	int rawpress = readLong(fd, 0xF7, 3) >> 4;
	int temp2 = bmp280_temp(rawtemp);
	double press = bmp280_press(rawpress);
	cout << press << endl;
	cout << readShort(fd, 0xFA) << " | " << readByte(fd, 0xFC) << endl;
	cout << rawtemp << " -> " << (temp2 / 100.0)   << " vs " << temp << endl;
	cout << rawpress << " -> " << press << endl;
	
	
	
	switch (flightMode){
		case CALIBRATION:
			xaccsum += accelx * deltaTime;
			yaccsum += accely * deltaTime;
			zaccsum += accelz * deltaTime;
			xgyrosum += gyrox * deltaTime;
			ygyrosum += gyroy * deltaTime;
			zgyrosum += gyroz * deltaTime;
			break;
			
		case PRE_LAUNCH:
		case POST_LAUNCH:
			//use gyro and acc estimations to get better result
			MTVec3D acc = mtRotatePointWithMTQuaternion(rot, {accelx, accely, accelz});
			xacc = acc.x;
			yacc = acc.y;
			zacc = acc.z;
			/*
			xacc = Cxx * accelx + Cxy * accely + Cxz * accelz;
			yacc = Cyx * accelx + Cyy * accely + Cyz * accelz;
			zacc = Czx * accelx + Czy * accely + Czz * accelz;
			*/
			double oxvel = xvel;
			double oyvel = yvel;
			double ozvel = zvel;
			xvel += 0.5 * (xacc + oldaccx) * G * deltaTime;
			yvel += 0.5 * (yacc + oldaccy) * G * deltaTime;
			zvel += 0.5 * (zacc + oldaccz - 2) * G * deltaTime;
			xpos += 0.5 * (oxvel + xvel) * deltaTime;
			ypos += 0.5 * (oyvel + yvel) * deltaTime;
			zpos += 0.5 * (ozvel + zvel) * deltaTime;
			
			if (((chrono::duration<double>)(now - lastPrint)).count() > 1){
				MTVec3D euler = mtQuaternionToEuler(&rot);
				euler = mtMultiplyVectorScalar(euler, 180 / M_PI);
				euler.x = mod(euler.x + 180, 360) - 180;
				euler.y = mod(euler.y + 180, 360) - 180;
				euler.z = mod(euler.z + 180, 360) - 180;
				cout << "rotated: " << euler.x << "|" << euler.y << "|" << euler.z << endl;
				//cout << xpos << "|" << ypos << "|" << zpos << " -> " << xacc << "|" << yacc << "|" << zacc << endl;
				cout << xpos << "|" << ypos << "|" << zpos << " <- " << xvel << "|" << yvel << "|" << zvel << endl;
				cout << "dt: " << deltaTime << endl;
				lastPrint = getTime();
			}
			//usleep(1000000);
			break;
	}
	oldgyrox = gyrox;
	oldgyroy = gyroy;
	oldgyroz = gyroz;
	oldaccx = xacc;
	oldaccy = yacc;
	oldaccz = zacc;
}

void finishCalibration(){
	double dt = ((chrono::duration<double>)(lastReadTime - initTime)).count();
	xaccsum /= dt;
	yaccsum /= dt;
	zaccsum /= dt;
	xgyrosum /= dt;
	ygyrosum /= dt;
	zgyrosum /= dt;
	xgyrodrift = xgyrosum;
	ygyrodrift = ygyrosum;
	zgyrodrift = zgyrosum;
	cout << xaccsum << "|" << yaccsum << "|" << zaccsum  << endl;
	
	//double z = -atan2(yaccsum, xaccsum);
	//double y = atan2(zaccsum, sqrt(xaccsum * xaccsum + yaccsum * yaccsum));
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

void updateFlightMode(){
	
	if (((chrono::duration<double>)(getTime() - initTime)).count() > CALIBRATION_TIME && flightMode == CALIBRATION){
		flightMode = PRE_LAUNCH;
		finishCalibration();
		cout << "Entering flight mode: PRE_LAUNCH" << endl;
	}
	if (acceleration > 2){
		flightMode = POST_LAUNCH;
		cout << "Entering flight mode: POST_LAUNCH" << endl;
	}
}

int main(int argc, char *argv[]) {
	
	initSensors();
	flightMode = CALIBRATION;
	
    while (1) {
		updateSensors();
		updateFlightMode();
    }
    return 0;
}
