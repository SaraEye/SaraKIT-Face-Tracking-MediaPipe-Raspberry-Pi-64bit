#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <unistd.h>
#include <stdbool.h>
#include <linux/spi/spidev.h>

#include "devices.hpp"

//BLDC GIMBAL MOTORS
int motorDriverPin = 17;

//SPI Communications
static const int CHANNEL = 0;
int fd;

void sleepms(int ms){
    usleep(ms*1000);
}

static const char *spi_name = "/dev/spidev0.1";

int wiringPiSPIData(int channel, unsigned char *data, int len, bool writeonly) {
    int spiDev = open(spi_name, O_RDWR);
    int mode = 0x01;
    ioctl(spiDev, SPI_IOC_WR_MODE, &mode);
    int maxSpeed = 0;
    ioctl(spiDev, SPI_IOC_RD_MAX_SPEED_HZ, &maxSpeed);
    int lsb_setting = 0;
    ioctl(spiDev, SPI_IOC_WR_LSB_FIRST, &lsb_setting);  
    int bits_per_word = 0;
    ioctl(spiDev, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
    struct spi_ioc_transfer spi[2];
    channel &= 1;

    memset(&spi, 0, sizeof(spi));
    spi[0].tx_buf = (unsigned long long)data;
    spi[0].len = len;
    spi[0].delay_usecs = 10;
    spi[0].speed_hz = 1000000;
    spi[0].bits_per_word = 8;
    if (writeonly) {
        spi[0].cs_change=1;
        int ret=ioctl(spiDev, SPI_IOC_MESSAGE(1), &spi);
       close(spiDev);
        return ret;
    }
    
    spi[0].cs_change=0;

    unsigned char dummyData[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    spi[1].tx_buf = (unsigned long long)dummyData;
    spi[1].rx_buf = (unsigned long long)data;
    spi[1].len = len;
    spi[1].cs_change=1;
    spi[1].delay_usecs = 10;
    spi[1].speed_hz = 1000000;
    spi[1].bits_per_word = 8;

    int ret=ioctl(spiDev, SPI_IOC_MESSAGE(2), spi);
    close(spiDev);
    return ret;
}

void sendCommandSet(uchar *buffer){
    wiringPiSPIData(CHANNEL, buffer, 16, true);
}

void sendCommandGet(uchar *buffer, int len=16){
    wiringPiSPIData(CHANNEL, buffer, len, false);
}

float bintofloat(unsigned int x) {
    union {
        unsigned int  x;
        float  f;
    } tmp;
    tmp.x = x;
    return tmp.f;
}

unsigned int floattoint(float f) {
    union {
        unsigned int  x;
        float  f;
    } tmp;
    tmp.f = f;
    return tmp.x;
}


float getFloat(uchar cmd[16],int cmdpos){
    uint16_t wl = (uint16_t)cmd[cmdpos  ] << 8 | cmd[cmdpos+1];
    uint16_t wh = (uint16_t)cmd[cmdpos+2] << 8 | cmd[cmdpos+3];
    uint32_t f=(uint32_t)(wh) << 16 | (wl);
    float ff=bintofloat(f);
    return ff;
}

uint16_t getInt16(uchar cmd[16],int cmdpos){
    uint16_t i16 = (uint16_t)cmd[cmdpos  ] << 8 | cmd[cmdpos+1];
    return i16;
}

//synchronize SPI
bool _SPICheck(){
    uchar command=127;
    uchar testbyte=0xf1;
    uchar cmd[18] = {command,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
    sendCommandGet(cmd);
    int sum=0;
    if (cmd[11]!=127){
        int l=1;
        int l2=1;
        while (cmd[11]!=127 && l<17) {
            cmd[0]=127;
            for (int i=1; i<16; i++) cmd[i]=i;
            sendCommandGet(cmd,l);
            l++;
/*            
            printf("test %d ",l);
            for (int i=0; i<16; i++) {
                printf("%d ",cmd[i]);
            }
            printf("\n");
*/            
            if (l==17) {
                l2++;
                l=l2;
            }
        }
        if (cmd[11]==127){
//            printf("\n");
            return true;        
        }
    } else {
        return true;
    }
    return false;
}

//=Accelerometer+Gyroscope+Example==================================================================

/*example:
//LSM6DS3_WHO_AM_I_REG = 0x6A ?

uint8_t odp=0;
LSM6DS3TR_readRegisters(0X0F,&odp,1);
printf("who i am %d\n",odp);

uint8_t status=0;
LSM6DS3TR_readRegisters(0X1E,&status,1);
if (status & 0x01) { //new data from acc?
    uint8_t buf[6];
    LSM6DS3TR_readRegisters(0X28,buf,6);
    uint16_t rawaccx = (uint16_t)buf[1] << 8 | buf[0];
    uint16_t rawaccy = (uint16_t)buf[3] << 8 | buf[2];
    uint16_t rawaccz = (uint16_t)buf[5] << 8 | buf[4];
    float accx=lsm6ds3tr_c_from_fs2g_to_mg(rawaccx);
    printf("%f \n",accx);
}
*/

uint8_t LSM6DS3TR_readRegisters(uint8_t address, uint8_t* data, uint8_t length){
    uchar command=6;
    uchar testbyte=0xf1;

    uchar cmd[16] = {command,0,0,address,0,length,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandGet(cmd,16);

    int stop=10;
    while (stop>0) {
        usleep(250); //time to I2C
        command=7;
        uchar cmd2[16] = {command,0,0,address,0,length,0,testbyte,0,0,0,0,0,0,0,0};
        sendCommandGet(cmd2,16);
        
        //DRV_BIG_ENDIAN<=>DRV_LITTLE_ENDIAN
        for (int i=0; i<16; i+=2){
            uint8_t c=cmd2[i];
            cmd2[i]=cmd2[i+1];
            cmd2[i+1]=c;
        }
        //0  1  2  3  4  5  6  7  8  9  10 11 12 13  14 15
        //x  x  0  1  2  3  4  5  6  7  8  9  10 11  12 13
        //x  x  i161  i162  i163  i164  i165  i166   result
        //x  x  float 1     float 2     float 3      result
        
        if (cmd2[15]==255 && cmd2[14]!=0) {
            for (int i=0; i<length; i++){
                data[i]=cmd2[i+2];
            }
            return cmd2[14];//return I2C1_MESSAGE_STATUS;
        }
        stop--;
    }
    return 0;
}

void LSM6DS3TR_writeRegisters(uint8_t address, uint8_t data){
    uchar command=8;
    uchar testbyte=0xf1;

    uchar cmd[16] = {command,0,0,address,0,data,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandSet(cmd);
    sleepms(50);
}

float lsm6ds3tr_c_from_fs2g_to_mg(int16_t lsb)
{
    return ((float)lsb * 0.061f);
}
float lsm6ds3tr_c_from_fs1000dps_to_mdps(int16_t lsb)
{
    return ((float)lsb * 35.0f);
}

//acc+gyro+temp enable
void Sensors_On(uchar on){
    uchar command=1;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,0,0,on,0,0,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandSet(cmd);
}

InfoAccGyro getAccGyro(){
    uchar command=2;
    uchar testbyte=0xf1;

    uchar cmd[16] = {command,0,0,0,0,0,0,testbyte,0,0,0,0,0,0,0,0};

    sendCommandGet(cmd,16);

    InfoAccGyro result;
    
    result.rawAccX=getInt16(cmd,4);
    result.rawAccY=getInt16(cmd,6);
    result.rawAccZ=getInt16(cmd,8);

    result.accX=lsm6ds3tr_c_from_fs2g_to_mg(result.rawAccX);
    result.accY=lsm6ds3tr_c_from_fs2g_to_mg(result.rawAccY);
    result.accZ=lsm6ds3tr_c_from_fs2g_to_mg(result.rawAccZ);

    result.rawGyroX=getInt16(cmd,10);
    result.rawGyroY=getInt16(cmd,12);
    result.rawGyroZ=getInt16(cmd,14);

    result.gyroX=lsm6ds3tr_c_from_fs1000dps_to_mdps(result.rawGyroX);
    result.gyroY=lsm6ds3tr_c_from_fs1000dps_to_mdps(result.rawGyroY);
    result.gyroZ=lsm6ds3tr_c_from_fs1000dps_to_mdps(result.rawGyroZ);

    //printf("acc x:%f, y:%f, z:%f [%x, %x | %x, %x | %x, %x ]\n", result.accX, result.accY, result.accZ, cmd[0], cmd[1], cmd[2], cmd[3],cmd[4], cmd[5], cmd[6], cmd[7]);
	return result;
}

//=Temperature=====================================================================================

float lsm6ds3tr_c_from_lsb_to_celsius(int16_t lsb)
{
  return (((float)lsb / 256.0f) + 25.0f);
}

float getTemperature(){
    uchar command=5;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,0,0,0,0,0,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandGet(cmd,16);
  
    float tempC=lsm6ds3tr_c_from_lsb_to_celsius(getInt16(cmd,4));

    //printf("temp x:%.2f\n", tempC);
	return tempC;
}

//=Encoders========================================================================================

void Encoder_Param(uchar EncoderId, uint16_t minpwm, uint16_t maxpwm){
    uchar command=10;
    uchar testbyte=0xf1;
  	uchar minpwmH = minpwm >> 8;
	uchar minpwmL = 0xFF & minpwm;
  	uchar maxpwmH = maxpwm >> 8;
	uchar maxpwmL = 0xFF & maxpwm;

    uchar cmd[16] = {command,EncoderId,minpwmH,minpwmL,maxpwmH,maxpwmL,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandSet(cmd);
}

//EncoderId 0-1; angle low pass filter time constant [0.01], velocity low pass filter time constant [0.01]
//eliminates vibrations - should be set depending on the weight of the wheel
void Encoder_LowPassFilter(uchar EncoderId, float angle, float velocity){
    if (EncoderId>3) {
        printf("invalid EncoderID value (0-1)\n");
        return;
    }
    uchar command=14;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,EncoderId,0,0,0,0,0,0,0,0,0,0,0,0,0,testbyte};
    memcpy(&cmd[4], &angle, 4);
    memcpy(&cmd[8], &velocity, 4);
    sendCommandSet(cmd);
}


void Encoder_On(uchar EncoderId, uchar on){
    uchar command=11;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,EncoderId,0,on,0,0,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandSet(cmd);
}

InfoEncoders Encoder_Get(uchar EncoderId){
    uchar command=12;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,EncoderId,0,0,0,0,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandGet(cmd,16);

    InfoEncoders result;
    
    result.angle=getFloat(cmd,4);
    result.velocity=getFloat(cmd,8);
    result.angleDeg=result.angle*(180.0 / _PI);
    result.velocityDeg=result.velocity*(180.0 / _PI);
    //printf("\rangle:%f velocity:%f [rad]     ", result.angle, result.velocity);
    //printf("\rangle:%f velocity:%f [deg]     ", result.angleDeg, result.velocityDeg);
	return result;
}    

InfoEncoders Encoder_GetParam(uchar EncoderId){
    uchar command=13;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,EncoderId,0,0,0,0,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandGet(cmd,16);

    InfoEncoders result;
    
    result.zeroAngle=getFloat(cmd,4);
    result.direction=(int16_t)getInt16(cmd,8);
    //printf("\rzeroAngle:%f direction:%d    ", result.zeroAngle, result.direction);
	return result;
}    

//=BLDC Gimbal Motors==============================================================================

void BLDCMotor_On(uchar motorId, bool on){
    if (motorId>3) {
        printf("invalid MotorID value (0-1)\n");
        return;
    }
    uchar command=25;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,0,on,0,0,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandSet(cmd);
}

//motorId 0-2; polePairs 7 for 12N14P or 11 for 24N22P;
void BLDCMotor_PolePairs(uchar motorId, uchar polePairs){
    if (motorId>3) {
        printf("invalid MotorID value (0-1)\n");
        return;
    }
    uchar command=20;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,0,polePairs,0,0,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandSet(cmd);
}

//motorId 0-1; torque 0-100; idle torque after ms [0-65535 ms] time;
void BLDCMotor_IdleTorque(uchar motorId, uchar torque, uint16_t torqueMs){
    if (motorId>1) {
        printf("invalid MotorID value (0-1)\n");
        return;
    }
    if (torque<0 or torque>100) {
        printf("invalid torque value (0-100)\n");
        return;
    }
    //set engine idle power and duration
    uchar command=21;
    uchar testbyte=0xf1;
  	uchar tmsH = torqueMs >> 8;
	uchar tmsL = 0xFF & torqueMs;
    uchar cmd[16] = {command,motorId,0,torque,tmsH,tmsL,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandSet(cmd);
}

//motorId 0-1; P [10]; I [0]; D [0]; ramp [0]
//PID for BLDCMotor_MoveToAngle, BLDCMotor_MoveByAngle (only with Encoder)
void BLDCMotor_PIDAngle(uchar motorId, float P, float I, float D, float ramp){
    if (motorId>1) {
        printf("invalid MotorID value (0-1)\n");
        return;
    }
    uchar command=26;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,0,0,0,0,0,0,0,0,0,0,0,0,0,testbyte};
    memcpy(&cmd[4], &P, 4);
    memcpy(&cmd[8], &I, 4);
    sendCommandSet(cmd);
    sleepms(1);

    command=27;
    uchar cmd2[16] = {command,motorId,0,0,0,0,0,0,0,0,0,0,0,0,0,testbyte};
    memcpy(&cmd2[4], &D, 4);
    memcpy(&cmd2[8], &ramp, 4);
    sendCommandSet(cmd2);
}

//motorId 0-1; P [0.2]; I [20]; D [0]; ramp [1000]
//PID for BLDCMotor_MoveContinuousVelocity, BLDCMotor_MoveToAngle, BLDCMotor_MoveByAngle (only with Encoder)
void BLDCMotor_PIDVelocity(uchar motorId, float P, float I, float D, float ramp){
    if (motorId>1) {
        printf("invalid MotorID value (0-1)\n");
        return;
    }
    uchar command=28;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,0,0,0,0,0,0,0,0,0,0,0,0,0,testbyte};
    memcpy(&cmd[4], &P, 4);
    memcpy(&cmd[8], &I, 4);
    sendCommandSet(cmd);
    sleepms(1);

    command=29;
    uchar cmd2[16] = {command,motorId,0,0,0,0,0,0,0,0,0,0,0,0,0,testbyte};
    memcpy(&cmd2[4], &D, 4);
    memcpy(&cmd2[8], &ramp, 4);
    sendCommandSet(cmd2);
}

//encoder port 0 or 1, direction -1 or 0 or 1  =0-set FOC ; return angle zero + direction
initFOC BLDCMotor_InitFOC(uchar motorId, uchar encoderId, int direction, float angle){
    initFOC initFoc;
    initFoc.ready=false;
    if (motorId>1) {
        printf("invalid MotorID value (0-3)\n");
        return initFoc;
    }

    uchar command=50;
    uchar testbyte=0xf1;
	char dir=(char)direction;
    uchar cmd[16] = {command,motorId,0,encoderId,0,dir,0,0,0,0,0,0,0,0,0,testbyte};
    memcpy(&cmd[6], &angle, 4);
    sendCommandSet(cmd);
    if (direction==0) {
        sleepms(100);
        InfoEncoders za;
        int ms=0;    
        while (ms<10000) {
            za=Encoder_GetParam(encoderId);
            if (za.zeroAngle==-999.0f || za.zeroAngle==0.0f)
                sleepms(1);
            else
                break;
            ms++;
        }
        printf("Decoder %d zeroAngle=%f direction=%d\n", motorId, za.zeroAngle, za.direction);
    }
    return initFoc;
}

//motorId 0-4; angle degs or radians; torque 0-100; speed 0.01-1000 if deg speed 1=360deg
void BLDCMotor_MoveToAngle(uchar motorId, float angle, float speed, uchar torque, bool degrees=false){
    if (motorId>3) {
        printf("invalid MotorID value (0..3)\n");
        return;
    }
    if (torque<0 or torque>100) {
        printf("invalid torque value (0..100)\n");
        return;
    }
    if (speed<0 or speed>1000) {
        printf("invalid speed value (0..1000)\n");
        return;
    }
    uchar command=51;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,0,torque,0,0,0,0,0,0,0,0,0,0,0,testbyte};
    if (degrees) {
        angle=angle*DEG_TO_RAD;
        speed=speed*_2PI; //2pi = 360 deg
    }
    memcpy(&cmd[4], &angle, 4);
    memcpy(&cmd[8], &speed, 4);
    sendCommandSet(cmd);
}

//motorId 0-4; angle degs or radians; torque 0-100; speed 0.01-1000 if deg speed 1=360deg
void BLDCMotor_MoveByAngle(uchar motorId, float angle, float speed, uchar torque, bool degrees=false){
    if (motorId>3) {
        printf("invalid MotorID value (0..3)\n");
        return;
    }
    if (torque<0 or torque>100) {
        printf("invalid torque value (0..100)\n");
        return;
    }
    if (speed<0 or speed>1000) {
        printf("invalid speed value (0..1000)\n");
        return;
    }
    uchar command=52;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,0,torque,0,0,0,0,0,0,0,0,0,0,0,testbyte};
    if (degrees) {
        angle=angle*DEG_TO_RAD;
        speed=speed*_2PI;
    }
    memcpy(&cmd[4], &angle, 4);
    memcpy(&cmd[8], &speed, 4);
    sendCommandSet(cmd);
}

void BLDCMotor_DriveMeters(uchar motorId, float centimeters, float speed, uchar torque, float WhellDiameter){
    float one=2*_PI*(WhellDiameter/2);
    float angle=centimeters/one*(2*_PI);
    BLDCMotor_MoveByAngle(motorId,angle,speed,torque,false);
}

//motorId 0-4; direction -1,1; torque 0..100;
void BLDCMotor_MoveContinuousTorque(uchar motorId, int direction, uchar torque){
    if (motorId>3) {
        printf("invalid MotorID value (0..3)\n");
        return;
    }
    if (direction!=-1 && direction!=1) {
        printf("invalid direction value (-1,1)\n");
        return;
    }
    if (torque<0 || torque>100) {
        printf("invalid torque value (0..100)\n");
        return;
    }
    char dir=(char)direction;
    uchar command=53;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,dir,torque,0,0,0,0,0,0,0,0,0,0,0,testbyte};
    sendCommandSet(cmd);
}

//motorId 0-1; direction 0..1; speed 0.01..1000;
void BLDCMotor_MoveContinuousVelocity(uchar motorId, int direction, uchar torque, float speed, bool degrees=false){
    if (motorId>3) {
        printf("invalid MotorID value (0..1)\n");
        return;
    }
    if (direction!=-1 && direction!=1) {
        printf("invalid direction value (-1,1)\n");
        return;
    }
    if (torque<0 || torque>100) {
        printf("invalid torque value (0..100)\n");
        return;
    }
    if (speed<0 or speed>1000) {
        printf("invalid speed value (0..1000)\n");
        return;
    }
    char dir=(char)direction;
    uchar command=54;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,dir,torque,0,0,0,0,0,0,0,0,0,0,0,testbyte};
    if (degrees) {
        speed=speed*DEG_TO_RAD;
    }
    memcpy(&cmd[8], &speed, 4);
    sendCommandSet(cmd);
}

//torque=0
void BLDCMotor_MoveStop(uchar motorId){
    if (motorId>3) {
        printf("invalid MotorID value (0-1)\n");
        return;
    }
    uchar command=60;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,0,0,0,0,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandSet(cmd);
}

//get info: 1 - position; 2 - velocity; 3 - current Torque;
GetInfoResult BLDCMotor_GetInfo(uchar motorId, uchar info, bool zprint){
    GetInfoResult result;
    if (motorId>3) {
        printf("invalid MotorID value (0-1)\n");
        return result;
    }

    uchar command=100;
    uchar testbyte=0xf1;
    uchar cmd[16] = {command,motorId,0,info,0,0,0,testbyte,0,0,0,0,0,0,0,0};
    sendCommandGet(cmd,16);
    
    switch (info) {
        case 1:
            result.fdata1=getFloat(cmd,2);
            result.fdata2=getFloat(cmd,6);
            if (zprint)
                printf("Position: electrical %.2f[rad] %.2f[deg] current:%.2f[rad] %.2f[deg]     \n", result.fdata1, result.fdata1 * RAD_TO_DEG, result.fdata2, result.fdata2 * RAD_TO_DEG);
        break;
        case 2:
            result.fdata1=getFloat(cmd,2);
            if (zprint)
                printf("Velocity: %.2f[rad] %.2f[deg] \n", result.fdata1, result.fdata1*RAD_TO_DEG );
        break;
        case 3:
            result.fdata1=getFloat(cmd,2);
            if (zprint)
                printf("Torque: %.2f[\%] \n", result.fdata1);
        break;
        default: {}
        // code block
    }
    /*
    printf("\rzeroAngle:%f direction:%d    ", result.zeroAngle, result.direction);
    */
	return result;
}

void GyroCalib(){
    printf("\nGyroscope calibration, put it on a level surface, don't move, wait 5 seconds.\n");
    int64_t i=0, x=0,y=0,z=0;

    while (i<5000) {
        InfoAccGyro iag=getAccGyro();
        x+=iag.gyroX;
        y+=iag.gyroY;
        z+=iag.gyroZ;
        sleepms(1);
        i++;
    }
    printf("Copy and paste in the program code:\n#define GYRO_X_OFFSET %.0f.0\n#define GYRO_Y_OFFSET %.0f.0\n#define GYRO_Z_OFFSET %.0f.0\n\n",-x/5000.0,-y/5000.0,-z/5000.0);
}

void AccCalib(){
    printf("\nAccelerator calibration, wait 5 seconds.\n");
    int64_t i=0, x=0,y=0,z=0;

    while (i<5000) {
        InfoAccGyro iag=getAccGyro();
        x+=iag.accX;
        y+=iag.accY;
        z+=iag.accZ;
        sleepms(1);
        i++;
    }
    printf("Copy and paste in the program code:\n#define ACCEL_X_OFFSET %.0f.0\n#define ACCEL_Y_OFFSET %.0f.0\n#define ACCEL_Z_OFFSET %.0f.0\n\n",-x/5000.0,-y/5000.0,-z/5000.0);
}

//reset driver
void BLDCMotor_Reset(){
//	pinMode(motorDriverPin, OUTPUT);
//	digitalWrite(motorDriverPin, 0);
//	digitalWrite(motorDriverPin, 1);
}

//reboot device - go to bootloader
void BLDCMotor_Reboot(){
/*    uchar header=rebootHeader;
    uchar buf1=0;
    uchar buf2=0;
    uchar buf3=0;
    uchar cmd[4] = {header, buf1, buf2, buf3};
    sendCommandSet(cmd);
*/    
}

