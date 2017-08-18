#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>

#define ADXL345_ADDRESS 0x53
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0 0x32
#define ADXL345_REG_DATAY0 0x34
#define ADXL345_REG_DATAZ0 0x36

#define ADXL345_MG2G_MULTIPLIER  0.004
#define SENSORS_GRAVITY_EARTH  9.80665F /**< Earth's gravity in m/s^2 */

unsigned char data_format;

void i2c_adxl_init();void adxl_init(int);double read_word_2c(int);
int fd;//wiringPi I2C Setup Fuction Output

int main(int argc, char **argv)
{
	wiringPiSetupPhys();
	i2c_adxl_init();	
	adxl_init(2);
	while(1){
		printf("ACC_X:%lf 		,ACC_Y:%lf 		,ACC_Z:%lf \n",read_word_2c(ADXL345_REG_DATAX0)*ADXL345_MG2G_MULTIPLIER*SENSORS_GRAVITY_EARTH,read_word_2c(ADXL345_REG_DATAY0)*ADXL345_MG2G_MULTIPLIER*SENSORS_GRAVITY_EARTH , read_word_2c(ADXL345_REG_DATAZ0)*ADXL345_MG2G_MULTIPLIER*SENSORS_GRAVITY_EARTH);
	
	}	
	return 0;
}

void i2c_adxl_init()
{
	fd = wiringPiI2CSetup (ADXL345_ADDRESS);
	wiringPiI2CWriteReg8 (fd,ADXL345_REG_POWER_CTL,0x0C);//disable sleep mode 
	printf("set Register Power Control  =%X\n",wiringPiI2CReadReg8 (fd,ADXL345_REG_POWER_CTL));
}

void adxl_init(int g)
{
	data_format = wiringPiI2CReadReg8 (fd,ADXL345_REG_DATA_FORMAT);
	printf("The Range was =%X\n",wiringPiI2CReadReg8 (fd,ADXL345_REG_DATA_FORMAT));	
	
	switch (g) {
		case 2 : //2G
			data_format |= 0X00; 
		break;
		case 4 : //4G
			data_format |= 0X01;
		break;
		case 8 ://8G
			data_format |= 0X02;
		break;
		case 16 : //16G
			data_format |= 0X03;
		break;		
	}
	data_format |= 0x08;//make sure that the FULL-RES bit is enabled for range scaling	
	wiringPiI2CWriteReg8 (fd,ADXL345_REG_DATA_FORMAT,data_format);//Set Range	
	printf("set Range to =%X\n",wiringPiI2CReadReg8 (fd,ADXL345_REG_DATA_FORMAT) );
	delay(5000);
}

double read_word_2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(fd, addr+1);
  val = val << 8;
  val += wiringPiI2CReadReg8(fd, addr);
  if (val >= 0x8000)
    val = -(65536 - val);

  return val;
}




/* *******************************************************************
			BMP180_Pressure Code :: 
*/
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <math.h>
#include "bmp180.h"

#define OSS BMP180_STANDARD
short AC1,AC2,AC3,B1,B2,MB,MC,MD;
unsigned short AC4,AC5,AC6;
int fd;
char I2C_readByte(int reg)
{
    return (char)wiringPiI2CReadReg8(fd,reg);
}

unsigned short I2C_readU16(int reg)
{
    int MSB,LSB;
    MSB = I2C_readByte(reg);
    LSB = I2C_readByte(reg + 1);
    int value = (MSB << 8) +LSB;
    return (unsigned short)value;
}

short I2C_readS16(int reg)
{
    int result;
    result = I2C_readU16(reg);
    if (result > 32767)result -= 65536;
    return (short)result;
}
void I2C_writeByte(int reg,int val)
{
    wiringPiI2CWriteReg8(fd,reg,val);
}

void load_calibration()
{
    AC1 = I2C_readS16(BMP180_CAL_AC1);
    AC2 = I2C_readS16(BMP180_CAL_AC2);
    AC3 = I2C_readS16(BMP180_CAL_AC3);
    AC4 = I2C_readU16(BMP180_CAL_AC4);
    AC5 = I2C_readU16(BMP180_CAL_AC5);
    AC6 = I2C_readU16(BMP180_CAL_AC6);
    B1  = I2C_readS16(BMP180_CAL_B1);
    B2  = I2C_readS16(BMP180_CAL_B2);
    MB  = I2C_readS16(BMP180_CAL_MB);
    MC  = I2C_readS16(BMP180_CAL_MC);
    MD  = I2C_readS16(BMP180_CAL_MD);
}
int read_raw_temp()
{
    int raw;
    I2C_writeByte(BMP180_CONTROL,BMP180_READTEMPCMD);
    delay(5);  //5ms;
    raw = I2C_readByte(BMP180_TEMPDATA) << 8;
    raw += I2C_readByte(BMP180_TEMPDATA+1);
    return raw;

}
int read_raw_pressure()
{
    int MSB,LSB,XLSB,raw;
    I2C_writeByte(BMP180_CONTROL,BMP180_READPRESSURECMD +(OSS << 6));
    switch(OSS)
    {
        case BMP180_ULTRALOWPOWER:
            delay(5);break;
        case BMP180_HIGHRES:
            delay(14);break;
        case BMP180_ULTRAHIGHRES:
            delay(26);break;
        default :
            delay(8);
    }
    MSB  = I2C_readByte(BMP180_PRESSUREDATA);
    LSB  = I2C_readByte(BMP180_PRESSUREDATA + 1);
    XLSB = I2C_readByte(BMP180_PRESSUREDATA + 2);
    raw = ((MSB << 16) + (LSB << 8) + XLSB) >> (8 - OSS);
    return raw;
}
float read_temperature()
{
    float T;
    int UT,X1,X2,B5;
    UT = read_raw_temp();
    X1 = ((UT - AC6)*AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    T = ((B5 + 8) >> 4) /10.0;
    return T;
}

int read_pressure()
{
    int P;
    int UT,UP,X1,X2,X3,B3,B5,B6;
    unsigned int B4;
    int B7;
    UT = read_raw_temp();
    UP = read_raw_pressure();

    X1 = ((UT - AC6)*AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;

    //Pressure Calculations
    B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6) >> 12) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (X3 + 32768)) >> 15;
    B7 = (UP - B3) * (50000 >> OSS);
    if (B7 < 0x80000000){P = (B7 * 2) / B4;}
    else {P = (B7 / B4) * 2;}
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;
    P = P + ((X1 + X2 + 3791) >> 4);
    return P;

}
float read_altitude()
{
    float pressure,altitude;
    float sealevel_pa = 101325.0;
    pressure = (float)read_pressure();
    altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa,(1.0/5.255)));
    return altitude;
}
float read_sealevel_pressure()
{
    float altitude_m = 0.0;
    float pressure,p0;   pressure =(float)read_pressure();
    p0 = pressure / pow(1.0 - altitude_m/44330.0,5.255);
    return p0;
}
int main(int argc,char **argv)
{
    printf("BMP180 Test Program ...\n");
    if(wiringPiSetup() < 0) return 1;
    fd = wiringPiI2CSetup(BMP180_Address);
    load_calibration();
    while(1)
    {
        printf("\nTemperature : %.2f C\n",read_temperature());
        printf("Pressure :    %.2f Pa\n",read_pressure()/100.0);
        printf("Altitude :    %.2f h\n",read_altitude());
        delay(1000);
    }
    return 0;
}

/* *****************************************************************
				HMC5883_Compass Code :
********************************************************************
*/

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>

#define cmp_ADDRESS 0x1e


void i2c_cmp_init();int read_word_2c(int);void i2c_cmp_read();
int fd;//wiringPi I2C Setup Fuction Output
double x_cmp=0,y_cmp=0,z_cmp=0;

int main(int argc, char **argv)
{
	i2c_cmp_init();
	while(1){
		i2c_cmp_read();
		printf("X_cmp: %lf \t Y_cmp: %lf \tZ_cmp: %lf \n",x_cmp,y_cmp,z_cmp);
		delay(50);
		printf("Status : %X \n",wiringPiI2CReadReg8(fd, 0x09));
	}
	return 0;
}

void i2c_cmp_init()
{
	fd = wiringPiI2CSetup (cmp_ADDRESS);
	if (fd < 0 )
		printf("Error Cnfiguring I2C Compass HMC5883 ");
	else
		printf("Config Register A  =%X\n",wiringPiI2CReadReg8 (fd,0x00));
	delay(3000);
	wiringPiI2CWriteReg8(fd, 0x00, 0x70); // Average every 11 sample and then equal to output
	wiringPiI2CWriteReg8(fd, 0x01, 0x00); // Average every 11 sample and then equal to output
	wiringPiI2CWriteReg8(fd, 0x02, 0x00); //Countinous Measurment Mode (Not Singel measurment ! )
}


int read_word_2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(fd, addr);
  val = val << 8;
  val += wiringPiI2CReadReg8(fd, addr+1);
  if (val >= 0xF800)
    val = -(65536 - val);

  return val;
}

void i2c_cmp_read()
{
	x_cmp = read_word_2c(0x03) * 360 / 2048;
	y_cmp = read_word_2c(0x07)	* 360 / 2048; // Y Registers are 7 and 8!
	z_cmp = read_word_2c(0x05)	* 360 / 2048;
}

/* *****************************Finsih
*/
