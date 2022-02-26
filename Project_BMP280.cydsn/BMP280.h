/* ========================================
 *
Library created by Fabricio Arismendi Svistoñuk,
for operation of the BMP280 temperature and Pressure sensor
under the PSoC 5lp platform (not compatible with BME280 or BMP180).

To be used with the BME280 sensor, it is necessary to modify the registers.
 *
 * ========================================
*/
#ifndef __BMP280_H__
#define __BMP280_H__

#include <project.h>
    

#define BMP280_MAX_ITER 300    
#define P_SURFACE       101325 // Surface Pressure (Pascals); (Adjust to location)
    
#define BMP280_I2C_MODE 0
#define BMP280_SPI_MODE 1

//Register names:
#define BMP280_DIG_T1_LSB_REG			0x88
#define BMP280_DIG_T1_MSB_REG			0x89
#define BMP280_DIG_T2_LSB_REG			0x8A
#define BMP280_DIG_T2_MSB_REG			0x8B
#define BMP280_DIG_T3_LSB_REG			0x8C
#define BMP280_DIG_T3_MSB_REG			0x8D
#define BMP280_DIG_P1_LSB_REG			0x8E
#define BMP280_DIG_P1_MSB_REG			0x8F
#define BMP280_DIG_P2_LSB_REG			0x90
#define BMP280_DIG_P2_MSB_REG			0x91
#define BMP280_DIG_P3_LSB_REG			0x92
#define BMP280_DIG_P3_MSB_REG			0x93
#define BMP280_DIG_P4_LSB_REG			0x94
#define BMP280_DIG_P4_MSB_REG			0x95
#define BMP280_DIG_P5_LSB_REG			0x96
#define BMP280_DIG_P5_MSB_REG			0x97
#define BMP280_DIG_P6_LSB_REG			0x98
#define BMP280_DIG_P6_MSB_REG			0x99
#define BMP280_DIG_P7_LSB_REG			0x9A
#define BMP280_DIG_P7_MSB_REG			0x9B
#define BMP280_DIG_P8_LSB_REG			0x9C
#define BMP280_DIG_P8_MSB_REG			0x9D
#define BMP280_DIG_P9_LSB_REG			0x9E
#define BMP280_DIG_P9_MSB_REG			0x9F

#define BMP280_CHIP_ID_REG				0xD0 //Chip ID
#define BMP280_RST_REG					0xE0 //Softreset Reg

#define BMP280_STAT_REG					0xF3 //Status Reg
#define BMP280_CTRL_MEAS_REG			0xF4 //Ctrl Measure Reg
#define BMP280_CONFIG_REG				0xF5 //Configuration Reg
#define BMP280_PRESSURE_MSB_REG			0xF7 //Pressure MSB
#define BMP280_PRESSURE_LSB_REG			0xF8 //Pressure LSB
#define BMP280_PRESSURE_XLSB_REG		0xF9 //Pressure XLSB
#define BMP280_TEMPERATURE_MSB_REG		0xFA //Temperature MSB
#define BMP280_TEMPERATURE_LSB_REG		0xFB //Temperature LSB
#define BMP280_TEMPERATURE_XLSB_REG		0xFC //Temperature XLSB



//Class SensorSettings.  This object is used to hold settings data.  The application
//uses this classes' data directly.  The settings are adopted and sent to the sensor
//at special times, such as .begin.  Some are used for doing math.
//
//This is a kind of bloated way to do this.  The trade-off is that the user doesn't
//need to deal with #defines or enums with bizarre names.
//
//A power user would strip out SensorSettings entirely, and send specific read and
//write command directly to the IC. (ST #defines below)
//
struct {	
  //Main Interface and mode settings
    uint8_t commInterface;
    uint8_t I2CAddress;
    uint8_t chipSelectPin;
	
	uint8_t runMode;
	uint8_t tStandby;
	uint8_t filter;
	uint8_t tempOverSample;
	uint8_t pressOverSample;
	

} typedef SensorSettings;

//Used to hold the calibration constants.  These are used
//by the driver as measurements are being taking
struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	

	
} typedef SensorCalibration;

//This is the man operational class of the driver.

struct {
    //settings
    SensorSettings settings;
	SensorCalibration calibration;
	int32_t t_fine;
    float temp_C;
    float press_Pa;

} typedef BMP280;


//Constructor generates default SensorSettings.
//(over-ride after construction if desired)
uint8_t BMP280_init();


//Call to apply SensorSettings.
//This also gets the SensorCalibration constants
void BMP280_update_settings();

// Start I2C component and put to sleep
void BMP280_start();
// Stop I2C component
void BMP280_stop();
//Software reset routine
void BMP280_reset();

//Returns the values as floats.
float BMP280_readFloatPressure( void );
float BMP280_readFloatAltitudeMeters( void );
float BMP280_readFloatAltitudeFeet( void );


//Temperature related methods
float BMP280_readTempC( void );
float BMP280_readTempF( void );

//The following utilities read and write

//ReadRegisterRegion takes a uint8 array address as input and reads
//a chunk of memory into that array.


//readRegister reads one register
uint8_t BMP280_readRegister(uint8_t);

//Reads two regs, LSByte then MSByte order, and concatenates them
//Used for two-byte reads


//Writes a byte;
void BMP280_writeRegister(uint8_t, uint8_t);

#endif  