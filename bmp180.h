#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>
#include <pigpio.h>
#include <string.h>
#include <unistd.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ktime.h>

#define BMP180_DEV_ADDR 0x77
#define BMP180_DEV_ID_REG 0xD0
#define BMP180_DEV_ID 0x55
#define BMP1880_CTL_REG 0xF4
#define BMP180_READ_TEMP_CMD 0x2E
#define BMP180_READ_PRES_CMD 0x34
#define BMP180_DATA_REG_MSB 0xF6
#define BMP180_DATA_REG_LSB 0xF7
#define BMP180_DATA_REG_XLSB 0xF8
#define BMP180_EEPROM_ADDR 0xAA
#define BMP180_EEPROM_SIZE 22

#define BMP180_CONVERSION_TIME(oss) \
	((oss) == 0 ? 4500 : (oss) == 1 ? 7500 : (oss) == 2 ? 13500 : 25500) 

typedef struct{
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;
} BMP180_EEPROM_t;

typedef struct {
	int i2c_handle;   // pigpio I2C handle number
	uint8_t i2c_bus;   // I2C bus number
	uint8_t i2c_addr;  // I2C address of BMP180
	uint32_t i2c_flags; // I2C flags none are currently defined
} BMP180_I2C_Config_t;

typedef struct{
	BMP180_EEPROM_t calib;
	BMP180_I2C_Config_t *hi2c;    		// pigpio I2C handle number
	uint8_t *raw_data;	// Buffer for uncompansated data
	int32_t ut, up;		// Uncompensated temperature and pressure
	int32_t b5;			// Intermediate value for temperature calculation
	uint8_t oss;		// Oversampling setting
} BMP180_Handle_t;

typedef struct {
	double temperature;
	int32_t pressure;
	uint32_t timestamp;
} BMP180_Sensor_Results_t;

typedef enum {
	BMP180_OK = 0,
	BMP180_ERROR = -1,
} BMP180_Status_t;

BMP180_Status_t BMP180_init(BMP180_Handle_t *hbmp180, BMP180_I2C_Config_t *i2c_config, uint8_t *raw_data_buffer);
BMP180_Status_t BMP180_read_ut(BMP180_Handle_t *hbmp180);
BMP180_Status_t BMP180_read_up(BMP180_Handle_t *hbmp180);
int32_t BMP180_calc_temp(BMP180_Handle_t *hbmp180);
int32_t BMP180_calc_pres(BMP180_Handle_t *hbmp180);
BMP180_Status_t BMP180_get_sensor_results(BMP180_Handle_t *hbmp180, BMP180_Sensor_Results_t *results);

#endif // BMP180_H