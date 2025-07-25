/*
 * Author: Utku Boran Unal
 * 
 * bmp180.h
 * 
 * This header file defines the structures, constants, and function prototypes
 * for interacting with the BMP180 pressure and temperature sensor.
 *
*/

#ifndef BMP180_H
#define BMP180_H

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/module.h> 

#define BMP180_DEV_ADDR 0x77
#define BMP180_DEV_ID_REG 0xD0
#define BMP180_DEV_ID 0x55
#define BMP180_CTL_REG 0xF4
#define BMP180_RST_REG 0xE0
#define BMP180_RST_CMD 0xB6
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

typedef struct{
	BMP180_EEPROM_t calib;
	struct i2c_client *client;
	u8 *raw_data;	// Buffer for uncompansated data
	s32 ut, up;		// Uncompensated temperature and pressure
	s32 b5;			// Intermediate value for temperature calculation
	u8 oss;		// Oversampling setting
} BMP180_Handle_t;

struct timer_data {
	struct timer_list timer;
	BMP180_Handle_t *hbmp180;
};

typedef struct {
	s32 temperature;
	s32 pressure;
} BMP180_Sensor_Results_t;

typedef enum {
	BMP180_OK = 0,
	BMP180_ERROR = -1,
} BMP180_Status_t;

static int i2c_read_byte(const struct i2c_client *client, u8 reg);
static int i2c_write_byte(const struct i2c_client *client, u8 reg, u8 value);
static int i2c_read_block(const struct i2c_client *client, u8 reg, u8 *buf, size_t len);
BMP180_Status_t BMP180_init(BMP180_Handle_t *hbmp180);
BMP180_Status_t BMP180_read_ut(BMP180_Handle_t *hbmp180);
BMP180_Status_t BMP180_read_up(BMP180_Handle_t *hbmp180);
s32 BMP180_calc_temp(BMP180_Handle_t *hbmp180);
s32 BMP180_calc_pres(BMP180_Handle_t *hbmp180);
BMP180_Status_t BMP180_get_sensor_results(BMP180_Handle_t *hbmp180, BMP180_Sensor_Results_t *results);
static int bmp180_probe(struct i2c_client *client);
static void bmp180_remove(struct i2c_client *client);

#endif // BMP180_H