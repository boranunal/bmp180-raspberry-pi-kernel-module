#include "bmp180.h"

BMP180_EEPROM_t _bmp180_calib;

uint8_t rst_sqnc[] = {0xE0, 0xB6}; // Reset sequence for BMP180

BMP180_Status_t BMP180_init(BMP180_Handle_t *hbmp180, BMP180_I2C_Config_t *i2c_config, uint8_t *raw_data_buffer) {
    uint8_t calib_buffer[22];
    uint8_t dev_id;
    // Try to read id register
    dev_id = i2cReadByteData(hbmp180->hi2c, BMP180_DEV_ID_REG);
    if(dev_id != BMP180_DEV_ID) {
        printk(KERN_ERR "I2C read failed! BMP180 device ID mismatch\n");
        return BMP180_ERROR;
    }
    // Soft reset
    if(i2cWriteI2CBlockData(hbmp180->hi2c, BMP180_DEV_ADDR, rst_sqnc, 2) < 0) {
        printk(KERN_ERR "Soft reset failed\n");
        return BMP180_ERROR;
    }
    sleep(100); // Wait for reset to complete

    // Read calibration data
    if(i2cReadI2CBlockData(hbmp180->hi2c, BMP180_DEV_ADDR, BMP180_EEPROM_ADDR, calib_buffer, BMP180_EEPROM_SIZE) < 0){
        printk(KERN_ERR "Failed to read calibration data\n");
        return BMP180_ERROR;
    }

    // Initialize handle
    hbmp180->ut = 0;
	hbmp180->up = 0;
	hbmp180->b5 = 0;
    hbmp180->oss = 0;
    hbmp180->hi2c = i2c_config;
    hbmp180->raw_data = raw_data_buffer;

    // Parse calibration data
    _bmp180_calib.AC1 = (calib_buffer[0] << 8) | calib_buffer[1];
    _bmp180_calib.AC2 = (calib_buffer[2] << 8) | calib_buffer[3];
    _bmp180_calib.AC3 = (calib_buffer[4] << 8) | calib_buffer[5];
    _bmp180_calib.AC4 = (calib_buffer[6] << 8) | calib_buffer[7];
    _bmp180_calib.AC5 = (calib_buffer[8] << 8) | calib_buffer[9];
    _bmp180_calib.AC6 = (calib_buffer[10] << 8) | calib_buffer[11];
    _bmp180_calib.B1 = (calib_buffer[12] << 8) | calib_buffer[13];
    _bmp180_calib.B2 = (calib_buffer[14] << 8) | calib_buffer[15];
    _bmp180_calib.MB = (calib_buffer[16] << 8) | calib_buffer[17];
    _bmp180_calib.MC = (calib_buffer[18] << 8) | calib_buffer[19];
    _bmp180_calib.MD = (calib_buffer[20] << 8) | calib_buffer[21];
    memcpy(&hbmp180->calib, &_bmp180_calib, sizeof(BMP180_EEPROM_t));

    printk(KERN_INFO "BMP180 initialized successfully\n");
    return BMP180_OK;
}

BMP180_Status_t BMP180_read_ut(BMP180_Handle_t *hbmp180) {
    uint8_t cmd = BMP180_READ_TEMP_CMD;
    int i2c_status = i2cWriteByteData(hbmp180->hi2c, BMP180_CTL_REG_ADDR, cmd)
    if(i2c_status != 0) {
        printk(KERN_ERR "%d: Failed to write temperature read command\n", i2c_status);
        return BMP180_ERROR;
    }
    usleep_range(4500, 5000); // Wait for conversion to complete
    
    i2c_status = i2cReadI2CBlockData(hbmp180->hi2c, BMP180_DEV_ADDR, BMP180_DATA_REG_MSB, hbmp180->raw_data, 2);
    if(i2c_status < 0) {
        printk(KERN_ERR "%d: Failed to read uncompensated temperature data\n", i2c_status);
        return BMP180_ERROR;
    }
    hbmp180->ut = (hbmp180->raw_data[0] << 8) | hbmp180->raw_data[1];
    return BMP180_OK;
}

BMP180_Status_t BMP180_read_up(BMP180_Handle_t *hbmp180) {
    uint8_t cmd = BMP180_READ_PRES_CMD + (hbmp180->oss << 6);
    
    int i2c_status = i2cWriteByteData(hbmp180->hi2c, BMP180_CTL_REG_ADDR, cmd);
    if(i2c_status != 0) {
        printk(KERN_ERR "%d: Failed to write pressure read command\n", i2c_status);
        return BMP180_ERROR;
    }

    usleep_range(BMP180_CONVERSION_TIME(hbmp180->oss), BMP180_CONVERSION_TIME(hbmp180->oss) + 500); // Wait for conversion to complete

    i2c_status = i2cReadI2CBlockData(hbmp180->hi2c, BMP180_DEV_ADDR, BMP180_DATA_REG_MSB, hbmp180->raw_data, 3);
    if(i2c_status < 0) {
        printk(KERN_ERR "%d: Failed to read uncompensated pressure data\n", i2c_status);
        return BMP180_ERROR;
    }
    hbmp180->up = ((hbmp180->raw_data[0] << 16) | (hbmp180->raw_data[1] << 8) | hbmp180->raw_data[2]) >> (8 - hbmp180->oss);
    return BMP180_OK;
}

int32_t BMP180_calc_temp(BMP180_Handle_t *hbmp180) {
    int32_t x1 = (hbmp180->ut - hbmp180->calib.AC6) * hbmp180->calib.AC5 / (1 << 15);
    int32_t x2 = (hbmp180->calib.MC * (1 << 11)) / (x1 + hbmp180->calib.MD);
    hbmp180->b5 = x1 + x2;          // Intermediate value for temperature calculation
	return (hbmp180->b5+8)/(1<<4);
}

int32_t BMP180_calc_pres(BMP180_Handle_t *hbmp180) {
    int32_t b6 = hbmp180->b5-4000;
	int32_t x1 = (hbmp180->calib.B2*(b6*b6>>12))>>11;
	int32_t x2 = hbmp180->calib.AC2*b6 >> 11;
	int32_t x3 = x1+x2;
	int32_t b3 = (((hbmp180->calib.AC1*4+x3)<<oss)+2)>>2;
	x1 = hbmp180->calib.AC3*b6>>13;
	x2 = (hbmp180->calib.B1*(b6*b6>>12))>>16;
	x3 = ((x1+x2)+2)>>2;
	unsigned long b4 = hbmp180->calib.AC4*(unsigned long)(x3+32768)>>15;
	unsigned long b7 = ((unsigned long)hbmp180->up-b3)*(50000>>oss);
	int32_t p;
	if(b7 < 0x80000000)	p = (b7*2)/b4;
	else p = (b7/b4)*2;
	x1 = (p >> 8)*(p >> 8);
	x1 = (x1*3038)>>16;
	x2 = (-7357*p)>>16;
	p=p+(x1+x2+3791)/16;
	return p;
}

BMP180_Status_t BMP180_get_sensor_results(BMP180_Handle_t *hbmp180, BMP180_Sensor_Results_t *results) {
    
    if(BMP180_read_ut(hbmp180) != BMP180_OK) {
        return BMP180_ERROR;
    }
    if(BMP180_read_up(hbmp180) != BMP180_OK) {
        return BMP180_ERROR;
    }
    results->temperature = BMP180_calc_temp(hbmp180) / 10.0;
    results->pressure = BMP180_calc_pres(hbmp180);
    result->timestamp = ktime_to_ms(ktime_get());
    return BMP180_OK;
}