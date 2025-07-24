#include "bmp180.h"


int i2c_read_byte(const struct i2c_client *client, u8 reg) {
    if(i2c_master_send(client, &reg, 1) < 0) {
        dev_err(client->dev, "Failed to write register address\n");
        return -1;
    }
    u8 data;
    if(i2c_master_recv(client, &data, 1) < 0) {
        dev_err(client->dev, "Failed to read data from register\n");
        return -1;
    }
    return data;
}

int i2c_write_byte(const struct i2c_client *client, u8 reg, u8 value) {
    u8 buf[2] = {reg, value};
    if(i2c_master_send(client, buf, 2) < 0) {
        dev_err(client->dev, "Failed to write data to register\n");
        return -1;
    }
    return 0;
}

int i2c_read_block(const struct i2c_client *client, u8 reg, u8 *buf, size_t len) {
    if(i2c_master_send(client, &reg, 1) < 0) {
        dev_err(client->dev, "Failed to write register address for block read\n");
        return -1;
    }
    if(i2c_master_recv(client, buf, len) < 0) {
        dev_err(client->dev, "Failed to read block data from register\n");
        return -1;
    }
    return 0;
}

/*
 * Initializes the BMP180 sensor
 *
 * @hbmp180: Pointer to the BMP180 handle structure
 * @i2c_config: Pointer to the I2C configuration structure
 * @raw_data_buffer: Pointer to a buffer for storing uncompensated data
 * 
*/
BMP180_Status_t BMP180_init(BMP180_Handle_t *hbmp180) {
    u8 calib_buffer[22];
    u8 dev_id;
    // Try to read id register
    dev_id = i2c_read_byte(hbmp180->client, BMP180_DEV_ID_REG);
    if(dev_id != BMP180_DEV_ID) {
        dev_err(hbmp180->client->dev, "I2C read failed! BMP180 device ID mismatch\n");
        return BMP180_ERROR;
    }
    // Soft reset
    if(i2c_write_byte(hbmp180->client, BMP180_RST_REG, BMP180_RST_CMD) < 0) {
        dev_err(hbmp180->client->dev, "Soft reset failed\n");
        return BMP180_ERROR;
    }
    msleep(100); // Wait for reset to complete

    // Read calibration data
    if(i2c_read_block(hbmp180->client, BMP180_EEPROM_ADDR, calib_buffer, BMP180_EEPROM_SIZE) < 0){
        dev_err(hbmp180->client->dev, "Failed to read calibration data\n");
        return BMP180_ERROR;
    }

    // Initialize handle
    hbmp180->ut = 0;
	hbmp180->up = 0;
	hbmp180->b5 = 0;
    hbmp180->oss = 1;

    // Parse calibration data
    hbmp180->calib.AC1 = (calib_buffer[0] << 8) | calib_buffer[1];
    hbmp180->calib.AC2 = (calib_buffer[2] << 8) | calib_buffer[3];
    hbmp180->calib.AC3 = (calib_buffer[4] << 8) | calib_buffer[5];
    hbmp180->calib.AC4 = (calib_buffer[6] << 8) | calib_buffer[7];
    hbmp180->calib.AC5 = (calib_buffer[8] << 8) | calib_buffer[9];
    hbmp180->calib.AC6 = (calib_buffer[10] << 8) | calib_buffer[11];
    hbmp180->calib.B1 = (calib_buffer[12] << 8) | calib_buffer[13];
    hbmp180->calib.B2 = (calib_buffer[14] << 8) | calib_buffer[15];
    hbmp180->calib.MB = (calib_buffer[16] << 8) | calib_buffer[17];
    hbmp180->calib.MC = (calib_buffer[18] << 8) | calib_buffer[19];
    hbmp180->calib.MD = (calib_buffer[20] << 8) | calib_buffer[21];

    dev_info(hbmp180->client->dev, "BMP180 initialized successfully\n");
    return BMP180_OK;
}

/*
 * Reads the uncompensated temperature from the BMP180 sensor
 *
*/
BMP180_Status_t BMP180_read_ut(BMP180_Handle_t *hbmp180) {
    if(i2c_write_byte(hbmp180->client, BMP180_CTL_REG, BMP180_READ_TEMP_CMD) < 0) {
        dev_err(hbmp180->client->dev, "Failed to write temperature read command\n");
        return BMP180_ERROR;
    }
    usleep_range(4500, 5000); // Wait for conversion to complete
    
    if(i2c_read_block(hbmp180->client, BMP180_DATA_REG_MSB, hbmp180->raw_data, 2) < 0) {
        dev_err(hbmp180->client->dev, "Failed to read uncompensated temperature data\n");
        return BMP180_ERROR;
    }
    hbmp180->ut = (hbmp180->raw_data[0] << 8) | hbmp180->raw_data[1];
    return BMP180_OK;
}

/*
 * Reads the uncompensated pressure from the BMP180 sensor
 *
*/
BMP180_Status_t BMP180_read_up(BMP180_Handle_t *hbmp180) {
    u8 cmd = BMP180_READ_PRES_CMD + (hbmp180->oss << 6);
    
    if(i2c_write_byte(hbmp180->client, BMP180_CTL_REG, cmd) < 0) {
        dev_err(hbmp180->client->dev, "Failed to write pressure read command\n");
        return BMP180_ERROR;
    }

    usleep_range(BMP180_CONVERSION_TIME(hbmp180->oss), BMP180_CONVERSION_TIME(hbmp180->oss) + 500); // Wait for conversion to complete

    if(i2c_read_block(hbmp180->client, BMP180_DATA_REG_MSB, hbmp180->raw_data, 3) < 0) {
        dev_err(hbmp180->client->dev, "Failed to read uncompensated pressure data\n");
        return BMP180_ERROR;
    }
    hbmp180->up = ((hbmp180->raw_data[0] << 16) | (hbmp180->raw_data[1] << 8) | hbmp180->raw_data[2]) >> (8 - hbmp180->oss);
    return BMP180_OK;
}

/*
 * Calculates the compensated temperature from the uncompensated value
 *
*/
s32 BMP180_calc_temp(BMP180_Handle_t *hbmp180) {
    s32 x1 = (hbmp180->ut - hbmp180->calib.AC6) * hbmp180->calib.AC5 / (1 << 15);
    s32 x2 = (hbmp180->calib.MC * (1 << 11)) / (x1 + hbmp180->calib.MD);
    hbmp180->b5 = x1 + x2;          // Intermediate value for temperature calculation
	return (hbmp180->b5+8)/(1<<4);
}

/*
 * Calculates the compensated pressure from the uncompensated value
 *
*/
s32 BMP180_calc_pres(BMP180_Handle_t *hbmp180) {
    s32 b6 = hbmp180->b5-4000;
	s32 x1 = (hbmp180->calib.B2*(b6*b6>>12))>>11;
	s32 x2 = hbmp180->calib.AC2*b6 >> 11;
	s32 x3 = x1+x2;
	s32 b3 = (((hbmp180->calib.AC1*4+x3)<<hbmp180->oss)+2)>>2;
	x1 = hbmp180->calib.AC3*b6>>13;
	x2 = (hbmp180->calib.B1*(b6*b6>>12))>>16;
	x3 = ((x1+x2)+2)>>2;
	unsigned long b4 = hbmp180->calib.AC4*(unsigned long)(x3+32768)>>15;
	unsigned long b7 = ((unsigned long)hbmp180->up-b3)*(50000>>hbmp180->oss);
	s32 p;
	if(b7 < 0x80000000)	p = (b7*2)/b4;
	else p = (b7/b4)*2;
	x1 = (p >> 8)*(p >> 8);
	x1 = (x1*3038)>>16;
	x2 = (-7357*p)>>16;
	p=p+(x1+x2+3791)/16;
	return p;
}

/*
 * Gets the sensor results including temperature and pressure
 *
*/
BMP180_Status_t BMP180_get_sensor_results(BMP180_Handle_t *hbmp180, BMP180_Sensor_Results_t *results) {
    
    if(BMP180_read_ut(hbmp180) != BMP180_OK) {
        return BMP180_ERROR;
    }
    if(BMP180_read_up(hbmp180) != BMP180_OK) {
        return BMP180_ERROR;
    }
    results->temperature = BMP180_calc_temp(hbmp180) / 10.0;
    results->pressure = BMP180_calc_pres(hbmp180);
    return BMP180_OK;
}

static void bmp180_timer_callback(struct timer_list *t) {
    BMP180_Handle_t *hbmp180 = from_timer(hbmp180, t, timer);
    BMP180_Sensor_Results_t bmp180_results;
    if(BMP180_get_sensor_results(hbmp180, &bmp180_results) == BMP180_OK) {
        dev_info(hbmp180->client->dev, "Temperature: %.2f C, Pressure: %d Pa",
            bmp180_results.temperature, bmp180_results.pressure);
    }
    else {
        dev_err(hbmp180->client->dev, "Failed to get sensor results\n");
    }
    mod_timer(&hbmp180->timer, jiffies + msecs_to_jiffies(5000));
}


static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    BMP180_Handle_t *hbmp180;

    hbmp180 = devm_kzalloc(&client->dev, sizeof(BMP180_Handle_t), GFP_KERNEL);
    if (!hbmp180)
        return -ENOMEM;
    
    hbmp180->raw_data = devm_kzalloc(&client->dev, 3, GFP_KERNEL);
    if (!hbmp180->raw_data)
        return -ENOMEM;
    hbmp180->client = client;
    if (BMP180_init(hbmp180) != BMP180_OK) {
        dev_err(&client->dev, "BMP180 init failed\n");
        return -EIO;
    }
    i2c_set_clientdata(client, hbmp180);

    timer_setup(&hbmp180->timer, bmp180_timer_callback, 0);
    mod_timer(&hbmp180->timer, jiffies + msecs_to_jiffies(5000));

    dev_info(hbmp180->client->dev, "BMP180 kernel module loaded\n");
    return 0;
}

static int bmp180_remove(struct i2c_client *client)
{
    BMP180_Handle_t *hbmp180 = i2c_get_clientdata(client);
    del_timer_sync(&hbmp180->timer);
    dev_info(hbmp180->client->dev, "BMP180: Device removed\n");
    return 0;
}

static const struct of_device_id bmp180_of_match[] = {
    { .compatible = "bosch,bmp180" },
    { }
};

MODULE_DEVICE_TABLE(of, bmp180_of_match);

static const struct i2c_device_id bmp180_id[] = {
    { "bmp180", 0 },
    { }
};

static struct i2c_driver bmp180_driver = {
    .driver = {
        .name = "bmp180",
        .of_match_table = bmp180_of_match,  // Add this line
    },
    .probe = bmp180_probe,
    .remove = bmp180_remove,
    .id_table = bmp180_id,
};

MODULE_DEVICE_TABLE(i2c, bmp180_id);

module_i2c_driver(bmp180_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Utku Boran Unal <boranutku@protonmail.com>");
MODULE_DESCRIPTION("BMP180 Driver");