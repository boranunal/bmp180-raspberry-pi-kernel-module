#include "bmp180.h"


static int i2c_read_byte(const struct i2c_client *client, u8 reg) {
    if(i2c_master_send(client, &reg, 1) < 0) {
        dev_err(&client->dev, "Failed to write register address\n");
        return -1;
    }
    u8 data;
    if(i2c_master_recv(client, &data, 1) < 0) {
        dev_err(&client->dev, "Failed to read data from register\n");
        return -1;
    }
    return data;
}

static int i2c_write_byte(const struct i2c_client *client, u8 reg, u8 value) {
    u8 buf[2] = {reg, value};
    if(i2c_master_send(client, buf, 2) < 0) {
        dev_err(&client->dev, "Failed to write data to register\n");
        return -1;
    }
    return 0;
}

static int i2c_read_block(const struct i2c_client *client, u8 reg, u8 *buf, size_t len) {
    if(i2c_master_send(client, &reg, 1) < 0) {
        dev_err(&client->dev, "Failed to write register address for block read\n");
        return -1;
    }
    if(i2c_master_recv(client, buf, len) < 0) {
        dev_err(&client->dev, "Failed to read block data from register\n");
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
        dev_err(&hbmp180->client->dev, "I2C read failed! BMP180 device ID mismatch\n");
        return BMP180_ERROR;
    }
    // Soft reset
    if(i2c_write_byte(hbmp180->client, BMP180_RST_REG, BMP180_RST_CMD) < 0) {
        dev_err(&hbmp180->client->dev, "Soft reset failed\n");
        return BMP180_ERROR;
    }
    usleep_range(10000, 10500); // Wait for reset to complete

    // Read calibration data
    if(i2c_read_block(hbmp180->client, BMP180_EEPROM_ADDR, calib_buffer, BMP180_EEPROM_SIZE) < 0){
        dev_err(&hbmp180->client->dev, "Failed to read calibration data\n");
        return BMP180_ERROR;
    }

    // Initialize handle
    hbmp180->data.ut = 0;
	hbmp180->data.up = 0;
	hbmp180->data.b5 = 0;
    hbmp180->data.oss = 0;

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

    dev_info(&hbmp180->client->dev, "BMP180 initialized successfully\n");
    
    return BMP180_OK;
}

/*
 * Reads the uncompensated temperature from the BMP180 sensor
 *
*/
BMP180_Status_t BMP180_read_ut(BMP180_Handle_t *hbmp180) {
    if(i2c_write_byte(hbmp180->client, BMP180_CTL_REG, BMP180_READ_TEMP_CMD) < 0) {
        dev_err(&hbmp180->client->dev, "Failed to write temperature read command\n");
        return BMP180_ERROR;
    }

    usleep_range(4500, 5000);

    if(i2c_read_block(hbmp180->client, BMP180_DATA_REG_MSB, hbmp180->data.raw_data, 2) < 0) {
        dev_err(&hbmp180->client->dev, "Failed to read uncompensated temperature data\n");
        return BMP180_ERROR;
    }
    hbmp180->data.ut = (hbmp180->data.raw_data[0] << 8) | hbmp180->data.raw_data[1];
    hbmp180->results.temperature = BMP180_calc_temp(hbmp180);
    dev_info(&hbmp180->client->dev, "Temperature: %d C", hbmp180->results.temperature);

    return BMP180_OK;
}

/*
 * Reads the uncompensated pressure from the BMP180 sensor
 *
*/
BMP180_Status_t BMP180_read_up(BMP180_Handle_t *hbmp180) {
    u8 cmd = BMP180_READ_PRES_CMD + (hbmp180->data.oss << 6);
    
    if(i2c_write_byte(hbmp180->client, BMP180_CTL_REG, cmd) < 0) {
        dev_err(&hbmp180->client->dev, "Failed to write pressure read command\n");
        return BMP180_ERROR;
    }

    usleep_range(BMP180_CONVERSION_TIME(hbmp180->data.oss), BMP180_CONVERSION_TIME(hbmp180->data.oss) + 500); // Wait for conversion to complete

    if(i2c_read_block(hbmp180->client, BMP180_DATA_REG_MSB, hbmp180->data.raw_data, 3) < 0) {
        dev_err(&hbmp180->client->dev, "Failed to read uncompensated pressure data\n");
        return BMP180_ERROR;
    }

    hbmp180->data.up = ((hbmp180->data.raw_data[0] << 16) | (hbmp180->data.raw_data[1] << 8) | hbmp180->data.raw_data[2]) >> (8 - hbmp180->data.oss);
    hbmp180->results.pressure = BMP180_calc_pres(hbmp180);
    dev_info(&hbmp180->client->dev, "Pressure: %d Pa", hbmp180->results.pressure);
    
    return BMP180_OK;
}

/*
 * Calculates the compensated temperature from the uncompensated value
 *
*/
s32 BMP180_calc_temp(BMP180_Handle_t *hbmp180) {
    s32 x1 = (hbmp180->data.ut - hbmp180->calib.AC6) * hbmp180->calib.AC5 / (1 << 15);
    s32 x2 = (hbmp180->calib.MC * (1 << 11)) / (x1 + hbmp180->calib.MD);
    hbmp180->data.b5 = x1 + x2;          // Intermediate value for temperature calculation
	return (hbmp180->data.b5+8)/(1<<4);
}

/*
 * Calculates the compensated pressure from the uncompensated value
 *
*/
s32 BMP180_calc_pres(BMP180_Handle_t *hbmp180) {
    s32 b6 = hbmp180->data.b5-4000;
	s32 x1 = (hbmp180->calib.B2*(b6*b6>>12))>>11;
	s32 x2 = hbmp180->calib.AC2*b6 >> 11;
	s32 x3 = x1+x2;
	s32 b3 = (((hbmp180->calib.AC1*4+x3)<<hbmp180->data.oss)+2)>>2;
	x1 = hbmp180->calib.AC3*b6>>13;
	x2 = (hbmp180->calib.B1*(b6*b6>>12))>>16;
	x3 = ((x1+x2)+2)>>2;
	unsigned long b4 = hbmp180->calib.AC4*(unsigned long)(x3+32768)>>15;
	unsigned long b7 = ((unsigned long)hbmp180->data.up-b3)*(50000>>hbmp180->data.oss);
	s32 p;
	if(b7 < 0x80000000)	p = (b7*2)/b4;
	else p = (b7/b4)*2;
	x1 = (p >> 8)*(p >> 8);
	x1 = (x1*3038)>>16;
	x2 = (-7357*p)>>16;
	p=p+(x1+x2+3791)/16;
	return p;
}


static void bmp180_work_handler(struct work_struct *work) {
    struct bmp180_work *bmp_work = container_of(work, struct bmp180_work, work);
    BMP180_Handle_t *hbmp180 = bmp_work->hbmp180;
    
    dev_info(&hbmp180->client->dev, "BMP180 work handler triggered\\n");
    
    if(BMP180_read_ut(hbmp180) != BMP180_OK) {
        return;
    }
    if(BMP180_read_up(hbmp180) != BMP180_OK) {
        return;
    }
}

static void bmp180_timer_callback(struct timer_list *t) {
    struct timer_data *tmd = from_timer(tmd, t, timer);
    if(!tmd || !tmd->hbmp180 || !tmd->hbmp180->work_data) return;

    queue_work(tmd->hbmp180->wq, &tmd->hbmp180->work_data->work);

    BMP180_Handle_t *hbmp180 = tmd->hbmp180;
    dev_info(&hbmp180->client->dev, "BMP180 timer callback triggered\n");
    printk(KERN_INFO "BMP180 timer callback triggered\n");


    mod_timer(&tmd->timer, jiffies + msecs_to_jiffies(5000));
}


static int bmp180_probe(struct i2c_client *client){
    printk(KERN_INFO "BMP180 probe function called\n");
    // Allocate memory for timer_data structure
    struct timer_data *tmd = kzalloc(sizeof(*tmd), GFP_KERNEL);;
    if (!tmd)
        return -ENOMEM;
    else
        printk(KERN_INFO "BMP180 timer_data allocated\n");
    // Allocate memory for BMP180_Handle_t structure
    BMP180_Handle_t *hbmp180 = devm_kzalloc(&client->dev, sizeof(BMP180_Handle_t), GFP_KERNEL);
    if (!hbmp180)
        return -ENOMEM;
    else 
        printk(KERN_INFO "BMP180 handle allocated\n");

    tmd->hbmp180 = hbmp180;
    tmd->hbmp180->data.raw_data = devm_kzalloc(&client->dev, 3, GFP_KERNEL);
    if (!tmd->hbmp180->data.raw_data)
        return -ENOMEM;
    else 
        printk(KERN_INFO "BMP180 raw_data buffer allocated\n");
    tmd->hbmp180->client = client;
    
    if (BMP180_init(tmd->hbmp180) != BMP180_OK) {
        dev_err(&client->dev, "BMP180 init failed\n");
        return -EIO;
    }

    tmd->hbmp180->work_data = kzalloc(sizeof(struct bmp180_work), GFP_KERNEL);
    if (!tmd->hbmp180->work_data) 
        return -ENOMEM;
    // Create workqueue
    tmd->hbmp180->wq = create_singlethread_workqueue("bmp180_wq");
    if (!tmd->hbmp180->wq) {
        kfree(tmd->hbmp180->work_data);
        return -ENOMEM;
    }
    // Initialize work
    tmd->hbmp180->work_data->hbmp180 = tmd->hbmp180;
    INIT_WORK(&tmd->hbmp180->work_data->work, bmp180_work_handler);

    i2c_set_clientdata(client, tmd);
    timer_setup(&tmd->timer, bmp180_timer_callback, 0);
    mod_timer(&tmd->timer, jiffies + msecs_to_jiffies(5000));

    dev_info(&client->dev, "BMP180 kernel module loaded\n");

    return 0;
}

static void bmp180_remove(struct i2c_client *client)
{
    struct timer_data *tmd = i2c_get_clientdata(client);
    if(tmd) {
        // Delete both timers
        del_timer_sync(&tmd->timer);
        if(tmd->hbmp180) {

            if (tmd->hbmp180->work_data) {
            cancel_work_sync(&tmd->hbmp180->work_data->work);
            kfree(tmd->hbmp180->work_data);
            }

            if (tmd->hbmp180->wq) {
            destroy_workqueue(tmd->hbmp180->wq);
            }
        }
        
        // Log before freeing
        dev_info(&client->dev, "BMP180: Device removed\n");
        
        kfree(tmd); 
    } else {
        dev_err(&client->dev, "BMP180: No device data found\n");
    }
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
        .owner = THIS_MODULE,
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