/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
//#include <stdlib.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include <linux/power/bq27541_battery.h>


#define BUFFERSIZE 32 				/*BUFFER SIZE*/
#define DEVICEIDNUMBYTES  8
#define DIGESTNUMBYTES  16
#define SECRETKEYNUMBYTES 16
#define RANDMESGNUMBYTES  20
unsigned char DeviceID[DEVICEIDNUMBYTES];           // Stores the Device ID data
unsigned char Digest[DIGESTNUMBYTES];               // SHA1 response from the bq27541

//unsigned char TxData[BUFFERSIZE];           // Stores data bytes to be TX'd
//unsigned char RxData[BUFFERSIZE];           // Stores data bytes that are RX'd
unsigned int  temperature;                  // Stores temperature
unsigned int  voltage;                      // Stores voltage
  signed int  atrate;                       // Stores AtRate
unsigned int  artte;                        // Stores AtRate Time to Empty
unsigned int  soc;                          // Stores State of Charge
unsigned int  dcap;                         // Stores Design Capacity
unsigned int  dnamelen;                     // Stores Device Name Length
unsigned char Key[SECRETKEYNUMBYTES];
unsigned char Message[RANDMESGNUMBYTES];
unsigned char Digest[DIGESTNUMBYTES]; 
unsigned int H[5];
unsigned int Random[5]; // 16 bytes random message for bq26100 to use in SHA1/HMAC
unsigned int Ws[80];			            // Global Work schedule variable
#define bq27541_DELAY			msecs_to_jiffies(5000)
#define I2C_MASK_FLAG	(0x00ff)
#define I2C_WR_FLAG		(0x1000)
struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(struct bq27x00_device_info *di, u8 reg, bool single);
};

enum bq27x00_chip { BQ27000, BQ27500, BQ27541 };

struct bq27x00_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int flags;

	int current_now;
};
/***
*bq27541 device information
*
*
**/
struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	enum bq27x00_chip	chip;

	struct bq27x00_reg_cache cache;
	int charge_design_full;

	unsigned long last_update;
	struct delayed_work work;

	struct power_supply	bat;

	struct bq27x00_access_methods bus;

	struct mutex lock;
};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
};

static unsigned int poll_interval = 360;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");
unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb);

static struct i2c_client *new_client = NULL;
/*
 * Common code for BQ27x00 devices
 */

static inline int bq27x00_read(struct bq27x00_device_info *di, u8 reg,
		bool single)
{
	return di->bus.read(di, reg, single);
}


static DEFINE_MUTEX(bq27541_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write bq27541] 
  *
  *********************************************************/  
  
int bq27541_set_cmd_read(struct bq27x00_device_info *di,u8 cmd, int *returnData)
{
    struct i2c_client *client = to_i2c_client(di->dev);
    char     cmd_buf[2]={0x00, 0x00};
    int      readData = 0;
    int      ret=0;

    
    
    //mutex_lock(&bq27541_i2c_access);
    
   // new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    //new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG | I2C_RS_FLAG;

    //cmd_buf[0] = cmd;
    //ret = mt_i2c_master_send(new_client, &cmd_buf[0], (2<<8 | 1), new_client->ext_flag);
        ret = i2c_smbus_read_byte_data(client, cmd);
	//ret=i2c_smbus_read_word_data(client, reg);

	if (ret < 0)
	{
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		//mutex_unlock(&bq27541_i2c_access);
		return 0;
        }
        *returnData = ret;
	
	//mutex_unlock(&bq27541_i2c_access);  
	
	return ret;    
} 


int bq27541_set_cmd_write(struct bq27x00_device_info *di, u8 cmd, int WriteData)
{
        struct i2c_client *client = to_i2c_client(di->dev);
	//char     cmd_buf[3]={0x00, 0x00};
        int      ret=0;
        //mutex_lock(&bq27541_i2c_access);
	//ret = i2c_smbus_write_byte_data(client, reg, value);
	//client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG; 
	//cmd_buf[0] = cmd;
	//cmd_buf[1] = WriteData;
	ret=i2c_smbus_write_byte_data(client, cmd,WriteData);

	if (ret < 0)
	{
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
                
                //mutex_unlock(&bq27541_i2c_access);
                return ret;
	}
	//mdelay(500);
	//mutex_unlock(&bq27541_i2c_access);    
	return ret;
}


int bq27541_write_block_reg(struct bq27x00_device_info *di, int reg, int count, u8 *buf)
{
        struct i2c_client *client = to_i2c_client(di->dev);
	int ret;
	ret = i2c_smbus_write_i2c_block_data(client, reg, count, buf);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		
        //mdelay(500);

	return ret;
}

int bq27541_read_block_reg(struct bq27x00_device_info *di, int reg, int count, u8 *buf)
{
        struct i2c_client *client = to_i2c_client(di->dev);
	int ret;
	ret = i2c_smbus_read_i2c_block_data(client, reg, count, buf);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}



static int bq27x00_battery_read_rsoc(struct bq27x00_device_info *di)
{
	int rsoc;

	if (di->chip == BQ27541)
		rsoc = bq27x00_read(di, BQ27541_REG_SOC, false);
	else
		rsoc = bq27x00_read(di, BQ27000_REG_RSOC, true);

	if (rsoc < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return rsoc;
}

/*
 * Return a battery charge value in mAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_charge(struct bq27x00_device_info *di, u8 reg)
{
	int charge;

	charge = bq27x00_read(di, reg, false);
	if (charge < 0) {
		dev_err(di->dev, "error reading nominal available capacity\n");
		return charge;
	}

	if (di->chip == BQ27500)
		charge *= 1000;
	else
		charge = charge * 3570 / BQ27000_RS;

	return charge;
}

/*
 * Return the battery Nominal available capaciy in mAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_nac(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_NAC);
}

/*
 * Return the battery Last measured discharge in mAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_lmd(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_LMD);
}

/*
 * Return the battery Initial last measured discharge in mAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_ilmd(struct bq27x00_device_info *di)
{
	int ilmd;

	if (di->chip == BQ27541)
		ilmd = bq27x00_read(di, BQ27541_REG_DCAP, false);
	else
		ilmd = bq27x00_read(di, BQ27000_REG_ILMD, true);

	if (ilmd < 0) {
		dev_err(di->dev, "error reading initial last measured discharge\n");
		return ilmd;
	}
/*
	if (di->chip == BQ27500)
		ilmd *= 1000;
	else
		ilmd = ilmd * 256 * 3570 / BQ27000_RS;

	return ilmd;
*/
}


/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_cyct(struct bq27x00_device_info *di)
{
	int cyct;

	cyct = bq27x00_read(di, BQ27x00_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_time(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27x00_read(di, reg, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading register %02x: %d\n", reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}
/*****
* update the battery information (current capacity voltage and so forth)
*
*
*******/
static void bq27x00_update(struct bq27x00_device_info *di)
{
        unsigned char TxData[BUFFERSIZE];           // Stores data bytes to be TX'd
        unsigned char RxData[BUFFERSIZE];           // Stores data bytes that are RX'd
	struct bq27x00_reg_cache cache = {0, };
	bool is_bq27500 = di->chip == BQ27541;

	cache.flags = bq27x00_read(di, BQ27x00_REG_FLAGS, is_bq27500);
	if (cache.flags >= 0) {
	        printk("CACHE FLAG BIGGER THAN ZERO \n");
                   bq27541_read_block_reg(di,BQ27541_REG_SOC, 2,RxData);
  
		cache.capacity = transBytes2UnsignedInt(RxData[1], RxData[0]);
		printk("Remaining Capacity in Percentage %d%\n",cache.capacity);
		bq27541_read_block_reg(di,BQ27x00_REG_TEMP, 2,RxData);
		cache.temperature = transBytes2UnsignedInt(RxData[1], RxData[0]);
		printk("temperture %d\n",cache.temperature);
		bq27541_read_block_reg(di,BQ27x00_REG_TTE, 2,RxData);
		cache.time_to_empty = transBytes2UnsignedInt(RxData[1], RxData[0]);
		printk("Time to Empty %d\n",cache.time_to_empty);
		
		bq27541_read_block_reg(di,BQ27x00_REG_TTE, 2,RxData);
		
		cache.time_to_empty_avg = transBytes2UnsignedInt(RxData[1], RxData[0]);
		printk("Time to Empty %d Average\n",cache.time_to_empty_avg);
		//cache.time_to_full = bq27x00_battery_read_time(di, BQ27x00_REG_TTF);
		bq27541_read_block_reg(di,BQ27x00_REG_LMD, 2,RxData);
		cache.charge_full = transBytes2UnsignedInt(RxData[1], RxData[0]);
		printk("Charge_full %d\n",cache.charge_full); 
		
		bq27541_read_block_reg(di,BQ27x00_REG_CYCT, 2,RxData);
                
		cache.cycle_count = transBytes2UnsignedInt(RxData[1], RxData[0]);
                printk("Cycle count %d\n",cache.cycle_count);
		//if (!is_bq27500)
		bq27541_read_block_reg(di,BQ27x00_REG_AI, 2,RxData);
		cache.current_now = transBytes2UnsignedInt(RxData[1], RxData[0]);
		printk("Current %d ma\n",cache.current_now);
		
		/* We only have to read charge design full once */
		if (di->charge_design_full <= 0)
		{
		        bq27541_read_block_reg(di,BQ27541_REG_DCAP, 2,RxData);
		        di->charge_design_full = transBytes2UnsignedInt(RxData[1], RxData[0]);
                        printk("Design Capacity %d maH\n",di->charge_design_full);
                }
	}

	/* Ignore current_now which is a snapshot of the current battery state
	 * and is likely to be different even between two consecutive reads */
	if (memcmp(&di->cache, &cache, sizeof(cache) - sizeof(int)) != 0) {
		di->cache = cache;
		power_supply_changed(&di->bat);
	}

	di->last_update = jiffies;
}
/******
* polling update battery information  360ms
*
*
*********/
static void bq27x00_battery_poll(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, work.work);
        printk("enter polling chy");
	bq27x00_update(di);

//	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
//		set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
//		schedule_delayed_work(&di->work, poll_interval * HZ);
//	}
        schedule_delayed_work(&di->work, bq27541_DELAY);
}


/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	if (di->cache.temperature < 0)
		return di->cache.temperature;

	if (di->chip == BQ27541)
	{
		val->intval = di->cache.temperature - 2731;
                printk("CAIHAOYUAN temperture:%d",val->intval);
	}
	else
	{
		val->intval = ((di->cache.temperature * 5) - 5463) / 2;
	}
	return 0;
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int curr;

	if (di->chip == BQ27541)
	    curr = bq27x00_read(di, BQ27x00_REG_AI, false);
	else
	    curr = di->cache.current_now;

	if (curr < 0)
		return curr;

	if (di->chip == BQ27541){
		/* bq27500 returns signed value */
		val->intval = (int)((s16)curr) * 1000;
	} else {
		if (di->cache.flags & BQ27000_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * 3570 / BQ27000_RS;
	}

	return 0;
}
/********
*
*update the battery status such as CHARGING   DISCHARGING
*
**********/
static int bq27x00_battery_status(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int status;

	if (di->chip == BQ27541) {
		if (di->cache.flags & BQ27500_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27500_FLAG_DSG)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (di->cache.flags & BQ27000_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (power_supply_am_i_supplied(&di->bat))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	val->intval = status;

	return 0;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int volt;

	volt = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	if (volt < 0)
		return volt;

	val->intval = volt * 1000;

	return 0;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 * current discharge is negative during discharge positive during charge
 *  unit(mW Design Energy Scale=1)(cW Design Energy Scale=10)
 */
static int bq27x00_battery_energy(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int ae;

	ae = bq27x00_read(di, BQ27541_REG_AP, false);
	if (ae < 0) {
		dev_err(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27541)
		ae *= 1000;
	else
		ae = ae * 29200 / BQ27000_RS;

	val->intval = ae;

	return 0;
}


static int bq27x00_simple_value(int value,
	union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27x00_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27x00_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27x00_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27x00_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq27x00_simple_value(di->cache.capacity, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bq27x00_battery_temperature(di, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_simple_value(di->cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27x00_simple_value(bq27x00_battery_read_nac(di), val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27x00_simple_value(di->cache.charge_full, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27x00_simple_value(di->charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27x00_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27x00_battery_energy(di, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27x00_external_power_changed(struct power_supply *psy)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

static int bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	int ret;

	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = bq27x00_external_power_changed;

	
	mutex_init(&di->lock);

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);
        INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);
	schedule_delayed_work(&di->work, bq27541_DELAY);
	//bq27x00_update(di);

	return 0;
}

static void bq27x00_powersupply_unregister(struct bq27x00_device_info *di)
{
	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(&di->bat);

	mutex_destroy(&di->lock);
}

#define CONFIG_BATTERY_BQ27X00_I2C
/* i2c specific code */
#ifdef CONFIG_BATTERY_BQ27X00_I2C

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}
unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb)
{
  unsigned int tmp;
  
  tmp = ((msb << 8) & 0xFF00);
  return ((unsigned int)(tmp + lsb) & 0x0000FFFF);  
}
/**
*
*probe function ,device find driver and excute this fuction
*
**********/
static int bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	int num;
	unsigned int sum = 0;
	unsigned char checksum = 0;
	int  atrate = 0; 
	unsigned int  voltage = 0;  
	unsigned int  artte = 0;                 // Stores AtRate Time to Empty
	unsigned int bytes = 0;
	unsigned int  soc = 0;                          // Stores State of Charge
        unsigned int  dcap =0;                         // Stores Design Capacity
        unsigned int  dnamelen = 0;                     // Stores Device Name Length
        unsigned char msb, lsb;
	unsigned char TxData[BUFFERSIZE];           // Stores data bytes to be TX'd
        unsigned char RxData[BUFFERSIZE];           // Stores data bytes that are RX'd
	int retval = 0;
	int i = 0;
	int FW_VERSION=0;

        printk("CAIHAOYUAN ADD BQ27541 DRIVER\n");
       //     if (!(new_client = kmalloc(sizeof(struct i2c_client),    GFP_KERNEL))) {
//        err = -ENOMEM;
  //      goto exit;
  //;
   // }    
       //  memset(new_client, 0, sizeof(struct i2c_client));

       //  new_client = client;  
       // bq27541_set_cmd_write(0x00,0x0002);
       // bq27541_set_cmd_read(0x00,&FW_VERSION);
      //  printk("BQ27541 FW VERSION: %x \n",FW_VERSION);
	/* Get new ID for the new battery device */
	// Read Temperature (units = 0.1K
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	di->id = num;
	di->dev = &client->dev;
	di->chip = id->driver_data;
	di->bat.name = name;
	di->bus.read = &bq27x00_read_i2c;



	printk("enter powersupplyinit\n");
	if (bq27x00_powersupply_init(di))
		goto batt_failed_3;
	
        //tempture1=bq27x00_read(di, BQ27x00_REG_TEMP, false);
        

	i2c_set_clientdata(client, di);
	
	/**************************Initial**************************/
/*	
  bq27541_read_block_reg(di,BQ27x00_REG_TEMP, 2,RxData);
  temperature = transBytes2UnsignedInt(RxData[1], RxData[0]);
    printk("temperture %d\n",temperature);
  // Read Voltage (units = mV)
  bq27541_read_block_reg(di,BQ27x00_REG_VOLT, 2,RxData);
  voltage = transBytes2UnsignedInt(RxData[1], RxData[0]);
     printk("voltage %d\n",voltage); 
  // Set AtRate (units = mA)
  atrate = ATRATE_MA;
  msb = ((atrate >> 8) & 0x00FF);
  lsb = (atrate & 0x00FF);
  bytes = 0;
  //TxData[bytes++] = bq27541CMD_AR_LSB;
  TxData[bytes++] = lsb;
  TxData[bytes++] = msb;
  bq27541_write_block_reg(di,BQ27x00_REG_AR,2,TxData);
//  MSP430_bq27541_blockWrite(TxData, bytes);

  // Verify AtRate was set correctly (units = mA)
  bq27541_read_block_reg(di,BQ27x00_REG_AR, 2,RxData);
  if ((RxData[1] != msb) || (RxData[0] != lsb))
  {
    printk("Initial Fail");
    goto batt_failed_3;
  }
  printk("Write ATRATE SUCCESSFULY\n");
  bq27541_read_block_reg(di,BQ27541_REG_UFSOC,2,RxData);
  // Read AtRate Time to Empty (units = Minutes)
  //MSP430_bq27541_read(bq27541CMD_ARTTE_LSB, 2);
  artte = transBytes2UnsignedInt(RxData[1], RxData[0]);
  printk("Unfiltered Remaining Capacity in Percentage %d%\n",artte);
  // Read State of Charge (units = %)
   bq27541_read_block_reg(di,BQ27541_REG_SOC, 2,RxData);
  soc = transBytes2UnsignedInt(RxData[1], RxData[0]);
  printk("Remaining Capacity in Percentage %d%\n",soc);

  // Read Design Capacity (units = mAH)
  bq27541_read_block_reg(di,BQ27541_REG_DCAP, 2,RxData);
  dcap = transBytes2UnsignedInt(RxData[1], RxData[0]);
  printk("Design Capacity %d maH\n",dcap);
  // Read Device Name Length
  bq27541_read_block_reg(di,BQ27541_REG_DNAMELEN, 1,RxData);
  dnamelen = RxData[0];
  printk("Device Name Length: %d\n",dnamelen);
  
  // Read Device Name (Rx buffer should end up with ASCII chars for 'bq27541')
  bq27541_read_block_reg(di,BQ27541_REG_DNAME, 7,RxData);
  
   
  // Write & read back 32 bytes of data in Manufacturer Info Block A
  for (i = 0; i < BUFFERSIZE; i++)          
  {
    TxData[i] = i;                          // Initialize data to be written
  }
  bq27541_set_cmd_write(di,BQ27541_REG_DFDCNTL, 0);// BlockDataControl() = 0x00
  printk("DFDCNTL SUCCESSFULLY\n");
  bq27541_set_cmd_write(di,BQ27541_REG_DFCLS, 58);// Write the subclass value
  printk("REG_DFCLS SUCCESSFULLY\n");
  bq27541_set_cmd_write(di,BQ27541_REG_DFBLK, 0);// Select offset within the flash  
  printk("REG_DFBLK SUCCESSFULLY\n");
  for (i = 0; i < BUFFERSIZE; i++)          // Compute the checksum of the block
  {
    sum += TxData[i];                       // Calculate the sum of the values  
  }
  //checksum = (0xFF - (sum & 0x00FF));       // Compute checksum based on the sum
  checksum = (0xFF - 0x41);  
  printk("checksum = %x \n",checksum);
  bq27541_set_cmd_write(di,BQ27541_REG_DFDCKS, checksum); // Write checksum value
  printk("BQ27541_REG_DFDCKS SUCCESSFULLY\n");
  for (i = 0; i < BUFFERSIZE; i++)          // Write 32 bytes to Info Block A
  {
    bq27541_set_cmd_write(di,(BQ27541_REG_ADF+i), TxData[i]);  
  }
  printk("BQ27541_REG_ADF SUCCESSFULLY\n");
  bq27541_read_block_reg(di,BQ27541_REG_ADF, 32,RxData);  // Read the contents of the block
  for (i = 0; i < BUFFERSIZE; i++)          // Check if writes were successful
  {
    if (TxData[i] != RxData[i])             // Tx & Rx data values match?
    {
      printk("Initial Fail");
      goto batt_failed_3;             // Signal error condition occurred
    }
  }

  // TODO: Insert the private 128-bit key that is stored in the bq27541
  // Key[15..8] = K1 (highest 64 bits of the key)
  // Key[ 7..0] = K0 (lowest 64 bits of the key)
  // In this example 0x0123456789ABCDEFFEDCBA9876543210 is used since a fresh
  // unprogrammed bq27541 device has this as its default for the 128-bit key.
  Key[15] = 0x01;
  Key[14] = 0x23;
  Key[13] = 0x45;
  Key[12] = 0x67;
  Key[11] = 0x89;
  Key[10] = 0xAB;
  Key[ 9] = 0xCD;
  Key[ 8] = 0xEF;
  Key[ 7] = 0xFE;
  Key[ 6] = 0xDC;
  Key[ 5] = 0xBA;
  Key[ 4] = 0x98;
  Key[ 3] = 0x76;
  Key[ 2] = 0x54;
  Key[ 1] = 0x32;
  Key[ 0] = 0x10;
  
  // Perform my own SHA1 (Host side)
  read = (UINT8)TI_getRandomIntegerFromVLO();// Use instability of VLO for rand
  srand(read);                              // Plant seed based on random value 
  for (i = 0; i < RANDMESGNUMBYTES; i++)    // Initialize random challenge bytes
  {
    Message[i] = rand();                    // Generate 1 random challenge byte
  }
  SHA1_authenticate();                      // Execute SHA-1/HMAC algorithm
  
  // Authenticate the bq27541
  bq27541_set_cmd_write( BQ27541_REG_DFDCNTL, 1); // BlockDataControl() = 0x01
  // Write block of random challenge to bq27541 (starting at location ADF)
  bytes = 0;
  TxData[bytes++] = BQ27541_REG_ADF;
  for (i = 1; i <= RANDMESGNUMBYTES; i++)
  {
    TxData[bytes++] = Message[i-1];  
  }
  //MSP430_bq27541_blockWrite(TxData, bytes);
  bq27541_write_block_reg(di,0,bytes,TxData);
  // Write checksum for the challenge to the bq27541
  sum = 0;
  for (i = 0; i < RANDMESGNUMBYTES; i++)    // Compute the checksum of the block
  {
    sum += Message[i];                      // Calculate the sum of the values  
  }
  checksum = (0xFF - (sum & 0x00FF));       // Compute checksum based on the sum
  bq27541_set_cmd_write(BQ27541_REG_ACKSDFD, checksum);
  // Read back the digest from the bq27541
  bq27541_read_block_reg(di,BQ27541_REG_ADF, RANDMESGNUMBYTES,RxData);// Read digest contents
  //bq27541_read_block_reg(di,BQ27541_REG_ADF, 32,RxData);  // Read the contents of the block
  // The 20 bytes of the digest returned by the bq27541 is arranged in 32-bit
  // words so that it can be compared with the results computed by the MCU
  Digest_32[4] = (u32)(RxData[ 0])*0x00000001 +
                 (u32)(RxData[ 1])*0x00000100 +
                 (u32)(RxData[ 2])*0x00010000 +
                 (u32)(RxData[ 3])*0x01000000;
  Digest_32[3] = (u32)(RxData[ 4])*0x00000001 +
                 (u32)(RxData[ 5])*0x00000100 +
                 (u32)(RxData[ 6])*0x00010000 +
                 (u32)(RxData[ 7])*0x01000000; 
  Digest_32[2] = (u32)(RxData[ 8])*0x00000001 +
                 (u32)(RxData[ 9])*0x00000100 +
                 (u32)(RxData[10])*0x00010000 +
                 (u32)(RxData[11])*0x01000000;
  Digest_32[1] = (u32)(RxData[12])*0x00000001 +
                 (u32)(RxData[13])*0x00000100 +
                 (u32)(RxData[14])*0x00010000 +
                 (u32)(RxData[15])*0x01000000;
  Digest_32[0] = (u32)(RxData[16])*0x00000001 +
                 (u32)(RxData[17])*0x00000100 +
                 (u32)(RxData[18])*0x00010000 +
                 (u32)(RxData[19])*0x01000000;

  // The results produced by the MCU and bq27541 must match for success
  if ( (Digest_32[0] == H[0]) && (Digest_32[1] == H[1]) &&
       (Digest_32[2] == H[2]) && (Digest_32[3] == H[3]) &&
       (Digest_32[4] == H[4]) )
  {
    // Set P1.0 LED on MSP430 EVM to signal that command sequence was successful
    printk("CAIHAOYUAN BQ27541 INIT successfully");
  }
  else
  {
    goto  batt_failed_3;               // Error condition
  }
*/
        
	return 0;

batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

/*
//*****************************************************************************
//  unsigned long Rotl(unsigned long x, int n)
//							
//  Description : This procedure is a rotate left n spaces of 32-bit word x.
//  Arguments :   x - word to be rotated
//	          n - amount of spaces to rotated to the left								  
//  Returns: Result of 32-bit word rotated n times
//*****************************************************************************
u32 Rotl(u32 x, int n)
{
  return ( (x<<n) | (x>>(32-n)) );
}	

//*****************************************************************************
// unsigned long W(int t)
//
// Description : Determines the work schedule for W(16) through W(79)
// Arguments : t - index of work schedule 16 through 79
// Global Variables : Ws[]
// Returns : Work schedule value with index t
//*****************************************************************************
u32 W(int t)
{
  return (Rotl(Ws[t-3] ^ Ws[t-8] ^ Ws[t-14] ^ Ws[t-16], 1));
}	

//*****************************************************************************
// unsigned long K(int t)
//
// Description : Selects one of the K values depending on the index t
// Arguments : t - index
// Returns : One of the 4 K values
//*****************************************************************************
u32 K(int t)
{
  if (t <= 19)
    return 0x5a827999;
  else if ( (t >= 20) && (t <= 39) )
    return 0x6ed9eba1;
  else if ( (t >= 40) && (t <= 59) )
    return 0x8f1bbcdc;
  else if ( (t >= 60) && (t <= 79) )
    return 0xca62c1d6;
  else
    return 0;		                    // Invalid value, not expected
}
	
//*****************************************************************************
// unsigned long f(unsigned long x, unsigned long y, unsigned long z, int t)
//
// Description : This procedure selects the ft(b,c,d) function based
//               on the SLUA389 and FIPS 180-2 documents
// Arguments : x - b as seen in document
//             y - c as seen in document
//             z - d as seed in document
//             t - index
// Returns : Result of ft function
//*****************************************************************************
u32 f(u32 x, u32 y, u32 z, int t)
{
  if (t <= 19)
    return ( (x & y) ^ ((~x) & z) );
  else if ( (t >= 20) && (t <= 39) )
    return (x ^ y ^ z);
  else if ( (t >= 40) && (t <= 59) )
    return ( (x & y) ^ (x & z) ^ (y & z) );
  else if ( (t >= 60) && (t <= 79) )
    return (x ^ y ^ z);
  else
    return 0;                               // Invalid value, not expected
}	

*****************************************************************************
// void SHA1_authenticate(void)
//
// Description : Computes the SHA1/HMAC as required by the bq26100
// Arguments : i - times that SHA1 is executing
//             t - index 0 through 79
//             temp - Used to update working variables
// Global Variables : Random[], Message[], Key[], Ws[], H[], A, B, C1, D, E
// Returns : Result of 32-bit word rotated n times
//*****************************************************************************
void SHA1_authenticate(void)
{
  int i; // Used for doing two times the SHA1 as required by the bq26100
  int t; // Used for the indexes 0 through 79
  u32 temp; // Used as the temp variable during the loop in which the
               // working variables A, B, C1, D and E are updated

  u32 B;
  u32 C1;
  u32 D;
  u32 E;
  //u32 H[5];
  
	
  // The 20 bytes of random message that are given to the bq26100 are arranged
  // in 32-bit words so that the microcontroller can compute the SHA1/HMAC
  Random[0] = (u32)(Message[16])*0x00000001 +
              (u32)(Message[17])*0x00000100 +
              (u32)(Message[18])*0x00010000 +
              (u32)(Message[19])*0x01000000;
  Random[1] = (u32)(Message[12])*0x00000001 +
              (u32)(Message[13])*0x00000100 +
              (u32)(Message[14])*0x00010000 +
              (u32)(Message[15])*0x01000000;
  Random[2] = (u32)(Message[ 8])*0x00000001 +
              (u32)(Message[ 9])*0x00000100 +
              (u32)(Message[10])*0x00010000 +
              (u32)(Message[11])*0x01000000;
  Random[3] = (u32)(Message[ 4])*0x00000001 +
              (u32)(Message[ 5])*0x00000100 +
              (u32)(Message[ 6])*0x00010000 +
              (u32)(Message[ 7])*0x01000000;
  Random[4] = (u32)(Message[ 0])*0x00000001 +
              (u32)(Message[ 1])*0x00000100 +
              (u32)(Message[ 2])*0x00010000 +
              (u32)(Message[ 3])*0x01000000;
  // The SHA1 is computed two times so that it complies with the bq26100 spec
  for (i = 0; i <= 1; i++)
  {
    // Work Schedule
    // The first four Working schedule variables Ws[0-3], are based on the key
    // that is implied that the bq26100 contains
    Ws[0] = (u32)(Key[12])*0x00000001 +
            (u32)(Key[13])*0x00000100 +
            (u32)(Key[14])*0x00010000 +
            (u32)(Key[15])*0x01000000;
    Ws[1] = (u32)(Key[ 8])*0x00000001 +
            (u32)(Key[ 9])*0x00000100 +
            (u32)(Key[10])*0x00010000 +
            (u32)(Key[11])*0x01000000;
    Ws[2] = (u32)(Key[ 4])*0x00000001 +
            (u32)(Key[ 5])*0x00000100 +
            (u32)(Key[ 6])*0x00010000 +
            (u32)(Key[ 7])*0x01000000;
    Ws[3] = (u32)(Key[ 0])*0x00000001 +
            (u32)(Key[ 1])*0x00000100 +
            (u32)(Key[ 2])*0x00010000 +
            (u32)(Key[ 3])*0x01000000;
    // On the first run of the SHA1 the random message is used 		
    if (i == 0)
    {
      Ws[4] = Random[0];
      Ws[5] = Random[1];
      Ws[6] = Random[2];
      Ws[7] = Random[3];
      Ws[8] = Random[4];
    }
    // On the second run of the SHA1, H(Kd || M) is used		
    else
    {
      Ws[4] = H[0];
      Ws[5] = H[1];
      Ws[6] = H[2];
      Ws[7] = H[3];
      Ws[8] = H[4];
    }
    // The Work schedule variables Ws[9-15] remain the same regardless of 
    // which run of the SHA1.  These values are as required by bq26100.
    Ws[9]  = 0x80000000;
    Ws[10] = 0x00000000;
    Ws[11] = 0x00000000;
    Ws[12] = 0x00000000;
    Ws[13] = 0x00000000;
    Ws[14] = 0x00000000;
    Ws[15] = 0x00000120;

    // The Work schedule variables Ws[16-79] are determined by the W(t) func
    for (t = 16; t <= 79; t++)
      Ws[t] = W(t);
    // Working Variables, always start the same regardless of which SHA1 run
    A  = 0x67452301;
    B  = 0xefcdab89;
    C1 = 0x98badcfe;
    D  = 0x10325476;
    E  = 0xc3d2e1f0;
    // Hash reads, always start the same regardless of what SHA1 run
    H[0] = A;
    H[1] = B;
    H[2] = C1;
    H[3] = D;
    H[4] = E;
    // Loop to change working variables A, B, C1, D and E
    // This is defined by FIPS 180-2 document
    for (t = 0; t <= 79; t++)
    {
      temp = Rotl(A,5) + f(B,C1,D,t) + E + K(t) + Ws[t];
      E = D;
      D = C1;
      C1 = Rotl(B,30);
      B = A;
      A = temp;
    }
    // 160-Bit SHA-1 Digest
    H[0] = (A  + H[0]);
    H[1] = (B  + H[1]);
    H[2] = (C1 + H[2]);
    H[3] = (D  + H[3]);
    H[4] = (E  + H[4]);
  }
}
*/
/**
*remove battery device
*
*
*
*
*
********/
static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	bq27x00_powersupply_unregister(di);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}
/**
* device id table match the id in BSP(i2c struct)
*
***/
static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27200", BQ27000 },	/* bq27200 is same as bq27000, but with i2c */
	{ "bq27541", BQ27541 },
	{}
};
MODULE_DEVICE_TABLE(i2c, bq27x00_id);

static struct i2c_driver bq27x00_battery_driver = {
	.driver = {
		.name = "bq27541",
	},
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.id_table = bq27x00_id,
};

static inline int bq27x00_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27x00_battery_driver);
	printk("enter bq27541 caihaoyuan");
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 i2c driver\n");

	return ret;
}

static inline void bq27x00_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}

#else

static inline int bq27x00_battery_i2c_init(void) { return 0; }
static inline void bq27x00_battery_i2c_exit(void) {};

#endif

/* platform specific code */
#ifdef CONFIG_BATTERY_BQ27X00_PLATFORM

static int bq27000_read_platform(struct bq27x00_device_info *di, u8 reg,
			bool single)
{
	struct device *dev = di->dev;
	struct bq27000_platform_data *pdata = dev->platform_data;
	unsigned int timeout = 3;
	int upper, lower;
	int temp;

	if (!single) {
		/* Make sure the value has not changed in between reading the
		 * lower and the upper part */
		upper = pdata->read(dev, reg + 1);
		do {
			temp = upper;
			if (upper < 0)
				return upper;

			lower = pdata->read(dev, reg);
			if (lower < 0)
				return lower;

			upper = pdata->read(dev, reg + 1);
		} while (temp != upper && --timeout);

		if (timeout == 0)
			return -EIO;

		return (upper << 8) | lower;
	}

	return pdata->read(dev, reg);
}

static int __devinit bq27000_battery_probe(struct platform_device *pdev)
{
	struct bq27x00_device_info *di;
	struct bq27000_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data supplied\n");
		return -EINVAL;
	}

	if (!pdata->read) {
		dev_err(&pdev->dev, "no hdq read callback supplied\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->chip = BQ27541;

	di->bat.name = pdata->name ?: dev_name(&pdev->dev);
	di->bus.read = &bq27000_read_platform;

	ret = bq27x00_powersupply_init(di);
	if (ret)
		goto err_free;

	return 0;

err_free:
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return ret;
}

static int __devexit bq27000_battery_remove(struct platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	bq27x00_powersupply_unregister(di);

	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

static struct platform_driver bq27000_battery_driver = {
	.probe	= bq27000_battery_probe,
	.remove = __devexit_p(bq27000_battery_remove),
	.driver = {
		.name = "bq27000-battery",
		.owner = THIS_MODULE,
	},
};

static inline int bq27x00_battery_platform_init(void)
{
	int ret = platform_driver_register(&bq27000_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27000 platform driver\n");

	return ret;
}

static inline void bq27x00_battery_platform_exit(void)
{
	platform_driver_unregister(&bq27000_battery_driver);
}

#else

static inline int bq27x00_battery_platform_init(void) { return 0; }
static inline void bq27x00_battery_platform_exit(void) {};

#endif

/*
 * Module stuff
 */

static int __init bq27x00_battery_init(void)
{
	int ret;

	ret = bq27x00_battery_i2c_init();
	if (ret)
		return ret;

	ret = bq27x00_battery_platform_init();
	if (ret)
		bq27x00_battery_i2c_exit();

	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	bq27x00_battery_platform_exit();
	bq27x00_battery_i2c_exit();
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
