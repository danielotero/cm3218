/*
 * Copyright (C) 2014 Capella Microsystems Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2, as published
 * by the Free Software Foundation.
 *
 * Special thanks Srinivas Pandruvada <srinivas.psndruvada@linux.intel.com>
 * help to add ACPI support.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/init.h>
#include <linux/acpi.h>

/* I2C Address */
#define CM3218_I2C_ADDR_ARA		0x0C

/* Registers Address */
#define CM3218_REG_ADDR_CMD		0x00
#define	CM3218_REG_ADDR_WH		0x01
#define	CM3218_REG_ADDR_WL		0x02
#define CM3218_REG_ADDR_ALS		0x04
#define CM3218_REG_ADDR_STATUS		0x06
#define CM3218_REG_ADDR_ID		0x07

/* Number of Configurable Registers */
#define CM3218_CONF_REG_NUM		0x0F

/* CMD register */
#define CM3218_CMD_ALS_DISABLE		BIT(0)
#define CM3218_CMD_ALS_INT_EN		BIT(1)
#define	CM3218_CMD_ALS_THRES_WINDOW	BIT(2)

#define	CM3218_CMD_ALS_PERS_SHIFT	4
#define	CM3218_CMD_ALS_PERS_MASK	(BIT(4) | BIT(5))
#define	CM3218_CMD_ALS_PERS_DEFAULT	(0x01 << CM3218_CMD_ALS_PERS_SHIFT)

#define CM3218_CMD_ALS_IT_SHIFT		6
#define CM3218_CMD_ALS_IT_MASK		(BIT(6) | BIT(7))
#define CM3218_CMD_ALS_IT_DEFAULT	(0x01 << CM3218_CMD_ALS_IT_SHIFT)

#define CM3218_CMD_ALS_HS		BIT(11)

#define	CM3218_WH_DEFAULT		0xFFFF
#define	CM3218_WL_DEFAULT		0x0000
#define CM3218_MLUX_PER_BIT_DEFAULT	1000	/* depend on resistor */
#define CM3218_MLUX_PER_BIT_BASE_IT	200000
#define	CM3218_CALIBSCALE_DEFAULT	100000
#define CM3218_CALIBSCALE_RESOLUTION	100000
#define MLUX_PER_LUX			1000
#define	CM3218_THRESHOLD_PERCENT	5	/* 5 percent */

static const u8 cm3218_reg[CM3218_CONF_REG_NUM] = {
	CM3218_REG_ADDR_CMD,
};

static const struct {
	int val;
	int val2;
	u8 it;
} cm3218_als_it_scales[] = {
	{0, 100000, 0},	/* 0.100000 */
	{0, 200000, 1},	/* 0.200000 */
	{0, 400000, 2},	/* 0.400000 */
	{0, 800000, 3},	/* 0.800000 */
};

struct cm3218_chip {
	struct i2c_client *client;
	struct i2c_client *ara_client;
	struct i2c_client *als_client;
	struct mutex lock;
	u16 conf_regs[CM3218_CONF_REG_NUM];
	int als_raw;
	int calibscale;
	int mlux_per_bit;
	int sensitivity_percent;
};

/**
 * cm3218_read_ara() - Read ARA register
 * @cm3218:	pointer of struct cm3218.
 *
 * Following SMBus protocol, ARA register is available only when interrupt
 * event happened.  Read it to clean interrupt event.  Otherwise, other
 * device address/registers will be blocked during interrupt event.
 *
 * Return: 0 for success; otherwise for error code.
 */
static int cm3218_read_ara(struct cm3218_chip *chip)
{
	int status;

	status = i2c_smbus_read_byte(chip->ara_client);

	if (status < 0)
		return -ENODEV;

	return status;
}

/**
 * cm3218_interrupt_config() - Enable/Disable CM3218 interrupt
 * @cm3218:	pointer of struct cm3218.
 * @enable:	0 to disable; otherwise to enable
 *
 * Config CM3218 interrupt control bit.
 *
 * Return: 0 for success; otherwise for error code.
 */
static int cm3218_interrupt_config(struct cm3218_chip *chip, int enable)
{
	int status;

	/* Force to clean interrupt */
	cm3218_read_ara(chip);

	if (enable)
		chip->conf_regs[CM3218_REG_ADDR_CMD] |=
			CM3218_CMD_ALS_INT_EN;
	else
		chip->conf_regs[CM3218_REG_ADDR_CMD] &=
			~CM3218_CMD_ALS_INT_EN;

	status = i2c_smbus_write_word_data(
			chip->als_client,
			CM3218_REG_ADDR_CMD,
			chip->conf_regs[CM3218_REG_ADDR_CMD]);

	if (status < 0)
		return -ENODEV;

	return status;
}

/**
 * cm3218_acpi_get_cpm_info() - Get CPM object from ACPI
 * @client	pointer of struct i2c_client.
 * @obj_name	pointer of ACPI object name.
 * @count	maximum size of return array.
 * @vals	pointer of array for return elements.
 *
 * Convert ACPI CPM table to array. Special thanks to Srinivas Pandruvada
 * for his help implementing this routine.
 *
 * Return: -ENODEV for fail.  Otherwise is number of elements.
 */
static int cm3218_acpi_get_cpm_info(struct i2c_client *client, char *obj_name,
							int count, u64 *vals)
{
	acpi_handle handle;
	struct acpi_buffer buffer = {ACPI_ALLOCATE_BUFFER, NULL};
	int i;
	acpi_status status;
	union acpi_object *cpm = NULL;

	handle = ACPI_HANDLE(&client->dev);
	if (!handle)
		return -ENODEV;

	status = acpi_evaluate_object(handle, obj_name, NULL, &buffer);
	if (ACPI_FAILURE(status)) {
		dev_err(&client->dev, "object %s not found\n", obj_name);
		return -ENODEV;
	}

	cpm = buffer.pointer;
	for (i = 0; i < cpm->package.count && i < count; ++i) {
		union acpi_object *elem;

		elem = &(cpm->package.elements[i]);
		vals[i] = elem->integer.value;
	}

	kfree(buffer.pointer);

	return i;
}

/**
 * cm3218_reg_init() - Initialize CM3218 registers
 * @cm3218:	pointer of struct cm3218.
 *
 * Initialize CM3218 ambient light sensor register to default values.
 *
  Return: 0 for success; otherwise for error code.
 */
static int cm3218_reg_init(struct cm3218_chip *chip)
{
	struct i2c_client *client = chip->client;
	int i;
	s32 ret;
	int cpm_elem_count;
	u64 cpm_elems[20];

	/* Disable interrupt */
	cm3218_interrupt_config(chip, 0);

	/* Disable device */
	i2c_smbus_write_word_data( chip->als_client,
			CM3218_REG_ADDR_CMD, CM3218_CMD_ALS_DISABLE);

	ret = i2c_smbus_read_word_data(
			chip->als_client,
			CM3218_REG_ADDR_ID);
	if (ret < 0)
		return ret;

	/* check device ID */
	if ((ret & 0xFF) != 0x18)
		return -ENODEV;

	/* Default Values */
	chip->conf_regs[CM3218_REG_ADDR_CMD] =
			CM3218_CMD_ALS_THRES_WINDOW |
			CM3218_CMD_ALS_PERS_DEFAULT |
			CM3218_CMD_ALS_IT_DEFAULT |
			CM3218_CMD_ALS_HS;
	chip->conf_regs[CM3218_REG_ADDR_WH] = CM3218_WH_DEFAULT;
	chip->conf_regs[CM3218_REG_ADDR_WL] = CM3218_WL_DEFAULT;
	chip->calibscale = CM3218_CALIBSCALE_DEFAULT;
	chip->mlux_per_bit = CM3218_MLUX_PER_BIT_DEFAULT;
	chip->sensitivity_percent = CM3218_THRESHOLD_PERCENT;

	if (ACPI_HANDLE(&client->dev)) {
		/* Load from ACPI */
		cpm_elem_count = cm3218_acpi_get_cpm_info(client, "CPM0",
							ARRAY_SIZE(cpm_elems),
							cpm_elems);
		if (cpm_elem_count > 0) {
			int header_num = 3;
			int reg_num = cpm_elem_count - header_num;
			int reg_bmp = cpm_elems[2];

			for (i = 0; i < reg_num; i++)
				if (reg_bmp & (1 << i))
					chip->conf_regs[i] =
						cpm_elems[header_num + i];
		}

		cpm_elem_count = cm3218_acpi_get_cpm_info(client, "CPM1",
							ARRAY_SIZE(cpm_elems),
							cpm_elems);
		if (cpm_elem_count > 0) {
			chip->mlux_per_bit = (int)cpm_elems[0] / 100;
			chip->calibscale = (int)cpm_elems[1];
		}

		cpm_elem_count = cm3218_acpi_get_cpm_info(client, "CPM5",
							ARRAY_SIZE(cpm_elems),
							cpm_elems);
		if (cpm_elem_count >= 7)
			chip->sensitivity_percent = (int)cpm_elems[6];

	}

	/* Force to disable interrupt */
	chip->conf_regs[CM3218_REG_ADDR_CMD] &= ~CM3218_CMD_ALS_INT_EN;

	/* Initialize registers*/
	for (i = 0; i < CM3218_CONF_REG_NUM; i++) {
		ret = i2c_smbus_write_word_data(
				chip->als_client,
				cm3218_reg[i],
				chip->conf_regs[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/**
 *  cm3218_read_als_it() - Get sensor integration time (ms)
 *  @cm3218:	pointer of struct cm3218
 *  @val:	pointer of int to load the integration time (sec).
 *  @val2:	pointer of int to load the integration time (microsecond).
 *
 *  Report the current integartion time by millisecond.
 *
 *  Return: IIO_VAL_INT_PLUS_MICRO for success, otherwise -EINVAL.
 */
static int cm3218_read_als_it(struct cm3218_chip *chip, int *val, int *val2)
{
	u16 als_it;
	int i;

	als_it = chip->conf_regs[CM3218_REG_ADDR_CMD];
	als_it &= CM3218_CMD_ALS_IT_MASK;
	als_it >>= CM3218_CMD_ALS_IT_SHIFT;
	for (i = 0; i < ARRAY_SIZE(cm3218_als_it_scales); i++) {
		if (als_it == cm3218_als_it_scales[i].it) {
			*val = cm3218_als_it_scales[i].val;
			*val2 = cm3218_als_it_scales[i].val2;
			return IIO_VAL_INT_PLUS_MICRO;
		}
	}

	return -EINVAL;
}

/**
 * cm3218_write_als_it() - Write sensor integration time
 * @cm3218:	pointer of struct cm3218.
 * @val:	integration time in second.
 * @val2:	integration time in millisecond.
 *
 * Convert integration time to sensor value.
 *
 * Return: i2c_smbus_write_word_data command return value.
 */
static int cm3218_write_als_it(struct cm3218_chip *chip, int val, int val2)
{
	u16 als_it, cmd;
	int i;
	s32 ret;

	for (i = 0; i < ARRAY_SIZE(cm3218_als_it_scales); i++) {
		if (val == cm3218_als_it_scales[i].val &&
			val2 == cm3218_als_it_scales[i].val2) {
			als_it = cm3218_als_it_scales[i].it;
			als_it <<= CM3218_CMD_ALS_IT_SHIFT;

			cmd = chip->conf_regs[CM3218_REG_ADDR_CMD] &
				~CM3218_CMD_ALS_IT_MASK;
			cmd |= als_it;
			ret = i2c_smbus_write_word_data(
						chip->als_client,
						CM3218_REG_ADDR_CMD,
						cmd);
			if (ret < 0)
				return ret;
			chip->conf_regs[CM3218_REG_ADDR_CMD] = cmd;
			return 0;
		}
	}
	return -EINVAL;
}

/**
 * cm3218_get_lux() - report current lux value
 * @cm3218:	pointer of struct cm3218.
 *
 * Convert sensor raw data to lux.  It depends on integration
 * time and calibscale variable.
 *
 * Return: Positive value is lux, otherwise is error code.
 */
static int cm3218_get_lux(struct cm3218_chip *chip)
{
	int ret;
	int als_it;
	int val, val2;
	unsigned long lux;

	ret = cm3218_read_als_it(chip, &val, &val2);
	als_it = val*1000000 + val2;
	if (ret < 0)
		return -EINVAL;

	lux = chip->mlux_per_bit;
	lux *= CM3218_MLUX_PER_BIT_BASE_IT;
	lux /= als_it;

	ret = i2c_smbus_read_word_data(
				chip->als_client,
				CM3218_REG_ADDR_ALS);

	if (ret < 0)
		return ret;

	chip->als_raw = ret;
	lux *= chip->als_raw;
	lux *= chip->calibscale;
	if (!(chip->conf_regs[CM3218_REG_ADDR_CMD] & CM3218_CMD_ALS_HS))
		lux *= 2;
	lux /= CM3218_CALIBSCALE_RESOLUTION;
	lux /= MLUX_PER_LUX;

	if (lux > 0xFFFF)
		lux = 0xFFFF;

	return lux;
}

static int cm3218_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct cm3218_chip *chip = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = cm3218_get_lux(chip);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val = chip->calibscale;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_INT_TIME:
		*val = 0;
		ret = cm3218_read_als_it(chip, val, val2);
		return ret;
	case IIO_CHAN_INFO_RAW:
		ret = i2c_smbus_read_word_data(chip->als_client,
					CM3218_REG_ADDR_ALS);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int cm3218_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct cm3218_chip *chip = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		chip->calibscale = val;
		return val;
	case IIO_CHAN_INFO_INT_TIME:
		ret = cm3218_write_als_it(chip, val, val2);
		return ret;
	}

	return -EINVAL;
}

/**
 * cm3218_get_it_available() - Get available ALS IT value
 * @dev:	pointer of struct device.
 * @attr:	pointer of struct device_attribute.
 * @buf:	pointer of return string buffer.
 *
 * Display the available integration time values by millisecond.
 *
 * Return: string length.
 */
static ssize_t cm3218_get_it_available(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i, len;

	for (i = 0, len = 0; i < ARRAY_SIZE(cm3218_als_it_scales); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u.%06u ",
			cm3218_als_it_scales[i].val,
			cm3218_als_it_scales[i].val2);
	return len + scnprintf(buf + len, PAGE_SIZE - len, "\n");
}

static int cm3218_threshold_update(struct cm3218_chip *chip, int percent)
{
	int ret;
	int wh, wl;

	ret = i2c_smbus_read_word_data(
		chip->als_client,
		CM3218_REG_ADDR_ALS);
	if (ret < 0)
		return ret;

	chip->als_raw = ret;

	wh = wl = ret;
	ret *= percent;
	ret /= 100;
	if (ret < 1)
		ret = 1;
	wh += ret;
	wl -= ret;
	if (wh > 65535)
		wh = 65535;
	if (wl < 0)
		wl = 0;

	chip->conf_regs[CM3218_REG_ADDR_WH] = wh;
	ret = i2c_smbus_write_word_data(
		chip->als_client,
		CM3218_REG_ADDR_WH,
		chip->conf_regs[CM3218_REG_ADDR_WH]);
	if (ret < 0)
		return ret;

	chip->conf_regs[CM3218_REG_ADDR_WL] = wl;
	ret = i2c_smbus_write_word_data(
		chip->als_client,
		CM3218_REG_ADDR_WL,
		chip->conf_regs[CM3218_REG_ADDR_WL]);
	if (ret < 0)
		return ret;

	return 0;
}

static irqreturn_t cm3218_event_handler(int irq, void *private)
{
	struct iio_dev *dev_info = private;
	struct cm3218_chip *chip = iio_priv(dev_info);
	int ret;

	if (!chip)
		return IRQ_NONE;

	mutex_lock(&chip->lock);

	/* Clear interrupt */
	ret = cm3218_read_ara(chip);

	/* Disable interrupt */
	ret = cm3218_interrupt_config(chip, 0);
	if (ret < 0)
		goto error_handler_unlock;

	/* Update Hi/Lo windows */
	ret = cm3218_threshold_update(chip, chip->sensitivity_percent);
	if (ret < 0)
		goto error_handler_unlock;

	/* Enable interrupt */
	ret = cm3218_interrupt_config(chip, 1);
	if (ret < 0)
		goto error_handler_unlock;

	mutex_unlock(&chip->lock);
	return IRQ_HANDLED;

error_handler_unlock:
	mutex_unlock(&chip->lock);
	return IRQ_NONE;
}

static int acpi_i2c_check_resource(struct acpi_resource *ares, void *data)
{
	u32 *addr = data;

	if (ares->type == ACPI_RESOURCE_TYPE_SERIAL_BUS) {
		struct acpi_resource_i2c_serialbus *sb;

		sb = &ares->data.i2c_serial_bus;
		if (sb->type == ACPI_RESOURCE_SERIAL_TYPE_I2C) {
			if (*addr)
				*addr |= (sb->slave_address << 16);
			else
				*addr = sb->slave_address;
		}
	}

	/* Tell the ACPI core that we already copied this address */
	return 1;
}

static int cm3218_acpi_config(struct i2c_client *client,
			       unsigned short *primary_addr,
			       unsigned short *secondary_addr)
{
	const struct acpi_device_id *id;
	struct acpi_device *adev;
	u32 i2c_addr = 0;
	LIST_HEAD(resources);
	int ret;

	id = acpi_match_device(client->dev.driver->acpi_match_table,
			       &client->dev);
	if (!id)
		return -ENODEV;

	adev = ACPI_COMPANION(&client->dev);
	if (!adev)
		return -ENODEV;

	ret = acpi_dev_get_resources(adev, &resources,
				     acpi_i2c_check_resource, &i2c_addr);
	if (ret < 0)
		return ret;

	acpi_dev_free_resource_list(&resources);
	*primary_addr = i2c_addr & 0x0000ffff;
	*secondary_addr = (i2c_addr & 0xffff0000) >> 16;

	if (*primary_addr == CM3218_I2C_ADDR_ARA) {
		*primary_addr = *secondary_addr;
		*secondary_addr = CM3218_I2C_ADDR_ARA;
	}

	return 0;
}


static const struct iio_chan_spec cm3218_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_CALIBSCALE) |
			BIT(IIO_CHAN_INFO_INT_TIME),
	}
};

static IIO_DEVICE_ATTR(in_illuminance_integration_time_available,
			S_IRUGO, cm3218_get_it_available, NULL, 0);

static struct attribute *cm3218_attributes[] = {
	&iio_dev_attr_in_illuminance_integration_time_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group cm3218_attribute_group = {
	.attrs = cm3218_attributes
};

static const struct iio_info cm3218_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= &cm3218_read_raw,
	.write_raw		= &cm3218_write_raw,
	.attrs			= &cm3218_attribute_group,
};

static int cm3218_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct cm3218_chip *chip;
	struct iio_dev *indio_dev;
	int ret;
	unsigned short primary_addr, secondary_addr;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (!indio_dev) {
		dev_err(&client->dev, "devm_iio_device_alloc failed\n");
		return -ENOMEM;
	}

	chip = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	chip->client = client;

	primary_addr = client->addr;
	secondary_addr = CM3218_I2C_ADDR_ARA;
	cm3218_acpi_config(client, &primary_addr, &secondary_addr);

	if (client->addr == primary_addr)
		chip->als_client = client;
	else
		chip->als_client = i2c_new_dummy(client->adapter,
						primary_addr);
	if (!chip->als_client) {
		dev_err(&client->dev, "%s: als_client failed\n", __func__);
		return -ENODEV;
	}

	if (client->addr == secondary_addr)
		chip->ara_client = client;
	else
		chip->ara_client = i2c_new_dummy(client->adapter,
						secondary_addr);
	if (!chip->ara_client) {
		dev_err(&client->dev, "%s: ara_client failed\n", __func__);
		return -ENODEV;
	}

	mutex_init(&chip->lock);
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = cm3218_channels;
	indio_dev->num_channels = ARRAY_SIZE(cm3218_channels);
	indio_dev->info = &cm3218_info;
	if (id && id->name)
		indio_dev->name = id->name;
	else
		indio_dev->name = (char *)dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = cm3218_reg_init(chip);
	if (ret) {
		dev_err(&client->dev,
			"%s: register init failed\n",
			__func__);
		return ret;
	}

	if (client->irq) {
		ret = request_threaded_irq(client->irq,
					NULL,
					cm3218_event_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"cm3218_event",
					indio_dev);

		if (ret < 0) {
			dev_err(&client->dev, "irq request error %d\n",
				-ret);
			goto error_disable_int;
		}

		ret = cm3218_threshold_update(chip,
					chip->sensitivity_percent);
		if (ret < 0)
			goto error_free_irq;

		ret = cm3218_interrupt_config(chip, 1);
		if (ret < 0)
			goto error_free_irq;
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s: regist device failed\n",
			__func__);
		goto error_free_irq;
	}

	return 0;

error_free_irq:
	if (client->irq)
		free_irq(client->irq, indio_dev);
error_disable_int:
	cm3218_interrupt_config(chip, 0);
	return ret;
}

static int cm3218_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm3218_chip *chip = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	cm3218_interrupt_config(chip, 0);
	if (client->irq)
		free_irq(client->irq, indio_dev);
	if (client != chip->ara_client)
		i2c_unregister_device(chip->ara_client);
	if (client != chip->als_client)
		i2c_unregister_device(chip->als_client);

	return 0;
}

static const struct i2c_device_id cm3218_id[] = {
	{ "cm3218", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, cm3218_id);

#ifdef CONFIG_OF
static const struct of_device_id cm3218_of_match[] = {
	{ .compatible = "capella,cm3218" },
	{ }
};
#endif

#if CONFIG_ACPI
static const struct acpi_device_id cm3218_acpi_match[] = {
	{ "CPLM3218", 0},
	{},
};
#endif

MODULE_DEVICE_TABLE(acpi, cm3218_acpi_match);

static struct i2c_driver cm3218_driver = {
	.driver = {
		.name	= "cm3218",
		.acpi_match_table = ACPI_PTR(cm3218_acpi_match),
		.of_match_table = of_match_ptr(cm3218_of_match),
		.owner	= THIS_MODULE,
	},
	.id_table	= cm3218_id,
	.probe		= cm3218_probe,
	.remove		= cm3218_remove,
};

module_i2c_driver(cm3218_driver);

MODULE_AUTHOR("Kevin Tsai <ktsai@capellamicro.com>");
MODULE_DESCRIPTION("CM3218 ambient light sensor driver");
MODULE_LICENSE("GPL");
