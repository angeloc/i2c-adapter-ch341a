/*
 * driver for the ch341a usb to i2c chip
 * 
 * Datasheet: 
 *
 * Copyright (C) 2016 Angelo Compagnucci <angelo.compagnucci@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 */
 
#define DEBUG 1

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/string.h>

#define TIMEOUT 500

#define CH341A_CONTROL_I2C        0xAA

#define CH341A_I2C_CMD_STA        0x74
#define CH341A_I2C_CMD_STO        0x75
#define CH341A_I2C_CMD_OUT        0x80
#define CH341A_I2C_CMD_IN         0xC0
#define CH341A_I2C_CMD_MAX_LENGTH 32
#define CH341A_I2C_CMD_SET        0x60
#define CH341A_I2C_CMD_US         0x40
#define CH341A_I2C_CMD_MS         0x50
#define CH341A_I2C_CMD_DLY        0x0f
#define CH341A_I2C_CMD_END        0x00

struct i2c_ch341a {
	struct usb_device *usb_dev;
	struct usb_interface *interface;
	struct i2c_adapter adapter;
	int ep_in, ep_out;
};

static int i2c_ch341a_usb_i2c_command(struct i2c_adapter *adapter, u8 *cmd, u8 len)
{
	struct i2c_ch341a *dev = (struct i2c_ch341a *)adapter->algo_data;
	int sent, ret;

	print_hex_dump_bytes(__func__, DUMP_PREFIX_OFFSET, cmd, len);

	ret = usb_bulk_msg(dev->usb_dev,
		usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
		cmd, len, &sent, TIMEOUT);
	if (ret) return ret;

	return 0;
}

static int i2c_ch341a_usb_i2c_start(struct i2c_adapter *adapter)
{
	u8 I2C_CMD_START[] = {CH341A_CONTROL_I2C, CH341A_I2C_CMD_STA, CH341A_I2C_CMD_END};
	return i2c_ch341a_usb_i2c_command(adapter, I2C_CMD_START, sizeof(I2C_CMD_START));
}

static int i2c_ch341a_usb_i2c_stop(struct i2c_adapter *adapter)
{
	u8 I2C_CMD_STOP[] = {CH341A_CONTROL_I2C, CH341A_I2C_CMD_STO, CH341A_I2C_CMD_END};
	return i2c_ch341a_usb_i2c_command(adapter, I2C_CMD_STOP, sizeof(I2C_CMD_STOP));
}

static int i2c_ch341a_usb_i2c_write(struct i2c_adapter *adapter, u8 data)
{
	struct i2c_ch341a *dev = (struct i2c_ch341a *)adapter->algo_data;
	u8 I2C_CMD_WRITE_BYTE[] = {CH341A_CONTROL_I2C, CH341A_I2C_CMD_OUT, 0, CH341A_I2C_CMD_END};
	u8 resp[CH341A_I2C_CMD_MAX_LENGTH];
	int ret, actual;

	I2C_CMD_WRITE_BYTE[2] = data;

	print_hex_dump_bytes(__func__, DUMP_PREFIX_OFFSET, I2C_CMD_WRITE_BYTE, sizeof(I2C_CMD_WRITE_BYTE));

	ret = usb_bulk_msg(dev->usb_dev, 
			usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			I2C_CMD_WRITE_BYTE, sizeof(I2C_CMD_WRITE_BYTE), &actual, TIMEOUT);
	if (ret) return ret;

	ret = usb_bulk_msg(dev->usb_dev, 
			usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
			&resp, CH341A_I2C_CMD_MAX_LENGTH, &actual, TIMEOUT);
	if (ret) return ret;

	if (resp[0] & 0x80) return -EPROTO;
	return 0;
}

static int i2c_ch341a_usb_write_byte(struct i2c_adapter *adapter, u16 addr, u16 len, u8 *data)
{
	int ret;

	print_hex_dump_bytes(__func__, DUMP_PREFIX_OFFSET, data, len);

	ret = i2c_ch341a_usb_i2c_start(adapter);
	if (ret) return ret;
	ret = i2c_ch341a_usb_i2c_write(adapter, addr);
	if (ret) return ret;
	ret = i2c_ch341a_usb_i2c_write(adapter, data[0]);
	if (ret) return ret;
	if (len == 2)
		ret = i2c_ch341a_usb_i2c_write(adapter, data[1]);
	if (ret) return ret;
	ret = i2c_ch341a_usb_i2c_stop(adapter);
	usleep_range(1500, 2000);

	return ret;
}

static int i2c_ch341a_usb_i2c_read(struct i2c_adapter *adapter, u16 len, u8 *data)
{
	struct i2c_ch341a *dev = (struct i2c_ch341a *)adapter->algo_data;
	u8 I2C_CMD_READ_BYTE[] = {CH341A_CONTROL_I2C, 0, CH341A_I2C_CMD_END};
	u8 resp[CH341A_I2C_CMD_MAX_LENGTH];
	int ret, actual;

	I2C_CMD_READ_BYTE[1] = CH341A_I2C_CMD_IN | (u8) len;

	print_hex_dump_bytes(__func__, DUMP_PREFIX_OFFSET, I2C_CMD_READ_BYTE, sizeof(I2C_CMD_READ_BYTE));

	ret = usb_bulk_msg(dev->usb_dev, 
			usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			I2C_CMD_READ_BYTE, sizeof(I2C_CMD_READ_BYTE), &actual, TIMEOUT);
	if (ret) return ret;

	ret = usb_bulk_msg(dev->usb_dev, 
			usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
			&resp, CH341A_I2C_CMD_MAX_LENGTH, &actual, TIMEOUT);
	if (ret) return ret;

	memcpy(data, resp, len);

	print_hex_dump_bytes(__func__, DUMP_PREFIX_OFFSET, data, len);

	return 0;
}

static int i2c_ch341a_usb_i2c_read_bytes(struct i2c_adapter *adapter, u16 addr, u16 len, u8 *data)
{
	int ret;
	int buf[1] = {0};

	print_hex_dump_bytes(__func__, DUMP_PREFIX_OFFSET, data, len);

	ret = i2c_ch341a_usb_i2c_start(adapter);
	if (ret) return ret;
	ret = i2c_ch341a_usb_i2c_write(adapter, addr);
	if (ret) return ret;
	ret = i2c_ch341a_usb_i2c_read(adapter, len, data);
	if (ret) return ret;
	ret = i2c_ch341a_usb_i2c_stop(adapter);
	if (ret) return ret;
	ret = i2c_ch341a_usb_write_byte(adapter, addr, 1, buf);

	return 0;
}

static int i2c_ch341a_usb_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct i2c_msg *pmsg;
	int i;

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		dev_dbg(&adapter->dev, "%s: (addr=%x, flags=%x, len=%d)", __func__,
			pmsg->addr, pmsg->flags, pmsg->len);
		print_hex_dump_bytes(__func__, DUMP_PREFIX_OFFSET, pmsg->buf, pmsg->len);
		if (pmsg->flags & I2C_M_RD) {
			if (i2c_ch341a_usb_i2c_read_bytes(adapter,
				(pmsg->addr<<1) + 1, 
				pmsg->len, pmsg->buf) != 0)
					return -EREMOTEIO;
		} else {
			if (i2c_ch341a_usb_write_byte(adapter,
				(pmsg->addr<<1), pmsg->len, pmsg->buf) != 0)
					return -EREMOTEIO;
		}
	}
	return i;
}

static u32 i2c_ch341a_usb_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm usb_algorithm = {
	.master_xfer	= i2c_ch341a_usb_xfer,
	.functionality	= i2c_ch341a_usb_func,
};

static const struct usb_device_id i2c_ch341a_table[] = {
	{ USB_DEVICE(0x1a86, 0x5512) },
	{ }
};

MODULE_DEVICE_TABLE(usb, i2c_ch341a_table);

static void i2c_ch341a_free(struct i2c_ch341a *dev)
{
	usb_put_dev(dev->usb_dev);
	kfree(dev);
}

static int i2c_ch341a_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct usb_host_interface *hostif = interface->cur_altsetting;
	struct i2c_ch341a *dev;
	int ret = -ENOMEM;

	dev_dbg(&interface->dev, "probing usb device\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&interface->dev, "Out of memory\n");
		goto error;
	}

	dev->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	dev->ep_out = hostif->endpoint[1].desc.bEndpointAddress;
	dev->ep_in = hostif->endpoint[0].desc.bEndpointAddress;

	usb_set_intfdata(interface, dev);

	dev->adapter.owner = THIS_MODULE;
	dev->adapter.class = I2C_CLASS_HWMON;
	dev->adapter.algo = &usb_algorithm;
	dev->adapter.algo_data = dev;
	i2c_set_adapdata(&dev->adapter, dev);
	snprintf(dev->adapter.name, sizeof(dev->adapter.name),
		 "i2c-ch341a at bus %03d device %03d",
		 dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

	dev->adapter.dev.parent = &dev->interface->dev;

	ret = i2c_add_adapter(&dev->adapter);
	if (ret < 0) {
		goto error;
	}

	dev_info(&dev->adapter.dev, "connected i2c-ch341a device\n");

	return 0;

 error:
	if (dev)
		usb_set_intfdata(interface, NULL);
		i2c_ch341a_free(dev);

	return ret;
}

static void i2c_ch341a_disconnect(struct usb_interface *interface)
{
	struct i2c_ch341a *dev = usb_get_intfdata(interface);

	i2c_del_adapter(&dev->adapter);
	usb_set_intfdata(interface, NULL);
	i2c_ch341a_free(dev);

	dev_dbg(&interface->dev, "disconnected\n");
}

static struct usb_driver i2c_ch341a_driver = {
	.name		= "i2c-ch341a",
	.probe		= i2c_ch341a_probe,
	.disconnect	= i2c_ch341a_disconnect,
	.id_table	= i2c_ch341a_table,
};

module_usb_driver(i2c_ch341a_driver);

MODULE_AUTHOR("Angelo Compagnucci <angelo.compagnucci@gmail.com>");
MODULE_DESCRIPTION("i2c-ch341a driver");
MODULE_LICENSE("GPL v2");
