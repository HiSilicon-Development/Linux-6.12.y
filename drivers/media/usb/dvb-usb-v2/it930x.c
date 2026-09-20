// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ITE IT9303 USB bridge driver
 *
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include "it930x.h"

#include "avl6381.h"
#include "mxl603_tuner.h"

#include <linux/unaligned.h>
#include <media/dvb-usb-ids.h>

#include "dvb_usb.h"

#define IT930X_REQUEST_HEADER_SIZE 4
#define IT930X_RESPONSE_HEADER_SIZE 3
#define IT930X_CHECKSUM_SIZE 2
#define IT930X_REGISTER_TRANSFER_SIZE 64
#define IT930X_REGISTER_WRITE_MAX (IT930X_REGISTER_TRANSFER_SIZE - 6)
#define IT930X_I2C_READ_MAX 64
#define IT930X_I2C_WRITE_MAX (IT930X_REGISTER_TRANSFER_SIZE - 3)
#define IT930X_I2C_BUS 0x01
#define IT930X_I2C_SPEED_366KHZ 7
#define IT930X_CHIP_ID 0x9306
#define IT930X_SCATTER_HEADER_SIZE 7

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static int it930x_tuner_select_input(struct dvb_frontend *fe,
				     enum fe_delivery_system delivery_system);

static const struct avl6381_config avl6381_config = {
	.demod_address = 0x14,
	.tuner_address = 0x60,
	.tuner_select_input = it930x_tuner_select_input,
};

static const struct mxl603_config mxl603_config = {
	.single_supply_3v3 = true,
	.xtal_frequency = MXL603_XTAL_24MHZ,
	.xtal_capacitance_pf = 12,
	.clock_output = true,
	.clock_output_div4 = false,
	.xtal_sharing = false,
	.if_frequency = MXL603_IF_5MHZ,
	.manual_if = false,
	.invert_if = true,
	.if_gain = 11,
	.manual_if_khz = 5000,
	.agc_type = MXL603_AGC_EXTERNAL,
	.agc_set_point = 66,
	.invert_agc = false,
	.mode_xtal_frequency = MXL603_XTAL_16MHZ,
	.mode_if_khz = 5000,
	.mode_if_gain = 11,
};

static u16 it930x_checksum(const u8 *buffer, size_t length)
{
	u16 checksum = 0;
	size_t i;

	for (i = 1; i < length; i++) {
		if (i & 1)
			checksum += buffer[i] << 8;
		else
			checksum += buffer[i];
	}

	return ~checksum;
}

static int it930x_control_message(struct dvb_usb_device *d,
				  const struct it930x_request *request)
{
	struct it930x_state *state = d_to_priv(d);
	u8 *buffer = state->control_buffer;
	u16 actual_checksum, expected_checksum;
	u8 sequence;
	int read_length, ret, write_length;

	if (request->write_length > sizeof(state->control_buffer) -
					    IT930X_REQUEST_HEADER_SIZE -
					    IT930X_CHECKSUM_SIZE ||
	    request->read_length > sizeof(state->control_buffer) -
					   IT930X_RESPONSE_HEADER_SIZE -
					   IT930X_CHECKSUM_SIZE)
		return -EMSGSIZE;

	mutex_lock(&d->usb_mutex);

	sequence = state->sequence++;
	buffer[0] = IT930X_REQUEST_HEADER_SIZE + request->write_length +
		    IT930X_CHECKSUM_SIZE - 1;
	buffer[1] = request->mailbox;
	buffer[2] = request->command;
	buffer[3] = sequence;
	if (request->write_length)
		memcpy(buffer + IT930X_REQUEST_HEADER_SIZE,
		       request->write_buffer, request->write_length);

	write_length = IT930X_REQUEST_HEADER_SIZE + request->write_length +
		       IT930X_CHECKSUM_SIZE;
	read_length = IT930X_RESPONSE_HEADER_SIZE + request->read_length +
		      IT930X_CHECKSUM_SIZE;
	expected_checksum = it930x_checksum(buffer, write_length - 2);
	buffer[write_length - 2] = expected_checksum >> 8;
	buffer[write_length - 1] = expected_checksum;

	ret = dvb_usbv2_generic_rw_locked(d, buffer, write_length, buffer,
					  read_length);
	if (ret)
		goto unlock;

	expected_checksum = it930x_checksum(buffer, read_length - 2);
	actual_checksum = get_unaligned_be16(buffer + read_length - 2);
	if (actual_checksum != expected_checksum) {
		dev_err(&d->udev->dev,
			"command 0x%02x response checksum mismatch\n",
			request->command);
		ret = -EIO;
		goto unlock;
	}

	if (buffer[1] != sequence) {
		dev_err(&d->udev->dev,
			"command 0x%02x response sequence mismatch\n",
			request->command);
		ret = -EIO;
		goto unlock;
	}

	if (buffer[2]) {
		dev_err(&d->udev->dev, "command 0x%02x failed: status %u\n",
			request->command, buffer[2]);
		ret = -EIO;
		goto unlock;
	}

	if (request->read_length)
		memcpy(request->read_buffer,
		       buffer + IT930X_RESPONSE_HEADER_SIZE,
		       request->read_length);

unlock:
	mutex_unlock(&d->usb_mutex);
	return ret;
}

static int it930x_write_registers(struct dvb_usb_device *d,
				  u32 register_address, const u8 *values,
				  u8 length)
{
	u8 buffer[IT930X_REGISTER_TRANSFER_SIZE];
	struct it930x_request request = {
		.command = IT930X_CMD_MEM_WR,
		.mailbox = register_address >> 16,
		.write_length = 6 + length,
		.write_buffer = buffer,
	};

	if (!length || length > IT930X_REGISTER_WRITE_MAX)
		return -EINVAL;

	buffer[0] = length;
	buffer[1] = 2;
	buffer[2] = 0;
	buffer[3] = 0;
	buffer[4] = register_address >> 8;
	buffer[5] = register_address;
	memcpy(buffer + 6, values, length);

	return it930x_control_message(d, &request);
}

static int it930x_read_registers(struct dvb_usb_device *d, u32 register_address,
				 u8 *values, u8 length)
{
	u8 buffer[] = {
		length, 2, 0, 0, register_address >> 8, register_address
	};
	struct it930x_request request = {
		.command = IT930X_CMD_MEM_RD,
		.mailbox = register_address >> 16,
		.write_length = sizeof(buffer),
		.write_buffer = buffer,
		.read_length = length,
		.read_buffer = values,
	};

	if (!length)
		return -EINVAL;

	return it930x_control_message(d, &request);
}

static int it930x_write_register(struct dvb_usb_device *d, u32 register_address,
				 u8 value)
{
	return it930x_write_registers(d, register_address, &value, 1);
}

static int it930x_read_register(struct dvb_usb_device *d, u32 register_address,
				u8 *value)
{
	return it930x_read_registers(d, register_address, value, 1);
}

static int it930x_update_bits(struct dvb_usb_device *d, u32 register_address,
			      u8 mask, u8 value)
{
	u8 register_value;
	int ret;

	if (!mask)
		return 0;
	if (mask == 0xff)
		return it930x_write_register(d, register_address, value);

	ret = it930x_read_register(d, register_address, &register_value);
	if (ret)
		return ret;

	register_value = (register_value & ~mask) | (value & mask);
	return it930x_write_register(d, register_address, register_value);
}

static int it930x_i2c_request(struct dvb_usb_device *d, u8 command, u16 address,
			      const u8 *write_buffer, u16 write_length,
			      u8 *read_buffer, u16 read_length)
{
	u8 buffer[IT930X_REGISTER_TRANSFER_SIZE];
	struct it930x_request request = {
		.command = command,
		.write_buffer = buffer,
		.read_buffer = read_buffer,
	};

	if (address > 0x7f || write_length > IT930X_I2C_WRITE_MAX ||
	    read_length > IT930X_I2C_READ_MAX)
		return -EOPNOTSUPP;

	request.write_length = 3 + write_length;
	request.read_length = read_length;
	buffer[0] = read_length ? read_length : write_length;
	buffer[1] = IT930X_I2C_BUS;
	buffer[2] = address << 1;
	if (write_length)
		memcpy(buffer + 3, write_buffer, write_length);

	return it930x_control_message(d, &request);
}

static int it930x_i2c_master_xfer(struct i2c_adapter *adapter,
				  struct i2c_msg messages[], int count)
{
	struct dvb_usb_device *d = i2c_get_adapdata(adapter);
	int ret;

	ret = mutex_lock_interruptible(&d->i2c_mutex);
	if (ret)
		return ret;

	if (count == 2 && !(messages[0].flags & I2C_M_RD) &&
	    messages[1].flags & I2C_M_RD &&
	    messages[0].addr == messages[1].addr) {
		ret = it930x_i2c_request(d, IT930X_CMD_GENERIC_I2C_RD,
					 messages[0].addr, messages[0].buf,
					 messages[0].len, messages[1].buf,
					 messages[1].len);
	} else if (count == 1 && !(messages[0].flags & I2C_M_RD)) {
		ret = it930x_i2c_request(d, IT930X_CMD_GENERIC_I2C_WR,
					 messages[0].addr, messages[0].buf,
					 messages[0].len, NULL, 0);
	} else if (count == 1 && messages[0].flags & I2C_M_RD) {
		ret = it930x_i2c_request(d, IT930X_CMD_GENERIC_I2C_RD,
					 messages[0].addr, NULL, 0,
					 messages[0].buf, messages[0].len);
	} else {
		ret = -EOPNOTSUPP;
	}

	mutex_unlock(&d->i2c_mutex);
	return ret ? ret : count;
}

static u32 it930x_i2c_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static struct i2c_algorithm it930x_i2c_algorithm = {
	.master_xfer = it930x_i2c_master_xfer,
	.functionality = it930x_i2c_functionality,
};

static const u32 it930x_gpio_enable_registers[IT930X_GPIO_COUNT] = {
	0xd8b1, 0xd8b9, 0xd8b5, 0xd8c1, 0xd8bd, 0xd8c9, 0xd8c5, 0xd8d1,
	0xd8cd, 0xd8d9, 0xd8d5, 0xd8e1, 0xd8dd, 0xd8e5, 0xd8e9, 0xd8ed,
};

static const u32 it930x_gpio_mode_registers[IT930X_GPIO_COUNT] = {
	0xd8b0, 0xd8b8, 0xd8b4, 0xd8c0, 0xd8bc, 0xd8c8, 0xd8c4, 0xd8d0,
	0xd8cc, 0xd8d8, 0xd8d4, 0xd8e0, 0xd8dc, 0xd8e4, 0xd8e8, 0xd8ec,
};

static const u32 it930x_gpio_output_registers[IT930X_GPIO_COUNT] = {
	0xd8af, 0xd8b7, 0xd8b3, 0xd8bf, 0xd8bb, 0xd8c7, 0xd8c3, 0xd8cf,
	0xd8cb, 0xd8d7, 0xd8d3, 0xd8df, 0xd8db, 0xd8e3, 0xd8e7, 0xd8eb,
};

static int it930x_set_gpio_mode(struct dvb_usb_device *d, enum it930x_gpio gpio,
				bool output)
{
	if (gpio >= IT930X_GPIO_COUNT)
		return -EINVAL;

	return it930x_write_register(d, it930x_gpio_mode_registers[gpio],
				     output);
}

static int it930x_enable_gpio(struct dvb_usb_device *d, enum it930x_gpio gpio)
{
	if (gpio >= IT930X_GPIO_COUNT)
		return -EINVAL;

	return it930x_write_register(d, it930x_gpio_enable_registers[gpio], 1);
}

static int it930x_write_gpio(struct dvb_usb_device *d, enum it930x_gpio gpio,
			     bool high)
{
	if (gpio >= IT930X_GPIO_COUNT)
		return -EINVAL;

	return it930x_write_register(d, it930x_gpio_output_registers[gpio],
				     high);
}

static bool it930x_scatter_header(const u8 *data, size_t size, size_t offset)
{
	if (size - offset < 3)
		return false;

	return data[offset] == 0x03 &&
	       (data[offset + 1] == 0x00 || data[offset + 1] == 0x01) &&
	       data[offset + 2] == 0x00;
}

static int it930x_download_firmware(struct dvb_usb_device *d,
				    const struct firmware *firmware)
{
	struct it930x_request request = {
		.command = IT930X_CMD_FW_SCATTER_WR,
	};
	u8 query = 1;
	u8 version[4];
	size_t offset, next;
	int ret;

	if (!firmware || firmware->size < IT930X_SCATTER_HEADER_SIZE ||
	    !it930x_scatter_header(firmware->data, firmware->size, 0))
		return -EINVAL;

	for (offset = 0; offset < firmware->size; offset = next) {
		for (next = offset + IT930X_SCATTER_HEADER_SIZE;
		     next < firmware->size; next++) {
			if (it930x_scatter_header(firmware->data,
						  firmware->size, next))
				break;
		}

		if (next - offset > U8_MAX)
			return -EINVAL;

		request.write_length = next - offset;
		request.write_buffer = firmware->data + offset;
		ret = it930x_control_message(d, &request);
		if (ret)
			return ret;
	}

	request.command = IT930X_CMD_FW_BOOT;
	request.write_length = 0;
	request.write_buffer = NULL;
	ret = it930x_control_message(d, &request);
	if (ret)
		return ret;

	request.command = IT930X_CMD_FW_QUERYINFO;
	request.write_length = sizeof(query);
	request.write_buffer = &query;
	request.read_length = sizeof(version);
	request.read_buffer = version;
	ret = it930x_control_message(d, &request);
	if (ret)
		return ret;

	if (!(version[0] || version[1] || version[2] || version[3])) {
		dev_err(&d->udev->dev, "firmware did not start\n");
		return -ENODEV;
	}

	dev_info(&d->udev->dev, "firmware version %u.%u.%u.%u\n", version[0],
		 version[1], version[2], version[3]);
	return 0;
}

static int it930x_prepare_bridge(struct dvb_usb_device *d)
{
	int ret;

	usleep_range(7000, 9000);
	ret = it930x_update_bits(d, 0xda05, BIT(0), BIT(0));
	if (ret)
		return ret;
	ret = it930x_write_gpio(d, IT930X_GPIO1, true);
	if (ret)
		return ret;

	ret = it930x_write_register(d, 0xda1d, 1);
	if (ret)
		return ret;
	usleep_range(2000, 3000);
	ret = it930x_write_register(d, 0xda1d, 0);
	if (ret)
		return ret;

	usleep_range(8000, 10000);
	ret = it930x_write_register(d, 0x4976, 0);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0x4bfb, 0);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0x4978, 0);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0x4977, 0);
	if (ret)
		return ret;

	return it930x_write_register(d, 0xf103, IT930X_I2C_SPEED_366KHZ);
}

static int it930x_identify_state(struct dvb_usb_device *d,
				 const char **firmware_name)
{
	struct it930x_state *state = d_to_priv(d);
	struct it930x_request request = {
		.command = IT930X_CMD_FW_QUERYINFO,
	};
	u8 query = 1;
	u8 version[4];
	u8 chip[3];
	int ret;

	ret = it930x_set_gpio_mode(d, IT930X_GPIO1, true);
	if (ret)
		return ret;
	ret = it930x_enable_gpio(d, IT930X_GPIO1);
	if (ret)
		return ret;
	ret = it930x_write_gpio(d, IT930X_GPIO1, false);
	if (ret)
		return ret;
	msleep(20);

	ret = it930x_read_registers(d, 0x1222, chip, sizeof(chip));
	if (ret)
		return ret;
	state->chip_version = chip[0];
	state->chip_type = get_unaligned_le16(chip + 1);

	ret = it930x_read_register(d, 0x384f, &state->prechip_version);
	if (ret)
		return ret;
	if (state->chip_type != IT930X_CHIP_ID)
		return -ENODEV;

	dev_info(&d->udev->dev,
		 "IT9303 prechip %02x, chip version %02x, type %04x\n",
		 state->prechip_version, state->chip_version, state->chip_type);
	*firmware_name = IT9303_FIRMWARE;

	request.write_length = sizeof(query);
	request.write_buffer = &query;
	request.read_length = sizeof(version);
	request.read_buffer = version;
	ret = it930x_control_message(d, &request);
	if (ret)
		return ret;

	ret = it930x_prepare_bridge(d);
	if (ret)
		return ret;

	return version[0] || version[1] || version[2] || version[3] ? WARM :
								      COLD;
}

static int it930x_initialize_stream(struct dvb_usb_device *d)
{
	u16 frame_size = (d->udev->speed == USB_SPEED_FULL ? 5 : 816) * 188 / 4;
	u8 packet_size = (d->udev->speed == USB_SPEED_FULL ? 64 : 512) / 4;
	u8 frame_size_bytes[] = { frame_size, frame_size >> 8 };
	int ret;

	ret = it930x_write_register(d, 0xf6a7, IT930X_I2C_SPEED_366KHZ);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0xf103, IT930X_I2C_SPEED_366KHZ);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0xda1a, 0);
	if (ret)
		return ret;
	ret = it930x_update_bits(d, 0xf41f, 0x04, 0x04);
	if (ret)
		return ret;
	ret = it930x_update_bits(d, 0xf41a, 0x05, 0x05);
	if (ret)
		return ret;
	ret = it930x_update_bits(d, 0xda1d, BIT(0), BIT(0));
	if (ret)
		return ret;
	ret = it930x_update_bits(d, 0xdd11, 0x0f, 0x0f);
	if (ret)
		return ret;
	ret = it930x_update_bits(d, 0xdd13, 0x1b, 0x1b);
	if (ret)
		return ret;
	ret = it930x_update_bits(d, 0xdd11, 0x2f, 0x2f);
	if (ret)
		return ret;
	ret = it930x_write_registers(d, 0xdd88, frame_size_bytes,
				     sizeof(frame_size_bytes));
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0xdd0c, packet_size);
	if (ret)
		return ret;
	ret = it930x_update_bits(d, 0xda05, BIT(0), 0);
	if (ret)
		return ret;
	ret = it930x_update_bits(d, 0xda06, BIT(0), 0);
	if (ret)
		return ret;
	ret = it930x_update_bits(d, 0xda1d, BIT(0), 0);
	if (ret)
		return ret;

	ret = it930x_write_register(d, 0xd920, 0);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0xd833, 1);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0xd830, 0);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0xd831, 1);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0xd832, 0);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0x4976, 1);
	if (ret)
		return ret;

	msleep(20);
	ret = it930x_update_bits(d, 0xda58, BIT(0), 0);
	if (ret)
		return ret;
	usleep_range(8000, 10000);
	ret = it930x_write_register(d, 0xda51, 0);
	if (ret)
		return ret;
	usleep_range(8000, 10000);
	ret = it930x_write_register(d, 0xda73, 1);
	if (ret)
		return ret;
	ret = it930x_write_register(d, 0xda78, 0x47);
	if (ret)
		return ret;
	msleep(30);
	ret = it930x_write_register(d, 0xda4c, 1);
	if (ret)
		return ret;
	usleep_range(8000, 10000);

	return it930x_write_register(d, 0xda5a, 0x1f);
}

static int it930x_tuner_select_input(struct dvb_frontend *fe,
				     enum fe_delivery_system delivery_system)
{
	struct dvb_usb_device *d = fe_to_d(fe);
	bool terrestrial;
	int ret;

	switch (delivery_system) {
	case SYS_DTMB:
		terrestrial = true;
		break;
	case SYS_DVBC_ANNEX_A:
		terrestrial = false;
		break;
	default:
		return -EINVAL;
	}

	ret = it930x_write_gpio(d, IT930X_GPIO2, terrestrial);
	if (ret)
		return ret;

	return it930x_write_gpio(d, IT930X_GPIO3, false);
}

static int it930x_frontend_attach(struct dvb_usb_adapter *adapter)
{
	struct dvb_usb_device *d = adap_to_d(adapter);
	int ret;

	ret = it930x_set_gpio_mode(d, IT930X_GPIO1, true);
	if (ret)
		return ret;
	ret = it930x_enable_gpio(d, IT930X_GPIO1);
	if (ret)
		return ret;
	ret = it930x_write_gpio(d, IT930X_GPIO1, false);
	if (ret)
		return ret;
	msleep(30);
	ret = it930x_write_gpio(d, IT930X_GPIO1, true);
	if (ret)
		return ret;
	msleep(150);

	ret = it930x_set_gpio_mode(d, IT930X_GPIO2, true);
	if (ret)
		return ret;
	ret = it930x_enable_gpio(d, IT930X_GPIO2);
	if (ret)
		return ret;
	ret = it930x_set_gpio_mode(d, IT930X_GPIO3, true);
	if (ret)
		return ret;
	ret = it930x_enable_gpio(d, IT930X_GPIO3);
	if (ret)
		return ret;

	adapter->fe[0] =
		dvb_attach(avl6381_attach, &avl6381_config, &d->i2c_adap);
	if (!adapter->fe[0]) {
		dev_err(&d->udev->dev, "AVL6381 demodulator not found\n");
		return -ENODEV;
	}

	return 0;
}

static int it930x_tuner_attach(struct dvb_usb_adapter *adapter)
{
	struct dvb_usb_device *d = adap_to_d(adapter);
	struct dvb_frontend *frontend;

	frontend = dvb_attach(mxl603_attach, adapter->fe[0], &d->i2c_adap,
			      avl6381_config.tuner_address, &mxl603_config);
	return frontend ? 0 : -ENODEV;
}

static int it930x_get_stream_config(struct dvb_frontend *frontend,
				    u8 *transport_type,
				    struct usb_data_stream_properties *stream)
{
	struct dvb_usb_device *d = fe_to_d(frontend);

	*transport_type = DVB_USB_FE_TS_TYPE_188;
	if (d->udev->speed == USB_SPEED_FULL)
		stream->u.bulk.buffersize = 5 * 188;

	return 0;
}

static const struct dvb_usb_device_properties it930x_properties = {
	.driver_name = KBUILD_MODNAME,
	.owner = THIS_MODULE,
	.adapter_nr = adapter_nr,
	.size_of_priv = sizeof(struct it930x_state),

	.generic_bulk_ctrl_endpoint = 0x02,
	.generic_bulk_ctrl_endpoint_response = 0x81,

	.identify_state = it930x_identify_state,
	.download_firmware = it930x_download_firmware,
	.i2c_algo = &it930x_i2c_algorithm,
	.frontend_attach = it930x_frontend_attach,
	.tuner_attach = it930x_tuner_attach,
	.init = it930x_initialize_stream,
	.get_stream_config = it930x_get_stream_config,

	.num_adapters = 1,
	.adapter = {
		{
			.stream = DVB_USB_STREAM_BULK(0x84, 4, 816 * 188),
		},
	},
};

static const struct usb_device_id it930x_id_table[] = {
	{ DVB_USB_DEVICE(USB_VID_ITETECH, USB_PID_ITETECH_IT9303,
			 &it930x_properties, "ITE IT9303 AVL6381 receiver",
			 NULL) },
	{}
};
MODULE_DEVICE_TABLE(usb, it930x_id_table);

static struct usb_driver it930x_usb_driver = {
	.name = KBUILD_MODNAME,
	.id_table = it930x_id_table,
	.probe = dvb_usbv2_probe,
	.disconnect = dvb_usbv2_disconnect,
	.suspend = dvb_usbv2_suspend,
	.resume = dvb_usbv2_resume,
	.reset_resume = dvb_usbv2_reset_resume,
	.no_dynamic_id = 1,
	.soft_unbind = 1,
};
module_usb_driver(it930x_usb_driver);

MODULE_DESCRIPTION("ITE IT9303 USB bridge driver");
MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(IT9303_FIRMWARE);
