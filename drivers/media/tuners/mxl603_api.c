// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MaxLinear MxL603 register programming
 *
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/math64.h>

#include "mxl603_api.h"

#define MXL603_REG_PAGE 0x00
#define MXL603_REG_XTAL_CAP 0x01
#define MXL603_REG_XTAL_CTRL 0x02
#define MXL603_REG_XTAL_CAL 0x03
#define MXL603_REG_IF_FREQ 0x04
#define MXL603_REG_IF_GAIN 0x05
#define MXL603_REG_IF_FCW_LOW 0x06
#define MXL603_REG_IF_FCW_HIGH 0x07
#define MXL603_REG_AGC_CONFIG 0x08
#define MXL603_REG_AGC_SET_POINT 0x09
#define MXL603_REG_TUNER_ENABLE 0x0b
#define MXL603_REG_MAIN_AMP 0x0e
#define MXL603_REG_BANDWIDTH 0x0f
#define MXL603_REG_FREQUENCY_LOW 0x10
#define MXL603_REG_FREQUENCY_HIGH 0x11
#define MXL603_REG_START_TUNE 0x12
#define MXL603_REG_CHIP_ID 0x18
#define MXL603_REG_CHIP_REVISION 0x1a
#define MXL603_REG_RF_POWER_LOW 0x1d
#define MXL603_REG_RF_POWER_HIGH 0x1e
#define MXL603_REG_LOCK_STATUS 0x2b
#define MXL603_REG_AGC_FLIP 0x5e
#define MXL603_REG_RESET 0xff

#define MXL603_READ_PREFIX 0xfb

struct mxl603_reg_sequence {
	u8 reg;
	u8 mask;
	u8 value;
};

#define MXL603_REG(_reg, _value) { (_reg), 0xff, (_value) }

static const struct mxl603_reg_sequence mxl603_defaults[] = {
	MXL603_REG(0x14, 0x13), MXL603_REG(0x6d, 0x8a), MXL603_REG(0x6d, 0x0a),
	MXL603_REG(0xdf, 0x19), MXL603_REG(0x45, 0x1b), MXL603_REG(0xa9, 0x59),
	MXL603_REG(0xaa, 0x6a), MXL603_REG(0xbe, 0x4c), MXL603_REG(0xcf, 0x25),
	MXL603_REG(0xd0, 0x34), MXL603_REG(0x77, 0xe7), MXL603_REG(0x78, 0xe3),
	MXL603_REG(0x6f, 0x51), MXL603_REG(0x7b, 0x84), MXL603_REG(0x7c, 0x9f),
	MXL603_REG(0x56, 0x41), MXL603_REG(0xcd, 0x64), MXL603_REG(0xc3, 0x2c),
	MXL603_REG(0x9d, 0x61), MXL603_REG(0xf7, 0x52), MXL603_REG(0x58, 0x81),
	MXL603_REG(0x00, 0x01), MXL603_REG(0x62, 0x02), MXL603_REG(0x00, 0x00),
};

static const struct mxl603_reg_sequence mxl603_dvbc_mode[] = {
	MXL603_REG(0x0c, 0x00), MXL603_REG(0x13, 0x04), MXL603_REG(0x53, 0x7e),
	MXL603_REG(0x57, 0x91), MXL603_REG(0x5c, 0xb1), MXL603_REG(0x62, 0xf2),
	MXL603_REG(0x6e, 0x03), MXL603_REG(0x6f, 0xd1), MXL603_REG(0x87, 0x77),
	MXL603_REG(0x88, 0x55), MXL603_REG(0x93, 0x33), MXL603_REG(0x97, 0x03),
	MXL603_REG(0xba, 0x40), MXL603_REG(0x98, 0xaf), MXL603_REG(0x9b, 0x20),
	MXL603_REG(0x9c, 0x1e), MXL603_REG(0xa0, 0x18), MXL603_REG(0xa5, 0x09),
	MXL603_REG(0xc2, 0xa9), MXL603_REG(0xc5, 0x7c), MXL603_REG(0xcd, 0x64),
	MXL603_REG(0xce, 0x7c), MXL603_REG(0xd5, 0x05), MXL603_REG(0xd9, 0x00),
	MXL603_REG(0xea, 0x00), MXL603_REG(0xdc, 0x1c),
};

static const struct mxl603_reg_sequence mxl603_dtmb_mode[] = {
	MXL603_REG(0x0c, 0x00), MXL603_REG(0x13, 0x04), MXL603_REG(0x53, 0xfe),
	MXL603_REG(0x57, 0x91), MXL603_REG(0x62, 0xc2), MXL603_REG(0x6e, 0x01),
	MXL603_REG(0x6f, 0x51), MXL603_REG(0x87, 0x77), MXL603_REG(0x88, 0x55),
	MXL603_REG(0x93, 0x22), MXL603_REG(0x97, 0x02), MXL603_REG(0xba, 0x30),
	MXL603_REG(0x98, 0xaf), MXL603_REG(0x9b, 0x20), MXL603_REG(0x9c, 0x1e),
	MXL603_REG(0xa0, 0x18), MXL603_REG(0xa5, 0x09), MXL603_REG(0xc2, 0xa9),
	MXL603_REG(0xc5, 0x7c), MXL603_REG(0xcd, 0x64), MXL603_REG(0xce, 0x7c),
	MXL603_REG(0xd5, 0x03), MXL603_REG(0xd9, 0x04),
};

static int mxl603_write(struct i2c_adapter *i2c, u8 addr, u8 reg, u8 value)
{
	u8 buf[] = { reg, value };
	struct i2c_msg msg = {
		.addr = addr,
		.buf = buf,
		.len = sizeof(buf),
	};
	int ret;

	ret = i2c_transfer(i2c, &msg, 1);
	if (ret == 1)
		return 0;

	dev_dbg(&i2c->dev, "write reg 0x%02x failed: %d\n", reg, ret);
	return ret < 0 ? ret : -EREMOTEIO;
}

static int mxl603_read(struct i2c_adapter *i2c, u8 addr, u8 reg, u8 *value)
{
	struct i2c_msg msg = {
		.addr = addr,
		.flags = I2C_M_RD,
		.buf = value,
		.len = 1,
	};
	int ret;

	ret = mxl603_write(i2c, addr, MXL603_READ_PREFIX, reg);
	if (ret)
		return ret;

	ret = i2c_transfer(i2c, &msg, 1);
	if (ret == 1)
		return 0;

	dev_dbg(&i2c->dev, "read reg 0x%02x failed: %d\n", reg, ret);
	return ret < 0 ? ret : -EREMOTEIO;
}

static int mxl603_update_bits(struct i2c_adapter *i2c, u8 addr, u8 reg, u8 mask,
			      u8 value)
{
	u8 old_value;
	int ret;

	ret = mxl603_read(i2c, addr, reg, &old_value);
	if (ret)
		return ret;

	value = (old_value & ~mask) | (value & mask);
	return mxl603_write(i2c, addr, reg, value);
}

static int mxl603_write_sequence(struct i2c_adapter *i2c, u8 addr,
				 const struct mxl603_reg_sequence *sequence,
				 size_t count)
{
	size_t i;
	int ret;

	for (i = 0; i < count; i++) {
		if (sequence[i].mask == 0xff)
			ret = mxl603_write(i2c, addr, sequence[i].reg,
					   sequence[i].value);
		else
			ret = mxl603_update_bits(i2c, addr, sequence[i].reg,
						 sequence[i].mask,
						 sequence[i].value);
		if (ret)
			return ret;
	}

	return 0;
}

static int mxl603_restore_page_zero(struct i2c_adapter *i2c, u8 addr, int ret)
{
	int page_ret;

	page_ret = mxl603_write(i2c, addr, MXL603_REG_PAGE, 0);
	return ret ? ret : page_ret;
}

int mxl603_reset(struct i2c_adapter *i2c, u8 addr)
{
	return mxl603_write(i2c, addr, MXL603_REG_RESET, 0);
}

int mxl603_get_version(struct i2c_adapter *i2c, u8 addr,
		       struct mxl603_version *version)
{
	int ret;

	if (!version)
		return -EINVAL;

	ret = mxl603_read(i2c, addr, MXL603_REG_CHIP_ID, &version->chip_id);
	if (ret)
		return ret;

	return mxl603_read(i2c, addr, MXL603_REG_CHIP_REVISION,
			   &version->revision);
}

static int mxl603_load_defaults(struct i2c_adapter *i2c, u8 addr,
				bool single_supply_3v3)
{
	int ret;

	ret = mxl603_write_sequence(i2c, addr, mxl603_defaults,
				    ARRAY_SIZE(mxl603_defaults));
	if (ret)
		return ret;

	ret = mxl603_write(i2c, addr, MXL603_REG_PAGE, 1);
	if (ret)
		return ret;
	ret = mxl603_update_bits(i2c, addr, 0x31, 0xd0, 0xd0);
	ret = mxl603_restore_page_zero(i2c, addr, ret);
	if (ret)
		return ret;

	if (single_supply_3v3)
		return mxl603_write(i2c, addr, MXL603_REG_MAIN_AMP, 0x04);

	return 0;
}

static int mxl603_configure_xtal(struct i2c_adapter *i2c, u8 addr,
				 const struct mxl603_config *config)
{
	u8 value;
	int ret;

	if (config->xtal_frequency > MXL603_XTAL_24MHZ ||
	    config->xtal_capacitance_pf > 31)
		return -EINVAL;

	value = config->xtal_frequency << 5;
	value |= config->xtal_capacitance_pf;
	if (config->clock_output)
		value |= BIT(7);
	ret = mxl603_write(i2c, addr, MXL603_REG_XTAL_CAP, value);
	if (ret)
		return ret;

	value = config->clock_output_div4 ? BIT(0) : 0;
	if (config->xtal_sharing)
		value |= BIT(6);
	ret = mxl603_write(i2c, addr, MXL603_REG_XTAL_CTRL, value);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, 0x6d, 0x0a);
	if (ret)
		return ret;

	if (config->single_supply_3v3)
		return mxl603_write(i2c, addr, MXL603_REG_MAIN_AMP, 0x14);

	return 0;
}

static int mxl603_disable_loop_through(struct i2c_adapter *i2c, u8 addr)
{
	int ret;

	ret = mxl603_write(i2c, addr, MXL603_REG_PAGE, 1);
	if (ret)
		return ret;
	ret = mxl603_update_bits(i2c, addr, 0x96, BIT(4), 0);

	return mxl603_restore_page_zero(i2c, addr, ret);
}

int mxl603_set_if_output(struct i2c_adapter *i2c, u8 addr,
			 const struct mxl603_config *config)
{
	u16 fcw;
	u8 value;
	int ret;

	if (config->manual_if) {
		if (!config->manual_if_khz || config->manual_if_khz > 44000)
			return -EINVAL;
		ret = mxl603_update_bits(i2c, addr, MXL603_REG_IF_FREQ, BIT(5),
					 BIT(5));
		if (ret)
			return ret;
		fcw = config->manual_if_khz * 8192 / 216000;
		ret = mxl603_write(i2c, addr, MXL603_REG_IF_FCW_LOW, fcw);
		if (ret)
			return ret;
		ret = mxl603_write(i2c, addr, MXL603_REG_IF_FCW_HIGH,
				   (fcw >> 8) & 0x0f);
	} else {
		if (config->if_frequency > MXL603_IF_44MHZ)
			return -EINVAL;
		ret = mxl603_update_bits(i2c, addr, MXL603_REG_IF_FREQ, 0x3f,
					 config->if_frequency);
	}
	if (ret)
		return ret;
	if (config->if_gain > 15)
		return -EINVAL;

	value = BIT(5) | config->if_gain;
	if (config->invert_if)
		value |= GENMASK(7, 6);

	return mxl603_write(i2c, addr, MXL603_REG_IF_GAIN, value);
}

static int mxl603_configure_agc(struct i2c_adapter *i2c, u8 addr,
				const struct mxl603_config *config)
{
	int ret;

	if (config->agc_type > MXL603_AGC_EXTERNAL ||
	    config->agc_set_point > 0x7f)
		return -EINVAL;

	ret = mxl603_update_bits(i2c, addr, MXL603_REG_AGC_CONFIG,
				 GENMASK(3, 2) | BIT(0),
				 config->agc_type << 2 | BIT(0));
	if (ret)
		return ret;
	ret = mxl603_update_bits(i2c, addr, MXL603_REG_AGC_SET_POINT,
				 GENMASK(6, 0), config->agc_set_point);
	if (ret)
		return ret;

	return mxl603_update_bits(i2c, addr, MXL603_REG_AGC_FLIP, BIT(4),
				  config->invert_agc ? BIT(4) : 0);
}

int mxl603_set_power(struct i2c_adapter *i2c, u8 addr,
		     enum mxl603_power_mode mode)
{
	u8 enable;
	int ret;

	if (mode == MXL603_POWER_ACTIVE)
		enable = 1;
	else if (mode == MXL603_POWER_STANDBY)
		enable = 0;
	else
		return -EINVAL;

	if (mode == MXL603_POWER_ACTIVE) {
		ret = mxl603_write(i2c, addr, MXL603_REG_TUNER_ENABLE, enable);
		if (ret)
			return ret;
		ret = mxl603_write(i2c, addr, MXL603_REG_START_TUNE, enable);
	} else {
		ret = mxl603_write(i2c, addr, MXL603_REG_START_TUNE, enable);
		if (ret)
			return ret;
		ret = mxl603_write(i2c, addr, MXL603_REG_TUNER_ENABLE, enable);
	}
	if (ret)
		return ret;

	ret = mxl603_write(i2c, addr, MXL603_REG_PAGE, 1);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, 0x60, 0x37);

	return mxl603_restore_page_zero(i2c, addr, ret);
}

static int mxl603_mode_gain(u8 gain, u8 *value)
{
	switch (gain) {
	case 11:
		*value = 0x47;
		break;
	case 9:
		*value = 0x44;
		break;
	case 8:
		*value = 0x43;
		break;
	case 7:
		*value = 0x42;
		break;
	case 6:
		*value = 0x41;
		break;
	case 5:
		*value = 0x40;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int mxl603_set_mode(struct i2c_adapter *i2c, u8 addr,
		    const struct mxl603_config *config,
		    enum mxl603_signal_mode mode)
{
	const struct mxl603_reg_sequence *sequence;
	size_t count;
	u8 csf, gain;
	int ret;

	if (config->mode_xtal_frequency == MXL603_XTAL_16MHZ)
		csf = 0x0d;
	else if (config->mode_xtal_frequency == MXL603_XTAL_24MHZ)
		csf = 0x0e;
	else
		return -EINVAL;

	if (mode == MXL603_MODE_DVBC) {
		sequence = mxl603_dvbc_mode;
		count = ARRAY_SIZE(mxl603_dvbc_mode);
	} else if (mode == MXL603_MODE_DTMB) {
		sequence = mxl603_dtmb_mode;
		count = ARRAY_SIZE(mxl603_dtmb_mode);
	} else {
		return -EINVAL;
	}

	ret = mxl603_write_sequence(i2c, addr, sequence, count);
	if (ret)
		return ret;
	if (config->mode_if_khz < 35250) {
		ret = mxl603_write(i2c, addr, 0x5a, 0xfe);
		if (ret)
			return ret;
		ret = mxl603_write(i2c, addr, 0x5b,
				   mode == MXL603_MODE_DVBC ? 0x10 : 0x18);
		if (ret)
			return ret;
		if (mode == MXL603_MODE_DTMB) {
			ret = mxl603_write(i2c, addr, 0x5c, 0xf1);
			if (ret)
				return ret;
		}
	} else {
		ret = mxl603_write(i2c, addr, 0x5a, 0xd9);
		if (ret)
			return ret;
		ret = mxl603_write(i2c, addr, 0x5b, 0x16);
		if (ret)
			return ret;
		if (mode == MXL603_MODE_DTMB) {
			ret = mxl603_write(i2c, addr, 0x5c, 0xb1);
			if (ret)
				return ret;
		}
	}

	ret = mxl603_write(i2c, addr, 0xea, csf);
	if (ret)
		return ret;
	if (mode == MXL603_MODE_DTMB) {
		ret = mxl603_mode_gain(config->mode_if_gain, &gain);
		if (ret)
			return ret;
		ret = mxl603_write(i2c, addr, 0xdc, gain);
		if (ret)
			return ret;
	}

	ret = mxl603_write(i2c, addr, MXL603_REG_XTAL_CAL, 0);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, MXL603_REG_XTAL_CAL, 1);
	if (ret)
		return ret;
	msleep(50);

	return 0;
}

int mxl603_initialize(struct i2c_adapter *i2c, u8 addr,
		      const struct mxl603_config *config)
{
	int ret;

	if (!config)
		return -EINVAL;

	ret = mxl603_reset(i2c, addr);
	if (ret)
		return ret;
	ret = mxl603_load_defaults(i2c, addr, config->single_supply_3v3);
	if (ret)
		return ret;
	ret = mxl603_configure_xtal(i2c, addr, config);
	if (ret)
		return ret;
	ret = mxl603_disable_loop_through(i2c, addr);
	if (ret)
		return ret;
	ret = mxl603_set_if_output(i2c, addr, config);
	if (ret)
		return ret;
	ret = mxl603_configure_agc(i2c, addr, config);
	if (ret)
		return ret;
	ret = mxl603_set_power(i2c, addr, MXL603_POWER_ACTIVE);
	if (ret)
		return ret;

	return mxl603_set_mode(i2c, addr, config, MXL603_MODE_DVBC);
}

int mxl603_set_frequency(struct i2c_adapter *i2c, u8 addr, u32 frequency,
			 enum mxl603_bandwidth bandwidth,
			 enum mxl603_signal_mode mode)
{
	u8 loop_through, agc, tune, cdc, vco;
	u64 frequency_code;
	int ret;

	if (mode != MXL603_MODE_DVBC && mode != MXL603_MODE_DTMB)
		return -EINVAL;
	switch (bandwidth) {
	case MXL603_CABLE_BW_6MHZ:
	case MXL603_CABLE_BW_7MHZ:
	case MXL603_CABLE_BW_8MHZ:
	case MXL603_TERR_BW_6MHZ:
	case MXL603_TERR_BW_7MHZ:
	case MXL603_TERR_BW_8MHZ:
		break;
	default:
		return -EINVAL;
	}
	if (frequency < 44000000 || frequency > 1002000000)
		return -EINVAL;

	ret = mxl603_write(i2c, addr, MXL603_REG_START_TUNE, 0);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, 0x7c,
			   frequency < 700000000 ? 0x1f : 0x9f);
	if (ret)
		return ret;
	vco = mode == MXL603_MODE_DVBC ? 0xc1 : 0x81;
	if (frequency >= 700000000)
		vco |= 0x10;
	ret = mxl603_write(i2c, addr, MXL603_REG_PAGE, 1);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, 0x31, vco);
	ret = mxl603_restore_page_zero(i2c, addr, ret);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, 0xea, 0x00);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, 0xeb, 0xd8);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, MXL603_REG_BANDWIDTH, bandwidth);
	if (ret)
		return ret;

	frequency_code = (u64)frequency * 64;
	do_div(frequency_code, 1000000);
	ret = mxl603_write(i2c, addr, MXL603_REG_FREQUENCY_LOW,
			   frequency_code & 0xff);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, MXL603_REG_FREQUENCY_HIGH,
			   frequency_code >> 8);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, MXL603_REG_TUNER_ENABLE, 1);
	if (ret)
		return ret;

	ret = mxl603_write(i2c, addr, MXL603_REG_PAGE, 1);
	if (ret)
		return ret;
	ret = mxl603_read(i2c, addr, 0x96, &loop_through);
	ret = mxl603_restore_page_zero(i2c, addr, ret);
	if (ret)
		return ret;
	ret = mxl603_read(i2c, addr, 0xb6, &agc);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, MXL603_REG_PAGE, 1);
	if (ret)
		return ret;
	ret = mxl603_read(i2c, addr, 0x60, &tune);
	if (!ret)
		ret = mxl603_read(i2c, addr, 0x5f, &cdc);
	if (ret)
		return mxl603_restore_page_zero(i2c, addr, ret);

	if (loop_through & BIT(4)) {
		agc = (agc & ~0x7f) | 0x0e;
		tune = (tune & 0xc0) | 0x0e;
		cdc = (cdc & 0xc0) | 0x0e;
	} else {
		agc = (agc & 0x80) | BIT(6);
		tune = (tune & 0xc0) | 0x37;
		cdc = (cdc & 0xc0) | 0x37;
	}

	ret = mxl603_write(i2c, addr, 0x60, tune);
	if (!ret)
		ret = mxl603_write(i2c, addr, 0x5f, cdc);
	ret = mxl603_restore_page_zero(i2c, addr, ret);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, 0xb6, agc);
	if (ret)
		return ret;
	ret = mxl603_write(i2c, addr, MXL603_REG_START_TUNE, 1);
	if (ret)
		return ret;
	usleep_range(15000, 17000);

	return mxl603_write(i2c, addr, 0xb6, agc | BIT(6));
}

int mxl603_get_lock(struct i2c_adapter *i2c, u8 addr, bool *rf_locked,
		    bool *ref_locked)
{
	u8 value;
	int ret;

	if (!rf_locked || !ref_locked)
		return -EINVAL;

	ret = mxl603_read(i2c, addr, MXL603_REG_LOCK_STATUS, &value);
	if (ret)
		return ret;

	*rf_locked = value & BIT(1);
	*ref_locked = value & BIT(0);
	return 0;
}

int mxl603_get_rf_power(struct i2c_adapter *i2c, u8 addr, s16 *dbm_x100)
{
	u16 raw;
	u8 value;
	int ret;

	if (!dbm_x100)
		return -EINVAL;

	ret = mxl603_read(i2c, addr, MXL603_REG_RF_POWER_LOW, &value);
	if (ret)
		return ret;
	raw = value;
	ret = mxl603_read(i2c, addr, MXL603_REG_RF_POWER_HIGH, &value);
	if (ret)
		return ret;
	raw |= (value & 0x03) << 8;

	*dbm_x100 = (raw & 0x01ff) * 25;
	if (raw & BIT(9))
		*dbm_x100 -= 12800;

	return 0;
}
