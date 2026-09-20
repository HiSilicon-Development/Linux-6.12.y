// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink avl6381 demod driver
 *
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>

#include "avl6381.h"
#include "avl6381_priv.h"

enum avl6381_mode { MODE_DTMB, MODE_DVBC };

static const u8 avl6381_pll_config[][40] = {
	{ 0x80, 0xC3, 0xC9, 0x01, 0x02, 0x32, 0x03, 0x05, 0x00, 0xA3,
	  0xE1, 0x11, 0x01, 0x22, 0x03, 0x0C, 0x19, 0x20, 0x00, 0x00,
	  0x6E, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x80, 0xFE,
	  0x21, 0x0A, 0x00, 0x1E, 0xDD, 0x04, 0x70, 0xBF, 0xCC, 0x03 },
	{ 0x80, 0xC3, 0xC9, 0x01, 0x02, 0x2C, 0x03, 0x06, 0x00, 0xEF,
	  0x1C, 0x0D, 0x01, 0x22, 0x03, 0x0C, 0x19, 0x20, 0x00, 0x00,
	  0x6E, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x80, 0xFE,
	  0x21, 0x0A, 0x00, 0x1E, 0xDD, 0x04, 0x70, 0xBF, 0xCC, 0x03 },
	{ 0x00, 0x24, 0xF4, 0x00, 0x01, 0x2A, 0x03, 0x05, 0x00, 0x90,
	  0x05, 0x10, 0x01, 0x36, 0x03, 0x0A, 0x18, 0x1B, 0x01, 0x00,
	  0x6E, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xB8,
	  0x4C, 0x0A, 0x00, 0xA2, 0x4A, 0x04, 0x00, 0x90, 0xD0, 0x03 },
	{ 0x00, 0x24, 0xF4, 0x00, 0x01, 0x2A, 0x03, 0x06, 0x00, 0xF8,
	  0x59, 0x0D, 0x01, 0x36, 0x03, 0x0A, 0x18, 0x1B, 0x01, 0x00,
	  0x6E, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xB8,
	  0x4C, 0x0A, 0x00, 0xA2, 0x4A, 0x04, 0x00, 0x90, 0xD0, 0x03 },
	{ 0x00, 0x36, 0x6E, 0x01, 0x02, 0x4B, 0x03, 0x06, 0x00, 0xA3,
	  0xE1, 0x11, 0x01, 0x23, 0x03, 0x0A, 0x15, 0x1C, 0x00, 0x00,
	  0x6E, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x7A,
	  0x03, 0x0A, 0x00, 0xB4, 0xC4, 0x04, 0x00, 0x87, 0x93, 0x03 },
	{ 0x00, 0x36, 0x6E, 0x01, 0x01, 0x2A, 0x02, 0x09, 0x00, 0xF8,
	  0x59, 0x0D, 0x01, 0x2A, 0x02, 0x0C, 0x19, 0x02, 0x00, 0x00,
	  0x5F, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x7A,
	  0x03, 0x0A, 0x00, 0xB4, 0xC4, 0x04, 0x00, 0x87, 0x93, 0x03 },
	{ 0xC0, 0xFC, 0x9B, 0x01, 0x03, 0x4A, 0x03, 0x05, 0x00, 0xF1,
	  0xE0, 0x0F, 0x01, 0x20, 0x03, 0x0A, 0x18, 0x1B, 0x00, 0x00,
	  0x6E, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xB8,
	  0x4C, 0x0A, 0x00, 0xA2, 0x4A, 0x04, 0x00, 0x90, 0xD0, 0x03 },
	{ 0xC0, 0xFC, 0x9B, 0x01, 0x03, 0x4A, 0x03, 0x06, 0x80, 0x73,
	  0x3B, 0x0D, 0x01, 0x20, 0x03, 0x0A, 0x18, 0x1B, 0x00, 0x00,
	  0x6E, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xB8,
	  0x4C, 0x0A, 0x00, 0xA2, 0x4A, 0x04, 0x00, 0x90, 0xD0, 0x03 }
};

static int avl6381_i2c_rd(struct avl6381_priv *priv, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg = {
		.addr = priv->config->demod_address,
		.flags = I2C_M_RD,
		.len = len,
		.buf = buf,
	};
	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "%s: i2c rd failed=%d len=%d\n",
			 KBUILD_MODNAME, ret, len);
		ret = -EREMOTEIO;
	}
	return ret;
}

static int avl6381_i2c_wr(struct avl6381_priv *priv, u8 *buf, int len)
{
	int ret;
	struct i2c_msg msg = {
		.addr = priv->config->demod_address,
		.flags = 0,
		.buf = buf,
		.len = len,
	};
	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_warn(&priv->i2c->dev, "%s: i2c wr failed=%d len=%d\n",
			 KBUILD_MODNAME, ret, len);
		ret = -EREMOTEIO;
	}
	return ret;
}

static int avl6381_i2c_wr_reg(struct avl6381_priv *priv, u32 addr, u32 data,
			      int reg_size)
{
	u8 buf[3 + 4];
	u8 *p = buf;

	if (reg_size != 1 && reg_size != 2 && reg_size != 4)
		return -EINVAL;

	*(p++) = (u8)(addr >> 16);
	*(p++) = (u8)(addr >> 8);
	*(p++) = (u8)(addr);

	switch (reg_size) {
	case 4:
		*(p++) = (u8)(data >> 24);
		*(p++) = (u8)(data >> 16);
		fallthrough;
	case 2:
		*(p++) = (u8)(data >> 8);
		fallthrough;
	case 1:
		*(p++) = (u8)(data);
		break;
	}

	return avl6381_i2c_wr(priv, buf, 3 + reg_size);
}

#define AVL6381_WR_REG8(_priv, _addr, _data) \
	avl6381_i2c_wr_reg(_priv, _addr, _data, 1)
#define AVL6381_WR_REG16(_priv, _addr, _data) \
	avl6381_i2c_wr_reg(_priv, _addr, _data, 2)
#define AVL6381_WR_REG32(_priv, _addr, _data) \
	avl6381_i2c_wr_reg(_priv, _addr, _data, 4)

struct avl6381_reg_sequence {
	u32 addr;
	u32 value;
	u8 width;
};

#define AVL6381_REG8(_addr, _value) \
	{ .addr = (_addr), .value = (_value), .width = 1 }
#define AVL6381_REG16(_addr, _value) \
	{ .addr = (_addr), .value = (_value), .width = 2 }
#define AVL6381_REG32(_addr, _value) \
	{ .addr = (_addr), .value = (_value), .width = 4 }

static int avl6381_write_sequence(struct avl6381_priv *priv,
				  const struct avl6381_reg_sequence *sequence,
				  size_t count)
{
	size_t i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = avl6381_i2c_wr_reg(priv, sequence[i].addr,
					 sequence[i].value, sequence[i].width);
		if (ret)
			return ret;
	}

	return 0;
}

static int avl6381_i2c_rd_reg(struct avl6381_priv *priv, u32 addr, u32 *data,
			      int reg_size)
{
	int ret;
	u8 buf[3 + 4];
	u8 *p = buf;

	if (reg_size != 1 && reg_size != 2 && reg_size != 4)
		return -EINVAL;

	*(p++) = (u8)(addr >> 16);
	*(p++) = (u8)(addr >> 8);
	*(p++) = (u8)(addr);
	ret = avl6381_i2c_wr(priv, buf, 3);
	if (ret)
		return ret;

	ret = avl6381_i2c_rd(priv, buf, reg_size);
	if (ret)
		return ret;

	*data = 0;
	p = buf;

	switch (reg_size) {
	case 4:
		*data |= (u32)(*(p++)) << 24;
		*data |= (u32)(*(p++)) << 16;
		fallthrough;
	case 2:
		*data |= (u32)(*(p++)) << 8;
		fallthrough;
	case 1:
		*data |= (u32)*p;
		break;
	}
	return ret;
}

#define AVL6381_RD_REG8(_priv, _addr, _data) \
	avl6381_i2c_rd_reg(_priv, _addr, _data, 1)
#define AVL6381_RD_REG16(_priv, _addr, _data) \
	avl6381_i2c_rd_reg(_priv, _addr, _data, 2)
#define AVL6381_RD_REG32(_priv, _addr, _data) \
	avl6381_i2c_rd_reg(_priv, _addr, _data, 4)

static int avl6381_get_rxop_status(struct avl6381_priv *priv)
{
	u32 reg_data;
	int ret;

	ret = AVL6381_RD_REG32(priv, 0x000204, &reg_data);
	if (ret)
		return ret;

	return reg_data ? -EBUSY : 0;
}

static int avl6381_send_rxop(struct avl6381_priv *priv, int a2)
{
	int ret, tries;

	for (tries = 0; tries < 42; tries++) {
		ret = avl6381_get_rxop_status(priv);
		if (!ret)
			return AVL6381_WR_REG32(priv, 0x000204, (u32)a2 << 24);
		if (ret != -EBUSY)
			return ret;
		usleep_range(10000, 12000);
	}

	return -ETIMEDOUT;
}

static int avl6381_reset_digital_core(struct avl6381_priv *priv)
{
	int ret;

	ret = AVL6381_WR_REG32(priv, 0x38fffc, 0);
	if (ret)
		return ret;
	usleep_range(10000, 12000);

	return AVL6381_WR_REG32(priv, 0x38fffc, 1);
}

static int avl6381_check_chip_ready(struct avl6381_priv *priv)
{
	u32 reset, signature;
	int ret;

	ret = AVL6381_RD_REG32(priv, 0x110840, &reset);
	if (ret)
		return ret;
	ret = AVL6381_RD_REG32(priv, 0x0000a0, &signature);
	if (ret)
		return ret;

	return reset == 1 || signature != 0x5aa57ff7 ? -EAGAIN : 0;
}

static int avl6381_wait_rxop(struct avl6381_priv *priv, unsigned int tries,
			     unsigned int delay_ms)
{
	int ret;

	while (tries--) {
		ret = avl6381_get_rxop_status(priv);
		if (!ret)
			return 0;
		if (ret != -EBUSY)
			return ret;
		msleep(delay_ms);
	}

	return -ETIMEDOUT;
}

static int avl6381_wait_ready(struct avl6381_priv *priv, unsigned int tries,
			      unsigned int delay_ms)
{
	int ret;

	while (tries--) {
		ret = avl6381_check_chip_ready(priv);
		if (!ret)
			return 0;
		if (ret != -EAGAIN)
			return ret;
		msleep(delay_ms);
	}

	return -ETIMEDOUT;
}

static int avl6381_get_family_id(struct avl6381_priv *priv, u32 *fid)
{
	return AVL6381_RD_REG32(priv, 0x040000, fid);
}

static int avl6381_get_chip_id(struct avl6381_priv *priv, u32 *chipid)
{
	int ret;
	u32 fid;

	ret = avl6381_get_family_id(priv, &fid);
	if (ret)
		return ret;
	if (fid != 0x63814e24)
		return -ENODEV;

	ret = AVL6381_RD_REG32(priv, 0x108004, chipid);
	if (!ret)
		dev_info(&priv->i2c->dev, "AVL6381 family=0x%08x chip=0x%08x\n",
			 fid, *chipid);

	return ret;
}

static int avl6381_dtmb_set_spectrum_polarity(struct avl6381_priv *priv,
					      unsigned int a2)
{
	u8 data = a2 > 1 ? 0x02 : 0x00;
	int ret;

	ret = AVL6381_WR_REG8(priv, 0x000322, data);
	if (ret)
		return ret;

	return AVL6381_WR_REG32(priv, 0x000324,
				a2 == 1 ? 0x004c4b40 : 0xffb3b4c0);
}

static int avl6381_set_pll(struct avl6381_priv *priv, const u8 *pll_conf)
{
	struct {
		u32 reg;
		u32 value;
	} const pll_regs[] = {
		{ 0x1000c0, pll_conf[4] - 1 },	{ 0x1000c4, pll_conf[5] - 1 },
		{ 0x1000d4, pll_conf[6] },	{ 0x1000c8, pll_conf[7] - 1 },
		{ 0x100080, pll_conf[12] - 1 }, { 0x100084, pll_conf[13] - 1 },
		{ 0x100094, pll_conf[14] },	{ 0x100088, pll_conf[15] - 1 },
		{ 0x10008c, pll_conf[16] - 1 }, { 0x100090, pll_conf[17] - 1 },
	};
	u32 value;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(pll_regs); i++) {
		ret = AVL6381_WR_REG32(priv, pll_regs[i].reg,
				       pll_regs[i].value);
		if (ret)
			return ret;
	}
	ret = AVL6381_WR_REG32(priv, 0x100000, 0);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG32(priv, 0x100000, 1);
	if (ret)
		return ret;
	usleep_range(5000, 7000);
	value = (u32)pll_conf[20] | (u32)pll_conf[21] << 8 |
		(u32)pll_conf[22] << 16 | (u32)pll_conf[23] << 24;
	ret = AVL6381_WR_REG32(priv, 0x100018, value);
	if (ret)
		return ret;
	value = (u32)pll_conf[24] | (u32)pll_conf[25] << 8 |
		(u32)pll_conf[26] << 16 | (u32)pll_conf[27] << 24;
	ret = AVL6381_WR_REG32(priv, 0x10001c, value);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG32(priv, 0x100010, 1);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG32(priv, 0x100008, 1);
	if (ret)
		return ret;

	return AVL6381_WR_REG32(priv, 0x100008, 0);
}

static int avl6381_wr_firmware(struct avl6381_priv *priv, const u8 *data,
			       int len)
{
	int ret;
	u8 buf[50];
	int size, pos;
	u32 addr;

	ret = 0;
	pos = 3;
	addr = (data[0] << 16) + (data[1] << 8) + data[2];
	while (pos < len) {
		if ((len - pos) >= 47)
			size = 50;
		else
			size = len - pos + 3;
		buf[0] = (addr >> 16) & 0xFF;
		buf[1] = (addr >> 8) & 0xFF;
		buf[2] = addr & 0xFF;
		memcpy(buf + 3, data + pos, size - 3);
		ret = avl6381_i2c_wr(priv, buf, size);
		if (ret)
			break;
		pos += 47;
		addr += 47;
	}
	return ret;
}

static int avl6381_patch_new(struct avl6381_priv *priv)
{
	static const struct avl6381_reg_sequence post_patch[] = {
		AVL6381_REG32(0x000228, 0x00280000),
		AVL6381_REG32(0x00022c, 0x002d0008),
		AVL6381_REG32(0x000230, 0x0028cb00),
		AVL6381_REG32(0x000234, 0x002f2c08),
		AVL6381_REG8(0x000225, 0x01),
		AVL6381_REG8(0x000226, 0x01),
		AVL6381_REG16(0x2d0000, 0x0001),
		AVL6381_REG16(0x2d0002, 0x0000),
		AVL6381_REG32(0x0000a0, 0x00000000),
		AVL6381_REG32(0x110840, 0x00000000),
	};
	const u8 *data = avl6381_freeze_data_dtmb;
	size_t pos = 4, size = sizeof(avl6381_freeze_data_dtmb);
	u32 payload_len;
	int ret;

	while (pos + 8 <= size) {
		payload_len = (u32)data[pos] << 24 | (u32)data[pos + 1] << 16 |
			      (u32)data[pos + 2] << 8 | data[pos + 3];
		if (!payload_len)
			break;
		if (payload_len > size - pos - 8)
			return -EINVAL;
		ret = avl6381_wr_firmware(priv, &data[pos + 5],
					  payload_len + 3);
		if (ret)
			return ret;
		pos += payload_len + 8;
	}

	return avl6381_write_sequence(priv, post_patch, ARRAY_SIZE(post_patch));
}

static int avl6381_i2c_bypass_on(struct avl6381_priv *priv)
{
	return AVL6381_WR_REG32(priv, 0x11801c, 0x00000007);
}

static int avl6381_i2c_bypass_off(struct avl6381_priv *priv)
{
	return AVL6381_WR_REG32(priv, 0x11801c, 0x00000006);
}

static int avl6381_initialize_base(struct avl6381_priv *priv,
				   const u8 *pll_conf)
{
	int ret;

	ret = AVL6381_WR_REG32(priv, 0x110840, 0x00000001);
	if (ret)
		return ret;
	ret = avl6381_set_pll(priv, pll_conf);
	if (ret)
		return ret;
	ret = avl6381_reset_digital_core(priv);
	if (ret)
		return ret;

	return avl6381_patch_new(priv);
}

static int avl6381_dvbc_initialize_receiver(struct avl6381_priv *priv)
{
	static const struct avl6381_reg_sequence sequence[] = {
		AVL6381_REG32(0x000560, 0x0d59f800),
		AVL6381_REG32(0x0005a8, 0x0a037a00),
		AVL6381_REG32(0x00055c, 0x016e3600),
		AVL6381_REG32(0x000580, 0x004c4b40),
		AVL6381_REG32(0x000558, 0x0068e778),
	};
	int ret;

	ret = avl6381_send_rxop(priv, 1);
	if (ret)
		return ret;

	return avl6381_write_sequence(priv, sequence, ARRAY_SIZE(sequence));
}

static int avl6381_dvbc_initialize_adc(struct avl6381_priv *priv)
{
	static const struct avl6381_reg_sequence sequence[] = {
		AVL6381_REG8(0x00057d, 0x01),
		AVL6381_REG8(0x00057f, 0x01),
		AVL6381_REG8(0x00057c, 0x00),
		AVL6381_REG8(0x000747, 0x00),
	};

	return avl6381_write_sequence(priv, sequence, ARRAY_SIZE(sequence));
}

static int avl6381_dtmb_initialize_receiver(struct avl6381_priv *priv)
{
	static const struct avl6381_reg_sequence sequence[] = {
		AVL6381_REG32(0x000338, 0x11e1a300),
		AVL6381_REG32(0x000384, 0x0a037a00),
		AVL6381_REG32(0x00033c, 0x04c4b400),
		AVL6381_REG32(0x000304, 0x016e3600),
		AVL6381_REG8(0x000321, 0x01),
		AVL6381_REG8(0x000323, 0x01),
		AVL6381_REG8(0x000319, 0x00),
		AVL6381_REG8(0x00032b, 0x01),
		AVL6381_REG8(0x0000a6, 0x00),
	};
	int ret;

	ret = avl6381_send_rxop(priv, 1);
	if (ret)
		return ret;
	ret = avl6381_write_sequence(priv, sequence, ARRAY_SIZE(sequence));
	if (ret)
		return ret;

	return avl6381_dtmb_set_spectrum_polarity(priv, 1);
}

static int avl6381_dtmb_initialize_adc(struct avl6381_priv *priv)
{
	int ret;

	ret = AVL6381_WR_REG8(priv, 0x000320, 0x00);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG8(priv, 0x0004d7, 0x00);
	if (ret)
		return ret;

	return avl6381_send_rxop(priv, 9);
}

static int avl6381_initialize_sdram(struct avl6381_priv *priv)
{
	static const struct avl6381_reg_sequence sequence[] = {
		AVL6381_REG32(0x000210, 0x00070a00),
		AVL6381_REG32(0x000214, 0x05060600),
		AVL6381_REG32(0x000218, 0x03010301),
	};
	int ret;

	ret = avl6381_write_sequence(priv, sequence, ARRAY_SIZE(sequence));
	if (ret)
		return ret;

	return avl6381_send_rxop(priv, 8);
}

static int avl6381_initialize_receiver(struct avl6381_priv *priv,
				       enum fe_delivery_system delivery_system)
{
	int ret;

	if (delivery_system == SYS_DVBC_ANNEX_A) {
		ret = avl6381_dvbc_initialize_receiver(priv);
		if (ret)
			return ret;
		ret = avl6381_dvbc_initialize_adc(priv);
	} else if (delivery_system == SYS_DTMB) {
		ret = avl6381_dtmb_initialize_receiver(priv);
		if (ret)
			return ret;
		ret = avl6381_dtmb_initialize_adc(priv);
	} else {
		return -EINVAL;
	}
	if (ret)
		return ret;

	return avl6381_initialize_sdram(priv);
}

static int avl6381_dtmb_set_symbol_rate(struct avl6381_priv *priv,
					unsigned int symbol_rate)
{
	return AVL6381_WR_REG32(priv, 0x000300, symbol_rate);
}

static int avl6381_dtmb_set_mpeg_mode(struct avl6381_priv *priv)
{
	int ret;

	ret = AVL6381_WR_REG8(priv, 0x000352, 1);
	if (ret)
		return ret;

	return AVL6381_WR_REG8(priv, 0x000353, 1);
}

static int avl6381_dvbc_set_mpeg_mode(struct avl6381_priv *priv)
{
	int ret;

	ret = AVL6381_WR_REG8(priv, 0x00056e, 0);
	if (ret)
		return ret;

	return AVL6381_WR_REG8(priv, 0x00056f, 1);
}

static int avl6381_set_mpeg_mode(struct avl6381_priv *priv,
				 enum fe_delivery_system delivery_system)
{
	if (delivery_system == SYS_DTMB)
		return avl6381_dtmb_set_mpeg_mode(priv);
	if (delivery_system == SYS_DVBC_ANNEX_A)
		return avl6381_dvbc_set_mpeg_mode(priv);

	return -EINVAL;
}

static int avl6381_write_mode_reg8(struct avl6381_priv *priv,
				   enum fe_delivery_system delivery_system,
				   u32 dtmb_reg, u32 dvbc_reg, u8 value)
{
	if (delivery_system == SYS_DTMB)
		return AVL6381_WR_REG8(priv, dtmb_reg, value);
	if (delivery_system == SYS_DVBC_ANNEX_A)
		return AVL6381_WR_REG8(priv, dvbc_reg, value);

	return -EINVAL;
}

static int avl6381_set_mpeg_serial_pin(struct avl6381_priv *priv,
				       enum fe_delivery_system delivery_system)
{
	return avl6381_write_mode_reg8(priv, delivery_system, 0x000351,
				       0x00056d, 0);
}

static int
avl6381_set_mpeg_serial_order(struct avl6381_priv *priv,
			      enum fe_delivery_system delivery_system)
{
	return avl6381_write_mode_reg8(priv, delivery_system, 0x000350,
				       0x00056c, 0);
}

static int
avl6381_set_mpeg_serial_sync_pulse(struct avl6381_priv *priv,
				   enum fe_delivery_system delivery_system)
{
	return avl6381_write_mode_reg8(priv, delivery_system, 0x0004e6,
				       0x00074e, 0);
}

static int avl6381_set_mpeg_error_bit(struct avl6381_priv *priv,
				      enum fe_delivery_system delivery_system)
{
	return avl6381_write_mode_reg8(priv, delivery_system, 0x000378,
				       0x000578, 1);
}

static int
avl6381_set_mpeg_error_polarity(struct avl6381_priv *priv,
				enum fe_delivery_system delivery_system)
{
	return avl6381_write_mode_reg8(priv, delivery_system, 0x000354,
				       0x000570, 0);
}

static int
avl6381_set_mpeg_valid_polarity(struct avl6381_priv *priv,
				enum fe_delivery_system delivery_system)
{
	return avl6381_write_mode_reg8(priv, delivery_system, 0x0004e7,
				       0x00074f, 0);
}

static int
avl6381_set_mpeg_packet_length(struct avl6381_priv *priv,
			       enum fe_delivery_system delivery_system)
{
	return avl6381_write_mode_reg8(priv, delivery_system, 0x000357,
				       0x000573, 0);
}

static int avl6381_dtmb_disable_mpeg_continuous(struct avl6381_priv *priv)
{
	return AVL6381_WR_REG8(priv, 0x00038b, 0);
}

static int avl6381_enable_mpeg_output(struct avl6381_priv *priv)
{
	return AVL6381_WR_REG32(priv, 0x108030, 0x00000fff);
}

static int avl6381_initialize_tuner_i2c(struct avl6381_priv *priv,
					enum fe_delivery_system delivery_system)
{
	u32 divider, reg;
	int ret;

	if (delivery_system == SYS_DTMB)
		divider = 0x34;
	else if (delivery_system == SYS_DVBC_ANNEX_A)
		divider = 0x27;
	else
		return -EINVAL;

	ret = AVL6381_WR_REG32(priv, 0x118000, 0x01);
	if (ret)
		return ret;
	ret = avl6381_i2c_bypass_off(priv);
	if (ret)
		return ret;
	ret = AVL6381_RD_REG32(priv, 0x118004, &reg);
	if (ret)
		return ret;
	reg &= ~BIT(0);
	ret = AVL6381_WR_REG32(priv, 0x118004, reg);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG32(priv, 0x118018, divider);
	if (ret)
		return ret;

	return AVL6381_WR_REG32(priv, 0x118000, 0);
}

static int avl6381_set_agc_polarity(struct avl6381_priv *priv,
				    enum fe_delivery_system delivery_system)
{
	return avl6381_write_mode_reg8(priv, delivery_system, 0x00030b,
				       0x00059f, 0);
}

static int avl6381_enable_agc(struct avl6381_priv *priv)
{
	return AVL6381_WR_REG32(priv, 0x108034, 0x00000001);
}

static int avl6381_reset_per(struct avl6381_priv *priv,
			     enum fe_delivery_system delivery_system)
{
	u32 value;
	int ret;

	ret = AVL6381_RD_REG32(priv, 0x149104, &value);
	if (ret)
		return ret;
	value |= BIT(0);
	ret = AVL6381_WR_REG32(priv, 0x149104, value);
	if (ret)
		return ret;

	switch (delivery_system) {
	case SYS_DTMB:
		ret = AVL6381_WR_REG8(priv, 0x0000a5, 0x00);
		break;
	case SYS_DVBC_ANNEX_A:
		ret = AVL6381_WR_REG16(priv, 0x0001a2, 0x0000);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return ret;
	ret = AVL6381_RD_REG32(priv, 0x149104, &value);
	if (ret)
		return ret;
	value |= BIT(3);
	ret = AVL6381_WR_REG32(priv, 0x149104, value);
	if (ret)
		return ret;
	value |= BIT(0);
	ret = AVL6381_WR_REG32(priv, 0x149104, value);
	if (ret)
		return ret;
	value &= ~BIT(0);

	return AVL6381_WR_REG32(priv, 0x149104, value);
}

static int avl6381_reset_error_stats(struct avl6381_priv *priv,
				     enum fe_delivery_system delivery_system)
{
	u32 reg;
	int ret;

	ret = AVL6381_RD_REG32(priv, 0x149160, &reg);
	if (ret)
		return ret;
	if (reg == 1) {
		ret = AVL6381_WR_REG32(priv, 0x149128, 0);
		if (ret)
			return ret;
		ret = AVL6381_WR_REG32(priv, 0x149128, 1);
		if (ret)
			return ret;
		ret = AVL6381_WR_REG32(priv, 0x149128, 0);
	}
	if (ret)
		return ret;

	return avl6381_reset_per(priv, delivery_system);
}

static int
avl6381_initialize_error_stats(struct avl6381_priv *priv,
			       enum fe_delivery_system delivery_system)
{
	static const struct avl6381_reg_sequence sequence[] = {
		AVL6381_REG32(0x149160, 0x00000001),
		AVL6381_REG32(0x14912c, 0x00000001),
		AVL6381_REG32(0x149130, 0x0a037a00),
		AVL6381_REG32(0x149134, 0x00000000),
		AVL6381_REG32(0x149138, 0x00000000),
		AVL6381_REG32(0x14913c, 0x00000000),
	};
	int ret;

	ret = avl6381_write_sequence(priv, sequence, ARRAY_SIZE(sequence));
	if (ret)
		return ret;

	return avl6381_reset_error_stats(priv, delivery_system);
}

static int avl6381_configure_mode(struct avl6381_priv *priv,
				  enum fe_delivery_system delivery_system)
{
	int ret;

	ret = avl6381_initialize_receiver(priv, delivery_system);
	if (ret)
		return ret;
	if (delivery_system == SYS_DTMB) {
		ret = avl6381_dtmb_set_symbol_rate(priv, 7560000);
		if (ret)
			return ret;
	}
	ret = avl6381_set_mpeg_mode(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_set_mpeg_serial_pin(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_set_mpeg_serial_order(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_set_mpeg_serial_sync_pulse(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_set_mpeg_error_bit(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_set_mpeg_error_polarity(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_set_mpeg_valid_polarity(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_set_mpeg_packet_length(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_dtmb_disable_mpeg_continuous(priv);
	if (ret)
		return ret;
	ret = avl6381_enable_mpeg_output(priv);
	if (ret)
		return ret;
	ret = avl6381_initialize_tuner_i2c(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_set_agc_polarity(priv, delivery_system);
	if (ret)
		return ret;
	ret = avl6381_enable_agc(priv);
	if (ret)
		return ret;
	ret = avl6381_initialize_error_stats(priv, delivery_system);
	if (ret)
		return ret;

	return AVL6381_WR_REG32(priv, 0x0006f4, 0x0a);
}

static int avl6381_initialize(struct avl6381_priv *priv)
{
	u32 chipid;
	int ret;

	if (priv->inited)
		return 0;

	ret = avl6381_get_chip_id(priv, &chipid);
	if (ret)
		return ret;
	priv->delivery_system = SYS_DVBC_ANNEX_A;
	ret = avl6381_initialize_base(priv, avl6381_pll_config[5]);
	if (ret)
		return ret;
	msleep(20);
	ret = avl6381_wait_ready(priv, 20, 20);
	if (ret)
		return ret;
	ret = avl6381_configure_mode(priv, SYS_DVBC_ANNEX_A);
	if (ret)
		return ret;

	priv->inited = true;
	return 0;
}

static int avl6381_dtmb_get_lock_status(struct avl6381_priv *priv, u32 *status)
{
	u32 data;
	int ret;

	ret = AVL6381_RD_REG8(priv, 0x0000a6, &data);
	if (!ret)
		*status = data;

	return ret;
}

static int avl6381_dvbc_get_lock_status(struct avl6381_priv *priv, u32 *status)
{
	u32 data;
	int ret;

	*status = 0;
	ret = AVL6381_RD_REG32(priv, 0x0001a4, &data);
	if (!ret && data == 21)
		*status = 1;

	return ret;
}

static int avl6381_get_lock_status(struct avl6381_priv *priv, u32 *status)
{
	int ret = -EINVAL;

	*status = 0;

	switch (priv->delivery_system) {
	case SYS_DTMB:
		ret = avl6381_dtmb_get_lock_status(priv, status);
		break;
	case SYS_DVBC_ANNEX_A:
		ret = avl6381_dvbc_get_lock_status(priv, status);
		break;
	default:
		break;
	}

	return ret;
}

static int avl6381_dtmb_get_snr(struct avl6381_priv *priv, u32 *snr)
{
	return AVL6381_RD_REG16(priv, 0x00011c, snr);
}

static int avl6381_dvbc_get_snr(struct avl6381_priv *priv, u32 *snr)
{
	int v9, ret;

	ret = AVL6381_RD_REG32(priv, 0x0005d8, &v9);
	if (ret || v9)
		return ret;
	ret = AVL6381_RD_REG16(priv, 0x0001ae, snr);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG32(priv, 0x0005d8, 0x00000001);
	if (!ret)
		msleep(50);

	return ret;
}

static int avl6381_get_snr(struct avl6381_priv *priv, u32 *snr)
{
	int ret = -EINVAL;

	switch (priv->delivery_system) {
	case SYS_DTMB:
		ret = avl6381_dtmb_get_snr(priv, snr);
		break;
	case SYS_DVBC_ANNEX_A:
		ret = avl6381_dvbc_get_snr(priv, snr);
		break;
	default:
		break;
	}

	return ret;
}

static s64 avl6381_qam_get_snr(struct avl6381_priv *priv)
{
	u32 snr = 0;
	int ret;

	ret = avl6381_get_snr(priv, &snr);
	return ret ? ret : snr;
}

static int avl6381_dvbc_halt(struct avl6381_priv *priv)
{
	u32 v6;
	int ret;

	ret = AVL6381_RD_REG32(priv, 0x0001a0, &v6);
	if (ret)
		return ret;

	return AVL6381_WR_REG32(priv, 0x0001a0, v6 & ~BIT(0));
}

static int avl6381_dtmb_halt(struct avl6381_priv *priv)
{
	u32 v6;
	int ret;

	ret = AVL6381_RD_REG32(priv, 0x0000a4, &v6);
	if (ret)
		return ret;

	return AVL6381_WR_REG32(priv, 0x0000a4, v6 & ~BIT(0));
}

static int avl6381_halt(struct avl6381_priv *priv,
			enum fe_delivery_system delivery_system)
{
	int ret;

	ret = avl6381_send_rxop(priv, 3);
	if (ret)
		return ret;
	usleep_range(2000, 4000);
	switch (delivery_system) {
	case SYS_DVBC_ANNEX_A:
		ret = avl6381_dvbc_halt(priv);
		break;
	case SYS_DTMB:
		ret = avl6381_dtmb_halt(priv);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int avl6381_dtmb_get_running_level(struct avl6381_priv *priv, u32 *level)
{
	int ret;
	u32 v5;

	ret = AVL6381_RD_REG8(priv, 0x000124, &v5);
	if (!ret && v5)
		*level = 2;
	else
		*level = 0;

	return ret;
}

static int avl6381_dvbc_get_running_level(struct avl6381_priv *priv, u32 *level)
{
	int ret;
	u32 v5;

	ret = AVL6381_RD_REG32(priv, 0x0001a4, &v5);
	if (ret)
		return ret;
	if (v5)
		*level = 2;
	else
		*level = 0;

	return ret;
}

static int avl6381_get_running_level(struct avl6381_priv *priv,
				     enum fe_delivery_system delivery_system,
				     u32 *level)
{
	int ret = -EINVAL;

	switch (delivery_system) {
	case SYS_DTMB:
		ret = avl6381_dtmb_get_running_level(priv, level);
		break;
	case SYS_DVBC_ANNEX_A:
		ret = avl6381_dvbc_get_running_level(priv, level);
		break;
	default:
		break;
	}

	return ret;
}

static int avl6381_dtmb_auto_lock_channel(struct avl6381_priv *priv)
{
	int ret;

	ret = AVL6381_WR_REG8(priv, 0x00021f, 0x01);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG8(priv, 0x0000ad, 0x00);
	if (ret)
		return ret;
	ret = AVL6381_WR_REG32(priv, 0x00010c, 0x00000000);
	if (ret)
		return ret;

	return avl6381_send_rxop(priv, 2);
}

static int avl6381_auto_lock_channel(struct avl6381_priv *priv,
				     enum fe_delivery_system delivery_system)
{
	int ret = -EINVAL;

	switch (delivery_system) {
	case SYS_DTMB:
		ret = avl6381_dtmb_auto_lock_channel(priv);
		break;
	case SYS_DVBC_ANNEX_A:
		ret = avl6381_send_rxop(priv, 12);
		break;
	default:
		break;
	}

	return ret;
}

static int avl6381_auto_lock(struct avl6381_priv *priv,
			     enum fe_delivery_system delivery_system)
{
	u32 level = 2;
	int ret, tries;

	ret = avl6381_halt(priv, delivery_system);
	if (ret)
		return ret;

	for (tries = 0; tries < 10; tries++) {
		ret = avl6381_get_running_level(priv, delivery_system, &level);
		if (ret)
			return ret;
		if (!level)
			return avl6381_auto_lock_channel(priv, delivery_system);
		usleep_range(10000, 12000);
	}

	return -ETIMEDOUT;
}

static int avl6381_get_mode(struct avl6381_priv *priv, u32 *mode)
{
	u32 value;
	int ret;

	ret = AVL6381_RD_REG32(priv, 0x000200, &value);
	if (!ret)
		*mode = value;

	return ret;
}

static int avl6381_set_mode(struct avl6381_priv *priv, enum avl6381_mode mode)
{
	enum fe_delivery_system delivery_system;
	u32 current_mode;
	int ret;

	switch (mode) {
	case MODE_DTMB:
		delivery_system = SYS_DTMB;
		break;
	case MODE_DVBC:
		delivery_system = SYS_DVBC_ANNEX_A;
		break;
	default:
		return -EINVAL;
	}

	ret = avl6381_get_mode(priv, &current_mode);
	if (ret)
		return ret;
	if (current_mode != mode) {
		ret = avl6381_halt(priv, delivery_system);
		if (ret)
			return ret;
		ret = avl6381_wait_rxop(priv, 22, 20);
		if (ret)
			return ret;
		ret = AVL6381_WR_REG32(priv, 0x110084, 0);
		if (ret)
			return ret;
		usleep_range(10000, 12000);
		ret = AVL6381_WR_REG32(priv, 0x110084, 1);
		if (ret)
			return ret;
		ret = AVL6381_WR_REG32(priv, 0x0000a0, 0);
		if (ret)
			return ret;
		ret = avl6381_send_rxop(priv, 10);
		if (ret)
			return ret;
		ret = avl6381_wait_rxop(priv, 202, 20);
		if (ret)
			return ret;
		ret = avl6381_wait_ready(priv, 22, 20);
		if (ret)
			return ret;
		ret = AVL6381_WR_REG32(priv, 0x110840, 1);
		if (ret)
			return ret;
		if (delivery_system == SYS_DTMB)
			ret = avl6381_set_pll(priv, avl6381_pll_config[4]);
		else
			ret = avl6381_set_pll(priv, avl6381_pll_config[5]);
		if (ret)
			return ret;
		msleep(20);
		ret = AVL6381_WR_REG32(priv, 0x110840, 0);
		if (ret)
			return ret;

		msleep(20);
		ret = avl6381_wait_ready(priv, 200, 20);
		if (ret)
			return ret;

		return avl6381_configure_mode(priv, delivery_system);
	}

	return 0;
}

static int avl6381_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct avl6381_priv *priv = fe->demodulator_priv;
	int i, ret;

	dev_dbg(&priv->i2c->dev, "%s: %d\n", __func__, enable);

	for (i = 0; i < 5; i++) {
		if (enable)
			ret = avl6381_i2c_bypass_on(priv);
		else
			ret = avl6381_i2c_bypass_off(priv);
		if (ret)
			return ret;
	}

	return 0;
}

static int avl6381_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct avl6381_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u32 locked = 0, reg1, reg2;
	u16 strength;
	s64 snr;
	int ret;

	mutex_lock(&priv->mutex);
	*status = 0;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	ret = avl6381_get_lock_status(priv, &locked);
	if (ret || !locked)
		goto out;

	*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
		  FE_HAS_SYNC | FE_HAS_LOCK;

	if (fe->ops.tuner_ops.get_rf_strength) {
		ret = fe->ops.tuner_ops.get_rf_strength(fe, &strength);
		if (ret)
			goto out;
		c->strength.stat[0].scale = FE_SCALE_DECIBEL;
		/* The MxL603 reports the magnitude of a signed 0.01 dBm value. */
		c->strength.stat[0].svalue = -(s64)strength * 10;
	}

	snr = avl6381_qam_get_snr(priv);
	if (snr >= 0) {
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		c->cnr.stat[0].svalue = snr * 10;
	}

	ret = AVL6381_RD_REG32(priv, 0x149110, &reg1);
	if (ret)
		goto out;
	ret = AVL6381_RD_REG32(priv, 0x149114, &reg2);
	if (ret)
		goto out;
	c->pre_bit_error.stat[0].scale = FE_SCALE_COUNTER;
	c->pre_bit_error.stat[0].uvalue = reg1;
	c->pre_bit_count.stat[0].scale = FE_SCALE_COUNTER;
	c->pre_bit_count.stat[0].uvalue = reg2;

out:
	mutex_unlock(&priv->mutex);

	return ret;
}

static int avl6381_set_frontend(struct dvb_frontend *fe)
{
	struct avl6381_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;

	mutex_lock(&priv->mutex);

	if (priv->config->tuner_select_input) {
		ret = priv->config->tuner_select_input(fe, c->delivery_system);
		if (ret)
			goto out;
	}

	switch (c->delivery_system) {
	case SYS_DTMB:
		ret = avl6381_set_mode(priv, MODE_DTMB);
		break;
	case SYS_DVBC_ANNEX_A:
		ret = avl6381_set_mode(priv, MODE_DVBC);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret)
		goto out;

	if (fe->ops.tuner_ops.set_params)
		ret = fe->ops.tuner_ops.set_params(fe);
	if (ret)
		goto out;

	if (c->delivery_system == SYS_DTMB)
		ret = avl6381_dtmb_set_symbol_rate(priv, 7560000);
	if (ret)
		goto out;

	ret = avl6381_auto_lock(priv, c->delivery_system);

	if (!ret)
		priv->delivery_system = c->delivery_system;
out:
	mutex_unlock(&priv->mutex);

	return ret;
}

static int avl6381_init(struct dvb_frontend *fe)
{
	struct avl6381_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0;

	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	c->cnr.len = 1;
	c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	c->block_error.len = 1;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_error.len = 1;
	c->pre_bit_error.stat[0].scale = FE_SCALE_COUNTER;
	c->pre_bit_count.len = 1;
	c->pre_bit_count.stat[0].scale = FE_SCALE_COUNTER;
	c->block_error.len = 1;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_count.len = 1;
	c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	ret = avl6381_initialize(priv);

	return ret;
}

static void avl6381_release(struct dvb_frontend *fe)
{
	struct avl6381_priv *priv = fe->demodulator_priv;

	mutex_destroy(&priv->mutex);
	kfree(priv);
}

static const struct dvb_frontend_ops avl6381_ops = {
	.delsys = { SYS_DTMB, SYS_DVBC_ANNEX_A },
	.info = { .name = "Availink AVL6381 DTMB/DVB-C demodulator",
		  .frequency_min_hz = 42 * MHz,
		  .frequency_max_hz = 858 * MHz,
		  .frequency_stepsize_hz = 0,
		  .frequency_tolerance_hz = 0,
		  .symbol_rate_min = 1000000,
		  .symbol_rate_max = 45000000,
		  .caps = FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			  FE_CAN_FEC_5_6 | FE_CAN_FEC_6_7 | FE_CAN_FEC_7_8 |
			  FE_CAN_FEC_AUTO | FE_CAN_QPSK | FE_CAN_QAM_16 |
			  FE_CAN_QAM_32 | FE_CAN_QAM_64 | FE_CAN_QAM_128 |
			  FE_CAN_QAM_256 | FE_CAN_QAM_AUTO |
			  FE_CAN_TRANSMISSION_MODE_AUTO |
			  FE_CAN_GUARD_INTERVAL_AUTO | FE_CAN_HIERARCHY_AUTO |
			  FE_CAN_MUTE_TS | FE_CAN_MULTISTREAM |
			  FE_CAN_INVERSION_AUTO | FE_CAN_RECOVER |
			  FE_HAS_EXTENDED_CAPS },

	.release = avl6381_release,
	.init = avl6381_init,
	.i2c_gate_ctrl = avl6381_i2c_gate_ctrl,
	.read_status = avl6381_read_status,
	.set_frontend = avl6381_set_frontend,
};

struct dvb_frontend *avl6381_attach(const struct avl6381_config *config,
				    struct i2c_adapter *i2c)
{
	struct avl6381_priv *priv;
	int ret;
	u32 id, fid;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	priv->frontend.ops = avl6381_ops;

	priv->frontend.demodulator_priv = priv;
	priv->config = config;
	priv->i2c = i2c;
	priv->delivery_system = SYS_UNDEFINED;
	priv->inited = false;
	mutex_init(&priv->mutex);

	ret = avl6381_get_family_id(priv, &fid);
	if (ret)
		goto err;
	if (fid != 0x63814e24) {
		dev_err(&i2c->dev, "unsupported AVL family 0x%08x\n", fid);
		goto err;
	}

	ret = AVL6381_RD_REG32(priv, 0x108004, &id);
	if (ret)
		goto err;

	dev_info(&i2c->dev, "found AVL6381 family=0x%08x chip=0x%08x\n", fid,
		 id);
	return &priv->frontend;

err:
	mutex_destroy(&priv->mutex);
	kfree(priv);
	return NULL;
}
EXPORT_SYMBOL_GPL(avl6381_attach);

MODULE_DESCRIPTION("Availink avl6381 DVB demodulator driver");
MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_LICENSE("GPL");
