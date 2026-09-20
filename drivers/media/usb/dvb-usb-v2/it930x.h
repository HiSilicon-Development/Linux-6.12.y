/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ITE IT9303 USB bridge driver
 *
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#ifndef __IT930X_H
#define __IT930X_H

#include <linux/types.h>

#define IT9303_FIRMWARE "dvb-usb-it9303-01.fw"

enum it930x_command {
	IT930X_CMD_MEM_RD = 0x00,
	IT930X_CMD_MEM_WR = 0x01,
	IT930X_CMD_FW_QUERYINFO = 0x22,
	IT930X_CMD_FW_BOOT = 0x23,
	IT930X_CMD_FW_SCATTER_WR = 0x29,
	IT930X_CMD_GENERIC_I2C_RD = 0x2a,
	IT930X_CMD_GENERIC_I2C_WR = 0x2b,
};

struct it930x_request {
	u8 command;
	u8 mailbox;
	u8 write_length;
	const u8 *write_buffer;
	u8 read_length;
	u8 *read_buffer;
};

struct it930x_state {
	u8 control_buffer[255];
	u8 sequence;
	u8 prechip_version;
	u8 chip_version;
	u16 chip_type;
};

enum it930x_gpio {
	IT930X_GPIO1,
	IT930X_GPIO2,
	IT930X_GPIO3,
	IT930X_GPIO4,
	IT930X_GPIO5,
	IT930X_GPIO6,
	IT930X_GPIO7,
	IT930X_GPIO8,
	IT930X_GPIO9,
	IT930X_GPIO10,
	IT930X_GPIO11,
	IT930X_GPIO12,
	IT930X_GPIO13,
	IT930X_GPIO14,
	IT930X_GPIO15,
	IT930X_GPIO16,
	IT930X_GPIO_COUNT,
};

#endif /* __IT930X_H */
