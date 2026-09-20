/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Availink avl6381 demod driver
 *
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#ifndef AVL6381_H
#define AVL6381_H

#include <media/dvb_frontend.h>

struct avl6381_config {
	u8 demod_address;
	u8 tuner_address;
	int (*tuner_select_input)(struct dvb_frontend *fe,
				  enum fe_delivery_system delivery_system);
};

struct dvb_frontend *avl6381_attach(const struct avl6381_config *config,
				    struct i2c_adapter *i2c);

#endif /* AVL6381_H */
