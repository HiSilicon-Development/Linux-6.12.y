/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Availink avl6381 demod driver
 *
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#ifndef AVL6381_PRIV_H
#define AVL6381_PRIV_H

#include <media/dvb_frontend.h>

#include "avl6381_freezeData_DTMB.h"

struct avl6381_priv {
	struct i2c_adapter *i2c;
	const struct avl6381_config *config;
	struct dvb_frontend frontend;
	enum fe_delivery_system delivery_system;
	bool inited;
	/* Protects demodulator and downstream tuner transactions. */
	struct mutex mutex;
};

#endif
