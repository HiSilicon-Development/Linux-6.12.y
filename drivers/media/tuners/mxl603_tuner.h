/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef MXL603_TUNER_H
#define MXL603_TUNER_H

#include <media/dvb_frontend.h>

#include "mxl603_api.h"

struct dvb_frontend *mxl603_attach(struct dvb_frontend *fe,
				   struct i2c_adapter *i2c, u8 addr,
				   const struct mxl603_config *config);

#endif
