// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MaxLinear MxL603 tuner driver
 *
 * Copyright (C) 2024 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include <linux/slab.h>

#include "mxl603_tuner.h"

struct mxl603_state {
	struct mxl603_config config;
	struct i2c_adapter *i2c;
	u8 addr;
	u32 frequency;
	u32 bandwidth;
};

static int mxl603_i2c_gate_ctrl(struct dvb_frontend *fe, bool enable)
{
	if (!fe->ops.i2c_gate_ctrl)
		return 0;

	return fe->ops.i2c_gate_ctrl(fe, enable);
}

static int mxl603_i2c_gate_close(struct dvb_frontend *fe, int ret)
{
	int close_ret;

	close_ret = mxl603_i2c_gate_ctrl(fe, false);
	return ret ? ret : close_ret;
}

static int mxl603_get_status(struct dvb_frontend *fe, u32 *status)
{
	struct mxl603_state *state = fe->tuner_priv;
	bool rf_locked, ref_locked;
	int ret;

	*status = 0;
	ret = mxl603_i2c_gate_ctrl(fe, true);
	if (ret)
		return ret;

	ret = mxl603_get_lock(state->i2c, state->addr, &rf_locked, &ref_locked);
	if (!ret && rf_locked && ref_locked)
		*status = TUNER_STATUS_LOCKED;

	return mxl603_i2c_gate_close(fe, ret);
}

static int mxl603_get_rf_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct mxl603_state *state = fe->tuner_priv;
	s16 dbm_x100;
	int ret;

	*strength = 0;
	ret = mxl603_i2c_gate_ctrl(fe, true);
	if (ret)
		return ret;

	ret = mxl603_get_rf_power(state->i2c, state->addr, &dbm_x100);
	ret = mxl603_i2c_gate_close(fe, ret);
	if (!ret)
		*strength = clamp_t(int, -dbm_x100, 0, U16_MAX);

	return ret;
}

static int mxl603_bandwidth(struct dtv_frontend_properties *c,
			    enum mxl603_signal_mode mode,
			    enum mxl603_bandwidth *bandwidth)
{
	switch (c->bandwidth_hz) {
	case 6000000:
		*bandwidth = mode == MXL603_MODE_DVBC ? MXL603_CABLE_BW_6MHZ :
							MXL603_TERR_BW_6MHZ;
		break;
	case 7000000:
		*bandwidth = mode == MXL603_MODE_DVBC ? MXL603_CABLE_BW_7MHZ :
							MXL603_TERR_BW_7MHZ;
		break;
	case 0:
	case 8000000:
		*bandwidth = mode == MXL603_MODE_DVBC ? MXL603_CABLE_BW_8MHZ :
							MXL603_TERR_BW_8MHZ;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mxl603_set_params(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct mxl603_state *state = fe->tuner_priv;
	enum mxl603_bandwidth bandwidth;
	enum mxl603_signal_mode mode;
	int ret;

	if (c->delivery_system == SYS_DVBC_ANNEX_A)
		mode = MXL603_MODE_DVBC;
	else if (c->delivery_system == SYS_DTMB)
		mode = MXL603_MODE_DTMB;
	else
		return -EINVAL;

	ret = mxl603_bandwidth(c, mode, &bandwidth);
	if (ret)
		return ret;
	ret = mxl603_i2c_gate_ctrl(fe, true);
	if (ret)
		return ret;

	ret = mxl603_set_if_output(state->i2c, state->addr, &state->config);
	if (!ret)
		ret = mxl603_set_mode(state->i2c, state->addr, &state->config,
				      mode);
	if (!ret)
		ret = mxl603_set_frequency(state->i2c, state->addr,
					   c->frequency, bandwidth, mode);
	ret = mxl603_i2c_gate_close(fe, ret);
	if (ret)
		return ret;

	state->frequency = c->frequency;
	state->bandwidth = c->bandwidth_hz ?: 8000000;
	return 0;
}

static int mxl603_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct mxl603_state *state = fe->tuner_priv;

	*frequency = state->frequency;
	return 0;
}

static int mxl603_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct mxl603_state *state = fe->tuner_priv;

	*bandwidth = state->bandwidth;
	return 0;
}

static int mxl603_tuner_init(struct dvb_frontend *fe)
{
	struct mxl603_state *state = fe->tuner_priv;
	int ret;

	ret = mxl603_i2c_gate_ctrl(fe, true);
	if (ret)
		return ret;
	ret = mxl603_initialize(state->i2c, state->addr, &state->config);

	return mxl603_i2c_gate_close(fe, ret);
}

static int mxl603_sleep(struct dvb_frontend *fe)
{
	struct mxl603_state *state = fe->tuner_priv;
	int ret;

	ret = mxl603_i2c_gate_ctrl(fe, true);
	if (ret)
		return ret;
	ret = mxl603_set_power(state->i2c, state->addr, MXL603_POWER_STANDBY);

	return mxl603_i2c_gate_close(fe, ret);
}

static void mxl603_release(struct dvb_frontend *fe)
{
	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
}

static const struct dvb_tuner_ops mxl603_tuner_ops = {
	.info = {
		.name = "MaxLinear MxL603",
		.frequency_min_hz = 44 * MHz,
		.frequency_max_hz = 1002 * MHz,
		.frequency_step_hz = 25 * kHz,
	},
	.init = mxl603_tuner_init,
	.sleep = mxl603_sleep,
	.set_params = mxl603_set_params,
	.get_status = mxl603_get_status,
	.get_rf_strength = mxl603_get_rf_strength,
	.get_frequency = mxl603_get_frequency,
	.get_bandwidth = mxl603_get_bandwidth,
	.release = mxl603_release,
};

struct dvb_frontend *mxl603_attach(struct dvb_frontend *fe,
				   struct i2c_adapter *i2c, u8 addr,
				   const struct mxl603_config *config)
{
	struct mxl603_state *state;
	struct mxl603_version version;
	int ret;

	if (!config)
		return NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	state->config = *config;
	state->i2c = i2c;
	state->addr = addr;

	ret = mxl603_i2c_gate_ctrl(fe, true);
	if (ret)
		goto err_free;
	ret = mxl603_reset(i2c, addr);
	if (!ret)
		ret = mxl603_get_version(i2c, addr, &version);
	ret = mxl603_i2c_gate_close(fe, ret);
	if (ret)
		goto err_free;

	dev_info(&i2c->dev, "MxL603 detected, id=0x%02x revision=0x%02x\n",
		 version.chip_id, version.revision);
	fe->tuner_priv = state;
	fe->ops.tuner_ops = mxl603_tuner_ops;
	return fe;

err_free:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL_GPL(mxl603_attach);

MODULE_DESCRIPTION("MaxLinear MxL603 silicon tuner driver");
MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_LICENSE("GPL");
