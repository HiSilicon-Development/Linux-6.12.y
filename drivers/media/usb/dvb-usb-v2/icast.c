// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SMIT iCast DTMB/DVB-C USB driver
 *
 * Copyright (C) 2026 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include <linux/unaligned.h>

#include "dvb_usb.h"

#define ICAST_USB_VENDOR 0x29df
#define ICAST_USB_PRODUCT 0x0001

#define ICAST_COMMAND_ENDPOINT 0x01
#define ICAST_RESPONSE_ENDPOINT 0x82
#define ICAST_STREAM_ENDPOINT 0x84
#define ICAST_USB_TIMEOUT_MS 500
#define ICAST_RESPONSE_SIZE 1004
#define ICAST_HEADER_SIZE 4
#define ICAST_MAX_PAYLOAD U8_MAX

#define ICAST_TARGET_READY 0xa0
#define ICAST_TARGET_STATUS 0x81

struct icast_state {
	struct dvb_usb_device *d;
	struct dvb_frontend frontend;
	/* Serializes the command/response protocol on the shared endpoints. */
	struct mutex protocol_lock;
	u8 transfer_buffer[ICAST_RESPONSE_SIZE];
	u32 frequency;
	u32 symbol_rate;
	enum fe_modulation modulation;
	u8 signal_strength;
	u8 snr;
	bool locked;
	bool supports_dtmb;
};

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static const u8 icast_init_ci[] = { 0x01 };
static const u8 icast_init_profile_1[] = {
	0x01, 0x92, 0x07, 0x00, 0x00, 0x01, 0x00, 0x41, 0x00, 0x01,
};

static const u8 icast_init_register_10[] = {
	0x01, 0x90, 0x02, 0x00, 0x01, 0x9f, 0x80, 0x10, 0x00,
};

static const u8 icast_init_register_12[] = {
	0x01, 0x90, 0x02, 0x00, 0x01, 0x9f, 0x80, 0x12, 0x00,
};

static const u8 icast_init_resources[] = {
	0x01, 0x90, 0x02, 0x00, 0x01, 0x9f, 0x80, 0x11, 0x1c, 0x00,
	0x01, 0x00, 0x41, 0x00, 0x02, 0x00, 0x41, 0x00, 0x03, 0x00,
	0x41, 0x00, 0x24, 0x00, 0x41, 0x00, 0x40, 0x00, 0x41, 0x00,
	0x96, 0x10, 0x01, 0x00, 0x20, 0x00, 0x41,
};

static const u8 icast_init_profile_2[] = {
	0x01, 0x92, 0x07, 0x00, 0x00, 0x02, 0x00, 0x41, 0x00, 0x02,
};

static const u8 icast_init_register_20[] = {
	0x01, 0x90, 0x02, 0x00, 0x02, 0x9f, 0x80, 0x20, 0x00,
};

static const u8 icast_init_sas_3[] = {
	0x01, 0x92, 0x07, 0x00, 0x00, 0x96, 0x10, 0x01, 0x00, 0x03,
};

static const u8 icast_init_handshake[] = {
	0x01, 0x90, 0x02, 0x00, 0x03, 0x9f, 0x9a, 0x00, 0x08,
	'S',  'M',  'i',  'T',	'Z',  'B',  'J',  'L',
};

static const u8 icast_init_sas_4[] = {
	0x01, 0x92, 0x07, 0x00, 0x00, 0x96, 0x10, 0x01, 0x00, 0x04,
};

static const u8 icast_query_hardware[] = {
	0x01, 0x90, 0x02, 0x00, 0x03, 0x9f, 0x9a, 0x07, 0x0b, 0x07,
	0x00, 0x08, 0x10, 0x07, 0x00, 0x04, 'G',  'D',	'H',  'W',
};

static const u8 icast_init_dvbc_profile[] = {
	0x01, 0x92, 0x07, 0x00, 0x00, 0x03, 0x00, 0x41, 0x00, 0x04,
};

static const u8 icast_init_dvbc_register[] = {
	0x01, 0x90, 0x02, 0x00, 0x04, 0x9f, 0x80, 0x30, 0x00,
};

static const u8 icast_init_sas_5[] = {
	0x01, 0x92, 0x07, 0x00, 0x00, 0x96, 0x10, 0x01, 0x00, 0x05,
};

static const u8 icast_default_tune[] = {
	0x01, 0x90, 0x02, 0x00, 0x03, 0x9f, 0x9a, 0x07, 0x17, 0x00, 0x00,
	0x14, 0x00, 0x03, 0x00, 0x10, 0xb8, 0xf3, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
};

static const u8 icast_status_request[] = {
	0x01, 0x90, 0x02, 0x00, 0x03, 0x9f, 0x9a, 0x07, 0x0b, 0x01,
	0x00, 0x08, 0x00, 0x05, 0x00, 0x04, 'G',  'S',	'T',  'A',
};

static int icast_bulk_out_locked(struct icast_state *state, const u8 *data,
				 size_t length)
{
	int actual_length, ret;

	if (length > sizeof(state->transfer_buffer))
		return -EMSGSIZE;

	memcpy(state->transfer_buffer, data, length);
	ret = usb_bulk_msg(state->d->udev,
			   usb_sndbulkpipe(state->d->udev,
					   ICAST_COMMAND_ENDPOINT),
			   state->transfer_buffer, length, &actual_length,
			   ICAST_USB_TIMEOUT_MS);
	if (ret)
		return ret;

	return actual_length == length ? 0 : -EREMOTEIO;
}

static int icast_bulk_in_locked(struct icast_state *state, size_t length)
{
	int actual_length, ret;

	if (length > sizeof(state->transfer_buffer))
		return -EMSGSIZE;

	ret = usb_bulk_msg(state->d->udev,
			   usb_rcvbulkpipe(state->d->udev,
					   ICAST_RESPONSE_ENDPOINT),
			   state->transfer_buffer, length, &actual_length,
			   ICAST_USB_TIMEOUT_MS);
	return ret ? ret : actual_length;
}

static int icast_validate_response(struct icast_state *state, int length)
{
	u8 payload_length;

	if (length < ICAST_HEADER_SIZE || state->transfer_buffer[0] != 0x01)
		return -EPROTO;

	payload_length = state->transfer_buffer[3];
	if (payload_length > length - ICAST_HEADER_SIZE)
		return -EMSGSIZE;

	return payload_length;
}

static int icast_send_frame_locked(struct icast_state *state, u8 target,
				   const u8 *payload, u8 payload_length)
{
	u8 frame[ICAST_HEADER_SIZE + ICAST_MAX_PAYLOAD];

	frame[0] = 0x01;
	frame[1] = 0x00;
	frame[2] = target;
	frame[3] = payload_length;
	if (payload_length)
		memcpy(frame + ICAST_HEADER_SIZE, payload, payload_length);

	return icast_bulk_out_locked(state, frame,
				     ICAST_HEADER_SIZE + payload_length);
}

static int icast_wait_ready_locked(struct icast_state *state,
				   unsigned int attempts)
{
	static const u8 ready_payload[] = { 0x01 };
	unsigned int attempt;
	int length, payload_length, ret;

	for (attempt = 0; attempt <= attempts; attempt++) {
		ret = icast_send_frame_locked(state, ICAST_TARGET_READY,
					      ready_payload,
					      sizeof(ready_payload));
		if (ret)
			return ret;
		length = icast_bulk_in_locked(state, ICAST_RESPONSE_SIZE);
		if (length < 0)
			return length;
		payload_length = icast_validate_response(state, length);
		if (payload_length < 0)
			return payload_length;
		if (payload_length > 1 && state->transfer_buffer[5] == 0x80)
			return 0;
		if (payload_length > 1 && state->transfer_buffer[5])
			return -EREMOTEIO;
		if (!attempts)
			return 0;
		usleep_range((attempt / 2 + 3) * USEC_PER_MSEC,
			     (attempt / 2 + 4) * USEC_PER_MSEC);
	}

	return -ETIMEDOUT;
}

static bool icast_response_complete(struct icast_state *state, int length,
				    u8 payload_length)
{
	unsigned int marker = ICAST_HEADER_SIZE + payload_length;

	return length >= marker + 4 && state->transfer_buffer[marker] == 0x80 &&
	       state->transfer_buffer[marker + 3] == 0x80;
}

static int icast_command_locked(struct icast_state *state, u8 target,
				const u8 *payload, u8 payload_length,
				unsigned int ready_attempts)
{
	int length, response_length, ret;

	ret = icast_send_frame_locked(state, target, payload, payload_length);
	if (ret)
		return ret;

	length = icast_bulk_in_locked(state, ICAST_RESPONSE_SIZE);
	if (length < 0)
		return length;
	response_length = icast_validate_response(state, length);
	if (response_length < 0)
		return response_length;

	if (state->transfer_buffer[2] == 0x80 && response_length > 1 &&
	    state->transfer_buffer[5] == 0x80)
		return 0;
	if (!icast_response_complete(state, length, response_length) &&
	    ready_attempts) {
		msleep(100);
		return icast_wait_ready_locked(state, ready_attempts);
	}

	return 0;
}

static int icast_query_status_locked(struct icast_state *state, u8 *payload,
				     size_t payload_size,
				     unsigned int ready_attempts)
{
	static const u8 status_payload[] = { 0x01 };
	int length, payload_length, ret;

	ret = icast_send_frame_locked(state, ICAST_TARGET_STATUS,
				      status_payload, sizeof(status_payload));
	if (ret)
		return ret;
	length = icast_bulk_in_locked(state, 900);
	if (length < 0)
		return length;
	payload_length = icast_validate_response(state, length);
	if (payload_length < 0)
		return payload_length;
	if (payload_length > payload_size)
		return -EMSGSIZE;

	memcpy(payload, state->transfer_buffer + ICAST_HEADER_SIZE,
	       payload_length);
	if (!icast_response_complete(state, length, payload_length) &&
	    ready_attempts) {
		ret = icast_wait_ready_locked(state, ready_attempts);
		if (ret)
			return ret;
	}

	return payload_length;
}

static bool icast_contains(const u8 *buffer, size_t length, const char *text)
{
	size_t text_length = strlen(text);
	size_t offset;

	if (text_length > length)
		return false;
	for (offset = 0; offset <= length - text_length; offset++) {
		if (!memcmp(buffer + offset, text, text_length))
			return true;
	}

	return false;
}

static int icast_initialize_system_locked(struct icast_state *state)
{
	static const u8 command[] = { 0xfe, 0x00, 0x10, 0x00 };
	int ret;

	ret = icast_bulk_out_locked(state, command, sizeof(command));
	if (ret)
		return ret;
	ret = icast_bulk_in_locked(state, sizeof(command));
	return ret == sizeof(command) ? 0 : ret < 0 ? ret : -EPROTO;
}

static int icast_initialize_hardware_locked(struct icast_state *state)
{
	u8 response[ICAST_MAX_PAYLOAD];
	int length, ret;

	ret = icast_command_locked(state, 0x82, icast_init_ci,
				   sizeof(icast_init_ci), 100);
	if (ret)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 0);
	if (ret < 0)
		return ret;

	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_init_profile_1,
				   sizeof(icast_init_profile_1), 0);
	if (ret)
		return ret;
	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_init_register_10,
				   sizeof(icast_init_register_10), 100);
	if (ret)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 0);
	if (ret < 0)
		return ret;
	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_init_register_12,
				   sizeof(icast_init_register_12), 100);
	if (ret)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 0);
	if (ret < 0)
		return ret;
	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_init_resources,
				   sizeof(icast_init_resources), 100);
	if (ret)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 0);
	if (ret < 0)
		return ret;

	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_init_profile_2,
				   sizeof(icast_init_profile_2), 0);
	if (ret)
		return ret;
	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_init_register_20,
				   sizeof(icast_init_register_20), 100);
	if (ret)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 0);
	if (ret < 0)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 0);
	if (ret < 0)
		return ret;

	ret = icast_command_locked(state, ICAST_TARGET_READY, icast_init_sas_3,
				   sizeof(icast_init_sas_3), 1);
	if (ret)
		return ret;
	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_init_handshake,
				   sizeof(icast_init_handshake), 100);
	if (ret)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 100);
	if (ret < 0)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 0);
	if (ret < 0)
		return ret;

	ret = icast_command_locked(state, ICAST_TARGET_READY, icast_init_sas_4,
				   sizeof(icast_init_sas_4), 300);
	if (ret)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 0);
	if (ret < 0)
		return ret;

	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_query_hardware,
				   sizeof(icast_query_hardware), 100);
	if (ret)
		return ret;
	length =
		icast_query_status_locked(state, response, sizeof(response), 0);
	if (length < 0)
		return length;
	state->supports_dtmb = icast_contains(response, length, "DTMB:Y");

	if (!state->supports_dtmb) {
		ret = icast_command_locked(state, ICAST_TARGET_READY,
					   icast_init_dvbc_profile,
					   sizeof(icast_init_dvbc_profile), 0);
		if (ret)
			return ret;
		ret = icast_command_locked(state, ICAST_TARGET_READY,
					   icast_init_dvbc_register,
					   sizeof(icast_init_dvbc_register), 0);
		if (ret)
			return ret;
		ret = icast_query_status_locked(state, response,
						sizeof(response), 0);
		if (ret < 0)
			return ret;
		ret = icast_query_status_locked(state, response,
						sizeof(response), 0);
		if (ret < 0)
			return ret;
		ret = icast_command_locked(state, ICAST_TARGET_READY,
					   icast_init_sas_5,
					   sizeof(icast_init_sas_5), 100);
		if (ret)
			return ret;
		ret = icast_query_status_locked(state, response,
						sizeof(response), 0);
		if (ret < 0)
			return ret;
	}

	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_default_tune,
				   sizeof(icast_default_tune), 100);
	if (ret)
		return ret;
	ret = icast_query_status_locked(state, response, sizeof(response), 20);
	return ret < 0 ? ret : 0;
}

static int icast_usb_reset(struct icast_state *state)
{
	return usb_control_msg(state->d->udev,
			       usb_sndctrlpipe(state->d->udev, 0), 0xa0,
			       USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			       0, 0, NULL, 0, ICAST_USB_TIMEOUT_MS);
}

static int icast_identify_state(struct dvb_usb_device *d,
				const char **firmware_name)
{
	struct icast_state *state = d_to_priv(d);
	int ret;

	state->d = d;
	mutex_init(&state->protocol_lock);
	mutex_lock(&state->protocol_lock);
	ret = icast_initialize_system_locked(state);
	if (!ret)
		ret = icast_initialize_hardware_locked(state);
	mutex_unlock(&state->protocol_lock);
	if (ret) {
		icast_usb_reset(state);
		return ret;
	}

	dev_info(&d->udev->dev, "iCast %s receiver initialized\n",
		 state->supports_dtmb ? "DTMB/DVB-C" : "DVB-C");
	return WARM;
}

static int icast_frontend_init(struct dvb_frontend *frontend)
{
	struct icast_state *state = frontend->demodulator_priv;

	state->frequency = 0;
	state->symbol_rate = 0;
	state->locked = false;
	return 0;
}

static int icast_modulation_value(enum fe_modulation modulation, u32 *value)
{
	switch (modulation) {
	case QAM_AUTO:
	case QAM_64:
		*value = 64;
		return 0;
	case QAM_16:
		*value = 16;
		return 0;
	case QAM_32:
		*value = 32;
		return 0;
	case QAM_128:
		*value = 128;
		return 0;
	case QAM_256:
		*value = 256;
		return 0;
	default:
		return -EINVAL;
	}
}

static int icast_set_frontend(struct dvb_frontend *frontend)
{
	struct dtv_frontend_properties *properties =
		&frontend->dtv_property_cache;
	struct icast_state *state = frontend->demodulator_priv;
	u8 response[ICAST_MAX_PAYLOAD];
	u8 command[sizeof(icast_default_tune)];
	u32 modulation, symbol_rate;
	int ret;

	if (properties->frequency < 10 * MHz ||
	    properties->frequency > 862 * MHz)
		return -EINVAL;

	memcpy(command, icast_default_tune, sizeof(command));
	put_unaligned_le32(properties->frequency / kHz, command + 16);
	if (properties->delivery_system == SYS_DVBC_ANNEX_A) {
		ret = icast_modulation_value(properties->modulation,
					     &modulation);
		if (ret)
			return ret;
		if (!properties->symbol_rate)
			return -EINVAL;
		symbol_rate = properties->symbol_rate / 1000;
		put_unaligned_le32(symbol_rate, command + 20);
		put_unaligned_le32(modulation, command + 24);
	} else if (properties->delivery_system == SYS_DTMB &&
		   state->supports_dtmb) {
		put_unaligned_le32(0, command + 20);
		put_unaligned_le32(0, command + 24);
		put_unaligned_le32(8, command + 28);
	} else {
		return -EINVAL;
	}

	mutex_lock(&state->protocol_lock);
	ret = icast_command_locked(state, ICAST_TARGET_READY, command,
				   sizeof(command), 100);
	if (!ret) {
		ret = icast_query_status_locked(state, response,
						sizeof(response), 2);
		if (ret >= 0)
			ret = 0;
	}
	mutex_unlock(&state->protocol_lock);
	if (ret)
		return ret;

	state->frequency = properties->frequency;
	state->symbol_rate = properties->symbol_rate;
	state->modulation = properties->modulation;
	state->locked = false;
	return 0;
}

static int icast_get_frontend(struct dvb_frontend *frontend,
			      struct dtv_frontend_properties *properties)
{
	struct icast_state *state = frontend->demodulator_priv;

	properties->frequency = state->frequency;
	properties->symbol_rate = state->symbol_rate;
	properties->modulation = state->modulation;
	return 0;
}

static int icast_get_tune_settings(struct dvb_frontend *frontend,
				   struct dvb_frontend_tune_settings *settings)
{
	settings->min_delay_ms = 800;
	settings->step_size = 0;
	settings->max_drift = 0;
	return 0;
}

static u16 icast_relative_value(u8 value)
{
	return DIV_ROUND_CLOSEST((u32)min_t(u8, value, 100) * U16_MAX, 100);
}

static int icast_read_status(struct dvb_frontend *frontend,
			     enum fe_status *status)
{
	struct dtv_frontend_properties *properties =
		&frontend->dtv_property_cache;
	struct icast_state *state = frontend->demodulator_priv;
	u8 response[ICAST_MAX_PAYLOAD];
	int length, ret;

	mutex_lock(&state->protocol_lock);
	ret = icast_command_locked(state, ICAST_TARGET_READY,
				   icast_status_request,
				   sizeof(icast_status_request), 50);
	if (!ret)
		ret = icast_query_status_locked(state, response,
						sizeof(response), 5);
	mutex_unlock(&state->protocol_lock);
	if (ret < 0)
		return ret;
	length = ret;
	if (length != 32)
		return -EPROTO;

	state->locked = response[29];
	state->snr = state->locked ? response[30] : 0;
	state->signal_strength = state->locked ? response[31] : 0;
	if (state->locked)
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
			  FE_HAS_SYNC | FE_HAS_LOCK;
	else
		*status = 0;

	properties->strength.len = 1;
	properties->strength.stat[0].scale = FE_SCALE_RELATIVE;
	properties->strength.stat[0].uvalue =
		icast_relative_value(state->signal_strength);
	properties->cnr.len = 1;
	properties->cnr.stat[0].scale = FE_SCALE_RELATIVE;
	properties->cnr.stat[0].uvalue = icast_relative_value(state->snr);
	return 0;
}

static int icast_read_signal_strength(struct dvb_frontend *frontend,
				      u16 *strength)
{
	struct icast_state *state = frontend->demodulator_priv;

	*strength = state->locked ?
			    icast_relative_value(state->signal_strength) :
			    0;
	return 0;
}

static int icast_read_snr(struct dvb_frontend *frontend, u16 *snr)
{
	struct icast_state *state = frontend->demodulator_priv;

	*snr = state->locked ? icast_relative_value(state->snr) : 0;
	return 0;
}

static const struct dvb_frontend_ops icast_dual_mode_ops = {
	.delsys = { SYS_DTMB, SYS_DVBC_ANNEX_A },
	.info = {
		.name = "SMIT iCast DTMB/DVB-C",
		.frequency_min_hz = 10 * MHz,
		.frequency_max_hz = 862 * MHz,
		.frequency_stepsize_hz = 10 * kHz,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO,
	},
	.init = icast_frontend_init,
	.set_frontend = icast_set_frontend,
	.get_frontend = icast_get_frontend,
	.get_tune_settings = icast_get_tune_settings,
	.read_status = icast_read_status,
	.read_signal_strength = icast_read_signal_strength,
	.read_snr = icast_read_snr,
};

static const struct dvb_frontend_ops icast_dvbc_ops = {
	.delsys = { SYS_DVBC_ANNEX_A },
	.info = {
		.name = "SMIT iCast DVB-C",
		.frequency_min_hz = 10 * MHz,
		.frequency_max_hz = 862 * MHz,
		.frequency_stepsize_hz = 10 * kHz,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO,
	},
	.init = icast_frontend_init,
	.set_frontend = icast_set_frontend,
	.get_frontend = icast_get_frontend,
	.get_tune_settings = icast_get_tune_settings,
	.read_status = icast_read_status,
	.read_signal_strength = icast_read_signal_strength,
	.read_snr = icast_read_snr,
};

static int icast_frontend_attach(struct dvb_usb_adapter *adapter)
{
	struct icast_state *state = d_to_priv(adap_to_d(adapter));

	state->frontend.ops = state->supports_dtmb ? icast_dual_mode_ops :
						     icast_dvbc_ops;
	state->frontend.demodulator_priv = state;
	adapter->fe[0] = &state->frontend;
	return 0;
}

static const struct dvb_usb_device_properties icast_properties = {
	.driver_name = KBUILD_MODNAME,
	.owner = THIS_MODULE,
	.adapter_nr = adapter_nr,
	.size_of_priv = sizeof(struct icast_state),
	.identify_state = icast_identify_state,
	.frontend_attach = icast_frontend_attach,
	.num_adapters = 1,
	.adapter = {
		{
			.stream = DVB_USB_STREAM_BULK(ICAST_STREAM_ENDPOINT, 8, 8192),
		},
	},
};

static const struct usb_device_id icast_id_table[] = {
	{ DVB_USB_DEVICE(ICAST_USB_VENDOR, ICAST_USB_PRODUCT, &icast_properties,
			 "SMIT iCast DTMB/DVB-C receiver", NULL) },
	{}
};
MODULE_DEVICE_TABLE(usb, icast_id_table);

static struct usb_driver icast_usb_driver = {
	.name = KBUILD_MODNAME,
	.id_table = icast_id_table,
	.probe = dvb_usbv2_probe,
	.disconnect = dvb_usbv2_disconnect,
	.suspend = dvb_usbv2_suspend,
	.resume = dvb_usbv2_resume,
	.reset_resume = dvb_usbv2_reset_resume,
	.no_dynamic_id = 1,
	.soft_unbind = 1,
};
module_usb_driver(icast_usb_driver);

MODULE_DESCRIPTION("SMIT iCast DTMB/DVB-C USB driver");
MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_LICENSE("GPL");
