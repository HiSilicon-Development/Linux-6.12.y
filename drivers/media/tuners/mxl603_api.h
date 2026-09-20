/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef MXL603_API_H
#define MXL603_API_H

#include <linux/i2c.h>
#include <linux/types.h>

enum mxl603_xtal_frequency {
	MXL603_XTAL_16MHZ,
	MXL603_XTAL_24MHZ,
};

enum mxl603_if_frequency {
	MXL603_IF_3_65MHZ,
	MXL603_IF_4MHZ,
	MXL603_IF_4_1MHZ,
	MXL603_IF_4_15MHZ,
	MXL603_IF_4_5MHZ,
	MXL603_IF_4_57MHZ,
	MXL603_IF_5MHZ,
	MXL603_IF_5_38MHZ,
	MXL603_IF_6MHZ,
	MXL603_IF_6_28MHZ,
	MXL603_IF_7_2MHZ,
	MXL603_IF_8_25MHZ,
	MXL603_IF_35_25MHZ,
	MXL603_IF_36MHZ,
	MXL603_IF_36_15MHZ,
	MXL603_IF_36_65MHZ,
	MXL603_IF_44MHZ,
};

enum mxl603_agc_type {
	MXL603_AGC_SELF,
	MXL603_AGC_EXTERNAL,
};

enum mxl603_signal_mode {
	MXL603_MODE_DVBC,
	MXL603_MODE_DTMB,
};

enum mxl603_bandwidth {
	MXL603_CABLE_BW_6MHZ = 0x00,
	MXL603_CABLE_BW_7MHZ = 0x01,
	MXL603_CABLE_BW_8MHZ = 0x02,
	MXL603_TERR_BW_6MHZ = 0x20,
	MXL603_TERR_BW_7MHZ = 0x21,
	MXL603_TERR_BW_8MHZ = 0x22,
};

enum mxl603_power_mode {
	MXL603_POWER_ACTIVE,
	MXL603_POWER_STANDBY,
};

struct mxl603_config {
	bool single_supply_3v3;
	enum mxl603_xtal_frequency xtal_frequency;
	u8 xtal_capacitance_pf;
	bool clock_output;
	bool clock_output_div4;
	bool xtal_sharing;
	enum mxl603_if_frequency if_frequency;
	bool manual_if;
	bool invert_if;
	u8 if_gain;
	u32 manual_if_khz;
	enum mxl603_agc_type agc_type;
	u8 agc_set_point;
	bool invert_agc;
	enum mxl603_xtal_frequency mode_xtal_frequency;
	u32 mode_if_khz;
	u8 mode_if_gain;
};

struct mxl603_version {
	u8 chip_id;
	u8 revision;
};

int mxl603_reset(struct i2c_adapter *i2c, u8 addr);
int mxl603_get_version(struct i2c_adapter *i2c, u8 addr,
		       struct mxl603_version *version);
int mxl603_initialize(struct i2c_adapter *i2c, u8 addr,
		      const struct mxl603_config *config);
int mxl603_set_power(struct i2c_adapter *i2c, u8 addr,
		     enum mxl603_power_mode mode);
int mxl603_set_if_output(struct i2c_adapter *i2c, u8 addr,
			 const struct mxl603_config *config);
int mxl603_set_mode(struct i2c_adapter *i2c, u8 addr,
		    const struct mxl603_config *config,
		    enum mxl603_signal_mode mode);
int mxl603_set_frequency(struct i2c_adapter *i2c, u8 addr, u32 frequency,
			 enum mxl603_bandwidth bandwidth,
			 enum mxl603_signal_mode mode);
int mxl603_get_lock(struct i2c_adapter *i2c, u8 addr, bool *rf_locked,
		    bool *ref_locked);
int mxl603_get_rf_power(struct i2c_adapter *i2c, u8 addr, s16 *dbm_x100);

#endif
