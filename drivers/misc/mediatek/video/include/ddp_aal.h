/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Joey Pan <joey.pan@mediatek.com>
 */

#ifndef __DDP_AAL_H__
#define __DDP_AAL_H__

#define AAL_HIST_BIN            33	/* [0..32] */
#define AAL_DRE_POINT_NUM       29

#define AAL_DRE30_GAIN_REGISTER_NUM		(544)
#define AAL_DRE30_HIST_REGISTER_NUM		(768)

#define AAL_SERVICE_FORCE_UPDATE 0x1

/* Custom "wide" pointer type for 64-bit compatibility. */
/* Always cast from uint32_t*. */
/* typedef unsigned long long aal_u32_ptr_t; */
#define aal_u32_handle_t unsigned long long

#define AAL_U32_PTR(x) ((unsigned int *)(unsigned long)x)

enum disp_aal_id_t {
	DISP_AAL0 = 0,
	DISP_AAL1,
	DISP_AAL_TOTAL
};

enum AAL_ESS_UD_MODE {
	CONFIG_BY_CUSTOM_LIB = 0,
	CONFIG_TO_LCD = 1,
	CONFIG_TO_AMOLED = 2
};

enum AAL_DRE_MODE {
	DRE_EN_BY_CUSTOM_LIB = 0xFFFF,
	DRE_OFF = 0,
	DRE_ON = 1
};

enum AAL_ESS_MODE {
	ESS_EN_BY_CUSTOM_LIB = 0xFFFF,
	ESS_OFF = 0,
	ESS_ON = 1
};

enum AAL_ESS_LEVEL {
	ESS_LEVEL_BY_CUSTOM_LIB = 0xFFFF
};

#define AAL_CONTROL_CMD(ID, CONTROL) (ID << 16 | CONTROL)

struct DISP_AAL_INITREG {
	/* DRE */
	int dre_map_bypass;
	/* ESS */
	int cabc_gainlmt[33];
	/* DRE 3.0 Reg. */
	int dre_s_lower;
	int dre_s_upper;
	int dre_y_lower;
	int dre_y_upper;
	int dre_h_lower;
	int dre_h_upper;
	int dre_h_slope;
	int dre_s_slope;
	int dre_y_slope;
	int dre_x_alpha_base;
	int dre_x_alpha_shift_bit;
	int dre_y_alpha_base;
	int dre_y_alpha_shift_bit;
	int act_win_x_end;
	int dre_blk_x_num;
	int dre_blk_y_num;
	int dre_blk_height;
	int dre_blk_width;
	int dre_blk_area;
	int dre_blk_area_min;
	int hist_bin_type;
	int dre_flat_length_slope;
	int dre_flat_length_th;
	int act_win_y_start;
	int act_win_y_end;
	int blk_num_x_start;
	int blk_num_x_end;
	int blk_cnt_x_start;
	int blk_cnt_x_end;
	int blk_num_y_start;
	int blk_num_y_end;
	int blk_cnt_y_start;
	int blk_cnt_y_end;
	int last_tile_x_flag;
	int last_tile_y_flag;
};

struct DISP_DRE30_INIT {
	/* DRE 3.0 SW */
	aal_u32_handle_t dre30_hist_addr;
};

struct DISP_AAL_DISPLAY_SIZE {
	int width;
	int height;
};

struct DISP_DRE30_HIST {
	unsigned int dre_hist[AAL_DRE30_HIST_REGISTER_NUM];
	int dre_blk_x_num;
	int dre_blk_y_num;
};

struct DISP_AAL_HIST {
	unsigned int serviceFlags;
	int backlight;
	int colorHist;
	unsigned int maxHist[AAL_HIST_BIN];
	int requestPartial;
	aal_u32_handle_t dre30_hist;
	unsigned int panel_type;
	int essStrengthIndex;
	int ess_enable;
	int dre_enable;
	unsigned int yHist[AAL_HIST_BIN];
};

struct DISP_AAL_HIST_MODULE {
	int colorHist;
	unsigned int maxHist[AAL_HIST_BIN];
	unsigned int count;
};

enum DISP_AAL_REFRESH_LATENCY {
	AAL_REFRESH_17MS = 17,
	AAL_REFRESH_33MS = 33
};

struct DISP_DRE30_PARAM {
	unsigned int dre30_gain[AAL_DRE30_GAIN_REGISTER_NUM];
};

struct DISP_AAL_PARAM {
	int DREGainFltStatus[AAL_DRE_POINT_NUM];
	int cabc_fltgain_force;	/* 10-bit ; [0,1023] */
	int cabc_gainlmt[33];
	int FinalBacklight;	/* 10-bit ; [0,1023] */
	int allowPartial;
	int refreshLatency;	/* DISP_AAL_REFRESH_LATENCY */
	aal_u32_handle_t dre30_gain;
};

#ifdef CONFIG_LGE_USE_ESS_STRENGTH_TABLE
// set value by backlight table map
enum DISP_UI_BAR {
  UI_10_PER = 13,
  UI_20_PER = 36,
  UI_30_PER = 77,
  UI_40_PER = 140,
  UI_60_PER = 320,
  UI_80_PER = 599,
  UI_90_PER = 793,
  UI_MAX    = 1005,
};

// set value by ess strength table
enum DISP_ESS_STR_INDEX {
  MIN_ESS_STR_INDEX       = 0, // 0
  UI_10_PER_ESS_STR_INDEX = 1, // 16
  UI_20_PER_ESS_STR_INDEX = 2, // 32
  UI_30_PER_ESS_STR_INDEX = 4, // 64
  UI_40_PER_ESS_STR_INDEX = 6, // 96
  UI_60_PER_ESS_STR_INDEX = 8, // 128 (default ess strength)
  UI_80_PER_ESS_STR_INDEX = 10,// 160
  MAX_ESS_STR_INDEX       = 12 // 192
};
#endif

void disp_aal_on_end_of_frame(void);
void disp_aal_on_end_of_frame_by_module(enum disp_aal_id_t id);

extern int aal_dbg_en;
void aal_test(const char *cmd, char *debug_output);
bool disp_aal_is_support(void);
int aal_is_partial_support(void);
int aal_request_partial_support(int partial);

void disp_aal_notify_backlight_changed(int bl_1024);

void disp_aal_set_lcm_type(unsigned int panel_type);
void disp_aal_set_ess_level(int level);
void disp_aal_set_ess_en(int enable);
void disp_aal_set_dre_en(int enable);
#ifdef CONFIG_LGE_DISPLAY_COMMON
int disp_aal_get_ess_en(void);
#endif

#endif
