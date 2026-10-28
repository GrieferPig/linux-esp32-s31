/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2025 Espressif Systems (Shanghai) CO LTD.
 * Register layouts from ESP-IDF 08e0d30a74ad (MIT option), with descriptive
 * comments removed and Linux integer types. Keep reserved words/bitfields:
 * they preserve the silicon register offsets. No IDF runtime dependency.
 */
#ifndef _S31_PPA_REGS_H
#define _S31_PPA_REGS_H
#include <linux/types.h>
typedef union {
    struct {
        u32 rdwr_word_blend0_clut:32;
    };
    u32 val;
} ppa_blend0_clut_data_reg_t;
typedef union {
    struct {
        u32 rdwr_word_blend1_clut:32;
    };
    u32 val;
} ppa_blend1_clut_data_reg_t;
typedef union {
    struct {
        u32 apb_fifo_mask:1;
        u32 blend0_clut_mem_rst:1;
        u32 blend1_clut_mem_rst:1;
        u32 blend0_clut_mem_rdaddr_rst:1;
        u32 blend1_clut_mem_rdaddr_rst:1;
        u32 blend_clut_mem_force_pd:1;
        u32 blend_clut_mem_force_pu:1;
        u32 blend_clut_mem_clk_ena:1;
        u32 reserved_8:24;
    };
    u32 val;
} ppa_clut_conf_reg_t;
typedef union {
    struct {
        u32 srm_rx_cm:4;
        u32 srm_tx_cm:4;
        u32 yuv_rx_range:1;
        u32 yuv_tx_range:1;
        u32 yuv2rgb_protocol:1;
        u32 rgb2yuv_protocol:1;
        u32 yuv422_rx_byte_order:2;
        u32 reserved_14:18;
    };
    u32 val;
} ppa_srm_color_mode_reg_t;
typedef union {
    struct {
        u32 blend0_rx_cm:4;
        u32 blend1_rx_cm:4;
        u32 blend_tx_cm:4;
        u32 blend0_rx_yuv_range:1;
        u32 blend_tx_yuv_range:1;
        u32 blend0_rx_yuv2rgb_protocol:1;
        u32 blend_tx_rgb2yuv_protocol:1;
        u32 blend0_rx_yuv422_byte_order:2;
        u32 reserved_18:14;
    };
    u32 val;
} ppa_blend_color_mode_reg_t;
typedef union {
    struct {
        u32 srm_rx_byte_swap_en:1;
        u32 srm_rx_rgb_swap_en:1;
        u32 srm_macro_bk_ro_bypass:1;
        u32 srm_bk_size_sel:1;
        u32 reserved_4:28;
    };
    u32 val;
} ppa_srm_byte_order_reg_t;
typedef union {
    struct {
        u32 blend0_rx_byte_swap_en:1;
        u32 blend1_rx_byte_swap_en:1;
        u32 blend0_rx_rgb_swap_en:1;
        u32 blend1_rx_rgb_swap_en:1;
        u32 reserved_4:28;
    };
    u32 val;
} ppa_blend_byte_order_reg_t;
typedef union {
    struct {
        u32 blend_en:1;
        u32 blend_bypass:1;
        u32 blend_fix_pixel_fill_en:1;
        u32 blend_trans_mode_update:1;
        u32 blend_rst:1;
        u32 blend_tx_inf_sel:2;
        u32 reserved_7:25;
    };
    u32 val;
} ppa_blend_trans_mode_reg_t;
typedef union {
    struct {
        u32 srm_rx_fix_alpha:8;
        u32 srm_rx_alpha_mod:2;
        u32 srm_rx_alpha_inv:1;
        u32 reserved_11:21;
    };
    u32 val;
} ppa_srm_fix_alpha_reg_t;
typedef union {
    struct {
        u32 blend_hb:14;
        u32 blend_vb:14;
        u32 reserved_28:4;
    };
    u32 val;
} ppa_blend_tx_size_reg_t;
typedef union {
    struct {
        u32 blend0_rx_fix_alpha:8;
        u32 blend1_rx_fix_alpha:8;
        u32 blend0_rx_alpha_mod:2;
        u32 blend1_rx_alpha_mod:2;
        u32 blend0_rx_alpha_inv:1;
        u32 blend1_rx_alpha_inv:1;
        u32 reserved_22:10;
    };
    u32 val;
} ppa_blend_fix_alpha_reg_t;
typedef union {
    struct {
        u32 blend1_rx_b:8;
        u32 blend1_rx_g:8;
        u32 blend1_rx_r:8;
        u32 reserved_24:8;
    };
    u32 val;
} ppa_blend_rgb_reg_t;
typedef union {
    struct {
        u32 blend_tx_fix_pixel:32;
    };
    u32 val;
} ppa_blend_fix_pixel_reg_t;
typedef union {
    struct {
        u32 colorkey_fg_b_low:8;
        u32 colorkey_fg_g_low:8;
        u32 colorkey_fg_r_low:8;
        u32 reserved_24:8;
    };
    u32 val;
} ppa_ck_fg_low_reg_t;
typedef union {
    struct {
        u32 colorkey_fg_b_high:8;
        u32 colorkey_fg_g_high:8;
        u32 colorkey_fg_r_high:8;
        u32 reserved_24:8;
    };
    u32 val;
} ppa_ck_fg_high_reg_t;
typedef union {
    struct {
        u32 colorkey_bg_b_low:8;
        u32 colorkey_bg_g_low:8;
        u32 colorkey_bg_r_low:8;
        u32 reserved_24:8;
    };
    u32 val;
} ppa_ck_bg_low_reg_t;
typedef union {
    struct {
        u32 colorkey_bg_b_high:8;
        u32 colorkey_bg_g_high:8;
        u32 colorkey_bg_r_high:8;
        u32 reserved_24:8;
    };
    u32 val;
} ppa_ck_bg_high_reg_t;
typedef union {
    struct {
        u32 colorkey_default_b:8;
        u32 colorkey_default_g:8;
        u32 colorkey_default_r:8;
        u32 colorkey_fg_bg_reverse:1;
        u32 reserved_25:7;
    };
    u32 val;
} ppa_ck_default_reg_t;
typedef union {
    struct {
        u32 srm_scal_x_int:8;
        u32 srm_scal_x_frag:4;
        u32 srm_scal_y_int:8;
        u32 srm_scal_y_frag:4;
        u32 srm_rotate_angle:2;
        u32 scal_rotate_rst:1;
        u32 scal_rotate_start:1;
        u32 srm_mirror_x:1;
        u32 srm_mirror_y:1;
        u32 reserved_30:2;
    };
    u32 val;
} ppa_srm_scal_rotate_reg_t;
typedef union {
    struct {
        u32 srm_mem_clk_ena:1;
        u32 srm_mem_force_pd:1;
        u32 srm_mem_force_pu:1;
        u32 reserved_3:29;
    };
    u32 val;
} ppa_srm_mem_pd_reg_t;
typedef union {
    struct {
        u32 clk_en:1;
        u32 reserved_1:31;
    };
    u32 val;
} ppa_reg_conf_reg_t;
typedef union {
    struct {
        u32 rnd_eco_low:32;
    };
    u32 val;
} ppa_eco_low_reg_t;
typedef union {
    struct {
        u32 rnd_eco_high:32;
    };
    u32 val;
} ppa_eco_high_reg_t;
typedef union {
    struct {
        u32 mem_aux_ctrl:14;
        u32 reserved_14:18;
    };
    u32 val;
} ppa_sram_ctrl_reg_t;
typedef union {
    struct {
        u32 srm_eof_int_raw:1;
        u32 blend_eof_int_raw:1;
        u32 srm_param_cfg_err_int_raw:1;
        u32 blend_param_cfg_err_int_raw:1;
        u32 reserved_4:28;
    };
    u32 val;
} ppa_int_raw_reg_t;
typedef union {
    struct {
        u32 srm_eof_int_st:1;
        u32 blend_eof_int_st:1;
        u32 srm_param_cfg_err_int_st:1;
        u32 blend_param_cfg_err_int_st:1;
        u32 reserved_4:28;
    };
    u32 val;
} ppa_int_st_reg_t;
typedef union {
    struct {
        u32 srm_eof_int_ena:1;
        u32 blend_eof_int_ena:1;
        u32 srm_param_cfg_err_int_ena:1;
        u32 blend_param_cfg_err_int_ena:1;
        u32 reserved_4:28;
    };
    u32 val;
} ppa_int_ena_reg_t;
typedef union {
    struct {
        u32 srm_eof_int_clr:1;
        u32 blend_eof_int_clr:1;
        u32 srm_param_cfg_err_int_clr:1;
        u32 blend_param_cfg_err_int_clr:1;
        u32 reserved_4:28;
    };
    u32 val;
} ppa_int_clr_reg_t;
typedef union {
    struct {
        u32 blend0_clut_cnt:9;
        u32 blend1_clut_cnt:9;
        u32 reserved_18:14;
    };
    u32 val;
} ppa_clut_cnt_reg_t;
typedef union {
    struct {
        u32 blend_size_diff_st:1;
        u32 blend_yuv_x_scale_err_st:1;
        u32 blend_yuv_y_scale_err_st:1;
        u32 reserved_3:29;
    };
    u32 val;
} ppa_blend_st_reg_t;
typedef union {
    struct {
        u32 tx_dscr_vb_err_st:1;
        u32 tx_dscr_hb_err_st:1;
        u32 y_rx_scal_equal_0_err_st:1;
        u32 rx_dscr_vb_err_st:1;
        u32 ydst_len_too_samll_err_st:1;
        u32 ydst_len_too_large_err_st:1;
        u32 x_rx_scal_equal_0_err_st:1;
        u32 rx_dscr_hb_err_st:1;
        u32 xdst_len_too_samll_err_st:1;
        u32 xdst_len_too_large_err_st:1;
        u32 x_yuv_rx_scale_err_st:1;
        u32 y_yuv_rx_scale_err_st:1;
        u32 x_yuv_tx_scale_err_st:1;
        u32 y_yuv_tx_scale_err_st:1;
        u32 reserved_14:18;
    };
    u32 val;
} ppa_srm_param_err_st_reg_t;
typedef union {
    struct {
        u32 srm_rx_dscr_sample_state:2;
        u32 srm_rx_scan_state:2;
        u32 srm_tx_dscr_sample_state:2;
        u32 srm_tx_scan_state:3;
        u32 reserved_9:23;
    };
    u32 val;
} ppa_srm_status_reg_t;
typedef union {
    struct {
        u32 rdn_result:1;
        u32 rdn_ena:1;
        u32 reserved_2:30;
    };
    u32 val;
} ppa_eco_cell_ctrl_reg_t;
typedef union {
    struct {
        u32 dbg_replace_sel:3;
        u32 reserved_3:29;
    };
    u32 val;
} ppa_debug_ctrl0_reg_t;
typedef union {
    struct {
        u32 dbg_replace_data:32;
    };
    u32 val;
} ppa_debug_ctrl1_reg_t;
typedef union {
    struct {
        u32 rgb2gray_b:8;
        u32 rgb2gray_g:8;
        u32 rgb2gray_r:8;
        u32 reserved_24:8;
    };
    u32 val;
} ppa_rgb2gray_reg_t;
typedef union {
    struct {
        u32 date:32;
    };
    u32 val;
} ppa_date_reg_t;
typedef struct ppa_dev_t {
    volatile ppa_blend0_clut_data_reg_t blend0_clut_data;
    volatile ppa_blend1_clut_data_reg_t blend1_clut_data;
    u32 reserved_008;
    volatile ppa_clut_conf_reg_t clut_conf;
    volatile ppa_int_raw_reg_t int_raw;
    volatile ppa_int_st_reg_t int_st;
    volatile ppa_int_ena_reg_t int_ena;
    volatile ppa_int_clr_reg_t int_clr;
    volatile ppa_srm_color_mode_reg_t srm_color_mode;
    volatile ppa_blend_color_mode_reg_t blend_color_mode;
    volatile ppa_srm_byte_order_reg_t srm_byte_order;
    volatile ppa_blend_byte_order_reg_t blend_byte_order;
    u32 reserved_030;
    volatile ppa_blend_trans_mode_reg_t blend_trans_mode;
    volatile ppa_srm_fix_alpha_reg_t srm_fix_alpha;
    volatile ppa_blend_tx_size_reg_t blend_tx_size;
    volatile ppa_blend_fix_alpha_reg_t blend_fix_alpha;
    u32 reserved_044;
    volatile ppa_blend_rgb_reg_t blend_rgb;
    volatile ppa_blend_fix_pixel_reg_t blend_fix_pixel;
    volatile ppa_ck_fg_low_reg_t ck_fg_low;
    volatile ppa_ck_fg_high_reg_t ck_fg_high;
    volatile ppa_ck_bg_low_reg_t ck_bg_low;
    volatile ppa_ck_bg_high_reg_t ck_bg_high;
    volatile ppa_ck_default_reg_t ck_default;
    volatile ppa_srm_scal_rotate_reg_t srm_scal_rotate;
    volatile ppa_srm_mem_pd_reg_t srm_mem_pd;
    volatile ppa_reg_conf_reg_t reg_conf;
    volatile ppa_clut_cnt_reg_t clut_cnt;
    volatile ppa_blend_st_reg_t blend_st;
    volatile ppa_srm_param_err_st_reg_t srm_param_err_st;
    volatile ppa_srm_status_reg_t srm_status;
    volatile ppa_eco_low_reg_t eco_low;
    volatile ppa_eco_high_reg_t eco_high;
    volatile ppa_eco_cell_ctrl_reg_t eco_cell_ctrl;
    volatile ppa_sram_ctrl_reg_t sram_ctrl;
    volatile ppa_debug_ctrl0_reg_t debug_ctrl0;
    volatile ppa_debug_ctrl1_reg_t debug_ctrl1;
    volatile ppa_rgb2gray_reg_t rgb2gray;
    u32 reserved_09c[25];
    volatile ppa_date_reg_t date;
} ppa_dev_t;
_Static_assert(sizeof(ppa_dev_t) == 0x104, "Invalid size of ppa_dev_t structure");
#endif
