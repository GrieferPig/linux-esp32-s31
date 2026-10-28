/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2025 Espressif Systems (Shanghai) CO LTD.
 * Register layouts from ESP-IDF 08e0d30a74ad (MIT option), with descriptive
 * comments removed and Linux integer types. Keep reserved words/bitfields:
 * they preserve the silicon register offsets. No IDF runtime dependency.
 */
#ifndef _S31_JPEG_REGS_H
#define _S31_JPEG_REGS_H
#include <linux/types.h>
typedef union {
    struct {
        u32 fsm_rst:1;
        u32 jpeg_start:1;
        u32 qnr_precision:1;
        u32 ff_check_en:1;
        u32 sample_sel:2;
        u32 debug_direct_out_en:1;
        u32 qnr_fifo_en:1;
        u32 lqnr_tbl_sel:2;
        u32 cqnr_tbl_sel:2;
        u32 color_space:3;
        u32 dht_fifo_en:1;
        u32 mem_clk_force_on:1;
        u32 decode_timeout_thres:6;
        u32 decode_timeout_task_sel:1;
        u32 soft_rst:1;
        u32 fifo_rst:1;
        u32 pixel_rev:1;
        u32 tailer_en:1;
        u32 pause_en:1;
        u32 mem_force_pd:1;
        u32 mem_force_pu:1;
        u32 mode:1;
    };
    u32 val;
} jpeg_config_reg_t;
typedef union {
    struct {
        u32 t0_dqt_info:8;
        u32 t1_dqt_info:8;
        u32 t2_dqt_info:8;
        u32 t3_dqt_info:8;
    };
    u32 val;
} jpeg_dqt_info_reg_t;
typedef union {
    struct {
        u32 va:16;
        u32 ha:16;
    };
    u32 val;
} jpeg_pic_size_reg_t;
typedef union {
    struct {
        u32 t0_qnr_val:32;
    };
    u32 val;
} jpeg_t0qnr_reg_t;
typedef union {
    struct {
        u32 t1_qnr_val:32;
    };
    u32 val;
} jpeg_t1qnr_reg_t;
typedef union {
    struct {
        u32 t2_qnr_val:32;
    };
    u32 val;
} jpeg_t2qnr_reg_t;
typedef union {
    struct {
        u32 t3_qnr_val:32;
    };
    u32 val;
} jpeg_t3qnr_reg_t;
typedef union {
    struct {
        u32 restart_interval:16;
        u32 component_num:8;
        u32 sw_dht_en:1;
        u32 sos_check_byte_num:2;
        u32 rst_check_byte_num:2;
        u32 multi_scan_err_check:1;
        u32 dezigzag_ready_ctl:1;
        u32 reserved_31:1;
    };
    u32 val;
} jpeg_decode_conf_reg_t;
typedef union {
    struct {
        u32 c0_dqt_tbl_sel:8;
        u32 c0_y_factor:4;
        u32 c0_x_factor:4;
        u32 c0_id:8;
        u32 reserved_24:8;
    };
    u32 val;
} jpeg_c0_reg_t;
typedef union {
    struct {
        u32 c1_dqt_tbl_sel:8;
        u32 c1_y_factor:4;
        u32 c1_x_factor:4;
        u32 c1_id:8;
        u32 reserved_24:8;
    };
    u32 val;
} jpeg_c1_reg_t;
typedef union {
    struct {
        u32 c2_dqt_tbl_sel:8;
        u32 c2_y_factor:4;
        u32 c2_x_factor:4;
        u32 c2_id:8;
        u32 reserved_24:8;
    };
    u32 val;
} jpeg_c2_reg_t;
typedef union {
    struct {
        u32 c3_dqt_tbl_sel:8;
        u32 c3_y_factor:4;
        u32 c3_x_factor:4;
        u32 c3_id:8;
        u32 reserved_24:8;
    };
    u32 val;
} jpeg_c3_reg_t;
typedef union {
    struct {
        u32 dc0_dht_id:4;
        u32 dc1_dht_id:4;
        u32 ac0_dht_id:4;
        u32 ac1_dht_id:4;
        u32 reserved_16:16;
    };
    u32 val;
} jpeg_dht_info_reg_t;
typedef union {
    struct {
        u32 done_int_raw:1;
        u32 rle_parallel_err_int_raw:1;
        u32 cid_err_int_raw:1;
        u32 c_dht_dc_id_err_int_raw:1;
        u32 c_dht_ac_id_err_int_raw:1;
        u32 c_dqt_id_err_int_raw:1;
        u32 rst_uxp_err_int_raw:1;
        u32 rst_check_none_err_int_raw:1;
        u32 rst_check_pos_err_int_raw:1;
        u32 out_eof_int_raw:1;
        u32 sr_color_mode_err_int_raw:1;
        u32 dct_done_int_raw:1;
        u32 bs_last_block_eof_int_raw:1;
        u32 scan_check_none_err_int_raw:1;
        u32 scan_check_pos_err_int_raw:1;
        u32 uxp_det_int_raw:1;
        u32 en_frame_eof_err_int_raw:1;
        u32 en_frame_eof_lack_int_raw:1;
        u32 de_frame_eof_err_int_raw:1;
        u32 de_frame_eof_lack_int_raw:1;
        u32 sos_unmatch_err_int_raw:1;
        u32 marker_err_fst_scan_int_raw:1;
        u32 marker_err_other_scan_int_raw:1;
        u32 undet_int_raw:1;
        u32 decode_timeout_int_raw:1;
        u32 encode_timeout_int_raw:1;
        u32 reserved_26:6;
    };
    u32 val;
} jpeg_int_raw_reg_t;
typedef union {
    struct {
        u32 done_int_ena:1;
        u32 rle_parallel_err_int_ena:1;
        u32 cid_err_int_ena:1;
        u32 c_dht_dc_id_err_int_ena:1;
        u32 c_dht_ac_id_err_int_ena:1;
        u32 c_dqt_id_err_int_ena:1;
        u32 rst_uxp_err_int_ena:1;
        u32 rst_check_none_err_int_ena:1;
        u32 rst_check_pos_err_int_ena:1;
        u32 out_eof_int_ena:1;
        u32 sr_color_mode_err_int_ena:1;
        u32 dct_done_int_ena:1;
        u32 bs_last_block_eof_int_ena:1;
        u32 scan_check_none_err_int_ena:1;
        u32 scan_check_pos_err_int_ena:1;
        u32 uxp_det_int_ena:1;
        u32 en_frame_eof_err_int_ena:1;
        u32 en_frame_eof_lack_int_ena:1;
        u32 de_frame_eof_err_int_ena:1;
        u32 de_frame_eof_lack_int_ena:1;
        u32 sos_unmatch_err_int_ena:1;
        u32 marker_err_fst_scan_int_ena:1;
        u32 marker_err_other_scan_int_ena:1;
        u32 undet_int_ena:1;
        u32 decode_timeout_int_ena:1;
        u32 encode_timeout_int_ena:1;
        u32 reserved_26:6;
    };
    u32 val;
} jpeg_int_ena_reg_t;
typedef union {
    struct {
        u32 done_int_st:1;
        u32 rle_parallel_err_int_st:1;
        u32 cid_err_int_st:1;
        u32 c_dht_dc_id_err_int_st:1;
        u32 c_dht_ac_id_err_int_st:1;
        u32 c_dqt_id_err_int_st:1;
        u32 rst_uxp_err_int_st:1;
        u32 rst_check_none_err_int_st:1;
        u32 rst_check_pos_err_int_st:1;
        u32 out_eof_int_st:1;
        u32 sr_color_mode_err_int_st:1;
        u32 dct_done_int_st:1;
        u32 bs_last_block_eof_int_st:1;
        u32 scan_check_none_err_int_st:1;
        u32 scan_check_pos_err_int_st:1;
        u32 uxp_det_int_st:1;
        u32 en_frame_eof_err_int_st:1;
        u32 en_frame_eof_lack_int_st:1;
        u32 de_frame_eof_err_int_st:1;
        u32 de_frame_eof_lack_int_st:1;
        u32 sos_unmatch_err_int_st:1;
        u32 marker_err_fst_scan_int_st:1;
        u32 marker_err_other_scan_int_st:1;
        u32 undet_int_st:1;
        u32 decode_timeout_int_st:1;
        u32 encode_timeout_int_st:1;
        u32 reserved_26:6;
    };
    u32 val;
} jpeg_int_st_reg_t;
typedef union {
    struct {
        u32 done_int_clr:1;
        u32 rle_parallel_err_int_clr:1;
        u32 cid_err_int_clr:1;
        u32 c_dht_dc_id_err_int_clr:1;
        u32 c_dht_ac_id_err_int_clr:1;
        u32 c_dqt_id_err_int_clr:1;
        u32 rst_uxp_err_int_clr:1;
        u32 rst_check_none_err_int_clr:1;
        u32 rst_check_pos_err_int_clr:1;
        u32 out_eof_int_clr:1;
        u32 sr_color_mode_err_int_clr:1;
        u32 dct_done_int_clr:1;
        u32 bs_last_block_eof_int_clr:1;
        u32 scan_check_none_err_int_clr:1;
        u32 scan_check_pos_err_int_clr:1;
        u32 uxp_det_int_clr:1;
        u32 en_frame_eof_err_int_clr:1;
        u32 en_frame_eof_lack_int_clr:1;
        u32 de_frame_eof_err_int_clr:1;
        u32 de_frame_eof_lack_int_clr:1;
        u32 sos_unmatch_err_int_clr:1;
        u32 marker_err_fst_scan_int_clr:1;
        u32 marker_err_other_scan_int_clr:1;
        u32 undet_int_clr:1;
        u32 decode_timeout_int_clr:1;
        u32 encode_timeout_int_clr:1;
        u32 reserved_26:6;
    };
    u32 val;
} jpeg_int_clr_reg_t;
typedef union {
    struct {
        u32 reserved_0:11;
        u32 bitstream_eof_vld_cnt:6;
        u32 dctout_zzscan_addr:6;
        u32 qnrval_zzscan_addr:6;
        u32 reg_state_yuv:3;
    };
    u32 val;
} jpeg_status0_reg_t;
typedef union {
    struct {
        u32 source_pixel:24;
        u32 last_block:1;
        u32 last_mcu:1;
        u32 last_dc:1;
        u32 packfifo_ready:1;
        u32 reserved_28:4;
    };
    u32 val;
} jpeg_status2_reg_t;
typedef union {
    struct {
        u32 yo:9;
        u32 y_ready:1;
        u32 cbo:9;
        u32 cb_ready:1;
        u32 cro:9;
        u32 cr_ready:1;
        u32 reserved_30:2;
    };
    u32 val;
} jpeg_status3_reg_t;
typedef union {
    struct {
        u32 hfm_bitstream:32;
    };
    u32 val;
} jpeg_status4_reg_t;
typedef union {
    struct {
        u32 dht_totlen_dc0:32;
    };
    u32 val;
} jpeg_dht_totlen_dc0_reg_t;
typedef union {
    struct {
        u32 dht_val_dc0:32;
    };
    u32 val;
} jpeg_dht_val_dc0_reg_t;
typedef union {
    struct {
        u32 dht_totlen_ac0:32;
    };
    u32 val;
} jpeg_dht_totlen_ac0_reg_t;
typedef union {
    struct {
        u32 dht_val_ac0:32;
    };
    u32 val;
} jpeg_dht_val_ac0_reg_t;
typedef union {
    struct {
        u32 dht_totlen_dc1:32;
    };
    u32 val;
} jpeg_dht_totlen_dc1_reg_t;
typedef union {
    struct {
        u32 dht_val_dc1:32;
    };
    u32 val;
} jpeg_dht_val_dc1_reg_t;
typedef union {
    struct {
        u32 dht_totlen_ac1:32;
    };
    u32 val;
} jpeg_dht_totlen_ac1_reg_t;
typedef union {
    struct {
        u32 dht_val_ac1:32;
    };
    u32 val;
} jpeg_dht_val_ac1_reg_t;
typedef union {
    struct {
        u32 dht_codemin_dc0:32;
    };
    u32 val;
} jpeg_dht_codemin_dc0_reg_t;
typedef union {
    struct {
        u32 dht_codemin_ac0:32;
    };
    u32 val;
} jpeg_dht_codemin_ac0_reg_t;
typedef union {
    struct {
        u32 dht_codemin_dc1:32;
    };
    u32 val;
} jpeg_dht_codemin_dc1_reg_t;
typedef union {
    struct {
        u32 dht_codemin_ac1:32;
    };
    u32 val;
} jpeg_dht_codemin_ac1_reg_t;
typedef union {
    struct {
        u32 decode_byte_cnt:26;
        u32 header_dec_st:4;
        u32 decode_sample_sel:2;
    };
    u32 val;
} jpeg_decoder_status0_reg_t;
typedef union {
    struct {
        u32 encode_data:16;
        u32 count_q:7;
        u32 mcu_fsm_ready:1;
        u32 decode_data:8;
    };
    u32 val;
} jpeg_decoder_status1_reg_t;
typedef union {
    struct {
        u32 comp_block_num:26;
        u32 scan_num:3;
        u32 rst_check_wait:1;
        u32 scan_check_wait:1;
        u32 mcu_in_proc:1;
    };
    u32 val;
} jpeg_decoder_status2_reg_t;
typedef union {
    struct {
        u32 lookup_data:32;
    };
    u32 val;
} jpeg_decoder_status3_reg_t;
typedef union {
    struct {
        u32 block_eof_cnt:26;
        u32 dezigzag_ready:1;
        u32 de_frame_eof_check:1;
        u32 de_dma2d_in_push:1;
        u32 reserved_29:3;
    };
    u32 val;
} jpeg_decoder_status4_reg_t;
typedef union {
    struct {
        u32 idct_hfm_data:16;
        u32 ns0:3;
        u32 ns1:3;
        u32 ns2:3;
        u32 ns3:3;
        u32 data_last_o:1;
        u32 rdn_result:1;
        u32 rdn_ena:1;
        u32 reserved_31:1;
    };
    u32 val;
} jpeg_decoder_status5_reg_t;
typedef union {
    struct {
        u32 pic_block_num:24;
        u32 reserved_24:8;
    };
    u32 val;
} jpeg_status5_reg_t;
typedef union {
    struct {
        u32 rdn_eco_low:32;
    };
    u32 val;
} jpeg_eco_low_reg_t;
typedef union {
    struct {
        u32 rdn_eco_high:32;
    };
    u32 val;
} jpeg_eco_high_reg_t;
typedef union {
    struct {
        u32 encode_timeout_thres:6;
        u32 reserved_6:26;
    };
    u32 val;
} jpeg_config1_reg_t;
typedef union {
    struct {
        u32 reserved_0:31;
        u32 clk_en:1;
    };
    u32 val;
} jpeg_sys_reg_t;
typedef union {
    struct {
        u32 jpeg_ver:28;
        u32 reserved_28:4;
    };
    u32 val;
} jpeg_version_reg_t;
typedef struct jpeg_dev_t {
    volatile jpeg_config_reg_t config;
    volatile jpeg_dqt_info_reg_t dqt_info;
    volatile jpeg_pic_size_reg_t pic_size;
    u32 reserved_00c;
    volatile jpeg_t0qnr_reg_t t0qnr;
    volatile jpeg_t1qnr_reg_t t1qnr;
    volatile jpeg_t2qnr_reg_t t2qnr;
    volatile jpeg_t3qnr_reg_t t3qnr;
    volatile jpeg_decode_conf_reg_t decode_conf;
    volatile jpeg_c0_reg_t c0;
    volatile jpeg_c1_reg_t c1;
    volatile jpeg_c2_reg_t c2;
    volatile jpeg_c3_reg_t c3;
    volatile jpeg_dht_info_reg_t dht_info;
    volatile jpeg_int_raw_reg_t int_raw;
    volatile jpeg_int_ena_reg_t int_ena;
    volatile jpeg_int_st_reg_t int_st;
    volatile jpeg_int_clr_reg_t int_clr;
    volatile jpeg_status0_reg_t status0;
    volatile jpeg_status2_reg_t status2;
    volatile jpeg_status3_reg_t status3;
    volatile jpeg_status4_reg_t status4;
    volatile jpeg_dht_totlen_dc0_reg_t dht_totlen_dc0;
    volatile jpeg_dht_val_dc0_reg_t dht_val_dc0;
    volatile jpeg_dht_totlen_ac0_reg_t dht_totlen_ac0;
    volatile jpeg_dht_val_ac0_reg_t dht_val_ac0;
    volatile jpeg_dht_totlen_dc1_reg_t dht_totlen_dc1;
    volatile jpeg_dht_val_dc1_reg_t dht_val_dc1;
    volatile jpeg_dht_totlen_ac1_reg_t dht_totlen_ac1;
    volatile jpeg_dht_val_ac1_reg_t dht_val_ac1;
    volatile jpeg_dht_codemin_dc0_reg_t dht_codemin_dc0;
    volatile jpeg_dht_codemin_ac0_reg_t dht_codemin_ac0;
    volatile jpeg_dht_codemin_dc1_reg_t dht_codemin_dc1;
    volatile jpeg_dht_codemin_ac1_reg_t dht_codemin_ac1;
    volatile jpeg_decoder_status0_reg_t decoder_status0;
    volatile jpeg_decoder_status1_reg_t decoder_status1;
    volatile jpeg_decoder_status2_reg_t decoder_status2;
    volatile jpeg_decoder_status3_reg_t decoder_status3;
    volatile jpeg_decoder_status4_reg_t decoder_status4;
    volatile jpeg_decoder_status5_reg_t decoder_status5;
    volatile jpeg_status5_reg_t status5;
    volatile jpeg_eco_low_reg_t eco_low;
    volatile jpeg_eco_high_reg_t eco_high;
    volatile jpeg_config1_reg_t config1;
    u32 reserved_0b0[18];
    volatile jpeg_sys_reg_t sys;
    volatile jpeg_version_reg_t version;
} jpeg_dev_t;
_Static_assert(sizeof(jpeg_dev_t) == 0x100, "Invalid size of jpeg_dev_t structure");
#endif
