/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2025 Espressif Systems (Shanghai) CO LTD.
 * Register layouts from ESP-IDF 08e0d30a74ad (MIT option), with descriptive
 * comments removed and Linux integer types. Keep reserved words/bitfields:
 * they preserve the silicon register offsets. No IDF runtime dependency.
 */
#ifndef _S31_DMA2D_REGS_H
#define _S31_DMA2D_REGS_H
#include <linux/types.h>
typedef union {
    struct {
        u32 out_auto_wrback_chn:1;
        u32 out_eof_mode_chn:1;
        u32 outdscr_burst_en_chn:1;
        u32 out_ecc_aes_en_chn:1;
        u32 out_check_owner_chn:1;
        u32 out_loop_test_chn:1;
        u32 out_mem_burst_length_chn:3;
        u32 out_macro_block_size_chn:2;
        u32 out_dscr_port_en_chn:1;
        u32 out_page_bound_en_chn:1;
        u32 reserved_13:3;
        u32 out_reorder_en_chn:1;
        u32 reserved_17:7;
        u32 out_rst_chn:1;
        u32 out_cmd_disable_chn:1;
        u32 out_arb_weight_opt_dis_chn:1;
        u32 reserved_27:5;
    };
    u32 val;
} dma2d_out_conf0_chn_reg_t;
typedef union {
    struct {
        u32 outfifo_wdata_chn:10;
        u32 outfifo_push_chn:1;
        u32 reserved_11:21;
    };
    u32 val;
} dma2d_out_push_chn_reg_t;
typedef union {
    struct {
        u32 reserved_0:20;
        u32 outlink_stop_chn:1;
        u32 outlink_start_chn:1;
        u32 outlink_restart_chn:1;
        u32 outlink_park_chn:1;
        u32 reserved_24:8;
    };
    u32 val;
} dma2d_out_link_conf_chn_reg_t;
typedef union {
    struct {
        u32 outlink_addr_chn:32;
    };
    u32 val;
} dma2d_out_link_addr_chn_reg_t;
typedef union {
    struct {
        u32 out_arb_token_num_chn:4;
        u32 out_arb_priority_chn:4;
        u32 reserved_8:24;
    };
    u32 val;
} dma2d_out_arb_chn_reg_t;
typedef union {
    struct {
        u32 reserved_0:4;
        u32 out_ro_ram_force_pd_chn:1;
        u32 out_ro_ram_force_pu_chn:1;
        u32 out_ro_ram_clk_fo_chn:1;
        u32 reserved_7:25;
    };
    u32 val;
} dma2d_out_ro_pd_conf_chn_reg_t;
typedef union {
    struct {
        u32 out_color_output_sel_chn:2;
        u32 out_color_3b_proc_en_chn:1;
        u32 out_color_input_sel_chn:3;
        u32 reserved_6:26;
    };
    u32 val;
} dma2d_out_color_convert_chn_reg_t;
typedef union {
    struct {
        u32 out_scramble_sel_pre_chn:3;
        u32 reserved_3:29;
    };
    u32 val;
} dma2d_out_scramble_chn_reg_t;
typedef union {
    struct {
        u32 out_color_param_h0_chn:21;
        u32 reserved_21:11;
    };
    u32 val;
} dma2d_out_color_param0_chn_reg_t;
typedef union {
    struct {
        u32 out_color_param_h1_chn:28;
        u32 reserved_28:4;
    };
    u32 val;
} dma2d_out_color_param1_chn_reg_t;
typedef union {
    struct {
        u32 out_color_param_m0_chn:21;
        u32 reserved_21:11;
    };
    u32 val;
} dma2d_out_color_param2_chn_reg_t;
typedef union {
    struct {
        u32 out_color_param_m1_chn:28;
        u32 reserved_28:4;
    };
    u32 val;
} dma2d_out_color_param3_chn_reg_t;
typedef union {
    struct {
        u32 out_color_param_l0_chn:21;
        u32 reserved_21:11;
    };
    u32 val;
} dma2d_out_color_param4_chn_reg_t;
typedef union {
    struct {
        u32 out_color_param_l1_chn:28;
        u32 reserved_28:4;
    };
    u32 val;
} dma2d_out_color_param5_chn_reg_t;
typedef union {
    struct {
        u32 out_etm_en_chn:1;
        u32 out_etm_loop_en_chn:1;
        u32 out_dscr_task_mak_chn:2;
        u32 reserved_4:28;
    };
    u32 val;
} dma2d_out_etm_conf_chn_reg_t;
typedef union {
    struct {
        u32 out_dscr_port_blk_h_chn:14;
        u32 out_dscr_port_blk_v_chn:14;
        u32 reserved_28:4;
    };
    u32 val;
} dma2d_out_dscr_port_blk_chn_reg_t;
typedef union {
    struct {
        u32 in_mem_trans_en_chn:1;
        u32 reserved_1:1;
        u32 indscr_burst_en_chn:1;
        u32 in_ecc_aes_en_chn:1;
        u32 in_check_owner_chn:1;
        u32 in_loop_test_chn:1;
        u32 in_mem_burst_length_chn:3;
        u32 in_macro_block_size_chn:2;
        u32 in_dscr_port_en_chn:1;
        u32 in_page_bound_en_chn:1;
        u32 reserved_13:3;
        u32 in_reorder_en_chn:1;
        u32 reserved_17:7;
        u32 in_rst_chn:1;
        u32 in_cmd_disable_chn:1;
        u32 in_arb_weight_opt_dis_chn:1;
        u32 reserved_27:5;
    };
    u32 val;
} dma2d_in_conf0_chn_reg_t;
typedef union {
    struct {
        u32 infifo_rdata_chn:11;
        u32 infifo_pop_chn:1;
        u32 reserved_12:20;
    };
    u32 val;
} dma2d_in_pop_chn_reg_t;
typedef union {
    struct {
        u32 reserved_0:20;
        u32 inlink_auto_ret_chn:1;
        u32 inlink_stop_chn:1;
        u32 inlink_start_chn:1;
        u32 inlink_restart_chn:1;
        u32 inlink_park_chn:1;
        u32 reserved_25:7;
    };
    u32 val;
} dma2d_in_link_conf_chn_reg_t;
typedef union {
    struct {
        u32 inlink_addr_chn:32;
    };
    u32 val;
} dma2d_in_link_addr_chn_reg_t;
typedef union {
    struct {
        u32 in_arb_token_num_chn:4;
        u32 in_arb_priority_chn:4;
        u32 reserved_8:24;
    };
    u32 val;
} dma2d_in_arb_chn_reg_t;
typedef union {
    struct {
        u32 reserved_0:4;
        u32 in_ro_ram_force_pd_chn:1;
        u32 in_ro_ram_force_pu_chn:1;
        u32 in_ro_ram_clk_fo_chn:1;
        u32 reserved_7:25;
    };
    u32 val;
} dma2d_in_ro_pd_conf_chn_reg_t;
typedef union {
    struct {
        u32 in_color_output_sel_chn:2;
        u32 in_color_3b_proc_en_chn:1;
        u32 in_color_input_sel_chn:3;
        u32 reserved_6:26;
    };
    u32 val;
} dma2d_in_color_convert_chn_reg_t;
typedef union {
    struct {
        u32 in_scramble_sel_pre_chn:3;
        u32 in_scramble_sel_post_chn:3;
        u32 reserved_6:26;
    };
    u32 val;
} dma2d_in_scramble_chn_reg_t;
typedef union {
    struct {
        u32 in_color_param_h0_chn:21;
        u32 reserved_21:11;
    };
    u32 val;
} dma2d_in_color_param0_chn_reg_t;
typedef union {
    struct {
        u32 in_color_param_h1_chn:28;
        u32 reserved_28:4;
    };
    u32 val;
} dma2d_in_color_param1_chn_reg_t;
typedef union {
    struct {
        u32 in_color_param_m0_chn:21;
        u32 reserved_21:11;
    };
    u32 val;
} dma2d_in_color_param2_chn_reg_t;
typedef union {
    struct {
        u32 in_color_param_m1_chn:28;
        u32 reserved_28:4;
    };
    u32 val;
} dma2d_in_color_param3_chn_reg_t;
typedef union {
    struct {
        u32 in_color_param_l0_chn:21;
        u32 reserved_21:11;
    };
    u32 val;
} dma2d_in_color_param4_chn_reg_t;
typedef union {
    struct {
        u32 in_color_param_l1_chn:28;
        u32 reserved_28:4;
    };
    u32 val;
} dma2d_in_color_param5_chn_reg_t;
typedef union {
    struct {
        u32 in_etm_en_chn:1;
        u32 in_etm_loop_en_chn:1;
        u32 in_dscr_task_mak_chn:2;
        u32 reserved_4:28;
    };
    u32 val;
} dma2d_in_etm_conf_chn_reg_t;
typedef union {
    struct {
        u32 axim_rd_rst:1;
        u32 axim_wr_rst:1;
        u32 clk_en:1;
        u32 reserved_3:29;
    };
    u32 val;
} dma2d_rst_conf_reg_t;
typedef union {
    struct {
        u32 access_intr_mem_start_addr:32;
    };
    u32 val;
} dma2d_intr_mem_start_addr_reg_t;
typedef union {
    struct {
        u32 access_intr_mem_end_addr:32;
    };
    u32 val;
} dma2d_intr_mem_end_addr_reg_t;
typedef union {
    struct {
        u32 access_extr_mem_start_addr:32;
    };
    u32 val;
} dma2d_extr_mem_start_addr_reg_t;
typedef union {
    struct {
        u32 access_extr_mem_end_addr:32;
    };
    u32 val;
} dma2d_extr_mem_end_addr_reg_t;
typedef union {
    struct {
        u32 out_arb_timeout_num:16;
        u32 out_weight_en:1;
        u32 reserved_17:15;
    };
    u32 val;
} dma2d_out_arb_config_reg_t;
typedef union {
    struct {
        u32 in_arb_timeout_num:16;
        u32 in_weight_en:1;
        u32 reserved_17:15;
    };
    u32 val;
} dma2d_in_arb_config_reg_t;
typedef union {
    struct {
        u32 rdn_ena:1;
        u32 rdn_result:1;
        u32 reserved_2:30;
    };
    u32 val;
} dma2d_rdn_result_reg_t;
typedef union {
    struct {
        u32 rdn_eco_high:32;
    };
    u32 val;
} dma2d_rdn_eco_high_reg_t;
typedef union {
    struct {
        u32 rdn_eco_low:32;
    };
    u32 val;
} dma2d_rdn_eco_low_reg_t;
typedef union {
    struct {
        u32 out_done_chn_int_raw:1;
        u32 out_eof_chn_int_raw:1;
        u32 out_dscr_err_chn_int_raw:1;
        u32 out_total_eof_chn_int_raw:1;
        u32 outfifo_ovf_l1_chn_int_raw:1;
        u32 outfifo_udf_l1_chn_int_raw:1;
        u32 outfifo_ovf_l2_chn_int_raw:1;
        u32 outfifo_udf_l2_chn_int_raw:1;
        u32 outfifo_ovf_l3_chn_int_raw:1;
        u32 outfifo_udf_l3_chn_int_raw:1;
        u32 outfifo_ro_ovf_chn_int_raw:1;
        u32 outfifo_ro_udf_chn_int_raw:1;
        u32 out_dscr_task_ovf_chn_int_raw:1;
        u32 reserved_13:19;
    };
    u32 val;
} dma2d_out_int_raw_chn_reg_t;
typedef union {
    struct {
        u32 out_done_chn_int_ena:1;
        u32 out_eof_chn_int_ena:1;
        u32 out_dscr_err_chn_int_ena:1;
        u32 out_total_eof_chn_int_ena:1;
        u32 outfifo_ovf_l1_chn_int_ena:1;
        u32 outfifo_udf_l1_chn_int_ena:1;
        u32 outfifo_ovf_l2_chn_int_ena:1;
        u32 outfifo_udf_l2_chn_int_ena:1;
        u32 outfifo_ovf_l3_chn_int_ena:1;
        u32 outfifo_udf_l3_chn_int_ena:1;
        u32 outfifo_ro_ovf_chn_int_ena:1;
        u32 outfifo_ro_udf_chn_int_ena:1;
        u32 out_dscr_task_ovf_chn_int_ena:1;
        u32 reserved_13:19;
    };
    u32 val;
} dma2d_out_int_ena_chn_reg_t;
typedef union {
    struct {
        u32 out_done_chn_int_st:1;
        u32 out_eof_chn_int_st:1;
        u32 out_dscr_err_chn_int_st:1;
        u32 out_total_eof_chn_int_st:1;
        u32 outfifo_ovf_l1_chn_int_st:1;
        u32 outfifo_udf_l1_chn_int_st:1;
        u32 outfifo_ovf_l2_chn_int_st:1;
        u32 outfifo_udf_l2_chn_int_st:1;
        u32 outfifo_ovf_l3_chn_int_st:1;
        u32 outfifo_udf_l3_chn_int_st:1;
        u32 outfifo_ro_ovf_chn_int_st:1;
        u32 outfifo_ro_udf_chn_int_st:1;
        u32 out_dscr_task_ovf_chn_int_st:1;
        u32 reserved_13:19;
    };
    u32 val;
} dma2d_out_int_st_chn_reg_t;
typedef union {
    struct {
        u32 out_done_chn_int_clr:1;
        u32 out_eof_chn_int_clr:1;
        u32 out_dscr_err_chn_int_clr:1;
        u32 out_total_eof_chn_int_clr:1;
        u32 outfifo_ovf_l1_chn_int_clr:1;
        u32 outfifo_udf_l1_chn_int_clr:1;
        u32 outfifo_ovf_l2_chn_int_clr:1;
        u32 outfifo_udf_l2_chn_int_clr:1;
        u32 outfifo_ovf_l3_chn_int_clr:1;
        u32 outfifo_udf_l3_chn_int_clr:1;
        u32 outfifo_ro_ovf_chn_int_clr:1;
        u32 outfifo_ro_udf_chn_int_clr:1;
        u32 out_dscr_task_ovf_chn_int_clr:1;
        u32 reserved_13:19;
    };
    u32 val;
} dma2d_out_int_clr_chn_reg_t;
typedef union {
    struct {
        u32 in_done_chn_int_raw:1;
        u32 in_suc_eof_chn_int_raw:1;
        u32 in_err_eof_chn_int_raw:1;
        u32 in_dscr_err_chn_int_raw:1;
        u32 infifo_ovf_l1_chn_int_raw:1;
        u32 infifo_udf_l1_chn_int_raw:1;
        u32 infifo_ovf_l2_chn_int_raw:1;
        u32 infifo_udf_l2_chn_int_raw:1;
        u32 infifo_ovf_l3_chn_int_raw:1;
        u32 infifo_udf_l3_chn_int_raw:1;
        u32 in_dscr_empty_chn_int_raw:1;
        u32 infifo_ro_ovf_chn_int_raw:1;
        u32 infifo_ro_udf_chn_int_raw:1;
        u32 in_dscr_task_ovf_chn_int_raw:1;
        u32 reserved_14:18;
    };
    u32 val;
} dma2d_in_int_raw_chn_reg_t;
typedef union {
    struct {
        u32 in_done_chn_int_ena:1;
        u32 in_suc_eof_chn_int_ena:1;
        u32 in_err_eof_chn_int_ena:1;
        u32 in_dscr_err_chn_int_ena:1;
        u32 infifo_ovf_l1_chn_int_ena:1;
        u32 infifo_udf_l1_chn_int_ena:1;
        u32 infifo_ovf_l2_chn_int_ena:1;
        u32 infifo_udf_l2_chn_int_ena:1;
        u32 infifo_ovf_l3_chn_int_ena:1;
        u32 infifo_udf_l3_chn_int_ena:1;
        u32 in_dscr_empty_chn_int_ena:1;
        u32 infifo_ro_ovf_chn_int_ena:1;
        u32 infifo_ro_udf_chn_int_ena:1;
        u32 in_dscr_task_ovf_chn_int_ena:1;
        u32 reserved_14:18;
    };
    u32 val;
} dma2d_in_int_ena_chn_reg_t;
typedef union {
    struct {
        u32 in_done_chn_int_st:1;
        u32 in_suc_eof_chn_int_st:1;
        u32 in_err_eof_chn_int_st:1;
        u32 in_dscr_err_chn_int_st:1;
        u32 infifo_ovf_l1_chn_int_st:1;
        u32 infifo_udf_l1_chn_int_st:1;
        u32 infifo_ovf_l2_chn_int_st:1;
        u32 infifo_udf_l2_chn_int_st:1;
        u32 infifo_ovf_l3_chn_int_st:1;
        u32 infifo_udf_l3_chn_int_st:1;
        u32 in_dscr_empty_chn_int_st:1;
        u32 infifo_ro_ovf_chn_int_st:1;
        u32 infifo_ro_udf_chn_int_st:1;
        u32 in_dscr_task_ovf_chn_int_st:1;
        u32 reserved_14:18;
    };
    u32 val;
} dma2d_in_int_st_chn_reg_t;
typedef union {
    struct {
        u32 in_done_chn_int_clr:1;
        u32 in_suc_eof_chn_int_clr:1;
        u32 in_err_eof_chn_int_clr:1;
        u32 in_dscr_err_chn_int_clr:1;
        u32 infifo_ovf_l1_chn_int_clr:1;
        u32 infifo_udf_l1_chn_int_clr:1;
        u32 infifo_ovf_l2_chn_int_clr:1;
        u32 infifo_udf_l2_chn_int_clr:1;
        u32 infifo_ovf_l3_chn_int_clr:1;
        u32 infifo_udf_l3_chn_int_clr:1;
        u32 in_dscr_empty_chn_int_clr:1;
        u32 infifo_ro_ovf_chn_int_clr:1;
        u32 infifo_ro_udf_chn_int_clr:1;
        u32 in_dscr_task_ovf_chn_int_clr:1;
        u32 reserved_14:18;
    };
    u32 val;
} dma2d_in_int_clr_chn_reg_t;
typedef union {
    struct {
        u32 outfifo_full_l2_chn:1;
        u32 outfifo_empty_l2_chn:1;
        u32 outfifo_cnt_l2_chn:4;
        u32 reserved_6:1;
        u32 out_remain_under_1b_chn:1;
        u32 out_remain_under_2b_chn:1;
        u32 out_remain_under_3b_chn:1;
        u32 out_remain_under_4b_chn:1;
        u32 out_remain_under_5b_chn:1;
        u32 out_remain_under_6b_chn:1;
        u32 out_remain_under_7b_chn:1;
        u32 out_remain_under_8b_chn:1;
        u32 outfifo_full_l1_chn:1;
        u32 outfifo_empty_l1_chn:1;
        u32 outfifo_cnt_l1_chn:5;
        u32 outfifo_full_l3_chn:1;
        u32 outfifo_empty_l3_chn:1;
        u32 outfifo_cnt_l3_chn:5;
        u32 reserved_29:3;
    };
    u32 val;
} dma2d_outfifo_status_chn_reg_t;
typedef union {
    struct {
        u32 outlink_dscr_addr_chn:18;
        u32 out_dscr_state_chn:2;
        u32 out_state_chn:4;
        u32 out_reset_avail_chn:1;
        u32 reserved_25:7;
    };
    u32 val;
} dma2d_out_state_chn_reg_t;
typedef union {
    struct {
        u32 out_eof_des_addr_chn:32;
    };
    u32 val;
} dma2d_out_eof_des_addr_chn_reg_t;
typedef union {
    struct {
        u32 outlink_dscr_chn:32;
    };
    u32 val;
} dma2d_out_dscr_chn_reg_t;
typedef union {
    struct {
        u32 outlink_dscr_bf0_chn:32;
    };
    u32 val;
} dma2d_out_dscr_bf0_chn_reg_t;
typedef union {
    struct {
        u32 outlink_dscr_bf1_chn:32;
    };
    u32 val;
} dma2d_out_dscr_bf1_chn_reg_t;
typedef union {
    struct {
        u32 outfifo_ro_cnt_chn:6;
        u32 out_ro_wr_state_chn:2;
        u32 out_ro_rd_state_chn:2;
        u32 out_pixel_byte_chn:4;
        u32 out_burst_block_num_chn:4;
        u32 reserved_18:14;
    };
    u32 val;
} dma2d_out_ro_status_chn_reg_t;
typedef union {
    struct {
        u32 infifo_full_l2_chn:1;
        u32 infifo_empty_l2_chn:1;
        u32 infifo_cnt_l2_chn:4;
        u32 reserved_6:1;
        u32 in_remain_under_1b_chn:1;
        u32 in_remain_under_2b_chn:1;
        u32 in_remain_under_3b_chn:1;
        u32 in_remain_under_4b_chn:1;
        u32 in_remain_under_5b_chn:1;
        u32 in_remain_under_6b_chn:1;
        u32 in_remain_under_7b_chn:1;
        u32 in_remain_under_8b_chn:1;
        u32 infifo_full_l1_chn:1;
        u32 infifo_empty_l1_chn:1;
        u32 infifo_cnt_l1_chn:5;
        u32 infifo_full_l3_chn:1;
        u32 infifo_empty_l3_chn:1;
        u32 infifo_cnt_l3_chn:5;
        u32 reserved_29:3;
    };
    u32 val;
} dma2d_infifo_status_chn_reg_t;
typedef union {
    struct {
        u32 inlink_dscr_addr_chn:18;
        u32 in_dscr_state_chn:2;
        u32 in_state_chn:3;
        u32 in_reset_avail_chn:1;
        u32 reserved_24:8;
    };
    u32 val;
} dma2d_in_state_chn_reg_t;
typedef union {
    struct {
        u32 in_suc_eof_des_addr_chn:32;
    };
    u32 val;
} dma2d_in_suc_eof_des_addr_chn_reg_t;
typedef union {
    struct {
        u32 in_err_eof_des_addr_chn:32;
    };
    u32 val;
} dma2d_in_err_eof_des_addr_chn_reg_t;
typedef union {
    struct {
        u32 inlink_dscr_chn:32;
    };
    u32 val;
} dma2d_in_dscr_chn_reg_t;
typedef union {
    struct {
        u32 inlink_dscr_bf0_chn:32;
    };
    u32 val;
} dma2d_in_dscr_bf0_chn_reg_t;
typedef union {
    struct {
        u32 inlink_dscr_bf1_chn:32;
    };
    u32 val;
} dma2d_in_dscr_bf1_chn_reg_t;
typedef union {
    struct {
        u32 infifo_ro_cnt_chn:5;
        u32 in_ro_wr_state_chn:2;
        u32 in_ro_rd_state_chn:2;
        u32 in_pixel_byte_chn:4;
        u32 in_burst_block_num_chn:4;
        u32 reserved_17:15;
    };
    u32 val;
} dma2d_in_ro_status_chn_reg_t;
typedef union {
    struct {
        u32 rid_err_cnt:4;
        u32 rresp_err_cnt:4;
        u32 wresp_err_cnt:4;
        u32 rd_fifo_cnt:3;
        u32 rd_bak_fifo_cnt:4;
        u32 wr_fifo_cnt:3;
        u32 wr_bak_fifo_cnt:4;
        u32 reserved_26:6;
    };
    u32 val;
} dma2d_axi_err_reg_t;
typedef union {
    struct {
        u32 date:32;
    };
    u32 val;
} dma2d_date_reg_t;
typedef union {
    struct {
        u32 out_peri_sel_chn:3;
        u32 reserved_3:29;
    };
    u32 val;
} dma2d_out_peri_sel_chn_reg_t;
typedef union {
    struct {
        u32 in_peri_sel_chn:3;
        u32 reserved_3:29;
    };
    u32 val;
} dma2d_in_peri_sel_chn_reg_t;
typedef union {
    struct {
        struct {
            u32 a                             :    10;
            u32 b                             :    11;
            u32 reserved21                    :    11;
        };
        struct {
            u32 c                             :    10;
            u32 d                             :    18;
            u32 reserved60                    :    4;
        };
    };
    u32 val[2];
} dma2d_color_param_reg_t;
typedef struct {
    volatile dma2d_color_param_reg_t param_h;
    volatile dma2d_color_param_reg_t param_m;
    volatile dma2d_color_param_reg_t param_l;
} dma2d_color_param_group_chn_reg_t;
typedef struct {
    volatile dma2d_out_conf0_chn_reg_t out_conf0;
    volatile dma2d_out_int_raw_chn_reg_t out_int_raw;
    volatile dma2d_out_int_ena_chn_reg_t out_int_ena;
    volatile dma2d_out_int_st_chn_reg_t out_int_st;
    volatile dma2d_out_int_clr_chn_reg_t out_int_clr;
    volatile dma2d_outfifo_status_chn_reg_t outfifo_status;
    volatile dma2d_out_push_chn_reg_t out_push;
    volatile dma2d_out_link_conf_chn_reg_t out_link_conf;
    volatile dma2d_out_link_addr_chn_reg_t out_link_addr;
    volatile dma2d_out_state_chn_reg_t out_state;
    volatile dma2d_out_eof_des_addr_chn_reg_t out_eof_des_addr;
    volatile dma2d_out_dscr_chn_reg_t out_dscr;
    volatile dma2d_out_dscr_bf0_chn_reg_t out_dscr_bf0;
    volatile dma2d_out_dscr_bf1_chn_reg_t out_dscr_bf1;
    volatile dma2d_out_peri_sel_chn_reg_t out_peri_sel;
    volatile dma2d_out_arb_chn_reg_t out_arb;
    volatile dma2d_out_ro_status_chn_reg_t out_ro_status;
    volatile dma2d_out_ro_pd_conf_chn_reg_t out_ro_pd_conf;
    volatile dma2d_out_color_convert_chn_reg_t out_color_convert;
    volatile dma2d_out_scramble_chn_reg_t out_scramble;
    volatile dma2d_color_param_group_chn_reg_t out_color_param_group;
    volatile dma2d_out_etm_conf_chn_reg_t out_etm_conf;
    volatile dma2d_out_dscr_port_blk_chn_reg_t out_dscr_port_blk;
    u32 reserved_out[36];
} dma2d_out_chn_reg_t;
typedef struct {
    volatile dma2d_in_conf0_chn_reg_t in_conf0;
    volatile dma2d_in_int_raw_chn_reg_t in_int_raw;
    volatile dma2d_in_int_ena_chn_reg_t in_int_ena;
    volatile dma2d_in_int_st_chn_reg_t in_int_st;
    volatile dma2d_in_int_clr_chn_reg_t in_int_clr;
    volatile dma2d_infifo_status_chn_reg_t infifo_status;
    volatile dma2d_in_pop_chn_reg_t in_pop;
    volatile dma2d_in_link_conf_chn_reg_t in_link_conf;
    volatile dma2d_in_link_addr_chn_reg_t in_link_addr;
    volatile dma2d_in_state_chn_reg_t in_state;
    volatile dma2d_in_suc_eof_des_addr_chn_reg_t in_suc_eof_des_addr;
    volatile dma2d_in_err_eof_des_addr_chn_reg_t in_err_eof_des_addr;
    volatile dma2d_in_dscr_chn_reg_t in_dscr;
    volatile dma2d_in_dscr_bf0_chn_reg_t in_dscr_bf0;
    volatile dma2d_in_dscr_bf1_chn_reg_t in_dscr_bf1;
    volatile dma2d_in_peri_sel_chn_reg_t in_peri_sel;
    volatile dma2d_in_arb_chn_reg_t in_arb;
    volatile dma2d_in_ro_status_chn_reg_t in_ro_status;
    volatile dma2d_in_ro_pd_conf_chn_reg_t in_ro_pd_conf;
    volatile dma2d_in_color_convert_chn_reg_t in_color_convert;
    volatile dma2d_in_scramble_chn_reg_t in_scramble;
    volatile dma2d_color_param_group_chn_reg_t in_color_param_group;
    volatile dma2d_in_etm_conf_chn_reg_t in_etm_conf;
    u32 reserved_in[36];
} dma2d_in_chn_reg_t;
typedef struct dma2d_dev_t {
    volatile dma2d_out_chn_reg_t out_channel[4];
    u32 reserved_400[64];
    volatile dma2d_in_chn_reg_t in_channel[3];
    u32 reserved_800[128];
    volatile dma2d_axi_err_reg_t axi_err;
    volatile dma2d_rst_conf_reg_t rst_conf;
    volatile dma2d_intr_mem_start_addr_reg_t intr_mem_start_addr;
    volatile dma2d_intr_mem_end_addr_reg_t intr_mem_end_addr;
    volatile dma2d_extr_mem_start_addr_reg_t extr_mem_start_addr;
    volatile dma2d_extr_mem_end_addr_reg_t extr_mem_end_addr;
    volatile dma2d_out_arb_config_reg_t out_arb_config;
    volatile dma2d_in_arb_config_reg_t in_arb_config;
    volatile dma2d_rdn_result_reg_t rdn_result;
    volatile dma2d_rdn_eco_high_reg_t rdn_eco_high;
    volatile dma2d_rdn_eco_low_reg_t rdn_eco_low;
    volatile dma2d_date_reg_t date;
} dma2d_dev_t;
_Static_assert(sizeof(dma2d_dev_t) == 0xa30, "Invalid size of dma2d_dev_t structure");
#endif
