/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <xtensa/config/core-isa.h>
#include "lv_macro_memset.S"

// This is LVGL ARGB8888 simple fill for ESP32, ESP32S2 processor

    .section .text
    .align  4
    .global lv_color_blend_to_argb8888_esp
    .type   lv_color_blend_to_argb8888_esp,@function

// The function implements the following C code:
// void lv_color_blend_to_argb8888(_lv_draw_sw_blend_fill_dsc_t * dsc);

// Input params
//
// dsc - a2

// typedef struct {
//     uint32_t opa;                l32i    0
//     void * dst_buf;              l32i    4
//     uint32_t dst_w;              l32i    8
//     uint32_t dst_h;              l32i    12
//     uint32_t dst_stride;         l32i    16
//     const void * src_buf;        l32i    20
//     uint32_t src_stride;         l32i    24
//     const lv_opa_t * mask_buf;   l32i    28
//     uint32_t mask_stride;        l32i    32
// } asm_dsc_t;

lv_color_blend_to_argb8888_esp:

    entry   a1,    32

    l32i.n   a3,    a2,    4                    // a3 - dest_buff
    l32i.n   a4,    a2,    8                    // a4 - dest_w                in uint32_t
    l32i.n   a5,    a2,    12                   // a5 - dest_h                in uint32_t
    l32i.n   a6,    a2,    16                   // a6 - dest_stride           in bytes
    l32i.n   a7,    a2,    20                   // a7 - src_buff (color)
    l32i.n   a8,    a7,    0                    // a8 - color as value
    slli     a11,   a4,    2                    // a11 - dest_w_bytes = sizeof(uint32_t) * dest_w

    beqz     a4,   _zero_matrix_len_check       // Check if dest_w a4 is zero
    beqz     a5,   _zero_matrix_len_check       // Check if dest_h a5 is zero
    movi     a7,    0xff000000                  // opacity mask
    or       a10,    a7,    a8                  // apply opacity
    sub      a6,     a6,   a11                  // dest_stride = dest_stride - dest_w_bytes

    // Check dest_w length
    bltui   a4,  8,  _matrix_width_check                // Branch if dest_w (a4) is lower than 8
    srli    a9,  a4,   3                                // a9 - loop_len = dest_w / 8

#if !XCHAL_HAVE_LOOPS
    slli    a14,  a9,  5                                // a14 = loop_len (a9) * 32 (main loop increments address pointers by 32)
#endif

    .outer_loop:

#if XCHAL_HAVE_LOOPS
        loopnez a9,  ._main_loop                        // zero-overhead loop (not supported for esp32s2)
#else
        // Init loop parameters
        beqz    a9,   ._main_loop                       // Branch to the end, if a9 is 0 (no need to run the main loop)
        add     a15,  a14,  a3                          // a15 = a14 + dest_buf address
        .main_loop_done:
#endif
        // Run main loop which sets 32 bytes (8 ARGB8888 pixels) in one loop run
            s32i.n      a10,  a3,  0                    // save 32 bits from a10 to dest_buff a3, offset 0 bytes
            s32i.n      a10,  a3,  4                    // save 32 bits from a10 to dest_buff a3, offset 4 bytes
            s32i.n      a10,  a3,  8                    // save 32 bits from a10 to dest_buff a3, offset 8 bytes
            s32i.n      a10,  a3,  12                   // save 32 bits from a10 to dest_buff a3, offset 12 bytes
            s32i.n      a10,  a3,  16                   // save 32 bits from a10 to dest_buff a3, offset 16 bytes
            s32i.n      a10,  a3,  20                   // save 32 bits from a10 to dest_buff a3, offset 20 bytes
            s32i.n      a10,  a3,  24                   // save 32 bits from a10 to dest_buff a3, offset 24 bytes
            s32i.n      a10,  a3,  28                   // save 32 bits from a10 to dest_buff a3, offset 28 bytes
            addi.n      a3,   a3,  32                   // increment dest_buff a3 pointer by 32 bytes
#if !XCHAL_HAVE_LOOPS
        blt     a3,   a15,  .main_loop_done             // Check end of the main loop, branch if dest_buf (a3) lower than a15
#endif
        ._main_loop:

        // Finish the remaining bytes out of the loop

        // Check modulo 16 of the dest_w_bytes (a11), if - then set 16 bytes (4 ARGB8888 pixels)
        // src_reg a10, dest_buff a3, dest_w_bytes a11
        macro_memset_mod_16 a10, a3, a11, __LINE__

        // Check modulo 8 of the dest_w_bytes (a11), if - then set 8 bytes (2 ARGB8888 pixels)
        // src_reg a10, dest_buff a3, dest_w_bytes a11
        macro_memset_mod_8 a10, a3, a11, __LINE__

        // Check modulo 4 of the dest_w_bytes (a11), if - then set 4 bytes (1 ARGB8888 pixel)
        // src_reg a10, dest_buff a3, dest_w_bytes a11
        macro_memset_mod_4 a10, a3, a11, __LINE__

        add     a3,  a3,  a6                             // dest_buff + dest_stride
        addi.n  a5,  a5,  -1                             // decrease the outer loop
    bnez a5, .outer_loop

    movi.n   a2, 1                                      // return LV_RESULT_OK = 1
    retw.n                                              // return

//**********************************************************************************************************************

    // Small matrix width, keep it simple for lengths less than 8 pixels

    _matrix_width_check:

#if !XCHAL_HAVE_LOOPS
    slli    a14,  a4,   2                               // a14 = loop_len (a9) * 4 (main loop increments address pointers by 4)
#endif

    .outer_loop_short_matrix:

#if XCHAL_HAVE_LOOPS
        loopnez a4,  ._main_loop_short_matrix           // zero-overhead loop (not supported for esp32s2)
#else
        // Init loop parameters
        add     a15,  a14,  a3                          // a15 = a14 + dest_buf address
        ._main_loop_short_matrix_done:
#endif
        // Run main loop which sets 4 bytes (one ARGB8888 pixel) in one loop run
            s32i.n      a10,  a3,  0                    // save 32 bits from a10 to dest_buff a3
            addi.n      a3,   a3,  4                    // increment dest_buff pointer by 4 bytes
#if !XCHAL_HAVE_LOOPS
        blt     a3,   a15,  ._main_loop_short_matrix_done             // Check end of the main loop, branch if dest_buf (a3) lower than a15
#endif
        ._main_loop_short_matrix:

        add     a3,  a3,  a6                            // dest_buff + dest_stride
        addi.n  a5,  a5,  -1                            // decrease the outer loop
    bnez a5, .outer_loop_short_matrix

    movi.n   a2, 1                                      // return LV_RESULT_OK = 1
    retw.n                                              // return

//**********************************************************************************************************************

    // One of the matrix dimensions is zero, return early
    _zero_matrix_len_check:
    movi.n   a2, 1                              // return LV_RESULT_OK = 1
    retw.n                                      // return
