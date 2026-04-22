/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <xtensa/config/core-isa.h>
#include "lv_macro_memset.S"

// This is LVGL RGB565 simple fill for ESP32, ESP32S2 processor

    .section .text
    .align  4
    .global lv_color_blend_to_rgb565_esp
    .type   lv_color_blend_to_rgb565_esp,@function
// The function implements the following C code:
// void lv_color_blend_to_rgb565(_lv_draw_sw_blend_fill_dsc_t * dsc);

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

lv_color_blend_to_rgb565_esp:

    entry   a1,    32

    l32i.n   a3,    a2,    4                    // a3 - dest_buff
    l32i.n   a4,    a2,    8                    // a4 - dest_w                in uint16_t
    l32i.n   a5,    a2,    12                   // a5 - dest_h                in uint16_t
    l32i.n   a6,    a2,    16                   // a6 - dest_stride           in bytes
    l32i.n   a7,    a2,    20                   // a7 - src_buff (color)
    l32i.n   a8,    a7,    0                    // a8 - color as value
    slli     a11,   a4,    1                    // a11 - dest_w_bytes = sizeof(uint16_t) * dest_w

    beqz     a4,   _zero_matrix_len_check       // Check if dest_w a4 is zero
    beqz     a5,   _zero_matrix_len_check       // Check if dest_h a5 is zero

    // Convert color to rgb565
    l8ui    a15,    a7,    2                    // red
    movi.n  a14,    0xf8
    and     a13,    a15,   a14
    slli    a10,    a13,   8

    l8ui    a15,    a7,    0                    // blue
    and     a13,    a15,   a14
    srli    a12,    a13,   3
    add     a10,    a10,   a12

    l8ui    a15,    a7,    1                    // green
    movi.n  a14,    0xfc
    and     a13,    a15,   a14
    slli    a12,    a13,   3
    add     a12,    a10,   a12                  // a12 = 16-bit color

    slli    a10,    a12,   16
    movi.n  a13,    0xFFFF0000
    and     a10,    a10,    a13
    or      a10,    a10,    a12                 // a10 = 32-bit color (16bit + (16bit << 16))

    movi.n  a8,    0x3                          // a8 = 0x3, dest_buff align mask
    sub     a6,    a6,   a11                    // dest_stride = dest_stride - dest_w_bytes

    // Check dest_w length
    bltui   a4,  8,  _matrix_width_check         // Branch if dest_w (a4) is lower than 8

    // cache init
    // Prepare main loop length and dest_w_bytes
    srli     a9,     a4,    4                    // a9 = loop_len = dest_w / 8, calculate main loop_len for original dest_w
    slli     a11,    a4,    1                    // a11 = dest_w_bytes = sizeof(uint16_t) * dest_w
    addi     a4,     a4,   -1                    // a4-- (decrement a4)
    s32i.n   a9,     a1,    0                    // cache.orig.loop_len
    s32i.n   a11,    a1,    4                    // cache.orig.dest_w_bytes

    // Prepare decreased main loop length and dest_w_bytes
    srli     a9,     a4,    4                    // a9 = loop_len = dest_w / 8, calculate main loop_len for dest_w - 1
    slli     a11,    a4,    1                    // a11 = dest_w_bytes = sizeof(uint16_t) * (dest_w - 1)
    s32i.n   a9,     a1,    8                    // cache.decr.loop_len
    s32i.n   a11,    a1,    12                   // cache.decr.dest_w_bytes
    and      a7,     a8,    a3                   // a7 = dest_buff AND 0x3 (chck if the address is 4-byte aligned)

    .outer_loop:

        // Check if the des_buff is 2-byte aligned
        beqz  a7, _dest_buff_2_byte_aligned      // branch if a7 is equal to zero
            s16i        a12,  a3,   0            // save 16 bits from 16-bit color a12 to dest_buff a3, offset 0
            l32i.n      a9,   a1,   8            // a9  = load cache.decr.loop_len
            l32i.n      a11,  a1,   12           // a11 = load cache.decr.dest_w_bytes
            addi.n      a3,   a3,   2            // increment dest_buff pointer by 2
            j           _dest_buff_unaligned
        _dest_buff_2_byte_aligned:

        l32i.n      a9,   a1,   0                // a11 = load cache.orig.loop_len
        l32i.n      a11,  a1,   4                // a11 = load cache.orig.dest_w_bytes

        _dest_buff_unaligned:

#if XCHAL_HAVE_LOOPS
        loopnez a9,   ._main_loop                       // zero-overhead loop (not supported in esp32s2)
#else
        // Init loop parameters
        beqz    a9,   ._main_loop                       // Branch to the end, if a9 is 0 (no need to run the main loop)
        slli    a15,  a9,   5                           // a15 = loop_len (a9) * 32 (main loop increments address pointers by 32)
        add     a15,  a15,  a3                          // a15 += dest_buf address
        ._main_loop_aligned_done:
#endif
        // Run main loop which sets 32 bytes (16 RGB565 pixels) in one loop run
            s32i.n      a10,  a3,  0                    // save 32 bits from 32-bit color a10 to dest_buff a3, offset 0
            s32i.n      a10,  a3,  4                    // save 32 bits from 32-bit color a10 to dest_buff a3, offset 4
            s32i.n      a10,  a3,  8                    // save 32 bits from 32-bit color a10 to dest_buff a3, offset 8
            s32i.n      a10,  a3,  12                   // save 32 bits from 32-bit color a10 to dest_buff a3, offset 12
            s32i.n      a10,  a3,  16                   // save 32 bits from 32-bit color a10 to dest_buff a3, offset 16
            s32i.n      a10,  a3,  20                   // save 32 bits from 32-bit color a10 to dest_buff a3, offset 20
            s32i.n      a10,  a3,  24                   // save 32 bits from 32-bit color a10 to dest_buff a3, offset 24
            s32i.n      a10,  a3,  28                   // save 32 bits from 32-bit color a10 to dest_buff a3, offset 28
            addi.n      a3,   a3,  32                   // increment dest_buff pointer by 32
#if !XCHAL_HAVE_LOOPS
        blt     a3,  a15,  ._main_loop_aligned_done     // Check end of the main loop, branch if dest_buf (a3) lower than a15
#endif
        ._main_loop:

        // Finish the remaining bytes out of the loop
        // Check modulo 16 of the dest_w_bytes (a11), if - then set 16 bytes (8 RGB565 pixels)
        // src_reg a10, dest_buff a3, dest_w_bytes a11
        macro_memset_mod_16 a10, a3, a11, __LINE__

        // Check modulo 8 of the dest_w_bytes (a11), if - then set 8 bytes (4 RGB565 pixels)
        // src_reg a10, dest_buff a3, dest_w_bytes a11
        macro_memset_mod_8 a10, a3, a11, __LINE__

        // Check modulo 4 of the dest_w_bytes (a11), if - then set 4 bytes (2 RGB565 pixels)
        // src_reg a10, dest_buff a3, dest_w_bytes a11
        macro_memset_mod_4 a10, a3, a11, __LINE__

        // Check modulo 2 of the dest_w_bytes (a11), if - then set 2 bytes (1 RGB565 pixel)
        // src_reg a10, dest_buff a3, dest_w_bytes a11
        macro_memset_mod_2 a10, a3, a11, __LINE__

        add     a3,  a3,  a6                            // dest_buff + dest_stride
        addi.n  a5,  a5,  -1                            // decrease the outer loop
        and     a7,  a8,  a3                            // a7 = dest_buff AND 0x3 (chck if the address is 4-byte aligned)
    bnez a5, .outer_loop

    movi.n   a2, 1                                      // return LV_RESULT_OK = 1
    retw.n                                              // return

//**********************************************************************************************************************

    // Small matrix width, keep it simple for lengths less than 8 pixels

    _matrix_width_check:

#if !XCHAL_HAVE_LOOPS
    slli    a14,  a4,   1                               // a14 = loop_len (a9) * 4 (main loop increments address pointers by 4)
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
            s16i        a12,  a3,  0                    // save 16 bits from a12 to dest_buff a3
            addi.n      a3,   a3,  2                    // increment dest_buff pointer by 2 bytes
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
    movi.n   a2, 1                                      // return LV_RESULT_OK = 1
    retw.n                                              // return
