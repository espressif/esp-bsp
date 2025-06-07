/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Memset macros for modulo checking
// After running the main loop, there is need to check remaining bytes to be set out of the main loop
// Macros work with both, aligned and unaligned (4-byte boundary) memories
// but performance is significantly lower when using unaligned memory, because of the unaligned memory access exception

// Macro for checking modulo 16
 .macro macro_memset_mod_16 src_reg, dest_buf, condition, JUMP_TAG
    // Check modulo 16 of the \condition, if - then set 16 bytes
    bbci \condition, 4, ._mod_16_check_\JUMP_TAG     // Branch if 4-th bit of \condition is clear
        s32i.n      \src_reg,   \dest_buf,  0        // Save 32 bits from \src_reg to \dest_buff, offset 0
        s32i.n      \src_reg,   \dest_buf,  4        // Save 32 bits from \src_reg to \dest_buff, offset 4
        s32i.n      \src_reg,   \dest_buf,  8        // Save 32 bits from \src_reg to \dest_buff, offset 8
        s32i.n      \src_reg,   \dest_buf,  12       // Save 32 bits from \src_reg to \dest_buff, offset 12
        addi.n      \dest_buf,  \dest_buf,  16       // Increment \dest_buff pointer 16
    ._mod_16_check_\JUMP_TAG:
.endm // macro_memset_mod_16

// Macro for checking modulo 8
 .macro macro_memset_mod_8 src_reg, dest_buf, condition, JUMP_TAG
    // Check modulo 8 of the \condition, if - then set 8 bytes
    bbci \condition, 3, ._mod_8_check_\JUMP_TAG      // Branch if 3-rd bit of \condition is clear
        s32i.n      \src_reg,   \dest_buf,  0        // Save 32 bits from \src_reg to \dest_buff, offset 0
        s32i.n      \src_reg,   \dest_buf,  4        // Save 32 bits from \src_reg to \dest_buff, offset 4
        addi.n      \dest_buf,  \dest_buf,  8        // Increment \dest_buff pointer 8
    ._mod_8_check_\JUMP_TAG:
.endm // macro_memset_mod_8

// Macro for checking modulo 4
 .macro macro_memset_mod_4 src_reg, dest_buf, condition, JUMP_TAG
    // Check modulo 4 of the \condition, if - then set 4 bytes
    bbci \condition, 2, ._mod_4_check_\JUMP_TAG      // Branch if 2-nd bit of \condition is clear
        s32i.n      \src_reg,   \dest_buf,  0        // Save 32 bits from \src_reg to \dest_buff, offset 0
        addi.n      \dest_buf,  \dest_buf,  4        // Increment \dest_buff pointer 4
    ._mod_4_check_\JUMP_TAG:
.endm // macro_memset_mod_4

// Macro for checking modulo 2
 .macro macro_memset_mod_2 src_reg, dest_buf, condition, JUMP_TAG
    // Check modulo 2 of the \condition, if - then set 2 bytes
    bbci \condition, 1, ._mod_2_check_\JUMP_TAG      // Branch if 1-st bit of \condition is clear
        s16i        \src_reg,   \dest_buf,  0        // Save 32 bits from \src_reg to \dest_buff, offset 0
        addi.n      \dest_buf,  \dest_buf,  2        // Increment \dest_buff pointer 2
    ._mod_2_check_\JUMP_TAG:
.endm // macro_memset_mod_2
