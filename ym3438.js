/*
 * Copyright (C) 2017-2022 Alexey Khokholov (Nuke.YKT)
 *
 * This file is part of Nuked OPN2.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 *  Nuked OPN2(Yamaha YM3438) emulator.
 *  Thanks:
 *      Silicon Pr0n:
 *          Yamaha YM3438 decap and die shot(digshadow).
 *      OPLx decapsulated(Matthew Gambrell, Olli Niemitalo):
 *          OPL2 ROMs.
 *
 * version: 1.0.12 (ym3438.c) / 1.0.9 (ym3438.h)
 *
 * JavaScript transcode of ym3438.c + ym3438.h (line-by-line, same cycle order).
 *
 * Usage:
 *     const YM = require("./ym3438.js");            // Node / CommonJS
 *     // <script src="ym3438.js"></script>          // browser -> global YM3438
 *
 *     const chip = new YM.ym3438_t();
 *     YM.OPN2_Reset(chip);
 *     const buf = new YM.Bit16s(2);                 // [0] = left, [1] = right
 *     YM.OPN2_Write(chip, 0, 0x22);                 // address port
 *     YM.OPN2_Write(chip, 1, 0x00);                 // data port
 *     YM.OPN2_Clock(chip, buf);                     // 1 call = 1 internal slot (24 per sample)
 *
 * Porting notes (C -> JS differences that are handled explicitly in this file):
 *   - Fixed-width integers: every struct array uses the matching typed array
 *     (Uint8Array/Uint16Array/Int16Array/...), so stores wrap exactly like C.
 *     Scalar struct members are plain JS numbers, so every store to a narrower
 *     scalar is masked by hand ("& 0xff", "& 0xffff", s16(), ...).
 *   - Integer division: C's "slot / 6" is written "((slot / 6) | 0)".
 *   - Booleans: C's "&&", "||", "==", "!" yield 0/1; JS yields operands/booleans,
 *     so every logical result stored into a field is normalised with "? 1 : 0".
 *   - Unsigned 32-bit shifts use ">>>" where the C type is Bit32u.
 *   - 2-D struct member fm_op1[6][2] is an array of 6 Int16Array(2).
 *   - memset(chip, 0, sizeof(*chip)) is emulated by ym3438_memset0().
 */

(function (root, factory) {
    if (typeof module === "object" && module.exports) {
        module.exports = factory();
    } else {
        root.YM3438 = factory();
    }
}(typeof globalThis !== "undefined" ? globalThis : (typeof self !== "undefined" ? self : this), function () {
"use strict";

/* ==========================================================================
 *  ym3438.h
 * ========================================================================== */

const ym3438_mode_ym2612 = 0x01;       /* Enables YM2612 emulation (MD1, MD2 VA2) */
const ym3438_mode_readmode = 0x02;     /* Enables status read on any port (TeraDrive, MD1 VA7, MD2, etc) */

/*
 * typedef uintptr_t       Bitu;
 * typedef intptr_t        Bits;
 * typedef uint64_t        Bit64u;
 * typedef int64_t         Bit64s;
 * typedef uint32_t        Bit32u;
 * typedef int32_t         Bit32s;
 * typedef uint16_t        Bit16u;
 * typedef int16_t         Bit16s;
 * typedef uint8_t         Bit8u;
 * typedef int8_t          Bit8s;
 *
 * JS has no scalar fixed-width types, so the typedef names are bound to the
 * matching typed-array constructors. "new Bit16u(24)" == "Bit16u x[24]".
 * (Bitu/Bits/Bit64u/Bit64s are not used by the emulator itself.)
 */
const Bitu   = (typeof BigUint64Array !== "undefined") ? BigUint64Array : Uint32Array;
const Bits   = (typeof BigInt64Array !== "undefined") ? BigInt64Array : Int32Array;
const Bit64u = (typeof BigUint64Array !== "undefined") ? BigUint64Array : null;
const Bit64s = (typeof BigInt64Array !== "undefined") ? BigInt64Array : null;
const Bit32u = Uint32Array;
const Bit32s = Int32Array;
const Bit16u = Uint16Array;
const Bit16s = Int16Array;
const Bit8u  = Uint8Array;
const Bit8s  = Int8Array;

/* Signed 16-bit wrap helper (JS numbers are doubles; emulates C truncation to Bit16s on store) */
function s16(x) { return (x << 16) >> 16; }

/*
 * typedef struct { ... } ym3438_t;
 *
 * Member order, names and types are identical to ym3438.h.
 * Arrays are typed arrays (auto-wrapping); scalars are numbers (wrapped by hand).
 */
class ym3438_t
{
    constructor()
    {
        this.cycles = 0;                                /* Bit32u */
        this.channel = 0;                               /* Bit32u */
        this.mol = 0; this.mor = 0;                     /* Bit16s mol, mor; */
        /* IO */
        this.write_data = 0;                            /* Bit16u */
        this.write_a = 0;                               /* Bit8u */
        this.write_d = 0;                               /* Bit8u */
        this.write_a_en = 0;                            /* Bit8u */
        this.write_d_en = 0;                            /* Bit8u */
        this.write_busy = 0;                            /* Bit8u */
        this.write_busy_cnt = 0;                        /* Bit8u */
        this.write_fm_address = 0;                      /* Bit8u */
        this.write_fm_data = 0;                         /* Bit8u */
        this.write_fm_mode_a = 0;                       /* Bit16u */
        this.address = 0;                               /* Bit16u */
        this.data = 0;                                  /* Bit8u */
        this.pin_test_in = 0;                           /* Bit8u */
        this.pin_irq = 0;                               /* Bit8u */
        this.busy = 0;                                  /* Bit8u */
        /* LFO */
        this.lfo_en = 0;                                /* Bit8u */
        this.lfo_freq = 0;                              /* Bit8u */
        this.lfo_pm = 0;                                /* Bit8u */
        this.lfo_am = 0;                                /* Bit8u */
        this.lfo_cnt = 0;                               /* Bit8u */
        this.lfo_inc = 0;                               /* Bit8u */
        this.lfo_quotient = 0;                          /* Bit8u */
        /* Phase generator */
        this.pg_fnum = 0;                               /* Bit16u */
        this.pg_block = 0;                              /* Bit8u */
        this.pg_kcode = 0;                              /* Bit8u */
        this.pg_inc = new Bit32u(24);                   /* Bit32u pg_inc[24] */
        this.pg_phase = new Bit32u(24);                 /* Bit32u pg_phase[24] */
        this.pg_reset = new Bit8u(24);                  /* Bit8u pg_reset[24] */
        this.pg_read = 0;                               /* Bit32u */
        /* Envelope generator */
        this.eg_cycle = 0;                              /* Bit8u */
        this.eg_cycle_stop = 0;                         /* Bit8u */
        this.eg_shift = 0;                              /* Bit8u */
        this.eg_shift_lock = 0;                         /* Bit8u */
        this.eg_timer_low_lock = 0;                     /* Bit8u */
        this.eg_timer = 0;                              /* Bit16u */
        this.eg_timer_inc = 0;                          /* Bit8u */
        this.eg_quotient = 0;                           /* Bit16u */
        this.eg_custom_timer = 0;                       /* Bit8u */
        this.eg_rate = 0;                               /* Bit8u */
        this.eg_ksv = 0;                                /* Bit8u */
        this.eg_inc = 0;                                /* Bit8u */
        this.eg_ratemax = 0;                            /* Bit8u */
        this.eg_sl = new Bit8u(2);                      /* Bit8u eg_sl[2] */
        this.eg_lfo_am = 0;                             /* Bit8u */
        this.eg_tl = new Bit8u(2);                      /* Bit8u eg_tl[2] */
        this.eg_state = new Bit8u(24);                  /* Bit8u eg_state[24] */
        this.eg_level = new Bit16u(24);                 /* Bit16u eg_level[24] */
        this.eg_out = new Bit16u(24);                   /* Bit16u eg_out[24] */
        this.eg_kon = new Bit8u(24);                    /* Bit8u eg_kon[24] */
        this.eg_kon_csm = new Bit8u(24);                /* Bit8u eg_kon_csm[24] */
        this.eg_kon_latch = new Bit8u(24);              /* Bit8u eg_kon_latch[24] */
        this.eg_csm_mode = new Bit8u(24);               /* Bit8u eg_csm_mode[24] */
        this.eg_ssg_enable = new Bit8u(24);             /* Bit8u eg_ssg_enable[24] */
        this.eg_ssg_pgrst_latch = new Bit8u(24);        /* Bit8u eg_ssg_pgrst_latch[24] */
        this.eg_ssg_repeat_latch = new Bit8u(24);       /* Bit8u eg_ssg_repeat_latch[24] */
        this.eg_ssg_hold_up_latch = new Bit8u(24);      /* Bit8u eg_ssg_hold_up_latch[24] */
        this.eg_ssg_dir = new Bit8u(24);                /* Bit8u eg_ssg_dir[24] */
        this.eg_ssg_inv = new Bit8u(24);                /* Bit8u eg_ssg_inv[24] */
        this.eg_read = new Bit32u(2);                   /* Bit32u eg_read[2] */
        this.eg_read_inc = 0;                           /* Bit8u */
        /* FM */
        this.fm_op1 = [                                 /* Bit16s fm_op1[6][2] */
            new Bit16s(2), new Bit16s(2), new Bit16s(2),
            new Bit16s(2), new Bit16s(2), new Bit16s(2)
        ];
        this.fm_op2 = new Bit16s(6);                    /* Bit16s fm_op2[6] */
        this.fm_out = new Bit16s(24);                   /* Bit16s fm_out[24] */
        this.fm_mod = new Bit16u(24);                   /* Bit16u fm_mod[24] */
        /* Channel */
        this.ch_acc = new Bit16s(6);                    /* Bit16s ch_acc[6] */
        this.ch_out = new Bit16s(6);                    /* Bit16s ch_out[6] */
        this.ch_lock = 0;                               /* Bit16s */
        this.ch_lock_l = 0;                             /* Bit8u */
        this.ch_lock_r = 0;                             /* Bit8u */
        this.ch_read = 0;                               /* Bit16s */
        /* Timer */
        this.timer_a_cnt = 0;                           /* Bit16u */
        this.timer_a_reg = 0;                           /* Bit16u */
        this.timer_a_load_lock = 0;                     /* Bit8u */
        this.timer_a_load = 0;                          /* Bit8u */
        this.timer_a_enable = 0;                        /* Bit8u */
        this.timer_a_reset = 0;                         /* Bit8u */
        this.timer_a_load_latch = 0;                    /* Bit8u */
        this.timer_a_overflow_flag = 0;                 /* Bit8u */
        this.timer_a_overflow = 0;                      /* Bit8u */

        this.timer_b_cnt = 0;                           /* Bit16u */
        this.timer_b_subcnt = 0;                        /* Bit8u */
        this.timer_b_reg = 0;                           /* Bit16u */
        this.timer_b_load_lock = 0;                     /* Bit8u */
        this.timer_b_load = 0;                          /* Bit8u */
        this.timer_b_enable = 0;                        /* Bit8u */
        this.timer_b_reset = 0;                         /* Bit8u */
        this.timer_b_load_latch = 0;                    /* Bit8u */
        this.timer_b_overflow_flag = 0;                 /* Bit8u */
        this.timer_b_overflow = 0;                      /* Bit8u */

        /* Register set */
        this.mode_test_21 = new Bit8u(8);               /* Bit8u mode_test_21[8] */
        this.mode_test_2c = new Bit8u(8);               /* Bit8u mode_test_2c[8] */
        this.mode_ch3 = 0;                              /* Bit8u */
        this.mode_kon_channel = 0;                      /* Bit8u */
        this.mode_kon_operator = new Bit8u(4);          /* Bit8u mode_kon_operator[4] */
        this.mode_kon = new Bit8u(24);                  /* Bit8u mode_kon[24] */
        this.mode_csm = 0;                              /* Bit8u */
        this.mode_kon_csm = 0;                          /* Bit8u */
        this.dacen = 0;                                 /* Bit8u */
        this.dacdata = 0;                               /* Bit16s */

        this.ks = new Bit8u(24);                        /* Bit8u ks[24] */
        this.ar = new Bit8u(24);                        /* Bit8u ar[24] */
        this.sr = new Bit8u(24);                        /* Bit8u sr[24] */
        this.dt = new Bit8u(24);                        /* Bit8u dt[24] */
        this.multi = new Bit8u(24);                     /* Bit8u multi[24] */
        this.sl = new Bit8u(24);                        /* Bit8u sl[24] */
        this.rr = new Bit8u(24);                        /* Bit8u rr[24] */
        this.dr = new Bit8u(24);                        /* Bit8u dr[24] */
        this.am = new Bit8u(24);                        /* Bit8u am[24] */
        this.tl = new Bit8u(24);                        /* Bit8u tl[24] */
        this.ssg_eg = new Bit8u(24);                    /* Bit8u ssg_eg[24] */

        this.fnum = new Bit16u(6);                      /* Bit16u fnum[6] */
        this.block = new Bit8u(6);                      /* Bit8u block[6] */
        this.kcode = new Bit8u(6);                      /* Bit8u kcode[6] */
        this.fnum_3ch = new Bit16u(6);                  /* Bit16u fnum_3ch[6] */
        this.block_3ch = new Bit8u(6);                  /* Bit8u block_3ch[6] */
        this.kcode_3ch = new Bit8u(6);                  /* Bit8u kcode_3ch[6] */
        this.reg_a4 = 0;                                /* Bit8u */
        this.reg_ac = 0;                                /* Bit8u */
        this.connect = new Bit8u(6);                    /* Bit8u connect[6] */
        this.fb = new Bit8u(6);                         /* Bit8u fb[6] */
        this.pan_l = new Bit8u(6);                      /* Bit8u pan_l[6] */
        this.pan_r = new Bit8u(6);                      /* Bit8u pan_r[6] */
        this.ams = new Bit8u(6);                        /* Bit8u ams[6] */
        this.pms = new Bit8u(6);                        /* Bit8u pms[6] */
        this.status = 0;                                /* Bit8u */
        this.status_time = 0;                           /* Bit32u */
    }
}

/* memset(chip, 0, sizeof(ym3438_t)) */
function ym3438_memset0(chip)
{
    const keys = Object.keys(chip);
    for (let k = 0; k < keys.length; k++)
    {
        const v = chip[keys[k]];
        if (typeof v === "number")
        {
            chip[keys[k]] = 0;
        }
        else if (Array.isArray(v))
        {
            /* fm_op1[6][2] */
            for (let j = 0; j < v.length; j++)
            {
                v[j].fill(0);
            }
        }
        else
        {
            v.fill(0);
        }
    }
}

/* ==========================================================================
 *  ym3438.c
 * ========================================================================== */

/*
 * #define SIGN_EXTEND(bit_index, value) (((value) & ((1u << (bit_index)) - 1u)) - ((value) & (1u << (bit_index))))
 *
 * The C macro is evaluated in unsigned int and the caller then stores it in a
 * Bit16s, i.e. the final value is the same as plain signed arithmetic here
 * (bit_index <= 13 for every use).
 */
function SIGN_EXTEND(bit_index, value)
{
    return ((value & ((1 << bit_index) - 1)) - (value & (1 << bit_index)));
}

/* enum { eg_num_attack = 0, eg_num_decay = 1, eg_num_sustain = 2, eg_num_release = 3 }; */
const eg_num_attack = 0;
const eg_num_decay = 1;
const eg_num_sustain = 2;
const eg_num_release = 3;

/* logsin table */
const logsinrom = new Bit16u([
    0x859, 0x6c3, 0x607, 0x58b, 0x52e, 0x4e4, 0x4a6, 0x471,
    0x443, 0x41a, 0x3f5, 0x3d3, 0x3b5, 0x398, 0x37e, 0x365,
    0x34e, 0x339, 0x324, 0x311, 0x2ff, 0x2ed, 0x2dc, 0x2cd,
    0x2bd, 0x2af, 0x2a0, 0x293, 0x286, 0x279, 0x26d, 0x261,
    0x256, 0x24b, 0x240, 0x236, 0x22c, 0x222, 0x218, 0x20f,
    0x206, 0x1fd, 0x1f5, 0x1ec, 0x1e4, 0x1dc, 0x1d4, 0x1cd,
    0x1c5, 0x1be, 0x1b7, 0x1b0, 0x1a9, 0x1a2, 0x19b, 0x195,
    0x18f, 0x188, 0x182, 0x17c, 0x177, 0x171, 0x16b, 0x166,
    0x160, 0x15b, 0x155, 0x150, 0x14b, 0x146, 0x141, 0x13c,
    0x137, 0x133, 0x12e, 0x129, 0x125, 0x121, 0x11c, 0x118,
    0x114, 0x10f, 0x10b, 0x107, 0x103, 0x0ff, 0x0fb, 0x0f8,
    0x0f4, 0x0f0, 0x0ec, 0x0e9, 0x0e5, 0x0e2, 0x0de, 0x0db,
    0x0d7, 0x0d4, 0x0d1, 0x0cd, 0x0ca, 0x0c7, 0x0c4, 0x0c1,
    0x0be, 0x0bb, 0x0b8, 0x0b5, 0x0b2, 0x0af, 0x0ac, 0x0a9,
    0x0a7, 0x0a4, 0x0a1, 0x09f, 0x09c, 0x099, 0x097, 0x094,
    0x092, 0x08f, 0x08d, 0x08a, 0x088, 0x086, 0x083, 0x081,
    0x07f, 0x07d, 0x07a, 0x078, 0x076, 0x074, 0x072, 0x070,
    0x06e, 0x06c, 0x06a, 0x068, 0x066, 0x064, 0x062, 0x060,
    0x05e, 0x05c, 0x05b, 0x059, 0x057, 0x055, 0x053, 0x052,
    0x050, 0x04e, 0x04d, 0x04b, 0x04a, 0x048, 0x046, 0x045,
    0x043, 0x042, 0x040, 0x03f, 0x03e, 0x03c, 0x03b, 0x039,
    0x038, 0x037, 0x035, 0x034, 0x033, 0x031, 0x030, 0x02f,
    0x02e, 0x02d, 0x02b, 0x02a, 0x029, 0x028, 0x027, 0x026,
    0x025, 0x024, 0x023, 0x022, 0x021, 0x020, 0x01f, 0x01e,
    0x01d, 0x01c, 0x01b, 0x01a, 0x019, 0x018, 0x017, 0x017,
    0x016, 0x015, 0x014, 0x014, 0x013, 0x012, 0x011, 0x011,
    0x010, 0x00f, 0x00f, 0x00e, 0x00d, 0x00d, 0x00c, 0x00c,
    0x00b, 0x00a, 0x00a, 0x009, 0x009, 0x008, 0x008, 0x007,
    0x007, 0x007, 0x006, 0x006, 0x005, 0x005, 0x005, 0x004,
    0x004, 0x004, 0x003, 0x003, 0x003, 0x002, 0x002, 0x002,
    0x002, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001,
    0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000
]);

/* exp table */
const exprom = new Bit16u([
    0x000, 0x003, 0x006, 0x008, 0x00b, 0x00e, 0x011, 0x014,
    0x016, 0x019, 0x01c, 0x01f, 0x022, 0x025, 0x028, 0x02a,
    0x02d, 0x030, 0x033, 0x036, 0x039, 0x03c, 0x03f, 0x042,
    0x045, 0x048, 0x04b, 0x04e, 0x051, 0x054, 0x057, 0x05a,
    0x05d, 0x060, 0x063, 0x066, 0x069, 0x06c, 0x06f, 0x072,
    0x075, 0x078, 0x07b, 0x07e, 0x082, 0x085, 0x088, 0x08b,
    0x08e, 0x091, 0x094, 0x098, 0x09b, 0x09e, 0x0a1, 0x0a4,
    0x0a8, 0x0ab, 0x0ae, 0x0b1, 0x0b5, 0x0b8, 0x0bb, 0x0be,
    0x0c2, 0x0c5, 0x0c8, 0x0cc, 0x0cf, 0x0d2, 0x0d6, 0x0d9,
    0x0dc, 0x0e0, 0x0e3, 0x0e7, 0x0ea, 0x0ed, 0x0f1, 0x0f4,
    0x0f8, 0x0fb, 0x0ff, 0x102, 0x106, 0x109, 0x10c, 0x110,
    0x114, 0x117, 0x11b, 0x11e, 0x122, 0x125, 0x129, 0x12c,
    0x130, 0x134, 0x137, 0x13b, 0x13e, 0x142, 0x146, 0x149,
    0x14d, 0x151, 0x154, 0x158, 0x15c, 0x160, 0x163, 0x167,
    0x16b, 0x16f, 0x172, 0x176, 0x17a, 0x17e, 0x181, 0x185,
    0x189, 0x18d, 0x191, 0x195, 0x199, 0x19c, 0x1a0, 0x1a4,
    0x1a8, 0x1ac, 0x1b0, 0x1b4, 0x1b8, 0x1bc, 0x1c0, 0x1c4,
    0x1c8, 0x1cc, 0x1d0, 0x1d4, 0x1d8, 0x1dc, 0x1e0, 0x1e4,
    0x1e8, 0x1ec, 0x1f0, 0x1f5, 0x1f9, 0x1fd, 0x201, 0x205,
    0x209, 0x20e, 0x212, 0x216, 0x21a, 0x21e, 0x223, 0x227,
    0x22b, 0x230, 0x234, 0x238, 0x23c, 0x241, 0x245, 0x249,
    0x24e, 0x252, 0x257, 0x25b, 0x25f, 0x264, 0x268, 0x26d,
    0x271, 0x276, 0x27a, 0x27f, 0x283, 0x288, 0x28c, 0x291,
    0x295, 0x29a, 0x29e, 0x2a3, 0x2a8, 0x2ac, 0x2b1, 0x2b5,
    0x2ba, 0x2bf, 0x2c4, 0x2c8, 0x2cd, 0x2d2, 0x2d6, 0x2db,
    0x2e0, 0x2e5, 0x2e9, 0x2ee, 0x2f3, 0x2f8, 0x2fd, 0x302,
    0x306, 0x30b, 0x310, 0x315, 0x31a, 0x31f, 0x324, 0x329,
    0x32e, 0x333, 0x338, 0x33d, 0x342, 0x347, 0x34c, 0x351,
    0x356, 0x35b, 0x360, 0x365, 0x36a, 0x370, 0x375, 0x37a,
    0x37f, 0x384, 0x38a, 0x38f, 0x394, 0x399, 0x39f, 0x3a4,
    0x3a9, 0x3ae, 0x3b4, 0x3b9, 0x3bf, 0x3c4, 0x3c9, 0x3cf,
    0x3d4, 0x3da, 0x3df, 0x3e4, 0x3ea, 0x3ef, 0x3f5, 0x3fa
]);

/* Note table */
const fn_note = new Bit32u([
    0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 3, 3, 3, 3, 3, 3
]);

/* Envelope generator */
const eg_stephi = [
    new Bit32u([ 0, 0, 0, 0 ]),
    new Bit32u([ 1, 0, 0, 0 ]),
    new Bit32u([ 1, 0, 1, 0 ]),
    new Bit32u([ 1, 1, 1, 0 ])
];

const eg_am_shift = new Bit8u([
    7, 3, 1, 0
]);

/* Phase generator */
const pg_detune = new Bit32u([ 16, 17, 19, 20, 22, 24, 27, 29 ]);

const pg_lfo_sh1 = [
    new Bit32u([ 7, 7, 7, 7, 7, 7, 7, 7 ]),
    new Bit32u([ 7, 7, 7, 7, 7, 7, 7, 7 ]),
    new Bit32u([ 7, 7, 7, 7, 7, 7, 1, 1 ]),
    new Bit32u([ 7, 7, 7, 7, 1, 1, 1, 1 ]),
    new Bit32u([ 7, 7, 7, 1, 1, 1, 1, 0 ]),
    new Bit32u([ 7, 7, 1, 1, 0, 0, 0, 0 ]),
    new Bit32u([ 7, 7, 1, 1, 0, 0, 0, 0 ]),
    new Bit32u([ 7, 7, 1, 1, 0, 0, 0, 0 ])
];

const pg_lfo_sh2 = [
    new Bit32u([ 7, 7, 7, 7, 7, 7, 7, 7 ]),
    new Bit32u([ 7, 7, 7, 7, 2, 2, 2, 2 ]),
    new Bit32u([ 7, 7, 7, 2, 2, 2, 7, 7 ]),
    new Bit32u([ 7, 7, 2, 2, 7, 7, 2, 2 ]),
    new Bit32u([ 7, 7, 2, 7, 7, 7, 2, 7 ]),
    new Bit32u([ 7, 7, 7, 2, 7, 7, 2, 1 ]),
    new Bit32u([ 7, 7, 7, 2, 7, 7, 2, 1 ]),
    new Bit32u([ 7, 7, 7, 2, 7, 7, 2, 1 ])
];

/* Address decoder */
const op_offset = new Bit32u([
    0x000, /* Ch1 OP1/OP2 */
    0x001, /* Ch2 OP1/OP2 */
    0x002, /* Ch3 OP1/OP2 */
    0x100, /* Ch4 OP1/OP2 */
    0x101, /* Ch5 OP1/OP2 */
    0x102, /* Ch6 OP1/OP2 */
    0x004, /* Ch1 OP3/OP4 */
    0x005, /* Ch2 OP3/OP4 */
    0x006, /* Ch3 OP3/OP4 */
    0x104, /* Ch4 OP3/OP4 */
    0x105, /* Ch5 OP3/OP4 */
    0x106  /* Ch6 OP3/OP4 */
]);

const ch_offset = new Bit32u([
    0x000, /* Ch1 */
    0x001, /* Ch2 */
    0x002, /* Ch3 */
    0x100, /* Ch4 */
    0x101, /* Ch5 */
    0x102  /* Ch6 */
]);

/* LFO */
const lfo_cycles = new Bit32u([
    108, 77, 71, 67, 62, 44, 8, 5
]);

/* FM algorithm */
const fm_algorithm = [
    [
        new Bit32u([ 1, 1, 1, 1, 1, 1, 1, 1 ]), /* OP1_0         */
        new Bit32u([ 1, 1, 1, 1, 1, 1, 1, 1 ]), /* OP1_1         */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* OP2           */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* Last operator */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* Last operator */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 1 ])  /* Out           */
    ],
    [
        new Bit32u([ 0, 1, 0, 0, 0, 1, 0, 0 ]), /* OP1_0         */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* OP1_1         */
        new Bit32u([ 1, 1, 1, 0, 0, 0, 0, 0 ]), /* OP2           */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* Last operator */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* Last operator */
        new Bit32u([ 0, 0, 0, 0, 0, 1, 1, 1 ])  /* Out           */
    ],
    [
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* OP1_0         */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* OP1_1         */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* OP2           */
        new Bit32u([ 1, 0, 0, 1, 1, 1, 1, 0 ]), /* Last operator */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* Last operator */
        new Bit32u([ 0, 0, 0, 0, 1, 1, 1, 1 ])  /* Out           */
    ],
    [
        new Bit32u([ 0, 0, 1, 0, 0, 1, 0, 0 ]), /* OP1_0         */
        new Bit32u([ 0, 0, 0, 0, 0, 0, 0, 0 ]), /* OP1_1         */
        new Bit32u([ 0, 0, 0, 1, 0, 0, 0, 0 ]), /* OP2           */
        new Bit32u([ 1, 1, 0, 1, 1, 0, 0, 0 ]), /* Last operator */
        new Bit32u([ 0, 0, 1, 0, 0, 0, 0, 0 ]), /* Last operator */
        new Bit32u([ 1, 1, 1, 1, 1, 1, 1, 1 ])  /* Out           */
    ]
];

let chip_type = ym3438_mode_readmode;

function OPN2_DoIO(chip)
{
    /* Write signal check */
    chip.write_a_en = ((chip.write_a & 0x03) === 0x01) ? 1 : 0;
    chip.write_d_en = ((chip.write_d & 0x03) === 0x01) ? 1 : 0;
    chip.write_a = (chip.write_a << 1) & 0xff;              /* Bit8u: must wrap at 8 bits */
    chip.write_d = (chip.write_d << 1) & 0xff;              /* Bit8u: must wrap at 8 bits */
    /* Busy counter */
    chip.busy = chip.write_busy;
    chip.write_busy_cnt = (chip.write_busy_cnt + chip.write_busy) & 0xff;
    chip.write_busy = ((chip.write_busy && !(chip.write_busy_cnt >> 5)) || chip.write_d_en) ? 1 : 0;
    chip.write_busy_cnt &= 0x1f;
}

function OPN2_DoRegWrite(chip)
{
    let i;
    let slot = chip.cycles % 12;
    let address;
    const channel = chip.channel;
    /* Update registers */
    if (chip.write_fm_data)
    {
        /* Slot */
        if (op_offset[slot] === (chip.address & 0x107))
        {
            if (chip.address & 0x08)
            {
                /* OP2, OP4 */
                slot += 12;
            }
            address = chip.address & 0xf0;
            switch (address)
            {
            case 0x30: /* DT, MULTI */
                chip.multi[slot] = chip.data & 0x0f;
                if (!chip.multi[slot])
                {
                    chip.multi[slot] = 1;
                }
                else
                {
                    chip.multi[slot] <<= 1;
                }
                chip.dt[slot] = (chip.data >> 4) & 0x07;
                break;
            case 0x40: /* TL */
                chip.tl[slot] = chip.data & 0x7f;
                break;
            case 0x50: /* KS, AR */
                chip.ar[slot] = chip.data & 0x1f;
                chip.ks[slot] = (chip.data >> 6) & 0x03;
                break;
            case 0x60: /* AM, DR */
                chip.dr[slot] = chip.data & 0x1f;
                chip.am[slot] = (chip.data >> 7) & 0x01;
                break;
            case 0x70: /* SR */
                chip.sr[slot] = chip.data & 0x1f;
                break;
            case 0x80: /* SL, RR */
                chip.rr[slot] = chip.data & 0x0f;
                chip.sl[slot] = (chip.data >> 4) & 0x0f;
                chip.sl[slot] |= (chip.sl[slot] + 1) & 0x10;
                break;
            case 0x90: /* SSG-EG */
                chip.ssg_eg[slot] = chip.data & 0x0f;
                break;
            default:
                break;
            }
        }

        /* Channel */
        if (ch_offset[channel] === (chip.address & 0x103))
        {
            address = chip.address & 0xfc;
            switch (address)
            {
            case 0xa0:
                chip.fnum[channel] = (chip.data & 0xff) | ((chip.reg_a4 & 0x07) << 8);
                chip.block[channel] = (chip.reg_a4 >> 3) & 0x07;
                chip.kcode[channel] = (chip.block[channel] << 2) | fn_note[chip.fnum[channel] >> 7];
                break;
            case 0xa4:
                chip.reg_a4 = chip.data & 0xff;
                break;
            case 0xa8:
                chip.fnum_3ch[channel] = (chip.data & 0xff) | ((chip.reg_ac & 0x07) << 8);
                chip.block_3ch[channel] = (chip.reg_ac >> 3) & 0x07;
                chip.kcode_3ch[channel] = (chip.block_3ch[channel] << 2) | fn_note[chip.fnum_3ch[channel] >> 7];
                break;
            case 0xac:
                chip.reg_ac = chip.data & 0xff;
                break;
            case 0xb0:
                chip.connect[channel] = chip.data & 0x07;
                chip.fb[channel] = (chip.data >> 3) & 0x07;
                break;
            case 0xb4:
                chip.pms[channel] = chip.data & 0x07;
                chip.ams[channel] = (chip.data >> 4) & 0x03;
                chip.pan_l[channel] = (chip.data >> 7) & 0x01;
                chip.pan_r[channel] = (chip.data >> 6) & 0x01;
                break;
            default:
                break;
            }
        }
    }

    if (chip.write_a_en || chip.write_d_en)
    {
        /* Data */
        if (chip.write_a_en)
        {
            chip.write_fm_data = 0;
        }

        if (chip.write_fm_address && chip.write_d_en)
        {
            chip.write_fm_data = 1;
        }

        /* Address */
        if (chip.write_a_en)
        {
            if ((chip.write_data & 0xf0) !== 0x00)
            {
                /* FM Write */
                chip.address = chip.write_data;
                chip.write_fm_address = 1;
            }
            else
            {
                /* SSG write */
                chip.write_fm_address = 0;
            }
        }

        /* FM Mode */
        /* Data */
        if (chip.write_d_en && (chip.write_data & 0x100) === 0)
        {
            switch (chip.write_fm_mode_a)
            {
            case 0x21: /* LSI test 1 */
                for (i = 0; i < 8; i++)
                {
                    chip.mode_test_21[i] = (chip.write_data >> i) & 0x01;
                }
                break;
            case 0x22: /* LFO control */
                if ((chip.write_data >> 3) & 0x01)
                {
                    chip.lfo_en = 0x7f;
                }
                else
                {
                    chip.lfo_en = 0;
                }
                chip.lfo_freq = chip.write_data & 0x07;
                break;
            case 0x24: /* Timer A */
                chip.timer_a_reg &= 0x03;
                chip.timer_a_reg |= (chip.write_data & 0xff) << 2;
                break;
            case 0x25:
                chip.timer_a_reg &= 0x3fc;
                chip.timer_a_reg |= chip.write_data & 0x03;
                break;
            case 0x26: /* Timer B */
                chip.timer_b_reg = chip.write_data & 0xff;
                break;
            case 0x27: /* CSM, Timer control */
                chip.mode_ch3 = (chip.write_data & 0xc0) >> 6;
                chip.mode_csm = (chip.mode_ch3 === 2) ? 1 : 0;
                chip.timer_a_load = chip.write_data & 0x01;
                chip.timer_a_enable = (chip.write_data >> 2) & 0x01;
                chip.timer_a_reset = (chip.write_data >> 4) & 0x01;
                chip.timer_b_load = (chip.write_data >> 1) & 0x01;
                chip.timer_b_enable = (chip.write_data >> 3) & 0x01;
                chip.timer_b_reset = (chip.write_data >> 5) & 0x01;
                break;
            case 0x28: /* Key on/off */
                for (i = 0; i < 4; i++)
                {
                    chip.mode_kon_operator[i] = (chip.write_data >> (4 + i)) & 0x01;
                }
                if ((chip.write_data & 0x03) === 0x03)
                {
                    /* Invalid address */
                    chip.mode_kon_channel = 0xff;
                }
                else
                {
                    chip.mode_kon_channel = (chip.write_data & 0x03) + ((chip.write_data >> 2) & 1) * 3;
                }
                break;
            case 0x2a: /* DAC data */
                chip.dacdata &= 0x01;
                chip.dacdata |= (chip.write_data ^ 0x80) << 1;
                chip.dacdata = s16(chip.dacdata);               /* Bit16s */
                break;
            case 0x2b: /* DAC enable */
                chip.dacen = chip.write_data >> 7;
                break;
            case 0x2c: /* LSI test 2 */
                for (i = 0; i < 8; i++)
                {
                    chip.mode_test_2c[i] = (chip.write_data >> i) & 0x01;
                }
                chip.dacdata &= 0x1fe;
                chip.dacdata |= chip.mode_test_2c[3];
                chip.eg_custom_timer = (!chip.mode_test_2c[7] && chip.mode_test_2c[6]) ? 1 : 0;
                break;
            default:
                break;
            }
        }

        /* Address */
        if (chip.write_a_en)
        {
            chip.write_fm_mode_a = chip.write_data & 0x1ff;
        }
    }

    if (chip.write_fm_data)
    {
        chip.data = chip.write_data & 0xff;
    }
}

function OPN2_PhaseCalcIncrement(chip)
{
    const chan = chip.channel;
    const slot = chip.cycles;
    let fnum = chip.pg_fnum;
    const fnum_h = fnum >> 4;
    let fm;
    let basefreq;
    const lfo = chip.lfo_pm;
    let lfo_l = lfo & 0x0f;
    const pms = chip.pms[chan];
    const dt = chip.dt[slot];
    const dt_l = dt & 0x03;
    let detune = 0;
    let block, note;
    let sum, sum_h, sum_l;
    let kcode = chip.pg_kcode;

    fnum <<= 1;
    /* Apply LFO */
    if (lfo_l & 0x08)
    {
        lfo_l ^= 0x0f;
    }
    fm = (fnum_h >> pg_lfo_sh1[pms][lfo_l]) + (fnum_h >> pg_lfo_sh2[pms][lfo_l]);
    if (pms > 5)
    {
        fm <<= pms - 5;
    }
    fm >>= 2;
    if (lfo & 0x10)
    {
        fnum -= fm;
    }
    else
    {
        fnum += fm;
    }
    fnum &= 0xfff;

    basefreq = (fnum << chip.pg_block) >> 2;

    /* Apply detune */
    if (dt_l)
    {
        if (kcode > 0x1c)
        {
            kcode = 0x1c;
        }
        block = kcode >> 2;
        note = kcode & 0x03;
        sum = (block + 9 + (((dt_l === 3) ? 1 : 0) | (dt_l & 0x02))) & 0xff;
        sum_h = sum >> 1;
        sum_l = sum & 0x01;
        detune = (pg_detune[(sum_l << 2) | note] >> (9 - sum_h)) & 0xff;
    }
    if (dt & 0x04)
    {
        basefreq -= detune;
    }
    else
    {
        basefreq += detune;
    }
    basefreq &= 0x1ffff;
    chip.pg_inc[slot] = (basefreq * chip.multi[slot]) >> 1;
    chip.pg_inc[slot] &= 0xfffff;
}

function OPN2_PhaseGenerate(chip)
{
    let slot;
    /* Mask increment */
    slot = (chip.cycles + 20) % 24;
    if (chip.pg_reset[slot])
    {
        chip.pg_inc[slot] = 0;
    }
    /* Phase step */
    slot = (chip.cycles + 19) % 24;
    if (chip.pg_reset[slot] || chip.mode_test_21[3])
    {
        chip.pg_phase[slot] = 0;
    }
    chip.pg_phase[slot] += chip.pg_inc[slot];
    chip.pg_phase[slot] &= 0xfffff;
}

function OPN2_EnvelopeSSGEG(chip)
{
    const slot = chip.cycles;
    let direction = 0;
    chip.eg_ssg_pgrst_latch[slot] = 0;
    chip.eg_ssg_repeat_latch[slot] = 0;
    chip.eg_ssg_hold_up_latch[slot] = 0;
    if (chip.ssg_eg[slot] & 0x08)
    {
        direction = chip.eg_ssg_dir[slot];
        if (chip.eg_level[slot] & 0x200)
        {
            /* Reset */
            if ((chip.ssg_eg[slot] & 0x03) === 0x00)
            {
                chip.eg_ssg_pgrst_latch[slot] = 1;
            }
            /* Repeat */
            if ((chip.ssg_eg[slot] & 0x01) === 0x00)
            {
                chip.eg_ssg_repeat_latch[slot] = 1;
            }
            /* Inverse */
            if ((chip.ssg_eg[slot] & 0x03) === 0x02)
            {
                direction ^= 1;
            }
            if ((chip.ssg_eg[slot] & 0x03) === 0x03)
            {
                direction = 1;
            }
        }
        /* Hold up */
        if (chip.eg_kon_latch[slot]
         && ((chip.ssg_eg[slot] & 0x07) === 0x05 || (chip.ssg_eg[slot] & 0x07) === 0x03))
        {
            chip.eg_ssg_hold_up_latch[slot] = 1;
        }
        direction &= chip.eg_kon[slot];
    }
    chip.eg_ssg_dir[slot] = direction;
    chip.eg_ssg_enable[slot] = (chip.ssg_eg[slot] >> 3) & 0x01;
    chip.eg_ssg_inv[slot] = (chip.eg_ssg_dir[slot] ^ (((chip.ssg_eg[slot] >> 2) & 0x01) & ((chip.ssg_eg[slot] >> 3) & 0x01)))
                          & chip.eg_kon[slot];
}

function OPN2_EnvelopeADSR(chip)
{
    const slot = (chip.cycles + 22) % 24;

    const nkon = chip.eg_kon_latch[slot];
    const okon = chip.eg_kon[slot];
    let kon_event;
    let koff_event;
    let eg_off;
    let level;
    let nextlevel = 0;
    let ssg_level;
    let nextstate = chip.eg_state[slot];
    let inc = 0;
    chip.eg_read[0] = chip.eg_read_inc;
    chip.eg_read_inc = (chip.eg_inc > 0) ? 1 : 0;

    /* Reset phase generator */
    chip.pg_reset[slot] = ((nkon && !okon) || chip.eg_ssg_pgrst_latch[slot]) ? 1 : 0;

    /* KeyOn/Off */
    kon_event = ((nkon && !okon) || (okon && chip.eg_ssg_repeat_latch[slot])) ? 1 : 0;
    koff_event = (okon && !nkon) ? 1 : 0;

    ssg_level = level = s16(chip.eg_level[slot]);

    if (chip.eg_ssg_inv[slot])
    {
        /* Inverse */
        ssg_level = s16(512 - level);
        ssg_level &= 0x3ff;
    }
    if (koff_event)
    {
        level = ssg_level;
    }
    if (chip.eg_ssg_enable[slot])
    {
        eg_off = (level >> 9) & 0xff;
    }
    else
    {
        eg_off = ((level & 0x3f0) === 0x3f0) ? 1 : 0;
    }
    nextlevel = level;
    if (kon_event)
    {
        nextstate = eg_num_attack;
        /* Instant attack */
        if (chip.eg_ratemax)
        {
            nextlevel = 0;
        }
        else if (chip.eg_state[slot] === eg_num_attack && level !== 0 && chip.eg_inc && nkon)
        {
            inc = s16((~level << chip.eg_inc) >> 5);
        }
    }
    else
    {
        switch (chip.eg_state[slot])
        {
        case eg_num_attack:
            if (level === 0)
            {
                nextstate = eg_num_decay;
            }
            else if (chip.eg_inc && !chip.eg_ratemax && nkon)
            {
                inc = s16((~level << chip.eg_inc) >> 5);
            }
            break;
        case eg_num_decay:
            if ((level >> 4) === (chip.eg_sl[1] << 1))
            {
                nextstate = eg_num_sustain;
            }
            else if (!eg_off && chip.eg_inc)
            {
                inc = 1 << (chip.eg_inc - 1);
                if (chip.eg_ssg_enable[slot])
                {
                    inc <<= 2;
                }
            }
            break;
        case eg_num_sustain:
        case eg_num_release:
            if (!eg_off && chip.eg_inc)
            {
                inc = 1 << (chip.eg_inc - 1);
                if (chip.eg_ssg_enable[slot])
                {
                    inc <<= 2;
                }
            }
            break;
        default:
            break;
        }
        if (!nkon)
        {
            nextstate = eg_num_release;
        }
    }
    if (chip.eg_kon_csm[slot])
    {
        nextlevel |= chip.eg_tl[1] << 3;
    }

    /* Envelope off */
    if (!kon_event && !chip.eg_ssg_hold_up_latch[slot] && chip.eg_state[slot] !== eg_num_attack && eg_off)
    {
        nextstate = eg_num_release;
        nextlevel = 0x3ff;
    }

    nextlevel = s16(nextlevel + inc);

    chip.eg_kon[slot] = chip.eg_kon_latch[slot];
    chip.eg_level[slot] = (nextlevel & 0xffff) & 0x3ff;     /* (Bit16u)nextlevel & 0x3ff */
    chip.eg_state[slot] = nextstate;
}

function OPN2_EnvelopePrepare(chip)
{
    let rate;
    let sum;
    let inc = 0;
    const slot = chip.cycles;
    let rate_sel;

    /* Prepare increment */
    rate = ((chip.eg_rate << 1) + chip.eg_ksv) & 0xff;

    if (rate > 0x3f)
    {
        rate = 0x3f;
    }

    sum = ((rate >> 2) + chip.eg_shift_lock) & 0x0f;
    if (chip.eg_rate !== 0 && chip.eg_quotient === 2)
    {
        if (rate < 48)
        {
            switch (sum)
            {
            case 12:
                inc = 1;
                break;
            case 13:
                inc = (rate >> 1) & 0x01;
                break;
            case 14:
                inc = rate & 0x01;
                break;
            default:
                break;
            }
        }
        else
        {
            inc = (eg_stephi[rate & 0x03][chip.eg_timer_low_lock] + (rate >> 2) - 11) & 0xff;
            if (inc > 4)
            {
                inc = 4;
            }
        }
    }
    chip.eg_inc = inc;
    chip.eg_ratemax = ((rate >> 1) === 0x1f) ? 1 : 0;

    /* Prepare rate & ksv */
    rate_sel = chip.eg_state[slot];
    if ((chip.eg_kon[slot] && chip.eg_ssg_repeat_latch[slot])
     || (!chip.eg_kon[slot] && chip.eg_kon_latch[slot]))
    {
        rate_sel = eg_num_attack;
    }
    switch (rate_sel)
    {
    case eg_num_attack:
        chip.eg_rate = chip.ar[slot];
        break;
    case eg_num_decay:
        chip.eg_rate = chip.dr[slot];
        break;
    case eg_num_sustain:
        chip.eg_rate = chip.sr[slot];
        break;
    case eg_num_release:
        chip.eg_rate = (chip.rr[slot] << 1) | 0x01;
        break;
    default:
        break;
    }
    chip.eg_ksv = chip.pg_kcode >> (chip.ks[slot] ^ 0x03);
    if (chip.am[slot])
    {
        chip.eg_lfo_am = chip.lfo_am >> eg_am_shift[chip.ams[chip.channel]];
    }
    else
    {
        chip.eg_lfo_am = 0;
    }
    /* Delay TL & SL value */
    chip.eg_tl[1] = chip.eg_tl[0];
    chip.eg_tl[0] = chip.tl[slot];
    chip.eg_sl[1] = chip.eg_sl[0];
    chip.eg_sl[0] = chip.sl[slot];
}

function OPN2_EnvelopeGenerate(chip)
{
    const slot = (chip.cycles + 23) % 24;
    let level;

    level = chip.eg_level[slot];

    if (chip.eg_ssg_inv[slot])
    {
        /* Inverse */
        level = (512 - level) & 0xffff;                     /* Bit16u: wraps if level > 512 */
    }
    if (chip.mode_test_21[5])
    {
        level = 0;
    }
    level &= 0x3ff;

    /* Apply AM LFO */
    level = (level + chip.eg_lfo_am) & 0xffff;

    /* Apply TL */
    if (!(chip.mode_csm && chip.channel === 2 + 1))
    {
        level = (level + (chip.eg_tl[0] << 3)) & 0xffff;
    }
    if (level > 0x3ff)
    {
        level = 0x3ff;
    }
    chip.eg_out[slot] = level;
}

function OPN2_UpdateLFO(chip)
{
    if ((chip.lfo_quotient & lfo_cycles[chip.lfo_freq]) === lfo_cycles[chip.lfo_freq])
    {
        chip.lfo_quotient = 0;
        chip.lfo_cnt = (chip.lfo_cnt + 1) & 0xff;
    }
    else
    {
        chip.lfo_quotient = (chip.lfo_quotient + chip.lfo_inc) & 0xff;
    }
    chip.lfo_cnt &= chip.lfo_en;
}

function OPN2_FMPrepare(chip)
{
    let slot = (chip.cycles + 6) % 24;
    const channel = chip.channel;
    let mod, mod1, mod2;
    const op = (slot / 6) | 0;                              /* C integer division */
    const connect = chip.connect[channel];
    const prevslot = (chip.cycles + 18) % 24;

    /* Calculate modulation */
    mod1 = mod2 = 0;

    if (fm_algorithm[op][0][connect])
    {
        mod2 |= chip.fm_op1[channel][0];
    }
    if (fm_algorithm[op][1][connect])
    {
        mod1 |= chip.fm_op1[channel][1];
    }
    if (fm_algorithm[op][2][connect])
    {
        mod1 |= chip.fm_op2[channel];
    }
    if (fm_algorithm[op][3][connect])
    {
        mod2 |= chip.fm_out[prevslot];
    }
    if (fm_algorithm[op][4][connect])
    {
        mod1 |= chip.fm_out[prevslot];
    }
    mod = s16(mod1 + mod2);
    if (op === 0)
    {
        /* Feedback */
        mod = mod >> (10 - chip.fb[channel]);
        if (!chip.fb[channel])
        {
            mod = 0;
        }
    }
    else
    {
        mod >>= 1;
    }
    chip.fm_mod[slot] = mod;                                /* Bit16u store: typed array wraps negatives */

    slot = (chip.cycles + 18) % 24;
    /* OP1 */
    if (((slot / 6) | 0) === 0)
    {
        chip.fm_op1[channel][1] = chip.fm_op1[channel][0];
        chip.fm_op1[channel][0] = chip.fm_out[slot];
    }
    /* OP2 */
    if (((slot / 6) | 0) === 2)
    {
        chip.fm_op2[channel] = chip.fm_out[slot];
    }
}

function OPN2_ChGenerate(chip)
{
    const slot = (chip.cycles + 18) % 24;
    const channel = chip.channel;
    const op = (slot / 6) | 0;                              /* C integer division */
    const test_dac = chip.mode_test_2c[5];
    let acc = chip.ch_acc[channel];
    let add = test_dac;
    let sum = 0;
    if (op === 0 && !test_dac)
    {
        acc = 0;
    }
    if (fm_algorithm[op][5][chip.connect[channel]] && !test_dac)
    {
        add += chip.fm_out[slot] >> 5;
    }
    sum = s16(acc + add);
    /* Clamp */
    if (sum > 255)
    {
        sum = 255;
    }
    else if (sum < -256)
    {
        sum = -256;
    }

    if (op === 0 || test_dac)
    {
        chip.ch_out[channel] = chip.ch_acc[channel];
    }
    chip.ch_acc[channel] = sum;
}

function OPN2_ChOutput(chip)
{
    const cycles = chip.cycles;
    const slot = chip.cycles;
    let channel = chip.channel;
    const test_dac = chip.mode_test_2c[5];
    let out;
    let sign;
    let out_en;
    chip.ch_read = chip.ch_lock;
    if (slot < 12)
    {
        /* Ch 4,5,6 */
        channel++;
    }
    if ((cycles & 3) === 0)
    {
        if (!test_dac)
        {
            /* Lock value */
            chip.ch_lock = chip.ch_out[channel];
        }
        chip.ch_lock_l = chip.pan_l[channel];
        chip.ch_lock_r = chip.pan_r[channel];
    }
    /* Ch 6 */
    if ((((cycles >> 2) === 1 && chip.dacen) || test_dac))
    {
        out = s16(chip.dacdata);
        out = SIGN_EXTEND(8, out);
    }
    else
    {
        out = chip.ch_lock;
    }
    chip.mol = 0;
    chip.mor = 0;

    if (chip_type & ym3438_mode_ym2612)
    {
        out_en = ((((cycles & 3) === 3) || test_dac)) ? 1 : 0;
        /* YM2612 DAC emulation(not verified) */
        sign = out >> 8;
        if (out >= 0)
        {
            out++;
            sign++;
        }
        if (chip.ch_lock_l && out_en)
        {
            chip.mol = out;
        }
        else
        {
            chip.mol = sign;
        }
        if (chip.ch_lock_r && out_en)
        {
            chip.mor = out;
        }
        else
        {
            chip.mor = sign;
        }
        /* Amplify signal */
        chip.mol = s16(chip.mol * 3);
        chip.mor = s16(chip.mor * 3);
    }
    else
    {
        out_en = ((((cycles & 3) !== 0) || test_dac)) ? 1 : 0;
        if (chip.ch_lock_l && out_en)
        {
            chip.mol = out;
        }
        if (chip.ch_lock_r && out_en)
        {
            chip.mor = out;
        }
    }
}

function OPN2_FMGenerate(chip)
{
    const slot = (chip.cycles + 19) % 24;
    /* Calculate phase */
    const phase = (chip.fm_mod[slot] + (chip.pg_phase[slot] >>> 10)) & 0x3ff;
    let quarter;
    let level;
    let output;
    if (phase & 0x100)
    {
        quarter = (phase ^ 0xff) & 0xff;
    }
    else
    {
        quarter = phase & 0xff;
    }
    level = logsinrom[quarter];
    /* Apply envelope */
    level = (level + (chip.eg_out[slot] << 2)) & 0xffff;    /* Bit16u */
    /* Transform */
    if (level > 0x1fff)
    {
        level = 0x1fff;
    }
    output = s16(((exprom[(level & 0xff) ^ 0xff] | 0x400) << 2) >> (level >> 8));
    if (phase & 0x200)
    {
        output = s16(((~output) ^ (chip.mode_test_21[4] << 13)) + 1);
    }
    else
    {
        output = s16(output ^ (chip.mode_test_21[4] << 13));
    }
    output = SIGN_EXTEND(13, output);
    chip.fm_out[slot] = output;
}

function OPN2_DoTimerA(chip)
{
    let time;
    let load;
    load = chip.timer_a_overflow;
    if (chip.cycles === 2)
    {
        /* Lock load value */
        load |= (!chip.timer_a_load_lock && chip.timer_a_load) ? 1 : 0;
        chip.timer_a_load_lock = chip.timer_a_load;
        if (chip.mode_csm)
        {
            /* CSM KeyOn */
            chip.mode_kon_csm = load;
        }
        else
        {
            chip.mode_kon_csm = 0;
        }
    }
    /* Load counter */
    if (chip.timer_a_load_latch)
    {
        time = chip.timer_a_reg;
    }
    else
    {
        time = chip.timer_a_cnt;
    }
    chip.timer_a_load_latch = load;
    /* Increase counter */
    if ((chip.cycles === 1 && chip.timer_a_load_lock) || chip.mode_test_21[2])
    {
        time = (time + 1) & 0xffff;
    }
    /* Set overflow flag */
    if (chip.timer_a_reset)
    {
        chip.timer_a_reset = 0;
        chip.timer_a_overflow_flag = 0;
    }
    else
    {
        chip.timer_a_overflow_flag |= chip.timer_a_overflow & chip.timer_a_enable;
    }
    chip.timer_a_overflow = (time >> 10) & 0xff;
    chip.timer_a_cnt = time & 0x3ff;
}

function OPN2_DoTimerB(chip)
{
    let time;
    let load;
    load = chip.timer_b_overflow;
    if (chip.cycles === 2)
    {
        /* Lock load value */
        load |= (!chip.timer_b_load_lock && chip.timer_b_load) ? 1 : 0;
        chip.timer_b_load_lock = chip.timer_b_load;
    }
    /* Load counter */
    if (chip.timer_b_load_latch)
    {
        time = chip.timer_b_reg;
    }
    else
    {
        time = chip.timer_b_cnt;
    }
    chip.timer_b_load_latch = load;
    /* Increase counter */
    if (chip.cycles === 1)
    {
        chip.timer_b_subcnt = (chip.timer_b_subcnt + 1) & 0xff;
    }
    if ((chip.timer_b_subcnt === 0x10 && chip.timer_b_load_lock) || chip.mode_test_21[2])
    {
        time = (time + 1) & 0xffff;
    }
    chip.timer_b_subcnt &= 0x0f;
    /* Set overflow flag */
    if (chip.timer_b_reset)
    {
        chip.timer_b_reset = 0;
        chip.timer_b_overflow_flag = 0;
    }
    else
    {
        chip.timer_b_overflow_flag |= chip.timer_b_overflow & chip.timer_b_enable;
    }
    chip.timer_b_overflow = (time >> 8) & 0xff;
    chip.timer_b_cnt = time & 0xff;
}

function OPN2_KeyOn(chip)
{
    const slot = chip.cycles;
    const chan = chip.channel;
    /* Key On */
    chip.eg_kon_latch[slot] = chip.mode_kon[slot];
    chip.eg_kon_csm[slot] = 0;
    if (chip.channel === 2 && chip.mode_kon_csm)
    {
        /* CSM Key On */
        chip.eg_kon_latch[slot] = 1;
        chip.eg_kon_csm[slot] = 1;
    }
    if (chip.cycles === chip.mode_kon_channel)
    {
        /* OP1 */
        chip.mode_kon[chan] = chip.mode_kon_operator[0];
        /* OP2 */
        chip.mode_kon[chan + 12] = chip.mode_kon_operator[1];
        /* OP3 */
        chip.mode_kon[chan + 6] = chip.mode_kon_operator[2];
        /* OP4 */
        chip.mode_kon[chan + 18] = chip.mode_kon_operator[3];
    }
}

function OPN2_Reset(chip)
{
    let i;
    ym3438_memset0(chip);                                   /* memset(chip, 0, sizeof(ym3438_t)); */
    for (i = 0; i < 24; i++)
    {
        chip.eg_out[i] = 0x3ff;
        chip.eg_level[i] = 0x3ff;
        chip.eg_state[i] = eg_num_release;
        chip.multi[i] = 1;
    }
    for (i = 0; i < 6; i++)
    {
        chip.pan_l[i] = 1;
        chip.pan_r[i] = 1;
    }
}

function OPN2_SetChipType(type)
{
    chip_type = type >>> 0;                                 /* Bit32u */
}

function OPN2_Clock(chip, buffer)
{
    const slot = chip.cycles;
    chip.lfo_inc = chip.mode_test_21[1];
    chip.pg_read >>>= 1;                                    /* Bit32u: logical shift */
    chip.eg_read[1] >>>= 1;                                 /* Bit32u: logical shift */
    chip.eg_cycle = (chip.eg_cycle + 1) & 0xff;
    /* Lock envelope generator timer value */
    if (chip.cycles === 1 && chip.eg_quotient === 2)
    {
        if (chip.eg_cycle_stop)
        {
            chip.eg_shift_lock = 0;
        }
        else
        {
            chip.eg_shift_lock = (chip.eg_shift + 1) & 0xff;
        }
        chip.eg_timer_low_lock = chip.eg_timer & 0x03;
    }
    /* Cycle specific functions */
    switch (chip.cycles)
    {
    case 0:
        chip.lfo_pm = chip.lfo_cnt >> 2;
        if (chip.lfo_cnt & 0x40)
        {
            chip.lfo_am = chip.lfo_cnt & 0x3f;
        }
        else
        {
            chip.lfo_am = chip.lfo_cnt ^ 0x3f;
        }
        chip.lfo_am = (chip.lfo_am << 1) & 0xff;
        break;
    case 1:
        chip.eg_quotient = (chip.eg_quotient + 1) & 0xffff;
        chip.eg_quotient %= 3;
        chip.eg_cycle = 0;
        chip.eg_cycle_stop = 1;
        chip.eg_shift = 0;
        chip.eg_timer_inc |= chip.eg_quotient >> 1;
        chip.eg_timer = (chip.eg_timer + chip.eg_timer_inc) & 0xffff;
        chip.eg_timer_inc = (chip.eg_timer >> 12) & 0xff;
        chip.eg_timer &= 0xfff;
        break;
    case 2:
        chip.pg_read = chip.pg_phase[21] & 0x3ff;
        chip.eg_read[1] = chip.eg_out[0];
        break;
    case 13:
        chip.eg_cycle = 0;
        chip.eg_cycle_stop = 1;
        chip.eg_shift = 0;
        chip.eg_timer = (chip.eg_timer + chip.eg_timer_inc) & 0xffff;
        chip.eg_timer_inc = (chip.eg_timer >> 12) & 0xff;
        chip.eg_timer &= 0xfff;
        break;
    case 23:
        chip.lfo_inc |= 1;
        break;
    }
    chip.eg_timer = (chip.eg_timer & ~(chip.mode_test_21[5] << chip.eg_cycle)) & 0xffff;
    if (((chip.eg_timer >> chip.eg_cycle) | (chip.pin_test_in & chip.eg_custom_timer)) & chip.eg_cycle_stop)
    {
        chip.eg_shift = chip.eg_cycle;
        chip.eg_cycle_stop = 0;
    }

    OPN2_DoIO(chip);

    OPN2_DoTimerA(chip);
    OPN2_DoTimerB(chip);
    OPN2_KeyOn(chip);

    OPN2_ChOutput(chip);
    OPN2_ChGenerate(chip);

    OPN2_FMPrepare(chip);
    OPN2_FMGenerate(chip);

    OPN2_PhaseGenerate(chip);
    OPN2_PhaseCalcIncrement(chip);

    OPN2_EnvelopeADSR(chip);
    OPN2_EnvelopeGenerate(chip);
    OPN2_EnvelopeSSGEG(chip);
    OPN2_EnvelopePrepare(chip);

    /* Prepare fnum & block */
    if (chip.mode_ch3)
    {
        /* Channel 3 special mode */
        switch (slot)
        {
        case 1: /* OP1 */
            chip.pg_fnum = chip.fnum_3ch[1];
            chip.pg_block = chip.block_3ch[1];
            chip.pg_kcode = chip.kcode_3ch[1];
            break;
        case 7: /* OP3 */
            chip.pg_fnum = chip.fnum_3ch[0];
            chip.pg_block = chip.block_3ch[0];
            chip.pg_kcode = chip.kcode_3ch[0];
            break;
        case 13: /* OP2 */
            chip.pg_fnum = chip.fnum_3ch[2];
            chip.pg_block = chip.block_3ch[2];
            chip.pg_kcode = chip.kcode_3ch[2];
            break;
        case 19: /* OP4 */
        default:
            chip.pg_fnum = chip.fnum[(chip.channel + 1) % 6];
            chip.pg_block = chip.block[(chip.channel + 1) % 6];
            chip.pg_kcode = chip.kcode[(chip.channel + 1) % 6];
            break;
        }
    }
    else
    {
        chip.pg_fnum = chip.fnum[(chip.channel + 1) % 6];
        chip.pg_block = chip.block[(chip.channel + 1) % 6];
        chip.pg_kcode = chip.kcode[(chip.channel + 1) % 6];
    }

    OPN2_UpdateLFO(chip);
    OPN2_DoRegWrite(chip);
    chip.cycles = (chip.cycles + 1) % 24;
    chip.channel = chip.cycles % 6;

    buffer[0] = chip.mol;
    buffer[1] = chip.mor;

    if (chip.status_time)
        chip.status_time--;
}

function OPN2_Write(chip, port, data)
{
    port = (port & 3) >>> 0;                                /* Bit32u port; port &= 3; */
    data &= 0xff;                                           /* Bit8u data */
    chip.write_data = (((port << 7) & 0x100) | data) & 0xffff;
    if (port & 1)
    {
        /* Data */
        chip.write_d |= 1;
    }
    else
    {
        /* Address */
        chip.write_a |= 1;
    }
}

function OPN2_SetTestPin(chip, value)
{
    chip.pin_test_in = value & 1;
}

function OPN2_ReadTestPin(chip)
{
    if (!chip.mode_test_2c[7])
    {
        return 0;
    }
    return (chip.cycles === 23) ? 1 : 0;
}

function OPN2_ReadIRQPin(chip)
{
    return chip.timer_a_overflow_flag | chip.timer_b_overflow_flag;
}

function OPN2_Read(chip, port)
{
    if ((port & 3) === 0 || (chip_type & ym3438_mode_readmode))
    {
        if (chip.mode_test_21[6])
        {
            /* Read test data */
            const slot = (chip.cycles + 18) % 24;
            let testdata = (((chip.pg_read & 0x01) << 15)
                          | ((chip.eg_read[chip.mode_test_21[0]] & 0x01) << 14)) & 0xffff;
            if (chip.mode_test_2c[4])
            {
                testdata |= chip.ch_read & 0x1ff;
            }
            else
            {
                testdata |= chip.fm_out[slot] & 0x3fff;
            }
            testdata &= 0xffff;                             /* Bit16u */
            if (chip.mode_test_21[7])
            {
                chip.status = testdata & 0xff;
            }
            else
            {
                chip.status = testdata >> 8;
            }
        }
        else
        {
            chip.status = ((chip.busy << 7) | (chip.timer_b_overflow_flag << 1)
                 | chip.timer_a_overflow_flag) & 0xff;
        }
        if (chip_type & ym3438_mode_ym2612)
        {
            chip.status_time = 300000;
        }
        else
        {
            chip.status_time = 40000000;
        }
    }
    if (chip.status_time)
    {
        return chip.status;
    }
    return 0;
}

return {
    /* ym3438.h */
    ym3438_mode_ym2612: ym3438_mode_ym2612,
    ym3438_mode_readmode: ym3438_mode_readmode,
    Bitu: Bitu, Bits: Bits, Bit64u: Bit64u, Bit64s: Bit64s,
    Bit32u: Bit32u, Bit32s: Bit32s, Bit16u: Bit16u, Bit16s: Bit16s, Bit8u: Bit8u, Bit8s: Bit8s,
    ym3438_t: ym3438_t,
    OPN2_Reset: OPN2_Reset,
    OPN2_SetChipType: OPN2_SetChipType,
    OPN2_Clock: OPN2_Clock,
    OPN2_Write: OPN2_Write,
    OPN2_SetTestPin: OPN2_SetTestPin,
    OPN2_ReadTestPin: OPN2_ReadTestPin,
    OPN2_ReadIRQPin: OPN2_ReadIRQPin,
    OPN2_Read: OPN2_Read
};

}));
