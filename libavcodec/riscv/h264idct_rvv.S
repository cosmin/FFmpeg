/*
 * Copyright © 2024 Rémi Denis-Courmont.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "libavutil/riscv/asm.S"

        .macro  sx rd, addr
#if (__riscv_xlen == 32)
        sw      \rd, \addr
#elif (__riscv_xlen == 64)
        sd      \rd, \addr
#else
        sq      \rd, \addr
#endif
        .endm

        .variant_cc ff_h264_idct4_rvv
func ff_h264_idct4_rvv, zve32x
        vsra.vi v5, v1, 1
        vsra.vi v7, v3, 1
        vadd.vv v8, v0, v2   # z0
        vsub.vv v9, v0, v2   # z1
        vsub.vv v10, v5, v3  # z2
        vadd.vv v11, v1, v7  # z3
        vadd.vv v1, v9, v10
        vsub.vv v2, v9, v10
        vadd.vv v0, v8, v11
        vsub.vv v3, v8, v11
        jr      t0
endfunc

func ff_h264_idct_add_8_rvv, zve32x
        csrwi       vxrm, 0
.Lidct_add4_8_rvv:
        vsetivli    zero, 4, e16, mf2, ta, ma
        addi        t1, a1, 1 * 4 * 2
        vle16.v     v0, (a1)
        addi        t2, a1, 2 * 4 * 2
        vle16.v     v1, (t1)
        addi        t3, a1, 3 * 4 * 2
        vle16.v     v2, (t2)
        vle16.v     v3, (t3)
        jal         t0, ff_h264_idct4_rvv
        vse16.v     v0, (a1)
        vse16.v     v1, (t1)
        vse16.v     v2, (t2)
        vse16.v     v3, (t3)
        vlseg4e16.v v0, (a1)
        .equ    offset, 0
        .rept   256 / __riscv_xlen
        sx      zero, offset(a1)
        .equ    offset, offset + (__riscv_xlen / 8)
        .endr
        jal         t0, ff_h264_idct4_rvv
        add         t1, a0, a2
        vle8.v      v4, (a0)
        add         t2, t1, a2
        vle8.v      v5, (t1)
        add         t3, t2, a2
        vle8.v      v6, (t2)
        vle8.v      v7, (t3)
        .irp    n,0,1,2,3
        vssra.vi    v\n, v\n, 6
        .endr
        vsetvli     zero, zero, e8, mf4, ta, ma
        vwaddu.wv   v0, v0, v4
        vwaddu.wv   v1, v1, v5
        vwaddu.wv   v2, v2, v6
        vwaddu.wv   v3, v3, v7
        vsetvli     zero, zero, e16, mf2, ta, ma
        .irp    n,0,1,2,3
        vmax.vx     v\n, v\n, zero
        .endr
        vsetvli     zero, zero, e8, mf4, ta, ma
        vnclipu.wi  v4, v0, 0
        vnclipu.wi  v5, v1, 0
        vnclipu.wi  v6, v2, 0
        vnclipu.wi  v7, v3, 0
        vse8.v      v4, (a0)
        vse8.v      v5, (t1)
        vse8.v      v6, (t2)
        vse8.v      v7, (t3)
        ret
endfunc

func ff_h264_idct_add_16_rvv, zve32x
        csrwi       vxrm, 0
        vsetivli    zero, 4, e32, m1, ta, ma
        addi        t1, a1, 1 * 4 * 4
        vle32.v     v0, (a1)
        addi        t2, a1, 2 * 4 * 4
        vle32.v     v1, (t1)
        addi        t3, a1, 3 * 4 * 4
        vle32.v     v2, (t2)
        vle32.v     v3, (t3)
        jal         t0, ff_h264_idct4_rvv
        vse32.v     v0, (a1)
        vse32.v     v1, (t1)
        vse32.v     v2, (t2)
        vse32.v     v3, (t3)
        vlseg4e32.v v0, (a1)
        .equ    offset, 0
        .rept   512 / __riscv_xlen
        sx      zero, offset(a1)
        .equ    offset, offset + (__riscv_xlen / 8)
        .endr
        jal         t0, ff_h264_idct4_rvv
        add         t1, a0, a2
        vle16.v     v4, (a0)
        add         t2, t1, a2
        vle16.v     v5, (t1)
        add         t3, t2, a2
        vle16.v     v6, (t2)
        vle16.v     v7, (t3)
        .irp    n,0,1,2,3
        vssra.vi    v\n, v\n, 6
        .endr
        vsetvli     zero, zero, e16, mf2, ta, ma
        vwaddu.wv   v0, v0, v4
        vwaddu.wv   v1, v1, v5
        vwaddu.wv   v2, v2, v6
        vwaddu.wv   v3, v3, v7
        vsetvli     zero, zero, e32, m1, ta, ma
        .irp    n,0,1,2,3
        vmax.vx     v\n, v\n, zero
        .endr
        .irp    n,0,1,2,3
        vmin.vx     v\n, v\n, a3
        .endr
        vsetvli     zero, zero, e16, mf2, ta, ma
        vncvt.x.x.w v4, v0
        vncvt.x.x.w v5, v1
        vncvt.x.x.w v6, v2
        vncvt.x.x.w v7, v3
        vse16.v     v4, (a0)
        vse16.v     v5, (t1)
        vse16.v     v6, (t2)
        vse16.v     v7, (t3)
        ret
endfunc

        .variant_cc ff_h264_idct8_rvv
func ff_h264_idct8_rvv, zve32x
        vsra.vi v9, v7, 1
        vsra.vi v11, v3, 1
        vsra.vi v12, v2, 1
        vsra.vi v13, v5, 1
        vsra.vi v14, v6, 1
        vsra.vi v15, v1, 1
        vadd.vv v9, v3, v9
        vsub.vv v11, v1, v11
        vsub.vv v13, v13, v1
        vadd.vv v15, v3, v15
        vsub.vv v9, v5, v9
        vadd.vv v11, v11, v7
        vadd.vv v13, v13, v7
        vadd.vv v15, v15, v5
        vadd.vv v8, v0, v4      # a0
        vsub.vv v9, v9, v7      # a1
        vsub.vv v10, v0, v4     # a2
        vsub.vv v11, v11, v3    # a3
        vsub.vv v12, v12, v6    # a4
        vadd.vv v13, v13, v5    # a5
        vadd.vv v14, v14, v2    # a6
        vadd.vv v15, v15, v1    # a7
        vsra.vi v7, v9, 2
        vsra.vi v5, v11, 2
        vsra.vi v3, v13, 2
        vsra.vi v1, v15, 2
        vadd.vv v0, v8, v14     # b0
        vadd.vv v6, v10, v12    # b2
        vsub.vv v2, v10, v12    # b4
        vsub.vv v4, v8, v14     # b6
        vsub.vv v8, v15, v7     # b7
        vsub.vv v14, v5, v13    # b5
        vadd.vv v12, v1, v9     # b1
        vadd.vv v10, v11, v3    # b3
        vadd.vv v1, v6, v14
        vsub.vv v6, v6, v14
        vsub.vv v7, v0, v8
        vadd.vv v0, v0, v8
        vsub.vv v5, v2, v10
        vadd.vv v2, v2, v10
        vadd.vv v3, v4, v12
        vsub.vv v4, v4, v12
        jr      t0
endfunc

func ff_h264_idct8_add_8_rvv, zve32x
        csrwi       vxrm, 0
.Lidct8_add_8_rvv:
        vsetivli    zero, 8, e16, m1, ta, ma
        addi        t1, a1, 1 * 8 * 2
        vle16.v     v0, (a1)
        addi        t2, a1, 2 * 8 * 2
        vle16.v     v1, (t1)
        addi        t3, a1, 3 * 8 * 2
        vle16.v     v2, (t2)
        addi        t4, a1, 4 * 8 * 2
        vle16.v     v3, (t3)
        addi        t5, a1, 5 * 8 * 2
        vle16.v     v4, (t4)
        addi        t6, a1, 6 * 8 * 2
        vle16.v     v5, (t5)
        addi        a7, a1, 7 * 8 * 2
        vle16.v     v6, (t6)
        vle16.v     v7, (a7)
        jal         t0, ff_h264_idct8_rvv
        vse16.v     v0, (a1)
        vse16.v     v1, (t1)
        vse16.v     v2, (t2)
        vse16.v     v3, (t3)
        vse16.v     v4, (t4)
        vse16.v     v5, (t5)
        vse16.v     v6, (t6)
        vse16.v     v7, (a7)
        vlseg8e16.v v0, (a1)
        .equ    offset, 0
        .rept   1024 / __riscv_xlen
        sx      zero, offset(a1)
        .equ    offset, offset + (__riscv_xlen / 8)
        .endr
        jal         t0, ff_h264_idct8_rvv
        add         t1, a0, a2
        vle8.v      v16, (a0)
        add         t2, t1, a2
        vle8.v      v17, (t1)
        add         t3, t2, a2
        vle8.v      v18, (t2)
        add         t4, t3, a2
        vle8.v      v19, (t3)
        add         t5, t4, a2
        vle8.v      v20, (t4)
        add         t6, t5, a2
        vle8.v      v21, (t5)
        add         a7, t6, a2
        vle8.v      v22, (t6)
        vle8.v      v23, (a7)
        .irp    n,0,1,2,3,4,5,6,7
        vssra.vi    v\n, v\n, 6
        .endr
        vsetvli     zero, zero, e8, mf2, ta, ma
        vwaddu.wv   v0, v0, v16
        vwaddu.wv   v1, v1, v17
        vwaddu.wv   v2, v2, v18
        vwaddu.wv   v3, v3, v19
        vwaddu.wv   v4, v4, v20
        vwaddu.wv   v5, v5, v21
        vwaddu.wv   v6, v6, v22
        vwaddu.wv   v7, v7, v23
        vsetvli     zero, zero, e16, m1, ta, ma
        .irp    n,0,1,2,3,4,5,6,7
        vmax.vx     v\n, v\n, zero
        .endr
        vsetvli     zero, zero, e8, mf2, ta, ma
        vnclipu.wi  v16, v0, 0
        vnclipu.wi  v17, v1, 0
        vnclipu.wi  v18, v2, 0
        vnclipu.wi  v19, v3, 0
        vnclipu.wi  v20, v4, 0
        vnclipu.wi  v21, v5, 0
        vnclipu.wi  v22, v6, 0
        vnclipu.wi  v23, v7, 0
        vse8.v      v16, (a0)
        vse8.v      v17, (t1)
        vse8.v      v18, (t2)
        vse8.v      v19, (t3)
        vse8.v      v20, (t4)
        vse8.v      v21, (t5)
        vse8.v      v22, (t6)
        vse8.v      v23, (a7)
        ret
endfunc

func ff_h264_idct8_add_16_rvv, zve32x
        li      a4, 8
        csrwi   vxrm, 0
        vsetivli    a5, 8, e32, m1, ta, ma
1:
        addi    t1, a1, 1 * 8 * 4
        vle32.v     v0, (a1)
        addi    t2, a1, 2 * 8 * 4
        vle32.v     v1, (t1)
        addi    t3, a1, 3 * 8 * 4
        vle32.v     v2, (t2)
        addi    t4, a1, 4 * 8 * 4
        vle32.v     v3, (t3)
        addi    t5, a1, 5 * 8 * 4
        vle32.v     v4, (t4)
        addi    t6, a1, 6 * 8 * 4
        vle32.v     v5, (t5)
        addi    a7, a1, 7 * 8 * 4
        vle32.v     v6, (t6)
        sub     a4, a4, a5
        vle32.v     v7, (a7)
        jal     t0, ff_h264_idct8_rvv
        vse32.v     v0, (a1)
        sh2add  a1, a5, a1
        vse32.v     v1, (t1)
        vse32.v     v2, (t2)
        vse32.v     v3, (t3)
        vse32.v     v4, (t4)
        vse32.v     v5, (t5)
        vse32.v     v6, (t6)
        vse32.v     v7, (a7)
        bnez    a4, 1b

        addi    a1, a1, -8 * 4
        li      a4, 8
        slli    a6, a5, 3 + 2
2:
        vsetvli     zero, zero, e32, m1, ta, ma
        vlseg8e32.v v0, (a1)
        jal     t0, ff_h264_idct8_rvv
        add     t1, a0, a2
        vle16.v     v16, (a0)
        add     t2, t1, a2
        vle16.v     v17, (t1)
        add     t3, t2, a2
        vle16.v     v18, (t2)
        add     t4, t3, a2
        vle16.v     v19, (t3)
        add     t5, t4, a2
        vle16.v     v20, (t4)
        add     t6, t5, a2
        vle16.v     v21, (t5)
        add     a7, t6, a2
        vle16.v     v22, (t6)
        sub     a4, a4, a5
        vle16.v     v23, (a7)
        .irp    n,0,1,2,3,4,5,6,7
        vssra.vi    v\n, v\n, 6
        .endr
        vsetvli     zero, zero, e16, mf2, ta, ma
        vwaddu.wv   v0, v0, v16
        add     a1, a6, a1
        vwaddu.wv   v1, v1, v17
        vwaddu.wv   v2, v2, v18
        vwaddu.wv   v3, v3, v19
        vwaddu.wv   v4, v4, v20
        vwaddu.wv   v5, v5, v21
        vwaddu.wv   v6, v6, v22
        vwaddu.wv   v7, v7, v23
        vsetvli     zero, zero, e32, m1, ta, ma
        .irp    n,0,1,2,3,4,5,6,7
        vmax.vx     v\n, v\n, zero
        .endr
        .irp    n,0,1,2,3,4,5,6,7
        vmin.vx     v\n, v\n, a3
        .endr
        vsetvli     zero, zero, e16, mf2, ta, ma
        vncvt.x.x.w v16, v0
        vncvt.x.x.w v17, v1
        vncvt.x.x.w v18, v2
        vncvt.x.x.w v19, v3
        vncvt.x.x.w v20, v4
        vncvt.x.x.w v21, v5
        vncvt.x.x.w v22, v6
        vncvt.x.x.w v23, v7
        vse16.v     v16, (a0)
        sh1add  a0, a5, a0
        vse16.v     v17, (t1)
        vse16.v     v18, (t2)
        vse16.v     v19, (t3)
        vse16.v     v20, (t4)
        vse16.v     v21, (t5)
        vse16.v     v22, (t6)
        vse16.v     v23, (a7)
        bnez    a4, 2b

        .equ    offset, 0
        .rept   2048 / __riscv_xlen
        sx      zero, offset - 8 * 8 * 4(a1)
        .equ    offset, offset + (__riscv_xlen / 8)
        .endr
        ret
endfunc

.irp    depth, 9, 10, 12, 14
func ff_h264_idct_add_\depth\()_rvv, zve32x
        li      a3, (1 << \depth) - 1
        j       ff_h264_idct_add_16_rvv
endfunc

func ff_h264_idct8_add_\depth\()_rvv, zve32x
        li      a3, (1 << \depth) - 1
        j       ff_h264_idct8_add_16_rvv
endfunc
.endr

const ff_h264_scan8
        .byte   014, 015, 024, 025, 016, 017, 026, 027
        .byte   034, 035, 044, 045, 036, 037, 046, 047
endconst

#if (__riscv_xlen == 64)
.irp    depth, 8
func ff_h264_idct_add16_\depth\()_rvv, zve32x
        csrwi   vxrm, 0
        addi    sp, sp, -80
        lla     t0, ff_h264_scan8
        sd      s0,   (sp)
        li      t1, 32 << (\depth > 8)
        mv      s0, sp
        sd      ra,  8(sp)
        sd      s1, 16(sp)
        sd      s2, 24(sp)
        sd      s3, 32(sp)
        sd      s4, 40(sp)
        sd      s5, 48(sp)
        sd      s6, 56(sp)
        sd      s7, 64(sp)
        vsetivli  zero, 16, e8, m1, ta, ma
        vle8.v    v8, (t0)
        vlse16.v  v16, (a2), t1
        vluxei8.v v12, (a4), v8
.if \depth == 8
        vsetvli   zero, zero, e16, m2, ta, ma
.else
        vsetvli   zero, zero, e32, m4, ta, ma
.endif
        vmsne.vi  v1, v16, 0
        vsetvli   zero, zero, e8, m1, ta, ma
        vmseq.vi  v2, v12, 1
        vmsne.vi  v0, v12, 0
        vmand.mm  v1, v1, v2
        vsetvli   zero, zero, e16, m2, ta, ma
        vmv.x.s   s2, v0
        vmv.x.s   s3, v1
        li      s1, 16
        mv      s4, a0
        mv      s5, a1
        mv      s6, a2
        mv      s7, a3
1:
        andi    t0, s2, 1
        addi    s1, s1, -1
        srli    s2, s2, 1
        beqz    t0, 3f     # if (nnz)
        lw      t2, (s5)   # block_offset[i]
        andi    t1, s3, 1
        mv      a1, s6
        mv      a2, s7
        add     a0, s4, t2
        beqz    t1, 2f    # if (nnz == 1 && block[i * 16])
        call    ff_h264_idct_dc_add_\depth\()_c
        j       3f
2:
        call    .Lidct_add4_\depth\()_rvv
3:
        srli    s3, s3, 1
        addi    s5, s5, 4
        addi    s6, s6, 16 * 2 << (\depth > 8)
        bnez    s1, 1b

        ld      s7, 64(sp)
        ld      s6, 56(sp)
        ld      s5, 48(sp)
        ld      s4, 40(sp)
        ld      s3, 32(sp)
        ld      s2, 24(sp)
        ld      s1, 16(sp)
        ld      ra,  8(sp)
        ld      s0,  0(sp)
        addi    sp, sp, 80
        ret
endfunc

func ff_h264_idct_add16intra_\depth\()_rvv, zve32x
        csrwi   vxrm, 0
        addi    sp, sp, -80
        lla     t0, ff_h264_scan8
        sd      s0,   (sp)
        li      t1, 32 << (\depth > 8)
        mv      s0, sp
        sd      ra,  8(sp)
        sd      s1, 16(sp)
        sd      s2, 24(sp)
        sd      s3, 32(sp)
        sd      s4, 40(sp)
        sd      s5, 48(sp)
        sd      s6, 56(sp)
        sd      s7, 64(sp)
        vsetivli  zero, 16, e8, m1, ta, ma
        vle8.v    v8, (t0)
        vlse16.v  v16, (a2), t1
        vluxei8.v v12, (a4), v8
.if \depth == 8
        vsetvli   zero, zero, e16, m2, ta, ma
.else
        vsetvli   zero, zero, e32, m4, ta, ma
.endif
        vmsne.vi  v1, v16, 0
        vsetvli   zero, zero, e8, m1, ta, ma
        vmsne.vi  v0, v12, 0
        vsetvli   zero, zero, e16, m2, ta, ma
        vmv.x.s   s2, v0
        vmv.x.s   s3, v1
        li      s1, 16
        mv      s4, a0
        mv      s5, a1
        mv      s6, a2
        mv      s7, a3
1:
        andi    t0, s2, 1
        addi    s1, s1, -1
        srli    s2, s2, 1
        lw      t2, (s5)   # block_offset[i]
        andi    t1, s3, 1
        mv      a1, s6
        mv      a2, s7
        add     a0, s4, t2
        beqz    t0, 2f     # if (nnzc[scan8[i]])
        call    .Lidct_add4_\depth\()_rvv
        j       3f
2:
        beqz    t1, 3f    # if (block[i * 16])
        call    ff_h264_idct_dc_add_\depth\()_c
3:
        srli    s3, s3, 1
        addi    s5, s5, 4
        addi    s6, s6, 16 * 2 << (\depth > 8)
        bnez    s1, 1b

        ld      s7, 64(sp)
        ld      s6, 56(sp)
        ld      s5, 48(sp)
        ld      s4, 40(sp)
        ld      s3, 32(sp)
        ld      s2, 24(sp)
        ld      s1, 16(sp)
        ld      ra,  8(sp)
        ld      s0,  0(sp)
        addi    sp, sp, 80
        ret
endfunc

func ff_h264_idct8_add4_\depth\()_rvv, zve32x
        csrwi       vxrm, 0
        addi    sp, sp, -80
        lla     t0, ff_h264_scan8
        sd      s0,   (sp)
        li      t1, 4 * 32 << (\depth > 8)
        mv      s0, sp
        li      t2, 4
        sd      ra,  8(sp)
        sd      s1, 16(sp)
        sd      s2, 24(sp)
        sd      s3, 32(sp)
        sd      s4, 40(sp)
        sd      s5, 48(sp)
        sd      s6, 56(sp)
        sd      s7, 64(sp)
        vsetivli  zero, 4, e8, mf4, ta, ma
        vlse8.v   v8, (t0), t2
        vlse16.v  v16, (a2), t1
        vluxei8.v v12, (a4), v8
.if \depth == 8
        vsetvli   zero, zero, e16, mf2, ta, ma
.else
        vsetvli   zero, zero, e32, m1, ta, ma
.endif
        vmsne.vi  v1, v16, 0
        vsetvli   zero, zero, e8, mf4, ta, ma
        vmseq.vi  v2, v12, 1
        vmsne.vi  v0, v12, 0
        vmand.mm  v1, v1, v2
        vmv.x.s   s2, v0
        vmv.x.s   s3, v1
        li      s1, 4
        mv      s4, a0
        mv      s5, a1
        mv      s6, a2
        mv      s7, a3
1:
        andi    t0, s2, 1
        addi    s1, s1, -1
        srli    s2, s2, 1
        beqz    t0, 3f     # if (nnz)
        lw      t2, (s5)   # block_offset[i]
        andi    t1, s3, 1
        mv      a1, s6
        mv      a2, s7
        add     a0, s4, t2
        beqz    t1, 2f    # if (nnz == 1 && block[i * 16])
        call    ff_h264_idct8_dc_add_\depth\()_c
        j       3f
2:
        call    .Lidct8_add_\depth\()_rvv
3:
        srli    s3, s3, 1
        addi    s5, s5, 4 * 4
        addi    s6, s6, 4 * 16 * 2 << (\depth > 8)
        bnez    s1, 1b

        ld      s7, 64(sp)
        ld      s6, 56(sp)
        ld      s5, 48(sp)
        ld      s4, 40(sp)
        ld      s3, 32(sp)
        ld      s2, 24(sp)
        ld      s1, 16(sp)
        ld      ra,  8(sp)
        ld      s0,  0(sp)
        addi    sp, sp, 80
        ret
endfunc
.endr
#endif
