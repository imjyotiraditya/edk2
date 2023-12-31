# Copyright (c) 2021 The Linux Foundation. All rights reserved.
# Not a contribution.
#
# Copyright 2014-2020 The OpenSSL Project Authors. All Rights Reserved.
#
# Licensed under the Apache License 2.0 (the "License").  You may not use
# this file except in compliance with the License.  You can obtain a copy
# in the file LICENSE in the source distribution or at
# https://www.openssl.org/source/license.html

#
# ====================================================================
# Written by Andy Polyakov <appro@openssl.org> for the OpenSSL
# project. The module is, however, dual licensed under OpenSSL and
# CRYPTOGAMS licenses depending on where you obtain it. For further
# details see http://www.openssl.org/~appro/cryptogams/.
# ====================================================================
#
# This module implements support for ARMv8 AES instructions. The
# module is endian-agnostic in sense that it supports both big- and
# little-endian cases. As does it support both 32- and 64-bit modes
# of operation. Latter is achieved by limiting amount of utilized
# registers to 16, which implies additional NEON load and integer
# instructions. This has no effect on mighty Apple A7, where results
# are literally equal to the theoretical estimates based on AES
# instruction latencies and issue rates. On Cortex-A53, an in-order
# execution core, this costs up to 10-15%, which is partially
# compensated by implementing dedicated code path for 128-bit
# CBC encrypt case. On Cortex-A57 parallelizable mode performance
# seems to be limited by sheer amount of NEON instructions...
#
# April 2019
#
# Key to performance of parallelize-able modes is round instruction
# interleaving. But which factor to use? There is optimal one for
# each combination of instruction latency and issue rate, beyond
# which increasing interleave factor doesn't pay off. While on cons
# side we have code size increase and resource waste on platforms for
# which interleave factor is too high. In other words you want it to
# be just right. So far interleave factor of 3x was serving well all
# platforms. But for ThunderX2 optimal interleave factor was measured
# to be 5x...
#
# Performance in cycles per byte processed with 128-bit key:
#
#		CBC enc		CBC dec		CTR
# Apple A7	2.39		1.20		1.20
# Cortex-A53	1.32		1.17/1.29(**)	1.36/1.46
# Cortex-A57(*)	1.95		0.82/0.85	0.89/0.93
# Cortex-A72	1.33		0.85/0.88	0.92/0.96
# Denver	1.96		0.65/0.86	0.76/0.80
# Mongoose	1.33		1.23/1.20	1.30/1.20
# Kryo		1.26		0.87/0.94	1.00/1.00
# ThunderX2	5.95		1.25		1.30
#
# (*)	original 3.64/1.34/1.32 results were for r0p0 revision
#	and are still same even for updated module;
# (**)	numbers after slash are for 32-bit code, which is 3x-
#	interleaved;

#include "arm_arch.h"

.section        .rodata
.align	5
.type   Lrcon, %object
.Lrcon:
.long	0x01,0x01,0x01,0x01
.long	0x0c0f0e0d,0x0c0f0e0d,0x0c0f0e0d,0x0c0f0e0d	// rotate-n-splat
.long	0x1b,0x1b,0x1b,0x1b

.section  .text.ARMv8_AES
.arch	armv8-a+crypto
.align  5

.globl	aes_v8_set_encrypt_key
.type	aes_v8_set_encrypt_key,%function
.align	5
aes_v8_set_encrypt_key:
.Lenc_key:

#ifdef UCLIB_NO_VFP_KERNEL
  /* This feature-flag is being used to support (only) AES128-CMAC in the TZ
   * kernel, by
   * 1. Saving the interrupt state, then masking interrupts
   * 2. Saving all NEON registers used in the function onto the stack
   * 3. Restoring those registers back from the stack before returning
   *
   * Currently this is only being done in the set-encrypt-key and cbc-encrypt
   * functions, so as to be able to support AES-CMAC.
   * x9 is being used to store DAIF since it is a 'scratch' register which we
   * expect to be caller-saved.
   */
  mrs  x9, DAIF
  msr  DAIFSet, #0x3

  stp q0,q1,[sp,#-32]!
  stp q2,q3,[sp,#-32]!
  stp q4,q5,[sp,#-32]!
  stp q6,q7,[sp,#-32]!

  dsb sy
  isb
#endif

	stp	x29,x30,[sp,#-16]!
	add	x29,sp,#0
	mov	x3,#-1
	cmp	x0,#0
	b.eq	.Lenc_key_abort
	cmp	x2,#0
	b.eq	.Lenc_key_abort
	mov	x3,#-2
	cmp	w1,#128
	b.lt	.Lenc_key_abort
	cmp	w1,#256
	b.gt	.Lenc_key_abort
	tst	w1,#0x3f
	b.ne	.Lenc_key_abort

	adr	x3,.Lrcon
	cmp	w1,#192

	eor	v0.16b,v0.16b,v0.16b
	ld1	{v3.16b},[x0],#16
	mov	w1,#8		// reuse w1
	ld1	{v1.4s,v2.4s},[x3],#32

	b.lt	.Loop128
	b.eq	.L192
	b	.L256

.align	4
.Loop128:
	tbl	v6.16b,{v3.16b},v2.16b
	ext	v5.16b,v0.16b,v3.16b,#12
	st1	{v3.4s},[x2],#16
	aese	v6.16b,v0.16b
	subs	w1,w1,#1

	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	 eor	v6.16b,v6.16b,v1.16b
	eor	v3.16b,v3.16b,v5.16b
	shl	v1.16b,v1.16b,#1
	eor	v3.16b,v3.16b,v6.16b
	b.ne	.Loop128

	ld1	{v1.4s},[x3]

	tbl	v6.16b,{v3.16b},v2.16b
	ext	v5.16b,v0.16b,v3.16b,#12
	st1	{v3.4s},[x2],#16
	aese	v6.16b,v0.16b

	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	 eor	v6.16b,v6.16b,v1.16b
	eor	v3.16b,v3.16b,v5.16b
	shl	v1.16b,v1.16b,#1
	eor	v3.16b,v3.16b,v6.16b

	tbl	v6.16b,{v3.16b},v2.16b
	ext	v5.16b,v0.16b,v3.16b,#12
	st1	{v3.4s},[x2],#16
	aese	v6.16b,v0.16b

	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	 eor	v6.16b,v6.16b,v1.16b
	eor	v3.16b,v3.16b,v5.16b
	eor	v3.16b,v3.16b,v6.16b
	st1	{v3.4s},[x2]
	add	x2,x2,#0x50

	mov	w12,#10
	b	.Ldone

.align	4
.L192:
	ld1	{v4.8b},[x0],#8
	movi	v6.16b,#8			// borrow v6.16b
	st1	{v3.4s},[x2],#16
	sub	v2.16b,v2.16b,v6.16b	// adjust the mask

.Loop192:
	tbl	v6.16b,{v4.16b},v2.16b
	ext	v5.16b,v0.16b,v3.16b,#12
	st1	{v4.8b},[x2],#8
	aese	v6.16b,v0.16b
	subs	w1,w1,#1

	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	eor	v3.16b,v3.16b,v5.16b

	dup	v5.4s,v3.s[3]
	eor	v5.16b,v5.16b,v4.16b
	 eor	v6.16b,v6.16b,v1.16b
	ext	v4.16b,v0.16b,v4.16b,#12
	shl	v1.16b,v1.16b,#1
	eor	v4.16b,v4.16b,v5.16b
	eor	v3.16b,v3.16b,v6.16b
	eor	v4.16b,v4.16b,v6.16b
	st1	{v3.4s},[x2],#16
	b.ne	.Loop192

	mov	w12,#12
	add	x2,x2,#0x20
	b	.Ldone

.align	4
.L256:
	ld1	{v4.16b},[x0]
	mov	w1,#7
	mov	w12,#14
	st1	{v3.4s},[x2],#16

.Loop256:
	tbl	v6.16b,{v4.16b},v2.16b
	ext	v5.16b,v0.16b,v3.16b,#12
	st1	{v4.4s},[x2],#16
	aese	v6.16b,v0.16b
	subs	w1,w1,#1

	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	eor	v3.16b,v3.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	 eor	v6.16b,v6.16b,v1.16b
	eor	v3.16b,v3.16b,v5.16b
	shl	v1.16b,v1.16b,#1
	eor	v3.16b,v3.16b,v6.16b
	st1	{v3.4s},[x2],#16
	b.eq	.Ldone

	dup	v6.4s,v3.s[3]		// just splat
	ext	v5.16b,v0.16b,v4.16b,#12
	aese	v6.16b,v0.16b

	eor	v4.16b,v4.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	eor	v4.16b,v4.16b,v5.16b
	ext	v5.16b,v0.16b,v5.16b,#12
	eor	v4.16b,v4.16b,v5.16b

	eor	v4.16b,v4.16b,v6.16b
	b	.Loop256

.Ldone:
	str	w12,[x2]
	mov	x3,#0

.Lenc_key_abort:
	mov	x0,x3			// return value
	ldr	x29,[sp],#16

#ifdef UCLIB_NO_VFP_KERNEL
  ldp q6,q7,[sp],#32
  ldp q4,q5,[sp],#32
  ldp q2,q3,[sp],#32
  ldp q0,q1,[sp],#32

  msr  DAIF, x9

  dsb sy
  isb

#endif

	ret
.size	aes_v8_set_encrypt_key,.-aes_v8_set_encrypt_key

.globl	aes_v8_set_decrypt_key
.type	aes_v8_set_decrypt_key,%function
.align	5
aes_v8_set_decrypt_key:

#ifdef UCLIB_NO_VFP_KERNEL
  mrs  x9, DAIF
  msr  DAIFSet, #0x3

  stp q0,q1,[sp,#-32]!

  dsb sy
  isb
#endif

	stp	x29,x30,[sp,#-16]!
	add	x29,sp,#0
	bl	.Lenc_key

	cmp	x0,#0
	b.ne	.Ldec_key_abort

	sub	x2,x2,#240		// restore original x2
	mov	x4,#-16
	add	x0,x2,x12,lsl#4	// end of key schedule

	ld1	{v0.4s},[x2]
	ld1	{v1.4s},[x0]
	st1	{v0.4s},[x0],x4
	st1	{v1.4s},[x2],#16

.Loop_imc:
	ld1	{v0.4s},[x2]
	ld1	{v1.4s},[x0]
	aesimc	v0.16b,v0.16b
	aesimc	v1.16b,v1.16b
	st1	{v0.4s},[x0],x4
	st1	{v1.4s},[x2],#16
	cmp	x0,x2
	b.hi	.Loop_imc

	ld1	{v0.4s},[x2]
	aesimc	v0.16b,v0.16b
	st1	{v0.4s},[x0]

	eor	x0,x0,x0		// return value
.Ldec_key_abort:
	ldp	x29,x30,[sp],#16

#ifdef UCLIB_NO_VFP_KERNEL
  ldp q0,q1,[sp],#32

  msr  DAIF, x9

  dsb sy
  isb

#endif

	ret
.size	aes_v8_set_decrypt_key,.-aes_v8_set_decrypt_key
.globl	aes_v8_encrypt
.type	aes_v8_encrypt,%function
.align	5
aes_v8_encrypt:
#ifdef UCLIB_NO_VFP_KERNEL
  mrs  x9, DAIF
  msr  DAIFSet, #0x3

  stp q0,q1,[sp,#-32]!
  stp q2,q3,[sp,#-32]!

  dsb sy
  isb
#endif

	ldr	w3,[x2,#240]
	ld1	{v0.4s},[x2],#16
	ld1	{v2.16b},[x0]
	sub	w3,w3,#2
	ld1	{v1.4s},[x2],#16

.Loop_enc:
	aese	v2.16b,v0.16b
	aesmc	v2.16b,v2.16b
	ld1	{v0.4s},[x2],#16
	subs	w3,w3,#2
	aese	v2.16b,v1.16b
	aesmc	v2.16b,v2.16b
	ld1	{v1.4s},[x2],#16
	b.gt	.Loop_enc

	aese	v2.16b,v0.16b
	aesmc	v2.16b,v2.16b
	ld1	{v0.4s},[x2]
	aese	v2.16b,v1.16b
	eor	v2.16b,v2.16b,v0.16b

	st1	{v2.16b},[x1]
#ifdef UCLIB_NO_VFP_KERNEL
  ldp q2,q3,[sp],#32
  ldp q0,q1,[sp],#32

  msr  DAIF, x9

  dsb sy
  isb

#endif

	ret
.size	aes_v8_encrypt,.-aes_v8_encrypt
.globl	aes_v8_decrypt
.type	aes_v8_decrypt,%function
.align	5
aes_v8_decrypt:
#ifdef UCLIB_NO_VFP_KERNEL
  mrs  x9, DAIF
  msr  DAIFSet, #0x3

  stp q0,q1,[sp,#-32]!
  stp q2,q3,[sp,#-32]!

  dsb sy
  isb
#endif

	ldr	w3,[x2,#240]
	ld1	{v0.4s},[x2],#16
	ld1	{v2.16b},[x0]
	sub	w3,w3,#2
	ld1	{v1.4s},[x2],#16

.Loop_dec:
	aesd	v2.16b,v0.16b
	aesimc	v2.16b,v2.16b
	ld1	{v0.4s},[x2],#16
	subs	w3,w3,#2
	aesd	v2.16b,v1.16b
	aesimc	v2.16b,v2.16b
	ld1	{v1.4s},[x2],#16
	b.gt	.Loop_dec

	aesd	v2.16b,v0.16b
	aesimc	v2.16b,v2.16b
	ld1	{v0.4s},[x2]
	aesd	v2.16b,v1.16b
	eor	v2.16b,v2.16b,v0.16b

	st1	{v2.16b},[x1]
#ifdef UCLIB_NO_VFP_KERNEL
  ldp q2,q3,[sp],#32
  ldp q0,q1,[sp],#32

  msr  DAIF, x9

  dsb sy
  isb

#endif

	ret
.size	aes_v8_decrypt,.-aes_v8_decrypt
.globl	aes_v8_cbc_encrypt
.type	aes_v8_cbc_encrypt,%function
.align	5
aes_v8_cbc_encrypt:

#ifdef UCLIB_NO_VFP_KERNEL
  mrs  x9, DAIF
  msr  DAIFSet, #0x3

  stp q0,q1,[sp,#-32]!
  stp q2,q3,[sp,#-32]!
  stp q4,q5,[sp,#-32]!
  stp q6,q7,[sp,#-32]!
  stp q16,q17,[sp,#-32]!
  stp q18,q19,[sp,#-32]!
  stp q20,q21,[sp,#-32]!
  stp q22,q23,[sp,#-32]!

  dsb sy
  isb
#endif

	stp	x29,x30,[sp,#-16]!
	add	x29,sp,#0
	subs	x2,x2,#16
	mov	x8,#16
	b.lo	.Lcbc_abort
	csel	x8,xzr,x8,eq

	cmp	w5,#0			// en- or decrypting?
	ldr	w5,[x3,#240]
	and	x2,x2,#-16
	ld1	{v6.16b},[x4]
	ld1	{v0.16b},[x0],x8

	ld1	{v16.4s-v17.4s},[x3]		// load key schedule...
	sub	w5,w5,#6
	add	x7,x3,x5,lsl#4	// pointer to last 7 round keys
	sub	w5,w5,#2
	ld1	{v18.4s-v19.4s},[x7],#32
	ld1	{v20.4s-v21.4s},[x7],#32
	ld1	{v22.4s-v23.4s},[x7],#32
	ld1	{v7.4s},[x7]

	add	x7,x3,#32
	mov	w6,w5
	b.eq	.Lcbc_dec

	cmp	w5,#2
	eor	v0.16b,v0.16b,v6.16b
	eor	v5.16b,v16.16b,v7.16b
	b.eq	.Lcbc_enc128

	ld1	{v2.4s-v3.4s},[x7]
	add	x7,x3,#16
	add	x6,x3,#16*4
	add	x12,x3,#16*5
	aese	v0.16b,v16.16b
	aesmc	v0.16b,v0.16b
	add	x14,x3,#16*6
	add	x3,x3,#16*7
	b	.Lenter_cbc_enc

.align	4
.Loop_cbc_enc:
	aese	v0.16b,v16.16b
	aesmc	v0.16b,v0.16b
	 st1	{v6.16b},[x1],#16
.Lenter_cbc_enc:
	aese	v0.16b,v17.16b
	aesmc	v0.16b,v0.16b
	aese	v0.16b,v2.16b
	aesmc	v0.16b,v0.16b
	ld1	{v16.4s},[x6]
	cmp	w5,#4
	aese	v0.16b,v3.16b
	aesmc	v0.16b,v0.16b
	ld1	{v17.4s},[x12]
	b.eq	.Lcbc_enc192

	aese	v0.16b,v16.16b
	aesmc	v0.16b,v0.16b
	ld1	{v16.4s},[x14]
	aese	v0.16b,v17.16b
	aesmc	v0.16b,v0.16b
	ld1	{v17.4s},[x3]
	nop

.Lcbc_enc192:
	aese	v0.16b,v16.16b
	aesmc	v0.16b,v0.16b
	 subs	x2,x2,#16
	aese	v0.16b,v17.16b
	aesmc	v0.16b,v0.16b
	 csel	x8,xzr,x8,eq
	aese	v0.16b,v18.16b
	aesmc	v0.16b,v0.16b
	aese	v0.16b,v19.16b
	aesmc	v0.16b,v0.16b
	 ld1	{v16.16b},[x0],x8
	aese	v0.16b,v20.16b
	aesmc	v0.16b,v0.16b
	 eor	v16.16b,v16.16b,v5.16b
	aese	v0.16b,v21.16b
	aesmc	v0.16b,v0.16b
	 ld1 {v17.4s},[x7]		// re-pre-load rndkey[1]
	aese	v0.16b,v22.16b
	aesmc	v0.16b,v0.16b
	aese	v0.16b,v23.16b
	eor	v6.16b,v0.16b,v7.16b
	b.hs	.Loop_cbc_enc

	st1	{v6.16b},[x1],#16
	b	.Lcbc_done

.align	5
.Lcbc_enc128:
	ld1	{v2.4s-v3.4s},[x7]
	aese	v0.16b,v16.16b
	aesmc	v0.16b,v0.16b
	b	.Lenter_cbc_enc128
.Loop_cbc_enc128:
	aese	v0.16b,v16.16b
	aesmc	v0.16b,v0.16b
	 st1	{v6.16b},[x1],#16
.Lenter_cbc_enc128:
	aese	v0.16b,v17.16b
	aesmc	v0.16b,v0.16b
	 subs	x2,x2,#16
	aese	v0.16b,v2.16b
	aesmc	v0.16b,v0.16b
	 csel	x8,xzr,x8,eq
	aese	v0.16b,v3.16b
	aesmc	v0.16b,v0.16b
	aese	v0.16b,v18.16b
	aesmc	v0.16b,v0.16b
	aese	v0.16b,v19.16b
	aesmc	v0.16b,v0.16b
	 ld1	{v16.16b},[x0],x8
	aese	v0.16b,v20.16b
	aesmc	v0.16b,v0.16b
	aese	v0.16b,v21.16b
	aesmc	v0.16b,v0.16b
	aese	v0.16b,v22.16b
	aesmc	v0.16b,v0.16b
	 eor	v16.16b,v16.16b,v5.16b
	aese	v0.16b,v23.16b
	eor	v6.16b,v0.16b,v7.16b
	b.hs	.Loop_cbc_enc128

	st1	{v6.16b},[x1],#16
	b	.Lcbc_done
.align	5
.Lcbc_dec:
	ld1	{v18.16b},[x0],#16
	subs	x2,x2,#32		// bias
	add	w6,w5,#2
	orr	v3.16b,v0.16b,v0.16b
	orr	v1.16b,v0.16b,v0.16b
	orr	v19.16b,v18.16b,v18.16b
	b.lo	.Lcbc_dec_tail

	orr	v1.16b,v18.16b,v18.16b
	ld1	{v18.16b},[x0],#16
	orr	v2.16b,v0.16b,v0.16b
	orr	v3.16b,v1.16b,v1.16b
	orr	v19.16b,v18.16b,v18.16b

.Loop3x_cbc_dec:
	aesd	v0.16b,v16.16b
	aesimc	v0.16b,v0.16b
	aesd	v1.16b,v16.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v16.16b
	aesimc	v18.16b,v18.16b
	ld1	{v16.4s},[x7],#16
	subs	w6,w6,#2
	aesd	v0.16b,v17.16b
	aesimc	v0.16b,v0.16b
	aesd	v1.16b,v17.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v17.16b
	aesimc	v18.16b,v18.16b
	ld1	{v17.4s},[x7],#16
	b.gt	.Loop3x_cbc_dec

	aesd	v0.16b,v16.16b
	aesimc	v0.16b,v0.16b
	aesd	v1.16b,v16.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v16.16b
	aesimc	v18.16b,v18.16b
	 eor	v4.16b,v6.16b,v7.16b
	 subs	x2,x2,#0x30
	 eor	v5.16b,v2.16b,v7.16b
	 csel	x6,x2,x6,lo			// x6, w6, is zero at this point
	aesd	v0.16b,v17.16b
	aesimc	v0.16b,v0.16b
	aesd	v1.16b,v17.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v17.16b
	aesimc	v18.16b,v18.16b
	 eor	v17.16b,v3.16b,v7.16b
	 add	x0,x0,x6		// x0 is adjusted in such way that
					// at exit from the loop v1.16b-v18.16b
					// are loaded with last "words"
	 orr	v6.16b,v19.16b,v19.16b
	 mov	x7,x3
	aesd	v0.16b,v20.16b
	aesimc	v0.16b,v0.16b
	aesd	v1.16b,v20.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v20.16b
	aesimc	v18.16b,v18.16b
	 ld1	{v2.16b},[x0],#16
	aesd	v0.16b,v21.16b
	aesimc	v0.16b,v0.16b
	aesd	v1.16b,v21.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v21.16b
	aesimc	v18.16b,v18.16b
	 ld1	{v3.16b},[x0],#16
	aesd	v0.16b,v22.16b
	aesimc	v0.16b,v0.16b
	aesd	v1.16b,v22.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v22.16b
	aesimc	v18.16b,v18.16b
	 ld1	{v19.16b},[x0],#16
	aesd	v0.16b,v23.16b
	aesd	v1.16b,v23.16b
	aesd	v18.16b,v23.16b
	 ld1 {v16.4s},[x7],#16	// re-pre-load rndkey[0]
	 add	w6,w5,#2
	eor	v4.16b,v4.16b,v0.16b
	eor	v5.16b,v5.16b,v1.16b
	eor	v18.16b,v18.16b,v17.16b
	 ld1 {v17.4s},[x7],#16	// re-pre-load rndkey[1]
	st1	{v4.16b},[x1],#16
	 orr	v0.16b,v2.16b,v2.16b
	st1	{v5.16b},[x1],#16
	 orr	v1.16b,v3.16b,v3.16b
	st1	{v18.16b},[x1],#16
	 orr	v18.16b,v19.16b,v19.16b
	b.hs	.Loop3x_cbc_dec

	cmn	x2,#0x30
	b.eq	.Lcbc_done
	nop

.Lcbc_dec_tail:
	aesd	v1.16b,v16.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v16.16b
	aesimc	v18.16b,v18.16b
	ld1	{v16.4s},[x7],#16
	subs	w6,w6,#2
	aesd	v1.16b,v17.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v17.16b
	aesimc	v18.16b,v18.16b
	ld1	{v17.4s},[x7],#16
	b.gt	.Lcbc_dec_tail

	aesd	v1.16b,v16.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v16.16b
	aesimc	v18.16b,v18.16b
	aesd	v1.16b,v17.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v17.16b
	aesimc	v18.16b,v18.16b
	aesd	v1.16b,v20.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v20.16b
	aesimc	v18.16b,v18.16b
	 cmn	x2,#0x20
	aesd	v1.16b,v21.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v21.16b
	aesimc	v18.16b,v18.16b
	 eor	v5.16b,v6.16b,v7.16b
	aesd	v1.16b,v22.16b
	aesimc	v1.16b,v1.16b
	aesd	v18.16b,v22.16b
	aesimc	v18.16b,v18.16b
	 eor	v17.16b,v3.16b,v7.16b
	aesd	v1.16b,v23.16b
	aesd	v18.16b,v23.16b
	b.eq	.Lcbc_dec_one
	eor	v5.16b,v5.16b,v1.16b
	eor	v17.16b,v17.16b,v18.16b
	 orr	v6.16b,v19.16b,v19.16b
	st1	{v5.16b},[x1],#16
	st1	{v17.16b},[x1],#16
	b	.Lcbc_done

.Lcbc_dec_one:
	eor	v5.16b,v5.16b,v18.16b
	 orr	v6.16b,v19.16b,v19.16b
	st1	{v5.16b},[x1],#16

.Lcbc_done:
	st1	{v6.16b},[x4]
.Lcbc_abort:
	ldr	x29,[sp],#16

#ifdef UCLIB_NO_VFP_KERNEL
  ldp q22,q23,[sp],#32
  ldp q20,q21,[sp],#32
  ldp q18,q19,[sp],#32
  ldp q16,q17,[sp],#32
  ldp q6,q7,[sp],#32
  ldp q4,q5,[sp],#32
  ldp q2,q3,[sp],#32
  ldp q0,q1,[sp],#32

  msr  DAIF, x9

  dsb sy
  isb

#endif

	ret
.size	aes_v8_cbc_encrypt,.-aes_v8_cbc_encrypt
.globl	aes_v8_ctr32_encrypt_blocks
.type	aes_v8_ctr32_encrypt_blocks,%function
.align	5
aes_v8_ctr32_encrypt_blocks:
	stp		x29,x30,[sp,#-16]!
	add		x29,sp,#0
	ldr		w5,[x3,#240]

	ldr		w8, [x4, #12]
	ld1		{v0.4s},[x4]

	ld1		{v16.4s-v17.4s},[x3]		// load key schedule...
	sub		w5,w5,#4
	mov		x12,#16
	cmp		x2,#2
	add		x7,x3,x5,lsl#4	// pointer to last 5 round keys
	sub		w5,w5,#2
	ld1		{v20.4s-v21.4s},[x7],#32
	ld1		{v22.4s-v23.4s},[x7],#32
	ld1		{v7.4s},[x7]
	add		x7,x3,#32
	mov		w6,w5
	csel	x12,xzr,x12,lo
#ifndef __ARMEB__
	rev		w8, w8
#endif
	orr		v1.16b,v0.16b,v0.16b
	add		w10, w8, #1
	orr		v18.16b,v0.16b,v0.16b
	add		w8, w8, #2
	orr		v6.16b,v0.16b,v0.16b
	rev		w10, w10
	mov		v1.s[3],w10
	b.ls		.Lctr32_tail
	rev		w12, w8
	sub		x2,x2,#3		// bias
	mov		v18.s[3],w12
	b		.Loop3x_ctr32

.align	4
.Loop3x_ctr32:
	aese		v0.16b,v16.16b
	aesmc		v0.16b,v0.16b
	aese		v1.16b,v16.16b
	aesmc		v1.16b,v1.16b
	aese		v18.16b,v16.16b
	aesmc		v18.16b,v18.16b
	ld1		{v16.4s},[x7],#16
	subs		w6,w6,#2
	aese		v0.16b,v17.16b
	aesmc		v0.16b,v0.16b
	aese		v1.16b,v17.16b
	aesmc		v1.16b,v1.16b
	aese		v18.16b,v17.16b
	aesmc		v18.16b,v18.16b
	ld1		{v17.4s},[x7],#16
	b.gt		.Loop3x_ctr32

	aese		v0.16b,v16.16b
	aesmc		v4.16b,v0.16b
	aese		v1.16b,v16.16b
	aesmc		v5.16b,v1.16b
	 ld1		{v2.16b},[x0],#16
	 orr		v0.16b,v6.16b,v6.16b
	aese		v18.16b,v16.16b
	aesmc		v18.16b,v18.16b
	 ld1		{v3.16b},[x0],#16
	 orr		v1.16b,v6.16b,v6.16b
	aese		v4.16b,v17.16b
	aesmc		v4.16b,v4.16b
	aese		v5.16b,v17.16b
	aesmc		v5.16b,v5.16b
	 ld1		{v19.16b},[x0],#16
	 mov		x7,x3
	aese		v18.16b,v17.16b
	aesmc		v17.16b,v18.16b
	 orr		v18.16b,v6.16b,v6.16b
	 add		w9,w8,#1
	aese		v4.16b,v20.16b
	aesmc		v4.16b,v4.16b
	aese		v5.16b,v20.16b
	aesmc		v5.16b,v5.16b
	 eor		v2.16b,v2.16b,v7.16b
	 add		w10,w8,#2
	aese		v17.16b,v20.16b
	aesmc		v17.16b,v17.16b
	 eor		v3.16b,v3.16b,v7.16b
	 add		w8,w8,#3
	aese		v4.16b,v21.16b
	aesmc		v4.16b,v4.16b
	aese		v5.16b,v21.16b
	aesmc		v5.16b,v5.16b
	 eor		v19.16b,v19.16b,v7.16b
	 rev		w9,w9
	aese		v17.16b,v21.16b
	aesmc		v17.16b,v17.16b
	 mov	v0.s[3], w9
	 rev		w10,w10
	aese		v4.16b,v22.16b
	aesmc		v4.16b,v4.16b
	aese		v5.16b,v22.16b
	aesmc		v5.16b,v5.16b
	 mov	v1.s[3], w10
	 rev		w12,w8
	aese		v17.16b,v22.16b
	aesmc		v17.16b,v17.16b
	 mov	v18.s[3], w12
	 subs		x2,x2,#3
	aese		v4.16b,v23.16b
	aese		v5.16b,v23.16b
	aese		v17.16b,v23.16b

	eor		v2.16b,v2.16b,v4.16b
	 ld1	 {v16.4s},[x7],#16	// re-pre-load rndkey[0]
	st1		{v2.16b},[x1],#16
	eor		v3.16b,v3.16b,v5.16b
	 mov		w6,w5
	st1		{v3.16b},[x1],#16
	eor		v19.16b,v19.16b,v17.16b
	 ld1	 {v17.4s},[x7],#16	// re-pre-load rndkey[1]
	st1		{v19.16b},[x1],#16
	b.hs		.Loop3x_ctr32

	adds		x2,x2,#3
	b.eq		.Lctr32_done
	cmp		x2,#1
	mov		x12,#16
	csel	x12,xzr,x12,eq

.Lctr32_tail:
	aese		v0.16b,v16.16b
	aesmc		v0.16b,v0.16b
	aese		v1.16b,v16.16b
	aesmc		v1.16b,v1.16b
	ld1		{v16.4s},[x7],#16
	subs		w6,w6,#2
	aese		v0.16b,v17.16b
	aesmc		v0.16b,v0.16b
	aese		v1.16b,v17.16b
	aesmc		v1.16b,v1.16b
	ld1		{v17.4s},[x7],#16
	b.gt		.Lctr32_tail

	aese		v0.16b,v16.16b
	aesmc		v0.16b,v0.16b
	aese		v1.16b,v16.16b
	aesmc		v1.16b,v1.16b
	aese		v0.16b,v17.16b
	aesmc		v0.16b,v0.16b
	aese		v1.16b,v17.16b
	aesmc		v1.16b,v1.16b
	 ld1		{v2.16b},[x0],x12
	aese		v0.16b,v20.16b
	aesmc		v0.16b,v0.16b
	aese		v1.16b,v20.16b
	aesmc		v1.16b,v1.16b
	 ld1		{v3.16b},[x0]
	aese		v0.16b,v21.16b
	aesmc		v0.16b,v0.16b
	aese		v1.16b,v21.16b
	aesmc		v1.16b,v1.16b
	 eor		v2.16b,v2.16b,v7.16b
	aese		v0.16b,v22.16b
	aesmc		v0.16b,v0.16b
	aese		v1.16b,v22.16b
	aesmc		v1.16b,v1.16b
	 eor		v3.16b,v3.16b,v7.16b
	aese		v0.16b,v23.16b
	aese		v1.16b,v23.16b

	cmp		x2,#1
	eor		v2.16b,v2.16b,v0.16b
	eor		v3.16b,v3.16b,v1.16b
	st1		{v2.16b},[x1],#16
	b.eq		.Lctr32_done
	st1		{v3.16b},[x1]

.Lctr32_done:
	ldr		x29,[sp],#16
	ret
.size	aes_v8_ctr32_encrypt_blocks,.-aes_v8_ctr32_encrypt_blocks
