/*
* Copyright (C) 2012, Texas Instruments, Inc.
* Texas Instruments, <www.ti.com>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
* SUCH DAMAGE.
*/

.macro DO_SMC id
	dsb
	isb
	dmb
	smc \id
.endm

/**
 * call_trusted() - Function to make a Secure Monitor call
 *
 * Prototype:
 * void call_trusted(void);
 *
 */
.global call_trusted

call_trusted:
	stmfd sp!, {r4-r12,lr}
	mov r6, #0xff
	mov r12, #0
	DO_SMC 1
	ldmfd sp!, {r4-r12,lr}
	bx lr

/**
 * start_hyp_mode() - Function will call the "Start Hyper-Visor"
 *			ROM code API to switch ARM into 'Hyp' mode
 * Prototype:
 * void start_hyp_mode(u32 hyp_svc_id);
 * @hyp_svc_id: The monitor API service ID for starting Hyp mode
 *
 */
.global start_hyp_mode
start_hyp_mode:
	stmfd sp!, {r4-r12,lr}
	mov r12, r0
	/* DONOT trust SP will be safe either, so save it */
	adr r0, save_sp
	str sp, [r0]
	adr r0, restore_from_hyp
	DO_SMC 0
restore_from_hyp:
	/* Assume everything is messed up, rebuild ARM registers back up */
	adr r0, save_sp
	ldr sp, [r0]
	ldmfd sp!, {r4-r12,lr}
	bx lr

save_sp:       /* Storage for stack pointer */
	.word   0x0
