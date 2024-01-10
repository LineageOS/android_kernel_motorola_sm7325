// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Motorola Mobility, Inc.
 *
 * Author: Maxwell Bland
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Implements the Motorola Kernel Integrity Protection Hypervisor API.
 */
#include <linux/memory.h>
#include <linux/qcom_scm.h>
#include <drivers/firmware/qcom_scm.h>
#include <soc/qcom/qseecom_scm.h>
#include <soc/qcom/qseecomi.h>
#include <linux/arm-smccc.h>

#include "rkp_hvc_api.h"

/**
 * mrkp_smc - raw arm SMC call for mrkp
 * @id: SMC call ID
 * @arg0: argument in x2 register
 * @arg1: argument in x3 register
 *
 * Unfortunately using the QCOM API leads to a very infrequent
 * crash as it attempts to acquire a mutex while we may be
 * performing a cross-CPU spinlock. arg0 and arg1 are at the
 * intentionally wrong places here as the QCOM API misplaced
 * them into x2/x3 rather than x1, and for historical reasons
 * we adapted.
 */
static int mrkp_smc(uint64_t id, uint64_t arg0, uint64_t arg1) {
	struct arm_smccc_res res;
	struct arm_smccc_quirk quirk = { .id = ARM_SMCCC_QUIRK_QCOM_A6 };
	quirk.state.a6 = 0;
	do {
		arm_smccc_smc_quirk(MOTO_RKP_SMCID | id, 0, arg0, arg1, 0, 0, quirk.state.a6, 0, &res, &quirk);
	} while (res.a0);
	return 0;
}

int mark_range_ro_smc(uint64_t start, uint64_t end)
{
	return mrkp_smc(MARK_RANGE_RO, __virt_to_phys(start), __virt_to_phys(end));
}

int add_jump_entry_lookup(uint64_t vaddr, uint64_t size)
{
	return mrkp_smc(ADD_JUMP_LABEL_LOOKUP, vaddr, size);
}

int lock_rkp(void)
{
	return mrkp_smc(LOCK_RKP, 0, 0);
}

int prepare_hugepage(uint64_t addr)
{
	return mrkp_smc(PREPARE_BLOCK, __virt_to_phys(addr), 0);
}

int commit_hugepage(void)
{
	return mrkp_smc(COMMIT_BLOCK, 0, 0);
}
