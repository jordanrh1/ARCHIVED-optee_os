// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * Peng Fan <peng.fan@nxp.com>
 */

#include <arm32.h>
#include <io.h>
#include <kernel/generic_boot.h>
#include <kernel/tz_ssvce_def.h>
#include <kernel/tz_ssvce_pl310.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <platform_config.h>
#include <stdint.h>
#include <sm/psci.h>
#include "imx_pl310.h"

#define PL310_DEBUG_CTRL_DISABLE_WRITEBACK      (1u << 1)
#define PL310_DEBUG_CTRL_DISABLE_LINEFILL       (1u << 0)

register_phys_mem(MEM_AREA_IO_SEC, PL310_BASE, CORE_MMU_DEVICE_SIZE);

void arm_cl2_config(vaddr_t pl310_base)
{
	/* Disable PL310 */
	write32(0, pl310_base + PL310_CTRL);

	write32(PL310_TAG_RAM_CTRL_INIT, pl310_base + PL310_TAG_RAM_CTRL);
	write32(PL310_DATA_RAM_CTRL_INIT, pl310_base + PL310_DATA_RAM_CTRL);
	write32(PL310_AUX_CTRL_INIT, pl310_base + PL310_AUX_CTRL);
	write32(PL310_PREFETCH_CTRL_INIT, pl310_base + PL310_PREFETCH_CTRL);
	write32(PL310_POWER_CTRL_INIT, pl310_base + PL310_POWER_CTRL);

	/* invalidate all cache ways */
	arm_cl2_invbyway(pl310_base);
}

void arm_cl2_enable(vaddr_t pl310_base)
{
	uint32_t val __maybe_unused;

	/* Enable PL310 ctrl -> only set lsb bit */
	write32(1, pl310_base + PL310_CTRL);

#ifndef CFG_SKIP_L2_INIT
	/* if L2 FLZW enable, enable in L1 */
	val = read32(pl310_base + PL310_AUX_CTRL);
	if (val & 1)
		write_actlr(read_actlr() | (1 << 3));
#endif
}

vaddr_t pl310_base(void)
{
	return core_mmu_get_va(PL310_BASE, MEM_AREA_IO_SEC);
}

int l2cache_op(int operation)
{
	switch (operation) {
	case L2CACHE_OP_ENABLE:
		return l2cache_enable();
	case L2CACHE_OP_DISABLE:
		return l2cache_disable();
	case L2CACHE_OP_ENABLE_WRITEBACK:
		return l2cache_enable_writeback();
	case L2CACHE_OP_DISABLE_WRITEBACK:
		return l2cache_disable_writeback();
	case L2CACHE_OP_ENABLE_WFLZ:
		return l2cache_enable_wflz();
	default:
		EMSG("Invalid L2 Cache Operation");
		return PSCI_RET_INVALID_PARAMETERS;
	}
}

int l2cache_enable(void)
{
	vaddr_t base = pl310_base();
	arm_cl2_config(base);
	arm_cl2_enable(base);
	return PSCI_RET_SUCCESS;
}

int l2cache_disable(void)
{
	return PSCI_RET_NOT_SUPPORTED;
}

int l2cache_enable_writeback(void)
{
	vaddr_t base = pl310_base();
	write32(0, base + PL310_DEBUG_CTRL);
	return PSCI_RET_SUCCESS;
}

int l2cache_disable_writeback(void)
{
	vaddr_t base = pl310_base();
	uint32_t val = PL310_DEBUG_CTRL_DISABLE_WRITEBACK |
	               PL310_DEBUG_CTRL_DISABLE_LINEFILL;

	write32(val, base + PL310_DEBUG_CTRL);
	return PSCI_RET_SUCCESS;
}

int l2cache_enable_wflz(void)
{
	write_actlr(read_actlr() | (1 << 3));
	return PSCI_RET_SUCCESS;
}

