// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2018 NXP
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <platform_config.h>

#include <arm.h>
#include <console.h>
#include <drivers/gic.h>
#include <drivers/ns16550.h>
#include <io.h>
#include <kernel/generic_boot.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/pm_stubs.h>
#include <kernel/thread.h>
#include <kernel/tz_ssvce_def.h>
#include <mm/core_memprot.h>
#include <sm/optee_smc.h>
#include <tee/entry_fast.h>
#include <tee/entry_std.h>
#include <kernel/tee_common_otp.h>
#include <mm/core_mmu.h>

static void main_fiq(void);

static const struct thread_handlers handlers = {
	.std_smc = tee_entry_std,
	.fast_smc = tee_entry_fast,
	.nintr = main_fiq,
#if defined(CFG_WITH_ARM_TRUSTED_FW)
	.cpu_on = cpu_on_handler,
	.cpu_off = pm_do_nothing,
	.cpu_suspend = pm_do_nothing,
	.cpu_resume = pm_do_nothing,
	.system_off = pm_do_nothing,
	.system_reset = pm_do_nothing,
#else
	.cpu_on = pm_panic,
	.cpu_off = pm_panic,
	.cpu_suspend = pm_panic,
	.cpu_resume = pm_panic,
	.system_off = pm_panic,
	.system_reset = pm_panic,
#endif
};

static struct gic_data gic_data;
static struct ns16550_data console_data;

register_phys_mem(MEM_AREA_IO_NSEC, CONSOLE_UART_BASE, CORE_MMU_DEVICE_SIZE);
register_phys_mem(MEM_AREA_IO_SEC, GIC_BASE, CORE_MMU_DEVICE_SIZE);

const struct thread_handlers *generic_boot_get_handlers(void)
{
	return &handlers;
}

static void main_fiq(void)
{
	panic();
}

#ifdef CFG_ARM32_core
void plat_cpu_reset_late(void)
{
	vaddr_t addr;

	if (!get_core_pos()) {
#if defined(CFG_BOOT_SECONDARY_REQUEST)
		/* set secondary entry address */
		write32(__compiler_bswap32(TEE_LOAD_ADDR),
				DCFG_BASE + DCFG_SCRATCHRW1);

		/* release secondary cores */
		write32(__compiler_bswap32(0x1 << 1), /* cpu1 */
				DCFG_BASE + DCFG_CCSR_BRR);
		dsb();
		sev();
#endif

		/* configure CSU */

		/* first grant all peripherals */
		for (addr = CSU_BASE + CSU_CSL_START;
			 addr != CSU_BASE + CSU_CSL_END;
			 addr += 4)
			write32(__compiler_bswap32(CSU_ACCESS_ALL), addr);

		/* restrict key preipherals from NS */
		write32(__compiler_bswap32(CSU_ACCESS_SEC_ONLY),
			CSU_BASE + CSU_CSL30);
		write32(__compiler_bswap32(CSU_ACCESS_SEC_ONLY),
			CSU_BASE + CSU_CSL37);

		/* lock the settings */
		for (addr = CSU_BASE + CSU_CSL_START;
			 addr != CSU_BASE + CSU_CSL_END;
			 addr += 4)
			write32(read32(addr) |
				__compiler_bswap32(CSU_SETTING_LOCK),
				addr);
	}
}
#endif

void console_init(void)
{
	ns16550_init(&console_data, CONSOLE_UART_BASE);
	register_serial_console(&console_data.chip);
}

void main_init_gic(void)
{
	vaddr_t gicc_base;
	vaddr_t gicd_base;

	gicc_base = (vaddr_t)phys_to_virt(GIC_BASE + GICC_OFFSET,
					  MEM_AREA_IO_SEC);
	gicd_base = (vaddr_t)phys_to_virt(GIC_BASE + GICD_OFFSET,
					  MEM_AREA_IO_SEC);

	if (!gicc_base || !gicd_base)
		panic();

	/* Initialize GIC */
	gic_init(&gic_data, gicc_base, gicd_base);
	itr_init(&gic_data.chip);
}

void main_secondary_init_gic(void)
{
	gic_cpu_init(&gic_data);
}

#ifdef CFG_HW_UNQ_KEY_REQUEST

#include <types_ext.h>
int get_hw_unique_key(uint64_t smc_func_id, uint64_t in_key, uint64_t size);

/*
 * Issued when requesting to Secure Storage Key for secure storage.
 *
 * SiP Service Calls
 *
 * Register usage:
 * r0/x0	SMC Function ID, OPTEE_SMC_FUNCID_SIP_LS_HW_UNQ_KEY
 */
#define OPTEE_SMC_FUNCID_SIP_LS_HW_UNQ_KEY			0xFF14
#define OPTEE_SMC_FAST_CALL_SIP_LS_HW_UNQ_KEY \
	OPTEE_SMC_CALL_VAL(OPTEE_SMC_32, OPTEE_SMC_FAST_CALL, \
			   OPTEE_SMC_OWNER_SIP, \
			   OPTEE_SMC_FUNCID_SIP_LS_HW_UNQ_KEY)

void tee_otp_get_hw_unique_key(struct tee_hw_unique_key *hwkey)
{
	int ret = 0;
	uint8_t hw_unq_key[sizeof(hwkey->data)] __aligned(64);

	ret = get_hw_unique_key(OPTEE_SMC_FAST_CALL_SIP_LS_HW_UNQ_KEY,
			virt_to_phys(hw_unq_key), sizeof(hwkey->data));

	if (ret < 0) {
		EMSG("\nH/W Unique key is not fetched from the platform.");
		ret = TEE_ERROR_SECURITY;
	} else {
		memcpy(&hwkey->data[0], hw_unq_key, sizeof(hwkey->data));
		ret = TEE_SUCCESS;
	}

	return ret;
}
#endif
