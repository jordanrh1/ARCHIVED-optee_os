// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2017 NXP
 *
 * Peng Fan <peng.fan@nxp.com>
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
#include <arm.h>
#include <arm32.h>
#include <console.h>
#include <drivers/imx_uart.h>
#include <io.h>
#include <imx.h>
#include <imx_pm.h>
#include <kernel/panic.h>
#include <kernel/cache_helpers.h>
#include <kernel/generic_boot.h>
#include <kernel/misc.h>
#include <kernel/interrupt.h>
#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <sm/sm.h>
#include <sm/pm.h>
#include <sm/psci.h>
#include <stdint.h>

#define MX7D_GPC_IMR1   0x30
#define MX7D_GPC_IMR2   0x34
#define MX7D_GPC_IMR3   0x38
#define MX7D_GPC_IMR4   0x3c

#define MX7D_GPC_ISR1_A7 0x70
#define MX7D_GPC_ISR2_A7 0x74
#define MX7D_GPC_ISR3_A7 0x78
#define MX7D_GPC_ISR4_A7 0x7C

static int suspended_init;

#define GPT_FREQ 1000000
#define GPT_PRESCALER24M (12 - 1)
#define GPT_PRESCALER (2 - 1)

#define GREEN_LED IMX_GPIO_NR(6, 14)

static struct itr_handler gpt1_itr;

static vaddr_t gpio_base;
static void gpio_toggle_direction(uint32_t gpio)
{
	uint32_t val;

	if (!gpio_base)
		gpio_base = core_mmu_get_va(GPIO_BASE, MEM_AREA_IO_SEC);

	DMSG("gpio_base = 0x%x");
	val = read32(gpio_base + GPIO_BANK_OFFSET(gpio) + GPIO_GDIR);
	val ^= (1 << (gpio % 32));
	write32(val, gpio_base + GPIO_BANK_OFFSET(gpio) + GPIO_GDIR);
}

static void toggle_led(void)
{
	gpio_toggle_direction(GREEN_LED);
}

static volatile int gpt_itr_fired;
static vaddr_t gpt_base;
static enum itr_return gpt_itr_cb(struct itr_handler *h)
{
	gpt_itr_fired = 1;

//	toggle_led();

	/* Disable and acknowledge interrupts */
	write32(0, gpt_base + GPT_IR);
	write32(0x3f, gpt_base + GPT_SR);

	return ITRR_HANDLED;
}

static void gpt_init(void)
{
	uint32_t val;

	gpt_base = core_mmu_get_va(GPT1_BASE, MEM_AREA_IO_SEC);

	/* Disable GPT */
	write32(0, gpt_base + GPT_CR);

	/* Software reset */
	write32(GPT_CR_SWR, gpt_base + GPT_CR);

	/* Wait for reset bit to clear */
	while ((read32(gpt_base + GPT_CR) & GPT_CR_SWR) != 0);

	/* Set prescaler to target frequency */
	val = ((GPT_PRESCALER24M & 0xf) << 12) | (GPT_PRESCALER & 0xfff);
	write32(val, gpt_base + GPT_PR);

	/* Select clock source */
	write32(GPT_CR_CLKSRC_24M, gpt_base + GPT_CR);

	/* Register interrupt handler */
	gpt1_itr.it = 55;
	gpt1_itr.flags = ITRF_TRIGGER_LEVEL;
	gpt1_itr.handler = gpt_itr_cb;
	itr_add(&gpt1_itr);
	itr_enable(gpt1_itr.it);

	val = read32(gpt_base + GPT_CR);
	val |= GPT_CR_EN;
	val |= GPT_CR_ENMOD;
	val |= GPT_CR_STOPEN;
	val |= GPT_CR_WAITEN;
	val |= GPT_CR_DOZEEN;
	val |= GPT_CR_DBGEN;
	val |= GPT_CR_FRR;
	val |= GPT_CR_EN_24M;
	write32(val, gpt_base + GPT_CR);

	DMSG("Initialized GPT");
}

static void gpt_delay(uint32_t ms)
{
	uint32_t initial;
	uint32_t done;

	initial = read32(gpt_base + GPT_CNT);
	done = initial + GPT_FREQ / 1000 * ms;
	if (done < initial) {	// overflow
		while (read32(gpt_base + GPT_CNT) >= initial);
	}
	while (read32(gpt_base + GPT_CNT) < done);
}

static void gpt_schedule_interrupt(uint32_t ms)
{
	uint32_t val;
	bool once = false;

	/* Disable timer */
	val = read32(gpt_base + GPT_CR);
	val &= ~GPT_CR_EN;
	write32(val, gpt_base + GPT_CR);

	/* Disable and acknowledge interrupts */
	write32(0, gpt_base + GPT_IR);
	write32(0x3f, gpt_base + GPT_SR);

	/* Set compare1 register */
	write32(GPT_FREQ / 1000 * ms, gpt_base + GPT_OCR1);

	/* Enable compare interrupt */
	write32(GPT_IR_OF1IE, gpt_base + GPT_IR);

	DMSG("Scheduling an interrupt %dms from now", ms);
	gpt_itr_fired = 0;

	/* Enable timer */
	val |= GPT_CR_EN;
	val |= GPT_CR_ENMOD;
	write32(val, gpt_base + GPT_CR);

	while (!gpt_itr_fired) {
		if (!once &&
                    (read32(gpt_base + GPT_CNT) > (GPT_FREQ / 1000 * ms))) {

			DMSG("Timer value reached target");
			once = true;
		}
	}

	DMSG("GPT interrupt fired!");
}

static void gpc_dump_unmasked_irqs(struct imx7_pm_info *p)
{
	vaddr_t gpc_base = p->gpc_va_base;
	DMSG("~IMR1=0x%x, ~IMR2=0x%x, ~IMR3=0x%x, ~IMR4=0x%x",
	     ~read32(gpc_base + MX7D_GPC_IMR1),
	     ~read32(gpc_base + MX7D_GPC_IMR2),
	     ~read32(gpc_base + MX7D_GPC_IMR3),
	     ~read32(gpc_base + MX7D_GPC_IMR4));

	DMSG("IRS1=0x%x, ISR2=0x%x, ISR3=0x%x, ISR4=0x%x",
	      read32(gpc_base + MX7D_GPC_ISR1_A7),
	      read32(gpc_base + MX7D_GPC_ISR2_A7),
	      read32(gpc_base + MX7D_GPC_ISR3_A7),
	      read32(gpc_base + MX7D_GPC_ISR4_A7));
}

static void gpc_mask_all_irqs(struct imx7_pm_info *p)
{
	vaddr_t gpc_base = p->gpc_va_base;
	DMSG("Masking all IRQs in GPC");
	write32(~0x0, gpc_base + MX7D_GPC_IMR1);
	write32(~0x0, gpc_base + MX7D_GPC_IMR2);
	write32(~0x0, gpc_base + MX7D_GPC_IMR3);
	write32(~0x0, gpc_base + MX7D_GPC_IMR4);
}

/* Called by pm_pm_cpu_suspend to do platform-specific suspend */
static int imx7_do_core_power_down(uint32_t arg)
{
	uint32_t val;
	int core_idx;
	struct imx7_pm_info *p = (struct imx7_pm_info *)arg;

	core_idx = get_core_pos();

	// XXX do we need to flush cache? Maybe. Looks like 
	// sm_pm_cpu_suspend_save flushes L1 cache before calling us.
	// Looks like core-local state gets saved to stack

	/* Program ACK selection for LPM */
	write32(GPC_PGC_ACK_SEL_A7_DUMMY_PUP_ACK |
                GPC_PGC_ACK_SEL_A7_DUMMY_PDN_ACK,
                p->gpc_va_base + GPC_PGC_ACK_SEL_A7);

	/* setup resume address and parameter */
	val = TRUSTZONE_OCRAM_START + SUSPEND_OCRAM_OFFSET + sizeof(*p);
	write32(val, p->src_va_base + SRC_GPR1_MX7 + core_idx * 8);
	write32(p->pa_base, p->src_va_base + SRC_GPR2_MX7 + core_idx * 8);
	
	/* Program LPCR_A7_BSC */
	// XXX need spinlock around common register
	// XXX at what point do we need to mask exceptions?
	val = read32(p->gpc_va_base + GPC_LPCR_A7_BSC);
	val &= ~GPC_LPCR_A7_BSC_LPM0;	// Set LPM0 to RUN mode
	val &= ~GPC_LPCR_A7_BSC_LPM1;   // Set LPM1 to RUN mode
	val |= GPC_LPCR_A7_BSC_CPU_CLK_ON_LPM;	// A7 clock ON in wait/stop
	val &= ~GPC_LPCR_A7_BSC_MASK_CORE0_WFI;
	val &= ~GPC_LPCR_A7_BSC_MASK_CORE1_WFI;
	val &= ~GPC_LPCR_A7_BSC_MASK_L2CC_WFI;
	val |= GPC_LPCR_A7_BSC_IRQ_SRC_C0;   // Core wakeup via GIC
	val |= GPC_LPCR_A7_BSC_IRQ_SRC_C1;
	val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_A7_WUP;
	val &= ~GPC_LPCR_A7_BSC_MASK_DSM_TRIGGER; // XXX not sure
	DMSG("GPC_LPCR_A7_BSC = 0x%x", val);
	write32(val, p->gpc_va_base + GPC_LPCR_A7_BSC);

	/* Program A7 advanced power control register */
	val = read32(p->gpc_va_base + GPC_LPCR_A7_AD);
	val &= ~GPC_LPCR_A7_AD_L2_PGE;	// do not power down L2
	val &= ~GPC_LPCR_A7_AD_EN_C1_PUP;  // XXX do not use LPM request
	val &= ~GPC_LPCR_A7_AD_EN_C1_IRQ_PUP; // XXX
	val &= ~GPC_LPCR_A7_AD_EN_C0_PUP;  // XXX
	val |= GPC_LPCR_A7_AD_EN_C0_IRQ_PUP; // XXX Wakeup by IRQ
	val &= ~GPC_LPCR_A7_AD_EN_PLAT_PDN;  // don't power down SCU and L2
	val &= ~GPC_LPCR_A7_AD_EN_C1_PDN;  // ignore LPM request
	val &= ~GPC_LPCR_A7_AD_EN_C0_PDN;  // ignore LPM request

	/* power down current core when core issues wfi */
	if (core_idx == 0)
		val |= GPC_LPCR_A7_AD_EN_C0_WFI_PDN;
	else
		val |= GPC_LPCR_A7_AD_EN_C1_WFI_PDN;

	DMSG("GPC_LPCR_A7_AD = 0x%x", val);
	write32(val, p->gpc_va_base + GPC_LPCR_A7_AD);

	gpc_mask_all_irqs(p);

	DMSG("Arming PGC and executing WFI");
	gpt_schedule_interrupt(5000);

	/* arm PGC for power down */
	if (core_idx == 0)
		val = GPC_PGC_C0;
	else
		val = GPC_PGC_C1;

	imx_gpcv2_set_core_pgc(true, val);

	dsb();
	wfi();

	/* return value ignored by sm_pm_cpu_suspend */
	return 0;
}

int imx7_core_power_down(uintptr_t entry,
		        uint32_t context_id, struct sm_nsec_ctx *nsec)
{
	uint32_t suspend_ocram_base = core_mmu_get_va(TRUSTZONE_OCRAM_START +
						      SUSPEND_OCRAM_OFFSET,
						      MEM_AREA_TEE_COHERENT);
	struct imx7_pm_info *p = (struct imx7_pm_info *)suspend_ocram_base;
	int ret;

	/* save banked registers for every mode except monitor mode */
	sm_save_modes_regs(&nsec->mode_regs);
	
	ret = sm_pm_cpu_suspend((uint32_t)p, imx7_do_core_power_down);

	/*
	 * Sometimes sm_pm_cpu_suspend may not really suspended,
	 * we need to check it's return value to restore reg or not
	 */
	if (ret < 0) {
		DMSG("=== Core did not power down ===\n");
		return 0;
	}

	DMSG("Resume from suspend");

	// XXX Need to disarm PGC, clear bits in A7_AD so that next
	// WFI does not power down the core

	/* Restore register of different mode in secure world */
	sm_restore_modes_regs(&nsec->mode_regs);

	/* Set entry for back to normal world */
	nsec->mon_lr = (uint32_t)entry;
	nsec->r0 = context_id;

	return PSCI_RET_SUCCESS;
}

static int imx7_core_do_wfi(void)
{
	for (;;) {
		DMSG("tick");
		toggle_led();
		gpt_delay(500);
		DMSG("tock");
		toggle_led();
		gpt_delay(500);
	}

	gpt_schedule_interrupt(1000);

	dsb();
	wfi();

	DMSG("Woke up from WFI");

	return PSCI_RET_SUCCESS;
}

int imx7_cpu_suspend(uint32_t power_state, uintptr_t entry,
		     uint32_t context_id, struct sm_nsec_ctx *nsec)
{
#if 0
	uint32_t suspend_ocram_base = core_mmu_get_va(TRUSTZONE_OCRAM_START +
						      SUSPEND_OCRAM_OFFSET,
						      MEM_AREA_TEE_COHERENT);
	struct imx7_pm_info *p = (struct imx7_pm_info *)suspend_ocram_base;
	int ret;
#endif

	if (!suspended_init) {
		toggle_led();
		gpt_init();
		imx7_suspend_init();
		suspended_init = 1;
	}

	switch (power_state) {
	case MX7_STATE_CORE_WFI:
		return imx7_core_do_wfi();
	case MX7_STATE_CORE_POWER_DOWN:
		return imx7_core_power_down(entry, context_id, nsec);
	default:
		return PSCI_RET_INVALID_PARAMETERS;
	}

#if 0
	/* Store non-sec ctx regs */
	sm_save_modes_regs(&nsec->mode_regs);

	// XXX dump the gpc state
	gpc_dump_unmasked_irqs(p);
	gpc_mask_all_irqs(p);

	DMSG("Calling sm_pm_cpu_suspend. suspend_ocram_base=0x%x",
	     (uint32_t)suspend_ocram_base);

	ret = sm_pm_cpu_suspend((uint32_t)p, (int (*)(uint32_t))
				(suspend_ocram_base + sizeof(*p)));
	/*
	 * Sometimes sm_pm_cpu_suspend may not really suspended,
	 * we need to check it's return value to restore reg or not
	 */
	if (ret < 0) {
		DMSG("=== Not suspended, GPC IRQ Pending ===\n");
		return 0;
	}

	plat_cpu_reset_late();

	/* Restore register of different mode in secure world */
	sm_restore_modes_regs(&nsec->mode_regs);

	/* Set entry for back to Linux */
	nsec->mon_lr = (uint32_t)entry;
	nsec->r0 = context_id;

	main_init_gic();

	DMSG("=== Back from Suspended ===\n");

	return 0;
#endif
}
