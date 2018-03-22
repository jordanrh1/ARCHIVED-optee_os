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
#include <drivers/gic.h>
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

#define GPT_FREQ 1000000
#define GPT_PRESCALER24M (12 - 1)
#define GPT_PRESCALER (2 - 1)

#define GIT_IRQ 27
#define GPT1_IRQ (55 + 32)
#define GPT4_IRQ (52 + 32)
#define USDHC1_IRQ (22 + 32)

#define GREEN_LED IMX_GPIO_NR(6, 14)

struct git_timer_state {
	uint32_t cntfrq;
};

extern uint32_t resume;
static int suspended_init;

static struct itr_handler gpt1_itr;

static void git_timer_save_state(struct git_timer_state *state)
{
	state->cntfrq = read_cntfrq();
}

static void git_timer_restore_state(struct git_timer_state *state)
{
	write_cntfrq(state->cntfrq);
}

static vaddr_t gpio_base;
static void gpio_toggle_direction(uint32_t gpio)
{
	uint32_t val;

	if (!gpio_base)
		gpio_base = core_mmu_get_va(GPIO_BASE, MEM_AREA_IO_SEC);

	val = read32(gpio_base + GPIO_BANK_OFFSET(gpio) + GPIO_GDIR);
	val ^= (1 << (gpio % 32));
	write32(val, gpio_base + GPIO_BANK_OFFSET(gpio) + GPIO_GDIR);
}

static void toggle_led(void)
{
	gpio_toggle_direction(GREEN_LED);
}

static vaddr_t gpt_base;
static enum itr_return gpt_itr_cb(struct itr_handler *h)
{
	vaddr_t base;

	base = (vaddr_t)phys_to_virt_io(GPT1_BASE);

	DMSG("interrupt fired!");
//	toggle_led();

	/* Disable and acknowledge interrupts */
	write32(0, base + GPT_IR);
	write32(0x3f, base + GPT_SR);

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
	gpt1_itr.it = GPT1_IRQ;
	gpt1_itr.flags = ITRF_TRIGGER_LEVEL;
	gpt1_itr.handler = gpt_itr_cb;
	itr_add(&gpt1_itr);
	//itr_enable(gpt1_itr.it);	// Rely on GPT to wake back up
	//itr_set_affinity(gpt1_itr.it, 0x1);

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

	/* Enable timer */
	val |= GPT_CR_EN;
	val |= GPT_CR_ENMOD;
	write32(val, gpt_base + GPT_CR);
}

void gpc_dump_unmasked_irqs(struct imx7_pm_info *p)
{
	vaddr_t gpc_base = p->gpc_va_base;
	DMSG("~IMR1=0x%x, ~IMR2=0x%x, ~IMR3=0x%x, ~IMR4=0x%x",
	     ~read32(gpc_base + GPC_IMR1_CORE0_A7),
	     ~read32(gpc_base + GPC_IMR2_CORE0_A7),
	     ~read32(gpc_base + GPC_IMR3_CORE0_A7),
	     ~read32(gpc_base + GPC_IMR4_CORE0_A7));

	DMSG("IRS1=0x%x, ISR2=0x%x, ISR3=0x%x, ISR4=0x%x",
	      read32(gpc_base + GPC_ISR1_A7),
	      read32(gpc_base + GPC_ISR2_A7),
	      read32(gpc_base + GPC_ISR3_A7),
	      read32(gpc_base + GPC_ISR4_A7));
}

void gpc_mask_all_irqs(struct imx7_pm_info *p)
{
	vaddr_t gpc_base = p->gpc_va_base;
	DMSG("Masking all IRQs in GPC");
	write32(~0x0, gpc_base + GPC_IMR1_CORE0_A7);
	write32(~0x0, gpc_base + GPC_IMR2_CORE0_A7);
	write32(~0x0, gpc_base + GPC_IMR3_CORE0_A7);
	write32(~0x0, gpc_base + GPC_IMR4_CORE0_A7);

	write32(~0x0, gpc_base + GPC_IMR1_CORE1_A7);
	write32(~0x0, gpc_base + GPC_IMR2_CORE1_A7);
	write32(~0x0, gpc_base + GPC_IMR3_CORE1_A7);
	write32(~0x0, gpc_base + GPC_IMR4_CORE1_A7);
}

void gpc_unmask_irq(struct imx7_pm_info *p, uint32_t irq)
{
	vaddr_t gpc_base = p->gpc_va_base;
	uint32_t val;
	val = read32(gpc_base + GPC_IMR1_CORE0_A7 + ((irq - 32) / 32) * 4);
	val &= ~(1 << (irq % 32));
	write32(val, gpc_base + GPC_IMR1_CORE0_A7 + ((irq - 32) / 32) * 4);
}

static void dump_phys_mem(uint32_t paddr)
{
	int i;
	vaddr_t vbase;
	enum teecore_memtypes mtype = MEM_AREA_RAM_NSEC;

	vbase = (vaddr_t)phys_to_virt(paddr, mtype);

	if (!vbase) {
		if (!core_mmu_add_mapping(mtype, paddr, CORE_MMU_DEVICE_SIZE)){
			EMSG("Failed to map 0x%x bytes at PA 0x%x",
			      CORE_MMU_DEVICE_SIZE,
			      paddr);
			return;
		}
		vbase = (vaddr_t)phys_to_virt(paddr, mtype);
	}

	DMSG("Dumping memory at paddr(0x%08x) vbase(0x%08x)", 
	     (uint32_t)paddr, (uint32_t)vbase);

	for (i = 0; i < 16; ++i) {
		DMSG("0x%08x: 0x%08x",
                     paddr + (i * 4),
		     *(uint32_t *)(vbase + (i * 4)));
	}
}

/* Called by pm_pm_cpu_suspend to do platform-specific suspend */
static int imx7_do_core_power_down(uint32_t arg)
{
	uint32_t val;
	int core_idx;
	struct imx7_pm_info *p = (struct imx7_pm_info *)arg;
	bool schedule_wakeup = false;
	bool wakeup_gic = true;
	bool force_sleep = false;

	core_idx = get_core_pos();

	// XXX do we need to flush cache? Maybe. Looks like 
	// sm_pm_cpu_suspend_save flushes L1 cache before calling us.
	// Looks like core-local state gets saved to stack

	/* Program ACK selection for LPM */
	write32(GPC_PGC_ACK_SEL_A7_DUMMY_PUP_ACK |
                GPC_PGC_ACK_SEL_A7_DUMMY_PDN_ACK,
                p->gpc_va_base + GPC_PGC_ACK_SEL_A7);

	/* setup resume address in OCRAM and parameter */
	val = ((uint32_t)&resume - (uint32_t)&imx7_suspend) + 
		p->pa_base + p->pm_info_size;

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

	if (wakeup_gic) {
		val |= GPC_LPCR_A7_BSC_IRQ_SRC_C0;   // Core wakeup via GIC
		val |= GPC_LPCR_A7_BSC_IRQ_SRC_C1;
		val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_A7_WUP;
	} else {
		// wakeup via IRQ
		// XXX need to program GPC
		val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_C0;
		val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_C1;
		val |= GPC_LPCR_A7_BSC_IRQ_SRC_A7_WUP;
	}

	val &= ~GPC_LPCR_A7_BSC_MASK_DSM_TRIGGER; // XXX not sure
	//DMSG("GPC_LPCR_A7_BSC = 0x%x", val);
	write32(val, p->gpc_va_base + GPC_LPCR_A7_BSC);

	/* Program A7 advanced power control register */
	val = read32(p->gpc_va_base + GPC_LPCR_A7_AD);
	val &= ~GPC_LPCR_A7_AD_L2_PGE;	// do not power down L2
	val &= ~GPC_LPCR_A7_AD_EN_PLAT_PDN;  // don't power down SCU and L2

	/* power down current core when core issues wfi */
	if (core_idx == 0) {
		val &= ~GPC_LPCR_A7_AD_EN_C0_PDN;  // ignore LPM request
		val &= ~GPC_LPCR_A7_AD_EN_C0_PUP;
		val |= GPC_LPCR_A7_AD_EN_C0_WFI_PDN;
		val |= GPC_LPCR_A7_AD_EN_C0_IRQ_PUP;
	} else {
		val &= ~GPC_LPCR_A7_AD_EN_C1_PDN;  // ignore LPM request
		val &= ~GPC_LPCR_A7_AD_EN_C1_PUP;
		val |= GPC_LPCR_A7_AD_EN_C1_WFI_PDN;
		val |= GPC_LPCR_A7_AD_EN_C1_IRQ_PUP;
	}

	//DMSG("GPC_LPCR_A7_AD = 0x%x", val);
	write32(val, p->gpc_va_base + GPC_LPCR_A7_AD);

	DMSG("Arming PGC and executing WFI");

	// Test whether GPC is really being used to wake up core
	if (schedule_wakeup) {
		if (!wakeup_gic) {
			itr_disable(GIT_IRQ); // mask GIT timer interrupt
			itr_disable(USDHC1_IRQ);
			itr_disable(GPT4_IRQ);
			gpc_unmask_irq(p, GPT1_IRQ);
		}
		gpt_schedule_interrupt(1000);
	}

	/* arm PGC for power down */
	if (core_idx == 0)
		val = GPC_PGC_C0;
	else
		val = GPC_PGC_C1;

	imx_gpcv2_set_core_pgc(true, val);

	val = 0;
	while (1) {
		dsb();
		wfi();
		if (!force_sleep) break;
		if (val == 0) {
			gic_dump_state(&gic_data);
			val = 1;
		}
	}

	/* return value ignored by sm_pm_cpu_suspend */
	return 0;
}

int imx7_core_power_down(uintptr_t entry,
		        uint32_t context_id, struct sm_nsec_ctx *nsec)
{
	uint32_t val;
	uint32_t core_idx;
	uint32_t suspend_ocram_base = core_mmu_get_va(TRUSTZONE_OCRAM_START +
						      SUSPEND_OCRAM_OFFSET,
						      MEM_AREA_TEE_COHERENT);
	struct imx7_pm_info *p = (struct imx7_pm_info *)suspend_ocram_base;
	int ret;
	struct git_timer_state git_state;

	DMSG("Before suspend. cntfrq = %d", read_cntfrq());

	git_timer_save_state(&git_state);

	/* save banked registers for every mode except monitor mode */
	sm_save_modes_regs(&nsec->mode_regs);
	
	ret = sm_pm_cpu_suspend((uint32_t)p, imx7_do_core_power_down);

	core_idx = get_core_pos();

	/*
	 * Whether we did or did not suspend, we need to unarm hardware
	 *  - reset BSC and AD registers
	 *  - unarm PGC
	 */
	if (core_idx == 0)
		val = GPC_PGC_C0;
	else
		val = GPC_PGC_C1;

	imx_gpcv2_set_core_pgc(false, val);

	/* Disable powerdown on WFI */
	val = read32(p->gpc_va_base + GPC_LPCR_A7_AD);
	if (core_idx == 0)
		val &= ~GPC_LPCR_A7_AD_EN_C0_WFI_PDN;
	else
		val &= ~GPC_LPCR_A7_AD_EN_C1_WFI_PDN;

	write32(val, p->gpc_va_base + GPC_LPCR_A7_AD);

	/*
	 * Sometimes sm_pm_cpu_suspend may not really suspended,
	 * we need to check it's return value to restore reg or not
	 */
	if (ret < 0) {
		DMSG("=== Core did not power down ===");
		return 0;
	}

	DMSG("Resume from suspend");

	git_timer_restore_state(&git_state);

	/* Restore register of different mode in secure world */
	sm_restore_modes_regs(&nsec->mode_regs);

	/* Set entry for back to normal world */
	DMSG("Setting entry=0x%x, context_id=0x%x",
	     (uint32_t)entry, context_id);

	nsec->mon_lr = (uint32_t)entry;
	nsec->mon_spsr = CPSR_MODE_SVC | CPSR_I | CPSR_F;

	//dump_phys_mem(entry);
	//gic_dump_state(&gic_data);

	return context_id;
}

//
// Do actual power down of A7 platform (ALL_OFF mode)
//
static int imx7_do_all_off(uint32_t arg)
{
	uint32_t val;
	int core_idx;
	struct imx7_pm_info *p = (struct imx7_pm_info *)arg;
	bool schedule_wakeup = true;
	bool force_sleep = true;

	core_idx = get_core_pos();

	// XXX do we need to flush cache? Maybe. Looks like 
	// sm_pm_cpu_suspend_save flushes L1 cache before calling us.
	// Looks like core-local state gets saved to stack
	// XXX we need to flush L2 cache

	/* Program ACK selection for LPM */
	write32(GPC_PGC_ACK_SEL_A7_PLAT_PGC_PUP_ACK |
                GPC_PGC_ACK_SEL_A7_PLAT_PGC_PDN_ACK,
                p->gpc_va_base + GPC_PGC_ACK_SEL_A7);

	/* setup resume address in OCRAM and parameter */
	val = ((uint32_t)&resume - (uint32_t)&imx7_suspend) + 
		p->pa_base + p->pm_info_size;

	write32(val, p->src_va_base + SRC_GPR1_MX7 + core_idx * 8);
	write32(p->pa_base, p->src_va_base + SRC_GPR2_MX7 + core_idx * 8);
	
	/* Program LPCR_A7_BSC */
	// XXX need spinlock around common register
	// XXX at what point do we need to mask exceptions?
	val = read32(p->gpc_va_base + GPC_LPCR_A7_BSC);
	val &= ~GPC_LPCR_A7_BSC_LPM0;	// Set LPM0 to WAIT mode
	val |= 2; // XXX stop mode
	val &= ~GPC_LPCR_A7_BSC_LPM1;   // Set LPM1 to WAIT mode
	val |= (2 << 2); // XXX stop mode
	val &= ~GPC_LPCR_A7_BSC_CPU_CLK_ON_LPM;	// A7 clock OFF in wait/stop
	val &= ~GPC_LPCR_A7_BSC_MASK_CORE0_WFI;
	val &= ~GPC_LPCR_A7_BSC_MASK_CORE1_WFI;
	val &= ~GPC_LPCR_A7_BSC_MASK_L2CC_WFI;

	// wakeup via IRQ
	// XXX need to program GPC
	val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_C0;
	val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_C1;
	val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_A7_WUP;

	val &= ~GPC_LPCR_A7_BSC_MASK_DSM_TRIGGER; // XXX not sure
	//DMSG("GPC_LPCR_A7_BSC = 0x%x", val);
	write32(val, p->gpc_va_base + GPC_LPCR_A7_BSC);

	/* Program A7 advanced power control register */
	val = read32(p->gpc_va_base + GPC_LPCR_A7_AD);
	val |= GPC_LPCR_A7_AD_L2_PGE;	// power down L2. XXX L2 flush required
	val |= GPC_LPCR_A7_AD_EN_PLAT_PDN;  // power down SCU and L2

	/* power down all cores on LPM request */
	val |= GPC_LPCR_A7_AD_EN_C0_PDN; // Core0 will power down with LPM
	val &= ~GPC_LPCR_A7_AD_EN_C0_PUP; // Not sure what this does
	val &= ~GPC_LPCR_A7_AD_EN_C0_WFI_PDN; // ignore core WFI
	val |= GPC_LPCR_A7_AD_EN_C0_IRQ_PUP; // Power up with IRQ request

	val |= GPC_LPCR_A7_AD_EN_C1_PDN;  // Core1 will power down with LPM
	val &= ~GPC_LPCR_A7_AD_EN_C1_PUP; // not sure abou this
	val &= ~GPC_LPCR_A7_AD_EN_C1_WFI_PDN; // ignore core WFI
	val &= ~GPC_LPCR_A7_AD_EN_C1_IRQ_PUP; // do not power up with IRQ

	//DMSG("GPC_LPCR_A7_AD = 0x%x", val);
	write32(val, p->gpc_va_base + GPC_LPCR_A7_AD);

	/* shut off the oscillator in DSM */
	val = read32(p->gpc_va_base + GPC_SLPCR);
	val |= GPC_SLPCR_EN_DSM;
	val |= GPC_SLPCR_SBYOS;	// power down on-chip oscillator on DSM
	write32(val, p->gpc_va_base + GPC_SLPCR);

	DMSG("Arming PGC and executing WFI");

	// Test whether GPC is really being used to wake up core
	if (schedule_wakeup) {
		itr_disable(GIT_IRQ); // mask GIT timer interrupt
		itr_disable(USDHC1_IRQ);
		itr_disable(GPT4_IRQ);
		
		gpc_unmask_irq(p, GPT1_IRQ);
		gpt_schedule_interrupt(5000);
	}

	/* arm PGC for power down */
	if (core_idx == 0)
		val = GPC_PGC_C0;
	else
		val = GPC_PGC_C1;

	imx_gpcv2_set_core_pgc(true, val);
	imx_gpcv2_set_core_pgc(true, GPC_PGC_SCU);

	// XXX need to use sequencer to program cores to go down first,
	// then SCU, and reverse for wakeup

	val = 0;
	while (1) {
		dsb();
		wfi();
		if (!force_sleep) break;
		if (val == 0) {
			gic_dump_state(&gic_data);
			val = 1;
		}
	}

	/* return value ignored by sm_pm_cpu_suspend */
	return 0;
}
//
// Power down and clock-gate the entire A7 platform
//
int imx7_all_off(uintptr_t entry,
		        uint32_t context_id, struct sm_nsec_ctx *nsec)
{
	uint32_t val;
	uint32_t core_idx;
	uint32_t suspend_ocram_base = core_mmu_get_va(TRUSTZONE_OCRAM_START +
						      SUSPEND_OCRAM_OFFSET,
						      MEM_AREA_TEE_COHERENT);
	struct imx7_pm_info *p = (struct imx7_pm_info *)suspend_ocram_base;
	int ret;
	struct git_timer_state git_state;

	git_timer_save_state(&git_state);

	// XXX need to save GIT state

	/* save banked registers for every mode except monitor mode */
	sm_save_modes_regs(&nsec->mode_regs);
	
	ret = sm_pm_cpu_suspend((uint32_t)p, imx7_do_core_power_down);

	core_idx = get_core_pos();

	/*
	 * Whether we did or did not suspend, we need to unarm hardware
	 *  - reset BSC and AD registers
	 *  - unarm PGC
	 */
	if (core_idx == 0)
		val = GPC_PGC_C0;
	else
		val = GPC_PGC_C1;

	imx_gpcv2_set_core_pgc(false, val);
	imx_gpcv2_set_core_pgc(false, GPC_PGC_SCU);
	// XXX unarm PGC for A7 SCU (0x880)
	

	// XXX disable LPM

	/*
	 * Sometimes sm_pm_cpu_suspend may not really suspended,
	 * we need to check it's return value to restore reg or not
	 */
	if (ret < 0) {
		DMSG("=== Core did not power down ===");
		return 0;
	}

	DMSG("Resume from suspend");

	git_timer_restore_state(&git_state);

	// XXX restore GIC state

	/* Restore register of different mode in secure world */
	sm_restore_modes_regs(&nsec->mode_regs);

	nsec->mon_lr = (uint32_t)entry;
	nsec->mon_spsr = CPSR_MODE_SVC | CPSR_I | CPSR_F;

	//dump_phys_mem(entry);
	//gic_dump_state(&gic_data);

	return context_id;
}


static int imx7_core_do_wfi(void)
{
	gpt_schedule_interrupt(5000);

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
		gpt_init();
		imx7_suspend_init();
		suspended_init = 1;
	}

	switch (power_state) {
	case MX7_STATE_CORE_WFI:
		return imx7_core_do_wfi();
	case MX7_STATE_CORE_POWER_DOWN:
		return imx7_core_power_down(entry, context_id, nsec);
	case MX7_STATE_A7_POWER_DOWN:
		return imx7_all_off(entry, context_id, nsec);
	default:
		return PSCI_RET_INVALID_PARAMETERS;
	}

#if 0
	/* Store non-sec ctx regs */
	sm_save_modes_regs(&nsec->mode_regs);

	// XXX dump the gpc state
	gpc_dump_unmasked_irqs(p);

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
