// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2017 NXP
 *
 * Peng Fan <peng.fan@nxp.com>
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
#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <sm/sm.h>
#include <sm/pm.h>
#include <sm/psci.h>
#include <stdint.h>
#include <atomic.h>

#define LPM_MODE_MASK			0x3
#define LPM_MODE_WAIT			0x1
#define LPM_MODE_STOP			0x2
#define LPM_STOP_ARM_CLOCK		BIT(2)
#define LPM_POWER_DOWN_CORES		BIT(3)
#define LPM_POWER_DOWN_SCU		BIT(4)
#define LPM_POWER_DOWN_L2		BIT(5)
#define LPM_INITIATING_CORE		BIT(6)

#define LPM_STATE_CLOCK_GATED		(LPM_MODE_WAIT | LPM_STOP_ARM_CLOCK)

#define LPM_STATE_ALL_OFF \
	(LPM_MODE_WAIT | LPM_STOP_ARM_CLOCK | LPM_POWER_DOWN_CORES | \
	 LPM_POWER_DOWN_SCU | LPM_POWER_DOWN_L2)

#define CORE_PUPSCR_SW2ISO		(0x1A << 7)

struct git_timer_state {
	uint32_t cntfrq;
	uint32_t cntkctl;
};

static int suspended_init;


// XXX GPT timer code begin
#ifndef GPT1_BASE_ADDR
#define GPT1_BASE_ADDR 0x302D0000
#define GPT2_BASE_ADDR 0x302E0000
#endif

#ifndef SHIFT_U32
#define SHIFT_U32(v, shift)   ((v) << (shift))
#endif

/* GPT register definitions */
#define GPT_CR                                   0x0
#define GPT_PR                                   0x4
#define GPT_SR                                   0x8
#define GPT_IR                                   0xc
#define GPT_OCR1                                 0x10
#define GPT_OCR2                                 0x14
#define GPT_OCR3                                 0x18
#define GPT_ICR1                                 0x1c
#define GPT_ICR2                                 0x20
#define GPT_CNT                                  0x24

#define GPT_CR_EN                                BIT(0)
#define GPT_CR_ENMOD                             BIT(1)
#define GPT_CR_DBGEN                             BIT(2)
#define GPT_CR_WAITEN                            BIT(3)
#define GPT_CR_DOZEEN                            BIT(4)
#define GPT_CR_STOPEN                            BIT(5)
#define GPT_CR_CLKSRC_24M                        SHIFT_U32(0x5, 6)
#define GPT_CR_CLKSRC_32K                        SHIFT_U32(0x4, 6)
#define GPT_CR_FRR                               BIT(9)
#define GPT_CR_EN_24M                            BIT(10)
#define GPT_CR_SWR                               BIT(15)

#define GPT_IR_OF1IE                             BIT(0)

#define GPT_USE_32K
#ifdef GPT_USE_32K
#define GPT_FREQ 32768
#define GPT_PRESCALER24M 0
#define GPT_PRESCALER 0
#else
#define GPT_FREQ 1000000
#define GPT_PRESCALER24M (12 - 1)
#define GPT_PRESCALER (2 - 1)
#endif

#define GPT1_IRQ	(55 + 32)
#define GPT2_IRQ        (54 + 32)

#define GPT_BASE_ADDR GPT2_BASE_ADDR
#define GPT_IRQ GPT2_IRQ

#define GIC_BASE                0x31000000
#define GIC_SIZE                0x8000
#define GICC_OFFSET             0x2000
#define GICD_OFFSET             0x1000
#define GICD_BASE		(GIC_BASE + GICD_OFFSET)
#define NUM_INTS_PER_REG	32
#define GICD_ISENABLER(n)       (0x100 + (n) * 4)

static uint32_t gpt_base;
static bool __maybe_unused gpt_ack_interrupt(void);

static enum itr_return gpt_itr_cb(struct itr_handler *h __unused)
{
	DMSG("GPT1 interrupt!");
	return gpt_ack_interrupt() ? ITRR_HANDLED : ITRR_NONE;
}

static struct itr_handler gpt_itr = {
	.it = GPT_IRQ,
	.flags = ITRF_TRIGGER_LEVEL,
	.handler = gpt_itr_cb,
};

static void gpt_init(void)
{
	uint32_t val;

	if (!core_mmu_add_mapping(MEM_AREA_IO_SEC, GPT_BASE_ADDR,
					  CORE_MMU_DEVICE_SIZE)) {

		DMSG("Failed to map GPT timer");
	}

	gpt_base = (uint32_t)phys_to_virt(GPT_BASE_ADDR, MEM_AREA_IO_SEC);

	DMSG("Successfully mapped GPT at 0x%x", gpt_base);

	/* Disable GPT */
	write32(0, gpt_base + GPT_CR);

	/* Software reset */
	write32(GPT_CR_SWR, gpt_base + GPT_CR);

	/* Wait for reset bit to clear */
	while ((read32(gpt_base + GPT_CR) & GPT_CR_SWR) != 0);

	/* Set prescaler to target frequency */
	val = ((GPT_PRESCALER24M & 0xf) << 12) |  (GPT_PRESCALER & 0xfff);
	write32(val, gpt_base + GPT_PR);

	/* Select clock source */
#ifdef GPT_USE_32K
	val = GPT_CR_CLKSRC_32K;
#else
	val = GPT_CR_CLKSRC_24M;
#endif
	write32(val, gpt_base + GPT_CR);

	val = read32(gpt_base + GPT_CR);
	val |= GPT_CR_EN;
	val |= GPT_CR_ENMOD;
	val |= GPT_CR_STOPEN;
	val |= GPT_CR_WAITEN;
	val |= GPT_CR_DOZEEN;
	val |= GPT_CR_DBGEN;
	val |= GPT_CR_FRR;
#ifndef GPT_USE_32K
	val |= GPT_CR_EN_24M;
#endif
	write32(val, gpt_base + GPT_CR);

	itr_add(&gpt_itr);
	itr_enable(GPT_IRQ);

	DMSG("Initialized GPT");
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

	/* Enable timer */
	val |= GPT_CR_EN;
	val |= GPT_CR_ENMOD;
	write32(val, gpt_base + GPT_CR);
}

static bool __maybe_unused gpt_ack_interrupt(void)
{
        uint32_t val;

        val = read32(gpt_base + GPT_SR);

        /* Disable and acknowledge interrupts */
        write32(0, gpt_base + GPT_IR);
        write32(0x3f, gpt_base + GPT_SR);

        return (val & 0x1) != 0;
}

static void __maybe_unused gpt_wait_for_interrupt(void)
{
	while((read32(gpt_base + GPT_SR) & 0x1) == 0);
}

// XXX GPT test code end

static int imx7_cpu_suspend_to_ocram(uint32_t power_state __unused,
		uintptr_t entry, uint32_t context_id __unused,
		struct sm_nsec_ctx *nsec)
{
	struct imx7_pm_info *p = suspend_info;
	int ret;

	/* Store non-sec ctx regs */
	sm_save_modes_regs(&nsec->mode_regs);

	ret = sm_pm_cpu_suspend((uint32_t)p, (int (*)(uint32_t))
				((uint32_t)p + sizeof(*p)));
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

	main_init_gic();

	DMSG("=== Back from Suspended ===\n");

	return 0;
}

static void git_timer_save_state(struct git_timer_state *state)
{
	state->cntfrq = read_cntfrq();
	state->cntkctl = read_cntkctl();
}

static void __maybe_unused git_timer_restore_state(struct git_timer_state *state)
{
	write_cntfrq(state->cntfrq);
	write_cntkctl(state->cntkctl);
	write_cntvoff(0);
}

static void enable_wake_irqs(struct imx7_pm_info *p)
{
	uint32_t irqs[5];

	assert((gic_data.max_it / 32) < ARRAY_SIZE(irqs));
	gic_get_enabled_irqs(&gic_data, irqs);

	write32(~irqs[1], p->gpc_va_base + GPC_IMR1_CORE0_A7);
	write32(~irqs[2], p->gpc_va_base + GPC_IMR2_CORE0_A7);
	write32(~irqs[3], p->gpc_va_base + GPC_IMR3_CORE0_A7);
	write32(~irqs[4], p->gpc_va_base + GPC_IMR4_CORE0_A7);
}

static void imx7_set_run_mode(struct imx7_pm_info *p)
{
	uint32_t val;

	imx_gpcv2_unmask_irq(GPR_IRQ);

	val = read32(p->gpc_va_base + GPC_LPCR_A7_BSC);
	val &= ~GPC_LPCR_A7_BSC_LPM0;
	val &= ~GPC_LPCR_A7_BSC_LPM1;
	val |= GPC_LPCR_A7_BSC_CPU_CLK_ON_LPM;
	write32(val, p->gpc_va_base + GPC_LPCR_A7_BSC);
}

static void enable_gpr_irq(struct imx7_pm_info *p)
{
	uint32_t val;

	val = read32(p->iomuxc_gpr_va_base + IOMUXC_GPR1_OFFSET);
	val |= IOMUXC_GPR1_IRQ;
	write32(val, p->iomuxc_gpr_va_base + IOMUXC_GPR1_OFFSET);
}

static void imx7_lpm_init(void)
{
	struct imx7_pm_info *p = lpi_info;
	uint32_t val;

	enable_gpr_irq(p);
	imx_gpcv2_mask_all_irqs();
	imx_gpcv2_unmask_irq(GPR_IRQ);

	val = read32(p->gpc_va_base + GPC_LPCR_A7_BSC);
	val &= ~GPC_LPCR_A7_BSC_LPM0;
	val &= ~GPC_LPCR_A7_BSC_LPM1;
	val |= GPC_LPCR_A7_BSC_CPU_CLK_ON_LPM;
	val &= ~GPC_LPCR_A7_BSC_MASK_CORE0_WFI;
	val &= ~GPC_LPCR_A7_BSC_MASK_CORE1_WFI;
	val &= ~GPC_LPCR_A7_BSC_MASK_L2CC_WFI;
	val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_C0;
	val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_C1;
	val |= GPC_LPCR_A7_BSC_IRQ_SRC_A7_WUP;
	val &= ~GPC_LPCR_A7_BSC_MASK_DSM_TRIGGER;
	write32(val, p->gpc_va_base + GPC_LPCR_A7_BSC);

	/* Program A7 advanced power control register */
	val = read32(p->gpc_va_base + GPC_LPCR_A7_AD);
	val &= ~GPC_LPCR_A7_AD_L2_PGE;
	val &= ~GPC_LPCR_A7_AD_EN_PLAT_PDN;
	val &= ~GPC_LPCR_A7_AD_EN_C0_PDN;
	val &= ~GPC_LPCR_A7_AD_EN_C1_PDN;
	val &= ~GPC_LPCR_A7_AD_EN_C0_PUP;
	val &= ~GPC_LPCR_A7_AD_EN_C0_WFI_PDN;
	val &= ~GPC_LPCR_A7_AD_EN_C0_IRQ_PUP;
	val &= ~GPC_LPCR_A7_AD_EN_C1_PUP;
	val &= ~GPC_LPCR_A7_AD_EN_C1_WFI_PDN;
	val &= ~GPC_LPCR_A7_AD_EN_C1_IRQ_PUP;
	write32(val, p->gpc_va_base + GPC_LPCR_A7_AD);

	/* program M4 power control register */
	val = read32(p->gpc_va_base + GPC_LPCR_M4);
	val |= GPC_LPCR_M4_MASK_DSM_TRIGGER;
	write32(val, p->gpc_va_base + GPC_LPCR_M4);

	/* set mega/fast mix in A7 domain */
	write32(0x1, p->gpc_va_base + GPC_PGC_CPU_MAPPING);

	/* set SCU timing values from datasheet */
	write32((0x59 << 10) | 0x5B | (0x2 << 20),
		 p->gpc_va_base + GPC_PGC_SCU_AUXSW);

	/* set C0/C1 power up timing */
	val = read32(p->gpc_va_base + GPC_PGC_C0_PUPSCR);
	val &= ~GPC_PGC_CORE_PUPSCR;
	val |= CORE_PUPSCR_SW2ISO;
	write32(val, p->gpc_va_base + GPC_PGC_C0_PUPSCR);

	val = read32(p->gpc_va_base + GPC_PGC_C1_PUPSCR);
	val &= ~GPC_PGC_CORE_PUPSCR;
	val |= CORE_PUPSCR_SW2ISO;
	write32(val, p->gpc_va_base + GPC_PGC_C1_PUPSCR);

	/* Disable DSM, voltage standby, RBC, and oscillator powerdown */
	val = 0;
	val |= GPC_SLPCR_EN_A7_FASTWUP_WAIT_MODE;
	write32(val, p->gpc_va_base + GPC_SLPCR);

	/* disable memory low power mode */
	val = read32(p->gpc_va_base + GPC_MLPCR);
	val |= GPC_MLPCR_MEMLP_CTL_DIS;
	write32(val, p->gpc_va_base + GPC_MLPCR);

	val = GPC_PGC_ACK_SEL_A7_DUMMY_PUP_ACK |
		GPC_PGC_ACK_SEL_A7_DUMMY_PDN_ACK;

	write32(val, p->gpc_va_base + GPC_PGC_ACK_SEL_A7);
}

static void imx7_prepare_lpm(uint32_t lpm_flags, struct imx7_pm_info *p)
{
	uint32_t val;

	enable_wake_irqs(p);

	assert(imx_gpcv2_irq_pending(GPR_IRQ));
	imx_gpcv2_unmask_irq(GPR_IRQ);

	/* Program LPCR_A7_BSC */
	val = read32(p->gpc_va_base + GPC_LPCR_A7_BSC);
	val &= ~GPC_LPCR_A7_BSC_LPM0;
	val |= lpm_flags & LPM_MODE_MASK;
	val &= ~GPC_LPCR_A7_BSC_LPM1;
	val |= ((lpm_flags & LPM_MODE_MASK) << 2);
	if ((lpm_flags & LPM_POWER_DOWN_SCU) ||
	    (lpm_flags & LPM_STOP_ARM_CLOCK)) {

		val &= ~GPC_LPCR_A7_BSC_CPU_CLK_ON_LPM;
	} else {
		val |= GPC_LPCR_A7_BSC_CPU_CLK_ON_LPM;
	}

	val &= ~GPC_LPCR_A7_BSC_MASK_CORE0_WFI;
	val &= ~GPC_LPCR_A7_BSC_MASK_CORE1_WFI;
	val &= ~GPC_LPCR_A7_BSC_MASK_L2CC_WFI;

	val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_C0;
	val &= ~GPC_LPCR_A7_BSC_IRQ_SRC_C1;
	val |= GPC_LPCR_A7_BSC_IRQ_SRC_A7_WUP;

	val &= ~GPC_LPCR_A7_BSC_MASK_DSM_TRIGGER;
	write32(val, p->gpc_va_base + GPC_LPCR_A7_BSC);

	/* Program A7 advanced power control register */
	val = read32(p->gpc_va_base + GPC_LPCR_A7_AD);
	assert((val & GPC_LPCR_A7_AD_EN_C0_WFI_PDN) == 0);
	assert((val & GPC_LPCR_A7_AD_EN_C0_IRQ_PUP) == 0);
	assert((val & GPC_LPCR_A7_AD_EN_C1_PUP) == 0);
	assert((val & GPC_LPCR_A7_AD_EN_C1_WFI_PDN) == 0);
	assert((val & GPC_LPCR_A7_AD_EN_C1_IRQ_PUP) == 0);

	if (lpm_flags & LPM_POWER_DOWN_CORES) {
		val |= GPC_LPCR_A7_AD_EN_C0_PDN;
		val |= GPC_LPCR_A7_AD_EN_C1_PDN;
		val |= GPC_LPCR_A7_AD_EN_C0_PUP;

		if (lpm_flags & LPM_POWER_DOWN_SCU) {
			val |= GPC_LPCR_A7_AD_EN_PLAT_PDN;
			if (lpm_flags & LPM_POWER_DOWN_L2)
				val |= GPC_LPCR_A7_AD_L2_PGE;
		} else {
			val &= ~GPC_LPCR_A7_AD_L2_PGE;
			val &= ~GPC_LPCR_A7_AD_EN_PLAT_PDN;
		}
	} else {
		val &= ~GPC_LPCR_A7_AD_L2_PGE;
		val &= ~GPC_LPCR_A7_AD_EN_PLAT_PDN;
		val &= ~GPC_LPCR_A7_AD_EN_C0_PDN;
		val &= ~GPC_LPCR_A7_AD_EN_C1_PDN;
		val &= ~GPC_LPCR_A7_AD_EN_C0_PUP;
	}

	write32(val, p->gpc_va_base + GPC_LPCR_A7_AD);

	/* A7_SCU as LPM power down ACK, A7_C0 as LPM power up ack */
	if (lpm_flags & LPM_POWER_DOWN_CORES) {
		/* A7_C0, A7_C1 power down in SLOT0 */
		write32(CORE0_A7_PDN_SLOT_CONTROL |
			CORE1_A7_PDN_SLOT_CONTROL,
			p->gpc_va_base + GPC_SLT0_CFG);

		if (lpm_flags & LPM_POWER_DOWN_SCU) {
			/* A7_SCU power down in SLOT3 */
			write32(SCU_PDN_SLOT_CONTROL,
				p->gpc_va_base + GPC_SLT3_CFG);

			/* A7_SCU power up in SLOT6 */
			write32(SCU_PUP_SLOT_CONTROL,
				p->gpc_va_base + GPC_SLT6_CFG);
		}

		/* A7_C0 power up in SLOT7 */
		write32(CORE0_A7_PUP_SLOT_CONTROL,
			p->gpc_va_base + GPC_SLT7_CFG);

		if (lpm_flags & LPM_POWER_DOWN_SCU) {
			val = GPC_PGC_ACK_SEL_A7_PLAT_PGC_PDN_ACK |
				GPC_PGC_ACK_SEL_A7_C0_PGC_PUP_ACK;
		} else {
			val = GPC_PGC_ACK_SEL_A7_C0_PGC_PUP_ACK |
				GPC_PGC_ACK_SEL_A7_C0_PGC_PDN_ACK;
		}

		write32(val, p->gpc_va_base + GPC_PGC_ACK_SEL_A7);
	} else {
		write32(GPC_PGC_ACK_SEL_A7_DUMMY_PUP_ACK |
			GPC_PGC_ACK_SEL_A7_DUMMY_PDN_ACK,
			p->gpc_va_base + GPC_PGC_ACK_SEL_A7);
	}

	/* arm PGC for power down */
	if (lpm_flags & LPM_POWER_DOWN_CORES) {
		imx_gpcv2_set_core_pgc(true, GPC_PGC_C0);
		imx_gpcv2_set_core_pgc(true, GPC_PGC_C1);
		if (lpm_flags & LPM_POWER_DOWN_SCU)
			imx_gpcv2_set_core_pgc(true, GPC_PGC_SCU);
	}

	imx_gpcv2_mask_irq(GPR_IRQ);
}

static int imx7_lpm_entry(uint32_t lpm_flags)
{
	uint32_t val;
	struct imx7_pm_info *p = lpi_info;
	
	typedef int idle_entry_func(uint32_t);
	idle_entry_func *enter_lpi = 
		(idle_entry_func *)(p->va_base + sizeof(*p));

	//console_putc('~');
	//DMSG("Before: cntpct=0x%llx, cntvct=0x%llx, cntfrq=0x%x, cntkctl=0x%x"
	//     ", cntvoff=0x%llx",
	//	read_cntpct(), read_cntvct(), read_cntfrq(), read_cntkctl(),
	//	read_cntvoff());

	/* setup resume address and parameter in OCRAM */
	// XXX handled in assembly
	if (false /*lpm_flags & LPM_POWER_DOWN_CORES*/) {
		int core_idx = get_core_pos();

		val = ((uint32_t)&resume - (uint32_t)&imx7_suspend) +
			p->pa_base + p->pm_info_size;

		write32(val, p->src_va_base + SRC_GPR1_MX7 + core_idx * 8);
		write32(p->pa_base,
			p->src_va_base + SRC_GPR2_MX7 + core_idx * 8);
	}

	if (lpm_flags & LPM_INITIATING_CORE)
		imx7_prepare_lpm(lpm_flags, p);

	console_putc('+');
	/* enter LPM */
	if (lpm_flags & LPM_POWER_DOWN_CORES) {
		enter_lpi((uint32_t)p);
	} else {
		dsb();
		wfi();
	}
	//imx7d_low_power_idle(p);
	//dsb();
	//wfi();

	console_putc('*');

	/* return value ignored by sm_pm_cpu_suspend */
	return 0;
}

bool wait = false;
static int imx7_do_context_losing_lpm(uint32_t lpm_flags,
	uintptr_t entry,
	uint32_t context_id,
	struct sm_nsec_ctx *nsec)
{
	int ret;
	struct git_timer_state git_state;

	assert(lpm_flags & LPM_POWER_DOWN_CORES);

	git_timer_save_state(&git_state);

	if (lpm_flags & LPM_POWER_DOWN_SCU)
		gic_save_state(&gic_data);

	/* save banked registers for every mode except monitor mode */
	sm_save_modes_regs(&nsec->mode_regs);
	
	ret = sm_pm_cpu_suspend(lpm_flags, imx7_lpm_entry);

	/* whether we did or did not suspend, we need to unarm hardware */
	/* XXX if we actually suspended, don't these bits clear themselves? */
	if (lpm_flags & LPM_INITIATING_CORE) {
		imx_gpcv2_set_core_pgc(false, GPC_PGC_C0);
		imx_gpcv2_set_core_pgc(false, GPC_PGC_C1);
		if (lpm_flags & LPM_POWER_DOWN_SCU)
			imx_gpcv2_set_core_pgc(false, GPC_PGC_SCU);
	}
	
	if (ret != 0) {
		//EMSG("=== Core did not power down ===");
		return PSCI_RET_SUCCESS;
	}

	git_timer_restore_state(&git_state);

	/* Restore register of different mode in secure world */
	sm_restore_modes_regs(&nsec->mode_regs);

	//main_init_gic();
	if (lpm_flags & LPM_POWER_DOWN_SCU)
		gic_restore_state(&gic_data);

	nsec->mon_lr = (uint32_t)entry;
	nsec->mon_spsr = CPSR_MODE_SVC | CPSR_I | CPSR_F;

	//DMSG("Back from power down. (entry=0x%x, context_id=0x%x)",
	//	(uint32_t)entry, context_id);
	
	console_putc('-');
	//DMSG("After: cntpct=0x%llx, cntvct=0x%llx, cntfrq=0x%x, cntkctl=0x%x"
	//	", cntvoff=0x%llx",
	//	read_cntpct(), read_cntvct(), read_cntfrq(), read_cntkctl(),
	//	read_cntvoff());

	return context_id;
}

/* Enter low power mode using the specified options */
static int imx7_lpm(uint32_t lpm_flags, uintptr_t entry,
	     uint32_t context_id, struct sm_nsec_ctx *nsec)
{
	int ret;
	uint32_t val;
	struct imx7_pm_info *p = lpi_info;

	val = atomic_dec32(&active_cores);
	if ((lpm_flags & LPM_INITIATING_CORE) && (val != 0)) {
		atomic_inc32(&active_cores);
		return PSCI_RET_DENIED;
	}

	if (lpm_flags & LPM_POWER_DOWN_CORES) {
		ret = imx7_do_context_losing_lpm(lpm_flags, entry,
			context_id, nsec);
	} else {
		imx7_lpm_entry(lpm_flags);
		ret = PSCI_RET_SUCCESS;
	}

	if (lpm_flags & LPM_INITIATING_CORE)
		imx7_set_run_mode(p);

	atomic_inc32(&active_cores);
	return ret;
}

int imx7_cpu_suspend(uint32_t power_state, uintptr_t entry,
		     uint32_t context_id, struct sm_nsec_ctx *nsec)
{
	if (!suspended_init) {
		imx7_suspend_init();
		imx7d_cpuidle_init();
		imx7_lpm_init();
		suspended_init = 1;
	}

	switch (power_state) {
	case 0:
	case 0 | PSCI_POWER_STATE_TYPE_MASK:
		return imx7_cpu_suspend_to_ocram(power_state, entry,
						 context_id, nsec);

	/*
	 * even though this state does not lose context, the OSPM may
	 * need the context-losing flag to be set so that it knows not
	 * to use the GIT timer for wakeup
	 */
	case MX7_STATEID_CORE_CLOCK_GATE |
		PSCI_EXT_POWER_STATE_TYPE_POWER_DOWN:
	case MX7_STATEID_CORE_CLOCK_GATE:
		return imx7_lpm(LPM_STATE_CLOCK_GATED,
				entry, context_id, nsec);

	case MX7_STATEID_CORE_CLOCK_GATE | MX7_STATEID_CLUSTER_CLOCK_GATE |
		MX7_LEVELID_CLUSTER | PSCI_EXT_POWER_STATE_TYPE_POWER_DOWN:
	case MX7_STATEID_CORE_CLOCK_GATE | MX7_STATEID_CLUSTER_CLOCK_GATE |
		MX7_LEVELID_CLUSTER:

		return imx7_lpm(LPM_STATE_CLOCK_GATED | LPM_INITIATING_CORE,
				entry, context_id, nsec);

	case 0x41000005:
	case 0x41000055:
		return imx7_lpm(
			LPM_MODE_WAIT |
			LPM_STOP_ARM_CLOCK |
			LPM_POWER_DOWN_CORES |
			//LPM_POWER_DOWN_SCU |
			//LPM_POWER_DOWN_L2 |
			LPM_INITIATING_CORE,
			entry,
			context_id,
			nsec);

	case 0x1234:

		gpt_init();
		for (;;) {

			gpt_schedule_interrupt(1000);
			gpt_wait_for_interrupt();

			DMSG("Attempting first LPM with interrupt asserted");
			imx7_lpm(
				LPM_MODE_WAIT |
				LPM_STOP_ARM_CLOCK |
				LPM_POWER_DOWN_CORES |
				//LPM_POWER_DOWN_SCU |
				//LPM_POWER_DOWN_L2 |
				LPM_INITIATING_CORE,
				entry,
				context_id,
				nsec);

			DMSG("Attempting second LPM");
			gpt_schedule_interrupt(500);

			imx7_lpm(
				LPM_MODE_WAIT |
				LPM_STOP_ARM_CLOCK |
				LPM_POWER_DOWN_CORES |
				//LPM_POWER_DOWN_SCU |
				//LPM_POWER_DOWN_L2 |
				LPM_INITIATING_CORE,
				entry,
				context_id,
				nsec);

			gpt_ack_interrupt();
			DMSG("Back from second LPM");
		}

	default:
		EMSG("Unknown state: 0x%x", power_state);
		return PSCI_RET_INVALID_PARAMETERS;
	}
}


