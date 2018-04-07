/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2016, Linaro Limited
 * Copyright (c) 2014, STMicroelectronics International N.V.
 */

#ifndef __DRIVERS_GIC_H
#define __DRIVERS_GIC_H
#include <types_ext.h>
#include <kernel/interrupt.h>

#define GIC_DIST_REG_SIZE	0x10000
#define GIC_CPU_REG_SIZE	0x10000

struct gic_per_cpu_state {
    /* CPU interface control register */
    uint32_t iccicr;
    /* Interrupt priority mask register */
    uint32_t iccpmr;
    /* binary point register */
    uint32_t iccbpr;
    /* banked per-processor enable-set registers */
    uint32_t icdiser;
    /* banked per-processor active-set reigsters */
    uint32_t icdabr;
    /* banked per-processor config registers */
    uint32_t icdicfr[2]; 
};

struct gic_saved_state {
    /* distributor control register */
    uint32_t icddcr;
    /* interrupt security registers */
    uint32_t icdisr[32];
    /* interrupt set-enable registers */
    uint32_t icdiser[32];
    /* active bit registers */
    uint32_t icdabr[32];
    /* interrupt priority registers */
    uint32_t icdipr[255];
    /* interrupt processor targets registers */
    uint32_t icdiptr[255];
    /* interrupt configuration registers */
    uint32_t icdicfr[64];
    
    struct gic_per_cpu_state per_cpu[8];
};

struct gic_data {
	vaddr_t gicc_base;
	vaddr_t gicd_base;
	size_t max_it;
	struct itr_chip chip;
	struct gic_saved_state saved_state;
};

/*
 * The two gic_init_* functions initializes the struct gic_data which is
 * then used by the other functions.
 */

void gic_init(struct gic_data *gd, vaddr_t gicc_base, vaddr_t gicd_base);
/* initial base address only */
void gic_init_base_addr(struct gic_data *gd, vaddr_t gicc_base,
			vaddr_t gicd_base);
/* initial cpu if only, mainly use for secondary cpu setup cpu interface */
void gic_cpu_init(struct gic_data *gd);

void gic_it_handle(struct gic_data *gd);

void gic_dump_state(struct gic_data *gd);

void gic_save_state(struct gic_data *gd);

void gic_restore_state(struct gic_data *gd);

/*
 * Get the bitmask of enabled irqs. irqs should be an array large enough
 * to hold all max_it IRQ lines
 */
void gic_get_enabled_irqs(struct gic_data *gd, uint32_t *irqs);

#endif /*__DRIVERS_GIC_H*/
