/*
 * sunxi DRAM controller initialization
 * (C) Copyright 2012 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * Based on sun4i Linux kernel sources mach-sunxi/pm/standby/dram*.c
 * and earlier U-Boot Allwiner A10 SPL work
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Berg Xing <bergxing@allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/dram.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>

static void mctl_ddr3_reset(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;

#ifdef CONFIG_SUN4I
	u32 reg_val;

	writel(0, TIMER_CPU_CFG_REG);
	reg_val = readl(TIMER_CPU_CFG_REG);
	reg_val >>= 6;
	reg_val &= 0x3;

	if (reg_val != 0) {
		setbits_le32(&dram->mcr, 0x1 << 12);
		sdelay(0x100);
		clrbits_le32(&dram->mcr, 0x1 << 12);
	} else
#endif
	{
		clrbits_le32(&dram->mcr, 0x1 << 12);
		sdelay(0x100);
		setbits_le32(&dram->mcr, 0x1 << 12);
	}
}

static void mctl_set_drive(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;

	clrsetbits_le32(&dram->mcr, 0x3, (0x6 << 12) | 0xFFC);
}

static void mctl_itm_disable(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;

	setbits_le32(&dram->ccr, 0x1 << 28);
}

static void mctl_itm_enable(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;

	clrbits_le32(&dram->ccr, 0x1 << 28);
}

static void mctl_enable_dll0(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;

	clrsetbits_le32(&dram->dllcr[0], 0x40000000, 0x80000000);
	sdelay(0x100);

	clrbits_le32(&dram->dllcr[0], 0xC0000000);
	sdelay(0x1000);

	clrsetbits_le32(&dram->dllcr[0], 0x80000000, 0x40000000);
	sdelay(0x1000);
}

/*
 * Note: This differs from pm/standby in that it checks the bus width
 */
static void mctl_enable_dllx(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;
	u32 i, n, bus_width;

	bus_width = readl(&dram->dcr);
	bus_width >>= 6;
	bus_width &= 7;

	if (bus_width == 3) {
		n = 5;
	} else {
		n = 3;
	}

	for (i = 1; i < n; i++)
		clrsetbits_le32(&dram->dllcr[i], 0x40000000, 0x80000000);
	sdelay(0x100);

	for (i = 1; i < n; i++)
		clrbits_le32(&dram->dllcr[i], 0xC0000000);
	sdelay(0x1000);

	for (i = 1; i < n; i++)
		clrsetbits_le32(&dram->dllcr[i], 0x80000000, 0x40000000);
	sdelay(0x1000);
}

static u32 hpcr_value[32] = {
#ifdef CONFIG_SUN5I
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0x1031, 0x1031, 0x0735, 0x1035,
	0x1035, 0x0731, 0x1031, 0,
	0x0301, 0x0301, 0x0301, 0x0301,
	0x0301, 0x0301, 0x0301, 0
#endif
#ifdef CONFIG_SUN4I
	0x0301, 0x0301, 0x0301, 0x0301,
	0x0301, 0x0301, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0x1031, 0x1031, 0x0735, 0x1035,
	0x1035, 0x0731, 0x1031, 0x0735,
	0x1035, 0x1031, 0x0731, 0x1035,
	0x1031, 0x0301, 0x0301, 0x0731
#endif
};

static void mctl_configure_hostport(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;
	u32 i;

	for (i = 0; i < 32; i++)
		writel(hpcr_value[i], &dram->hpcr[i]);
}

static void mctl_setup_dram_clock(u32 clk)
{
	u32 reg_val;

	/* setup DRAM PLL */
	reg_val = readl(DRAM_CCM_SDRAM_PLL_REG);
	reg_val &= ~0x3;
	reg_val |= 0x1;			/* m factor */
	reg_val &= ~(0x3 << 4);
	reg_val |= 0x1 << 4;		/* k factor */
	reg_val &= ~(0x1f << 8);
	reg_val |= ((clk / 24) & 0x1f) << 8;	/* n factor */
	reg_val &= ~(0x3 << 16);
	reg_val |= 0x1 << 16;		/* p factor */
	reg_val &= ~(0x1 << 29);	/* PLL on */
	reg_val |= (u32) 0x1 << 31;	/* PLL En */
	writel(reg_val, DRAM_CCM_SDRAM_PLL_REG);
	sdelay(0x100000);

	setbits_le32(DRAM_CCM_SDRAM_PLL_REG, 0x1 << 29);

#ifdef CONFIG_SUN4I
	/* reset GPS */
	clrbits_le32(DRAM_CCM_GPS_CLK_REG, 0x3);
	setbits_le32(DRAM_CCM_AHB_GATE_REG, 0x1 << 26);
	sdelay(0x20);
	clrbits_le32(DRAM_CCM_AHB_GATE_REG, 0x1 << 26);
#endif

	/* setup MBUS clock */
	reg_val = (0x1 << 31) | (0x2 << 24) | (0x1);
	writel(reg_val, DRAM_CCM_MUS_CLK_REG);

	/*
	 * open DRAMC AHB & DLL register clock
	 * close it first
	 */
#ifdef CONFIG_SUN5I
	clrbits_le32(DRAM_CCM_AHB_GATE_REG, 0x3 << 14);
#else
	clrbits_le32(DRAM_CCM_AHB_GATE_REG, 0x1 << 14);
#endif
	sdelay(0x1000);

	/* then open it */
#ifdef CONFIG_SUN5I
	setbits_le32(DRAM_CCM_AHB_GATE_REG, 0x3 << 14);
#else
	setbits_le32(DRAM_CCM_AHB_GATE_REG, 0x1 << 14);
#endif
	sdelay(0x1000);
}

static int dramc_scan_readpipe(void)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;
	u32 reg_val;

	/* data training trigger */
	setbits_le32(&dram->ccr, 0x1 << 30);

	/* check whether data training process is end */
	while (readl(&dram->ccr) & (0x1 << 30))
		;

	/* check data training result */
	reg_val = readl(&dram->csr);
	if (reg_val & (0x1 << 20))
		return -1;

	return 0;
}

static void dramc_clock_output_en(u32 on)
{
#ifdef CONFIG_SUN5I
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;

	if (on)
		setbits_le32(&dram->mcr, 0x1 << DCLK_OUT_OFFSET);
	else
		clrbits_le32(&dram->mcr, 0x1 << DCLK_OUT_OFFSET);
#endif
#ifdef CONFIG_SUN4I
	if (on)
		setbits_le32(DRAM_CCM_SDRAM_CLK_REG, 0x1 << DCLK_OUT_OFFSET);
	else
		clrbits_le32(DRAM_CCM_SDRAM_CLK_REG, 0x1 << DCLK_OUT_OFFSET);
#endif
}

#ifdef CONFIG_SUN4I
static void dramc_set_autorefresh_cycle(u32 clk)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;
	u32 reg_val;
	u32 tmp_val;
	u32 dram_size;

	if (clk < 600) {
		dram_size = readl(&dram->dcr);
		dram_size >>= 3;
		dram_size &= 0x7;
		if (dram_size <= 0x2)
			reg_val = (131 * clk) >> 10;
		else
			reg_val = (336 * clk) >> 10;

		tmp_val = (7987 * clk) >> 10;
		tmp_val = tmp_val * 9 - 200;
		reg_val |= tmp_val << 8;
		reg_val |= 0x8 << 24;
		writel(reg_val, &dram->drr);
	} else {
		writel(0x0, &dram->drr);
	}
}
#endif /* SUN4I */

#ifdef CONFIG_SUN5I
static void dramc_set_autorefresh_cycle(u32 clk)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;
	u32 reg_val;
	u32 tmp_val;
	reg_val = 131;

	tmp_val = (7987 * clk) >> 10;
	tmp_val = tmp_val * 9 - 200;
	reg_val |= tmp_val << 8;
	reg_val |= 0x8 << 24;
	writel(reg_val, &dram->drr);
}
#endif /* SUN5I */

int dramc_init(struct dram_para *para)
{
	struct sunxi_dram_reg *dram = (struct sunxi_dram_reg *)DRAMC_IO_BASE;
	u32 reg_val;
	int ret_val;

	/* check input dram parameter structure */
	if (!para)
		return -1;

	/* setup DRAM relative clock */
	mctl_setup_dram_clock(para->clock);

#ifdef CONFIG_SUN5I
	// test-only: new code? does it work? change or remove comment
	/* This is new unknown code! */
	writel(0, &dram->reg_23c);
#endif

	/* reset external DRAM */
	mctl_ddr3_reset();
	mctl_set_drive();

	/* dram clock off */
	dramc_clock_output_en(0);

#ifdef CONFIG_SUN4I
	/* select dram controller 1 */
	writel(0x16237495, &dram->csel);
#endif

	mctl_itm_disable();
	mctl_enable_dll0();

	/* configure external DRAM */
	reg_val = 0;
	if (para->type == 3)
		reg_val |= 0x1;
	reg_val |= (para->io_width >> 3) << 1;

	if (para->density == 256)
		reg_val |= 0x0 << 3;
	else if (para->density == 512)
		reg_val |= 0x1 << 3;
	else if (para->density == 1024)
		reg_val |= 0x2 << 3;
	else if (para->density == 2048)
		reg_val |= 0x3 << 3;
	else if (para->density == 4096)
		reg_val |= 0x4 << 3;
	else if (para->density == 8192)
		reg_val |= 0x5 << 3;
	else
		reg_val |= 0x0 << 3;

	reg_val |= ((para->bus_width >> 3) - 1) << 6;

	reg_val |= (para->rank_num - 1) << 10;

	reg_val |= 0x1 << 12;
	reg_val |= ((0x1) & 0x3) << 13;

	writel(reg_val, &dram->dcr);

#ifdef CONFIG_SUN5I
	/* set odt impendance divide ratio */
	reg_val = ((para->zq) >> 8) & 0xfffff;
	reg_val |= ((para->zq) & 0xff) << 20;
	reg_val |= (para->zq) & 0xf0000000;
	writel(reg_val, &dram->zqcr0);
#endif

	/* dram clock on */
	dramc_clock_output_en(1);

	sdelay(0x10);

	while (readl(&dram->ccr) & (0x1U << 31))
		;

	mctl_enable_dllx();

#ifdef CONFIG_SUN4I
	/* set odt impendance divide ratio */
	reg_val = ((para->zq) >> 8) & 0xfffff;
	reg_val |= ((para->zq) & 0xff) << 20;
	reg_val |= (para->zq) & 0xf0000000;
	writel(reg_val, &dram->zqcr0);
#endif

#ifdef CONFIG_SUN4I
	/* set I/O configure register */
	reg_val = 0x00cc0000;
	reg_val |= (para->odt_en) & 0x3;
	reg_val |= ((para->odt_en) & 0x3) << 30;
	writel(reg_val, &dram->iocr);
#endif

	/* set refresh period */
	dramc_set_autorefresh_cycle(para->clock);

	/* set timing parameters */
	writel(para->tpr0, &dram->tpr0);
	writel(para->tpr1, &dram->tpr1);
	writel(para->tpr2, &dram->tpr2);

	/* set mode register */
	if (para->type == 3) {
		/* ddr3 */
		reg_val = 0x0;
#ifdef CONFIG_SUN5I
		reg_val |= 0x1000;
#endif
		reg_val |= (para->cas - 4) << 4;
		reg_val |= 0x5 << 9;
	} else if (para->type == 2) {
		/* ddr2 */
		reg_val = 0x2;
		reg_val |= para->cas << 4;
		reg_val |= 0x5 << 9;
	}
	writel(reg_val, &dram->mr);

	writel(para->emr1, &dram->emr);
	writel(para->emr2, &dram->emr2);
	writel(para->emr3, &dram->emr3);

	/* set DQS window mode */
	clrsetbits_le32(&dram->ccr, 0x1U << 17, 0x1U << 14);

	/* initial external DRAM */
	setbits_le32(&dram->ccr, 0x1U << 31);

	while (readl(&dram->ccr) & (0x1U << 31))
		;

	/* scan read pipe value */
	mctl_itm_enable();
	ret_val = dramc_scan_readpipe();

	if (ret_val < 0)
		return 0;

	/* configure all host port */
	mctl_configure_hostport();

	return get_ram_size((long *)PHYS_SDRAM_1, 1 << 30);
}
