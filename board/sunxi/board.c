/*
 * (C) Copyright 2012-2013 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Some board init for the Allwinner A10-evb board.
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
#include <asm/arch/dram.h>
#include <asm/arch/clock.h>
#include <asm/arch/mmc.h>
#include <axp209.h>

DECLARE_GLOBAL_DATA_PTR;

/* add board specific code here */
int board_init(void)
{
	gd->bd->bi_boot_params = (PHYS_SDRAM_1 + 0x100);

	return 0;
}

#ifdef CONFIG_DISPLAY_BOARDINFO
int checkboard(void)
{
	printf("Board: %s\n", CONFIG_SYS_BOARD_NAME);

	return 0;
}
#endif

int dram_init(void)
{
	gd->ram_size = get_ram_size((long *)PHYS_SDRAM_1, 1 << 30);

	return 0;
}

#ifdef CONFIG_GENERIC_MMC
int board_mmc_init(bd_t *bis)
{
	sunxi_mmc_init(CONFIG_MMC_SUNXI_SLOT);

	return 0;
}
#endif

#ifdef CONFIG_SPL_BUILD
void sunxi_board_init(void)
{
	int power_failed = 0;
	int ramsize;

	printf("DRAM:");
	ramsize = sunxi_dram_init();
	if (!ramsize) {
		printf(" ?");
		ramsize = sunxi_dram_init();
	}
	if (!ramsize) {
		printf(" ?");
		ramsize = sunxi_dram_init();
	}
	printf(" %dMB\n", ramsize>>20);
	if (!ramsize)
		hang();

#ifdef CONFIG_AXP209_POWER
	power_failed |= axp209_init();
	if(!power_failed){
		power_failed |= axp209_set_dcdc2(1400);
		power_failed |= axp209_set_dcdc3(1250);
		power_failed |= axp209_set_ldo2(3000);
		power_failed |= axp209_set_ldo3(2800);
		power_failed |= axp209_set_ldo4(2800);
		clock_set_pll1(1008000000);
		return;
	}
#endif

	/*
	 * Only clock up the CPU to full speed if we are reasonably
	 * assured it's being powered with suitable core voltage
	 */
	clock_set_pll1(1008000000);
}

#ifdef CONFIG_SPL_DISPLAY_PRINT
void spl_display_print(void)
{
	printf("Board: %s\n", CONFIG_SYS_BOARD_NAME);
}
#endif

#endif
