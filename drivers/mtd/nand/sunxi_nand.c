#define DEBUG
/*
 * (C) Copyright 2012
 * Sergey Lapin <slapin@ossfans.org>
 *
 * Portions (C) 2007-2011 Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * MTD-based NAND driver.
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
#include <nand.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>

#ifdef DEBUG
uint32_t debug_readl(const char *name, void *addr) {
	uint32_t r = readl(addr);
	debug("READ %s=%08x\n", name, r);
	return r;
}

void debug_writel(const char *name, uint32_t r, void *addr) {
	debug("WRITE %s=%08x\n", name, r);
	writel(r, addr);
}
#define readl(addr)		debug_readl(#addr, (void *)addr)
#define writel(value, addr)	debug_writel(#addr, value, (void *)addr)
#endif
#define NFC_REG_CTL		0x01c03000
#define NFC_EN			(1 << 0)
#define NFC_RESET		(1 << 1)
#define NFC_RAM_METHOD		(1 << 14)
#define NFC_CE_SEL		(7 << 24)

#define NFC_REG_ECC_CTL 	0x01c03034
#define NFC_ECC_MODE		(0xf << 12)
#define NFC_RANDOM_EN		(1 << 9)

#define NFC_REG_ST		0x01c03004
#define NFC_CMD_INT_FLAG	(1 << 1)
#define NFC_CMD_FIFO_STATUS	(1 << 3)
#define NFC_RB_STATE0		(1 << 8)

#define NFC_REG_TIMING_CTL	0x01c0300c
#define NFC_REG_TIMING_CFG	0x01c03010

#define NFC_REG_SPARE_AREA	0x01c030a0

#define NFC_REG_CMD		0x01c03024
#define NFC_SEND_ADR		(1 << 19)
#define NFC_DATA_TRANS		(1 << 21)
#define NFC_SEND_CMD1		(1 << 22)
#define NFC_WAIT_FLAG		(1 << 23)

#define NFC_REG_ADDR_LOW	0x01c03014
#define NFC_REG_ADDR_HIGH	0x01c03018

#define NFC_REG_CNT		0x01c03020

#define NFC_REG_IO_DATA		0x01c03030

#define NFC_RAM0_BASE		0x01c03400

static int read_offset = 0, write_offset = 0;
static uint8_t sunxi_nand_read_byte(struct mtd_info *mtd)
{
	uint8_t data = readb(NFC_RAM0_BASE + read_offset);
	read_offset++;
	return data;
}

static void sunxi_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	read_offset = 0;
	for (i = 0; i < len; i++)
		buf[i] = readb(NFC_REG_IO_DATA);
	read_offset = 0;
}

static void sunxi_nand_write_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
}


static int sunxi_nand_dev_ready(struct mtd_info *mtd)
{
	return (readl(NFC_REG_ST) & NFC_RB_STATE0) ? 1: 0;
}

static void sunxi_nand_select_chip(struct mtd_info *mtd, int chip)
{
	u32 ctl;
	ctl = readl(NFC_REG_CTL);
	ctl &= ~NFC_CE_SEL;
	ctl |= ((chip & 7) << 24);
	writel(ctl, NFC_REG_CTL);
}


static void do_nand_cmd(int command, int column, int page_addr)
{
	int addr_cycle, wait_rb_flag, data_fetch_flag, byte_count;
	addr_cycle = wait_rb_flag = data_fetch_flag = 0;
	u32 cfg = command, ctl;
	while(readl(NFC_REG_ST) & NFC_CMD_FIFO_STATUS);
	switch(command) {
	case NAND_CMD_READ0:
		addr_cycle = 5;
		data_fetch_flag = 0;
		byte_count = 0;
		ctl = column & 0xffff;
		ctl |= ((page_addr & 0xffff) << 16);
		writel(ctl, NFC_REG_ADDR_LOW);
		writel((page_addr >> 16) & 0xff, NFC_REG_ADDR_HIGH);
		break;
	case NAND_CMD_READID:
		writel(0, NFC_REG_ADDR_LOW);
		writel(0, NFC_REG_ADDR_HIGH);
		addr_cycle = 1;
		data_fetch_flag = 1;
		byte_count = 8;
		break;
	case NAND_CMD_RESET:
		break;
	case NAND_CMD_READSTART:
		addr_cycle = 0;
		writel(0, NFC_REG_ADDR_LOW);
		writel(0, NFC_REG_ADDR_HIGH);
		data_fetch_flag = 1;
		byte_count = 8192;
		break;
	case NAND_CMD_RNDOUT:
		addr_cycle = 2;
		writel(column & 0xffff, NFC_REG_ADDR_LOW);
		writel(0, NFC_REG_ADDR_HIGH);
		break;
	case NAND_CMD_RNDOUTSTART:
		data_fetch_flag = 1;
		byte_count = 218;
		break;
	default:
		break;

	}
	cfg |= (addr_cycle << 16); /*  addr cycle */
	if (addr_cycle > 0)
		cfg |= NFC_SEND_ADR;
	if (wait_rb_flag)
		cfg |= NFC_WAIT_FLAG;
	if (data_fetch_flag) {
		u32 ctl;
		ctl = readl(NFC_REG_CTL);
		clrbits_le32(cfg, NFC_RAM_METHOD);
		writel(ctl, NFC_REG_CTL);
		cfg |= NFC_DATA_TRANS;
		writel(byte_count, NFC_REG_CNT);
	}
	cfg |= NFC_SEND_CMD1;
	writel(cfg, NFC_REG_CMD);
	while(readl(NFC_REG_ST) & NFC_CMD_FIFO_STATUS);
	/* Waiting for interrupt flag to be set */
	while(!(readl(NFC_REG_ST) & NFC_CMD_INT_FLAG));
	/* Clearing interrupt if any */
	writel(readl(NFC_REG_ST) & NFC_CMD_INT_FLAG, NFC_REG_ST);
}

static void sunxi_nand_command(struct mtd_info *mtd, unsigned command,
		int column, int page_addr)
{
	u32 cfg = command;
	int i;
	struct nand_chip *nand = mtd->priv;
	debug("nand command = %u, col %d. page_addr %d\n", command, column, page_addr);
	switch(command) {
	case NAND_CMD_RESET:
	case NAND_CMD_READID:
		do_nand_cmd(command, column, page_addr);
		break;
	case NAND_CMD_READOOB:
		do_nand_cmd(NAND_CMD_READ0, column + mtd->writesize, page_addr);
		do_nand_cmd(NAND_CMD_READSTART, -1, -1);
		break;
	case NAND_CMD_READ0:
		do_nand_cmd(NAND_CMD_READ0, column, page_addr);
		do_nand_cmd(NAND_CMD_READSTART, -1, -1);
		break;
	}
#if 0
#if 0
	void _add_cmd_list(NFC_CMD_LIST *cmd,__u32 value,__u32 addr_cycle,__u8 *addr,__u8 data_fetch_flag,
			__u8 main_data_fetch,__u32 bytecnt,__u8 wait_rb_flag);
#endif
	addr_cycle = wait_rb_flag = data_fetch_flag = 0;
	switch(command) {
#if 0
	case NAND_CMD_RNDOUT:
		_add_cmd_list(cmd_list + 1,0x05,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE);
	case NAND_CMD_STATUS:
		_add_cmd_list(&cmd_list, 0x70, 0, addr, 1,NFC_IGNORE,1,NFC_IGNORE);
	case NAND_CMD_STATUS_MULTI:
		_add_cmd_list(&cmd_list, 0x71, 0, addr, 1,NFC_IGNORE,1,NFC_IGNORE);
	case NAND_CMD_READ1:
#endif
	case NAND_CMD_READ0:
#if 0
		_cal_addr_in_chip(readop->block,readop->page,0,addr,5);
		_add_cmd_list(cmd_list,0x00,5,addr,NFC_NO_DATA_FETCH,NFC_IGNORE,NFC_IGNORE,NFC_NO_WAIT_RB);
	case NAND_CMD_READOOB:
#endif
	case NAND_CMD_READID:
		writel(0, NFC_REG_ADDR_LOW);
		writel(0, NFC_REG_ADDR_HIGH);
		addr_cycle = 1;
		data_fetch_flag = 1;
		byte_count = 8;
		break;
#if 0
		_add_cmd_list(&cmd, 0x90,1 , &addr, NFC_DATA_FETCH, NFC_IGNORE, 6, NFC_NO_WAIT_RB);
	case NAND_CMD_PARAM:
	case NAND_CMD_ERASE1:
		_add_cmd_list(cmd_list+i,0x60,3,addr[i],NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE)
	case NAND_CMD_ERASE2:
		_add_cmd_list(cmd_list + i,0xd0,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE);
	case NAND_CMD_SEQIN:
#endif
	case NAND_CMD_PAGEPROG:
		break;
#if 0
		_add_cmd_list(cmd_list+1,0x10, 0,NFC_IGNORE,NFC_NO_DATA_FETCH,NFC_IGNORE, NFC_IGNORE,NFC_IGNORE);
#endif
	case NAND_CMD_RNDOUTSTART:
		break;
#if 0
		_add_cmd_list(cmd_list + 2,0xe0,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE);
#endif
	case NAND_CMD_READSTART:
		break;
#if 0
		_add_cmd_list(cmd_list + 3,0x30,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE,NFC_IGNORE);
#endif
	case NAND_CMD_RESET:
		break;
	}
#if 0
	/* ??? */
	_add_cmd_list(&cmd, 0xed,1 , &addr, NFC_DATA_FETCH, NFC_IGNORE, 32, NFC_WAIT_RB);
#endif
#endif
	read_offset = 0;
	write_offset = 0;
}

static void sunxi_nand_write_byte(struct mtd_info *mtd, uint8_t data)
{
}

#define NAND_MAX_CLOCK (10 * 1000000)
static void sunxi_pll5_nand_clock(void)
{
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	u32 ctl, div_p, div_n = 0, div_m,
	    clock, nand_clk_divid_ratio, edo_clk;
	clock = clock_get_pll5();
	edo_clk = NAND_MAX_CLOCK * 2;
	/*FIXME: Ugly :( */
	nand_clk_divid_ratio = clock / edo_clk;
	if (clock % edo_clk)
		nand_clk_divid_ratio++;
	for (div_m = nand_clk_divid_ratio; div_m > 16 && div_n < 3; div_n++) {
		if (div_m % 2)
			div_m++;
		div_m >>= 1;
	}
	div_m--;
	if (div_m > 15)
		div_m = 15;	/* Overflow */
	/* nand clock source is PLL5 */
	/* TODO: define proper clock sources for NAND reg */
	clrsetbits_le32(&ccm->nand_sclk_cfg, 3 << 24, 2 << 24); /* 2 = PLL5 */
	clrsetbits_le32(&ccm->nand_sclk_cfg, 3 << 16, div_n << 16);
	clrsetbits_le32(&ccm->nand_sclk_cfg, 0xf << 0, div_m << 0);
	setbits_le32(&ccm->nand_sclk_cfg, (1 << 31));
	/* open clock for nand */
	setbits_le32(&ccm->ahb_gate0, (1 << AHB_GATE_OFFSET_NAND));
	debug("NAND Clock: PLL5=%dHz, divid_ratio=%d, n=%d m=%d, clock=%dHz\n", clock, nand_clk_divid_ratio, div_n, div_m, (clock>>div_n)/(div_m+1));
}

int board_nand_init(struct nand_chip *nand)
{
	u32 ctl;

	debug("board_nand_init\n");
	sunxi_pll5_nand_clock();
	ctl = readl(NFC_REG_ECC_CTL);
	ctl &= ~NFC_ECC_MODE;
	writel(ctl, NFC_REG_ECC_CTL);
	ctl = readl(NFC_REG_CTL);
	ctl |= NFC_RESET;
	writel(ctl, NFC_REG_CTL);
	while(readl(NFC_REG_CTL) & NFC_RESET);
	ctl = NFC_EN;
	ctl |= ( (0 & 0x1) << 2); /* Bus width */
	ctl |= ( (0 & 0x1) << 6); /* CE_CTL */
	ctl |= ( (0 & 0x1) << 7); /* CE_CTL1 */
	ctl |= ( 0x1 << 8 );
	ctl |= ((0 & 0x3) << 18); /* DDR_TYPE */
	ctl |= ((0 & 0x1) << 31); /* DEBUG */
	writel(ctl, NFC_REG_CTL);
	ctl = (1 << 8); /* serial_access_mode = 1 */
	writel(ctl, NFC_REG_TIMING_CTL);
	writel(0xff, NFC_REG_TIMING_CFG);
	writel(0x800, NFC_REG_SPARE_AREA); /* Controller SRAM area where spare data should get accumulated during DMA data transfer */
	ctl = readl(NFC_REG_ECC_CTL);
	ctl &= ~NFC_RANDOM_EN;
	writel(ctl, NFC_REG_ECC_CTL);
	/* Selecting chip 0 */
	ctl = readl(NFC_REG_CTL);
	ctl &= ~NFC_CE_SEL;
	ctl |= ((0 & 7) << 24); /* Chip 0 */
	writel(ctl, NFC_REG_CTL);
	/* Waiting for command fifo to be empty */
	while(readl(NFC_REG_ST) & NFC_CMD_FIFO_STATUS);
	/* Sending reset command */
	ctl = 0xff;
	ctl |= NFC_SEND_CMD1;
	writel(ctl, NFC_REG_CMD);
	while(readl(NFC_REG_ST) & NFC_CMD_FIFO_STATUS);
	/* Waiting for interrupt flag to be set */
	while(!(readl(NFC_REG_ST) & NFC_CMD_INT_FLAG));
	/* Clearing interrupt if any */
	writel(readl(NFC_REG_ST) & NFC_CMD_INT_FLAG, NFC_REG_ST);
	/* Select ready/busy 0 */
	nand->ecc.mode = NAND_ECC_SOFT;
	nand->chip_delay = 20;
	nand->dev_ready = sunxi_nand_dev_ready;
	nand->cmdfunc = sunxi_nand_command;
	nand->select_chip = sunxi_nand_select_chip;
	nand->read_buf = sunxi_nand_read_buf;
	nand->write_buf = sunxi_nand_write_buf;
	nand->IO_ADDR_R = NFC_RAM0_BASE + 8;
	nand->IO_ADDR_W = NFC_RAM0_BASE + 8;
	nand->read_byte = sunxi_nand_read_byte;
	return 0;
}
