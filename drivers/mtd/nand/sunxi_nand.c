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

#define NFC_REG_CTL		0x01c03000
#define NFC_EN			(1 << 0)
#define NFC_RESET		(1 << 1)
#define NFC_RB_SEL		(1 << 3)
#define NFC_RAM_METHOD		(1 << 14)
#define NFC_CE_SEL		(7 << 24)

#define NFC_REG_ECC_CTL 	0x01c03034
#define NFC_ECC_EN		(1 << 0)
#define NFC_ECC_PIPELINE	(1 << 3)
#define NFC_ECC_EXCEPTION	(1 << 4)
#define NFC_ECC_BLOCK_SIZE	(1 << 5)
#define NFC_RANDOM_EN		(1 << 9)
#define NFC_RANDOM_DIRECTION	(1 << 10)
#define NFC_ECC_MODE		(0xf << 12)
#define NFC_RANDOM_SEED		(0x7fff << 16)

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
#define NFC_SEND_CMD2		(1 << 24)
#define NFC_DATA_SWAP_METHOD	(1 << 26)

#define NFC_REG_ADDR_LOW	0x01c03014
#define NFC_REG_ADDR_HIGH	0x01c03018

#define NFC_REG_CNT		0x01c03020

#define NFC_REG_IO_DATA		0x01c03030

#define NFC_RAM0_BASE		0x01c03400

#define NFC_REG_RCMD_SET	0x01c03028
#define NFC_REG_WCMD_SET	0x01c0302c

#define NFC_REG_SECTOR_NUM	0x01c0301c

#include "sunxi_debug_reg.c"

static int read_offset = 0, write_offset = 0;
static uint8_t page_buffer[8192 + 1024];
static uint8_t sunxi_nand_read_byte(struct mtd_info *mtd)
{
	uint8_t data = readb(NFC_RAM0_BASE + read_offset);
	read_offset++;
	return data;
}

static void sunxi_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	memcpy(buf, &page_buffer[0], len);
	read_offset = 0;
	memset(page_buffer, 0, sizeof(page_buffer));
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
	debug("nand_select_chip(%d)\n", chip);
	u32 ctl;
	ctl = readl(NFC_REG_CTL);
	ctl &= ~NFC_CE_SEL;
	ctl |= ((chip & 7) << 24);
	writel(ctl, NFC_REG_CTL);
}


static void sunxi_nand_select_rb(int rb)
{
	u32 ctl;

	debug("nand_select_rb(%d)\n", rb);
	ctl = readl(NFC_REG_CTL);
	ctl &= ( (~NFC_RB_SEL) & 0xffffffff);
	ctl |= ((rb & 0x1) << 3);
	writel(ctl, NFC_REG_CTL);
}

static int sunxi_nand_check_rb(int rb)
{
        if (readl(NFC_REG_ST) & ((NFC_RB_STATE0 << (rb & 0x3))))
		return 0;
	else
		return 1;
}
static unsigned sunxi_nand_random_seed(int page)
{
	static const unsigned short random_seed[128] = {
    0x2b75, 0x0bd0, 0x5ca3, 0x62d1, 0x1c93, 0x07e9, 0x2162, 0x3a72, 0x0d67, 0x67f9, 
    0x1be7, 0x077d, 0x032f, 0x0dac, 0x2716, 0x2436, 0x7922, 0x1510, 0x3860, 0x5287, 
    0x480f, 0x4252, 0x1789, 0x5a2d, 0x2a49, 0x5e10, 0x437f, 0x4b4e, 0x2f45, 0x216e, 
    0x5cb7, 0x7130, 0x2a3f, 0x60e4, 0x4dc9, 0x0ef0, 0x0f52, 0x1bb9, 0x6211, 0x7a56, 
    0x226d, 0x4ea7, 0x6f36, 0x3692, 0x38bf, 0x0c62, 0x05eb, 0x4c55, 0x60f4, 0x728c, 
    0x3b6f, 0x2037, 0x7f69, 0x0936, 0x651a, 0x4ceb, 0x6218, 0x79f3, 0x383f, 0x18d9, 
    0x4f05, 0x5c82, 0x2912, 0x6f17, 0x6856, 0x5938, 0x1007, 0x61ab, 0x3e7f, 0x57c2, 
    0x542f, 0x4f62, 0x7454, 0x2eac, 0x7739, 0x42d4, 0x2f90, 0x435a, 0x2e52, 0x2064, 
    0x637c, 0x66ad, 0x2c90, 0x0bad, 0x759c, 0x0029, 0x0986, 0x7126, 0x1ca7, 0x1605, 
    0x386a, 0x27f5, 0x1380, 0x6d75, 0x24c3, 0x0f8e, 0x2b7a, 0x1418, 0x1fd1, 0x7dc1, 
    0x2d8e, 0x43af, 0x2267, 0x7da3, 0x4e3d, 0x1338, 0x50db, 0x454d, 0x764d, 0x40a3, 
    0x42e6, 0x262b, 0x2d2e, 0x1aea, 0x2e17, 0x173d, 0x3a6e, 0x71bf, 0x25f9, 0x0a5d, 
    0x7c57, 0x0fbe, 0x46ce, 0x4939, 0x6b17, 0x37bb, 0x3e91, 0x76db 
	};

	return random_seed[page % 128 ];
}

static void sunxi_nand_enable_random(int page)
{
#if RANDOMIZE_IO_DATA
	u32 ctl = readl(NFC_REG_ECC_CTL);
	ctl |= NFC_RANDOM_EN;
	ctl &= ~NFC_RANDOM_DIRECTION;
	ctl &= ~NFC_RANDOM_SEED;
	ctl |= sunxi_nand_random_seed(page) << 16;
	writel(ctl, NFC_REG_ECC_CTL);
#endif
}

static void sunxi_nand_disable_random(void)
{
	u32 ctl = readl(NFC_REG_ECC_CTL);
	ctl &= ~NFC_RANDOM_EN;
	writel(ctl, NFC_REG_ECC_CTL);
}

/* Command - command to do
 * column - address within page
 * page_addr = page number (row)*/

static void do_nand_cmd(int command, int column, int page_addr)
{
	int addr_cycle, wait_rb_flag, data_fetch_flag, byte_count;
	addr_cycle = wait_rb_flag = data_fetch_flag = 0;
	u32 cfg = command, ctl;
	while(readl(NFC_REG_ST) & NFC_CMD_FIFO_STATUS);
	sunxi_nand_select_rb(0);
	switch(command) {
	case NAND_CMD_READ0:
		addr_cycle = 5;
		data_fetch_flag = 1;
		byte_count = 1024;
		wait_rb_flag = 1;
		ctl = column & 0xffff;
		ctl |= ((page_addr & 0xffff) << 16);
		writel(ctl, NFC_REG_ADDR_LOW);
		writel((page_addr >> 16) & 0xff, NFC_REG_ADDR_HIGH);
		ctl = 0;
		ctl |= 0x30 << 0;
		ctl |= 0x05 << 8;
		ctl |= 0xe0 << 16;
		writel(ctl, NFC_REG_RCMD_SET);
		cfg |= NFC_SEND_CMD2;
		clrsetbits_le32(cfg, (3<< 30), (2 << 30)); /* page command */
		writel(8, NFC_REG_SECTOR_NUM);
		break;
	case NAND_CMD_PARAM:
		writel(column, NFC_REG_ADDR_LOW);
		writel(0, NFC_REG_ADDR_HIGH);
		addr_cycle = 1;
		data_fetch_flag = 1;
		byte_count = 1024;
		wait_rb_flag = 1;
		break;
	case NAND_CMD_READID:
		writel(column, NFC_REG_ADDR_LOW);
		writel(0, NFC_REG_ADDR_HIGH);
		addr_cycle = 1;
		data_fetch_flag = 1;
		byte_count = 8;
		break;
	case NAND_CMD_RESET:
		break;
	case NAND_CMD_RNDOUT:
		addr_cycle = 2;
		writel(column & 0xffff, NFC_REG_ADDR_LOW);
		writel(0, NFC_REG_ADDR_HIGH);
		writel(0xE0, NFC_REG_RCMD_SET);
		data_fetch_flag = 1;
		wait_rb_flag = 0;
		byte_count = 0x400;
		cfg |= NFC_SEND_CMD2;
		break;
	default:
		break;

	}
	if (addr_cycle > 0) {
		cfg |= NFC_SEND_ADR;
		cfg |= ((addr_cycle - 1) << 16); /*  addr cycle */
	}
	if (wait_rb_flag)
		cfg |= NFC_WAIT_FLAG;
	if (data_fetch_flag) {
		u32 ctl;
		ctl = readl(NFC_REG_CTL);
		if (ctl & NFC_RAM_METHOD) {
			clrbits_le32(cfg, NFC_RAM_METHOD);
			writel(ctl, NFC_REG_CTL);
		}
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
	switch(command) {
	case NAND_CMD_RESET:
		/*wait rb0 ready*/
		sunxi_nand_select_rb(0);
		while(sunxi_nand_check_rb(0));

		/*wait rb1 ready*/
		sunxi_nand_select_rb(1);
		while(sunxi_nand_check_rb(1));
		break;
	}
}

static void sunxi_nand_command(struct mtd_info *mtd, unsigned command,
		int column, int page_addr)
{
	u32 cfg = command;
	int i, j;
	struct nand_chip *nand = mtd->priv;
	int bufloc, dlen;
	debug("nand command = %x, col %d. page_addr %d\n", command, column, page_addr);
	sunxi_nand_disable_random();
	switch(command) {
	case NAND_CMD_RESET:
	case NAND_CMD_READID:
	case NAND_CMD_PARAM:
		do_nand_cmd(command, column, page_addr);
		break;
	case NAND_CMD_READOOB:
		printf("reading OOB\n");
		memset(page_buffer, 0, sizeof(page_buffer));
		do_nand_cmd(NAND_CMD_READ0, column + mtd->writesize, page_addr);
		for (j = 0; j < 1024; j++)
			page_buffer[j] = nand->read_byte(mtd);
		read_offset = 0;
		break;
	case NAND_CMD_READ0:
		bufloc = 0;
		dlen = mtd->writesize;
		memset(page_buffer, 0, sizeof(page_buffer));
		sunxi_nand_enable_random(page_addr);
		do_nand_cmd(NAND_CMD_READ0, column, page_addr);
		for (j = 0; j < 1024; j++)
			page_buffer[bufloc++] = nand->read_byte(mtd);
		read_offset = 0;
		for (i = 1; i < mtd->writesize / 1024 + 1; i++) {
			do_nand_cmd(NAND_CMD_RNDOUT, column + i * 1024, page_addr);
			for (j = 0; j < 1024; j++)
				page_buffer[bufloc++] = nand->read_byte(mtd);
			read_offset = 0;
		}
		break;
	default:
		debug("sunxi_nand_command: Unhandled command %02x!\n", command);
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

static void sunxi_nand_set_clock(int hz)
{
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	int clock = clock_get_pll5();
	int edo_clk = hz * 2;
	int div_n = 0, div_m;
	int nand_clk_divid_ratio = clock / edo_clk;

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
	debug("NAND Clock: PLL5=%dHz, divid_ratio=%d, n=%d m=%d, clock=%dHz (target %dHz\n", clock, nand_clk_divid_ratio, div_n, div_m, (clock>>div_n)/(div_m+1), hz);
}

int board_nand_init(struct nand_chip *nand)
{
	u32 ctl;

	debug("board_nand_init\n");
	sunxi_nand_set_clock(NAND_MAX_CLOCK);
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
	ctl |= ( 0x1 << 8 );	  /* PAGE_SIZE = 2K */ /* Needs to be reset to actual page size */
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
