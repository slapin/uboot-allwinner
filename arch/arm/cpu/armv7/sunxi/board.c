/*
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Some init for sunxi platform.
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

#include <linux/compiler.h>
#include <common.h>
#include <asm/io.h>
#include <serial.h>
#include <asm/arch/clock.h>
#include <asm/arch/timer.h>
#include <asm/arch/gpio.h>
#include <asm/arch/key.h>
#include <asm/arch/dram.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/early_print.h>
#include <version.h>
#include <mmc.h>
#include <fat.h>

#ifdef CONFIG_SPL_BUILD
DECLARE_GLOBAL_DATA_PTR;

/* Define global data structure pointer to it*/
static gd_t gdata __attribute__ ((section(".data")));
static bd_t bdata __attribute__ ((section(".data")));
#endif

/* The sunxi internal brom will try to loader external bootloader
 * from mmc0, nannd flash, mmc2.
 * We check where we boot from by checking the config
 * of the gpio pin.
 */
sunxi_boot_type_t boot_from(void) {

	u32 cfg;

	cfg = sunxi_gpio_get_cfgpin(SUNXI_GPC(7));
	if( cfg == SUNXI_GPC7_SDC2_CLK )
		return SUNXI_BOOT_TYPE_MMC2;

	cfg = sunxi_gpio_get_cfgpin(SUNXI_GPC(2));
	if( cfg == SUNXI_GPC2_NCLE )
		return SUNXI_BOOT_TYPE_NAND;

	cfg = sunxi_gpio_get_cfgpin(SUNXI_GPF(2));
	if( cfg == SUNXI_GPF2_SDC0_CLK )
		return SUNXI_BOOT_TYPE_MMC2;

	/* if we are here, something goes wrong */
	return SUNXI_BOOT_TYPE_NULL;
}

int watchdog_init(void) {

	struct sunxi_wdog *wdog =
		&((struct sunxi_timer_reg *)SUNXI_TIMER_BASE)->wdog;
	/* disable watchdog */
	writel(0, &(wdog->mode));

	return 0;
}

int gpio_init(void) {

	/* config uart pin */
	sunxi_gpio_set_cfgpin(SUNXI_GPB(22), SUNXI_GPB22_UART0_TX);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(23), SUNXI_GPB23_UART0_RX);

	return 0;
}

/* do some early init */
void s_init(void) {

	watchdog_init();
	clock_init();
	gpio_init();

#ifdef CONFIG_SPL_BUILD
	sunxi_dram_init();
#endif
}

extern void sunxi_reset(void);
void reset_cpu(ulong addr) {

	sunxi_reset();
}

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void) {

	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif

#ifdef CONFIG_SPL_BUILD
void save_boot_params(u32 r0, u32 r1, u32 r2, u32 r3) {}

inline void hang(void)
{
	puts("\n### ERROR ### Please RESET the board ###\n");
	for (;;)
		;
}

void board_init_f(unsigned long bootflag)
{
	/*
	 * We call relocate_code() with relocation target same as the
	 * CONFIG_SYS_SPL_TEXT_BASE. This will result in relocation getting
	 * skipped. Instead, only .bss initialization will happen. That's
	 * all we need
	 */
	relocate_code(CONFIG_SPL_STACK, &gdata, CONFIG_SPL_TEXT_BASE);
}

struct spl_image_info {
	const char *name;
	u8 os;
	u32 load_addr;
	u32 entry_point;
	u32 size;
};

struct spl_image_info spl_image;
u32* boot_params_ptr = NULL;

void spl_parse_image_header(const struct image_header *header)
{
	u32 header_size = sizeof(struct image_header);

	if (__be32_to_cpu(header->ih_magic) == IH_MAGIC) {
		spl_image.size = __be32_to_cpu(header->ih_size) + header_size;
		spl_image.entry_point = __be32_to_cpu(header->ih_load);
		/* Load including the header */
		spl_image.load_addr = spl_image.entry_point - header_size;
		spl_image.os = header->ih_os;
		spl_image.name = (const char *)&header->ih_name;
		debug("spl: payload image: %s load addr: 0x%x size: %d\n",
			spl_image.name, spl_image.load_addr, spl_image.size);
	} else {
		/* Signature not found - assume u-boot.bin */
		printf("mkimage signature not found - ih_magic = %x\n",
			header->ih_magic);
		debug("Assuming u-boot.bin ..\n");
		/* Let's assume U-Boot will not be more than 512 KB */
		spl_image.size = CONFIG_MMC_U_BOOT_SECTOR_COUNT * 512;
		spl_image.entry_point = CONFIG_SYS_TEXT_BASE;
		spl_image.load_addr = CONFIG_SYS_TEXT_BASE;
		spl_image.os = IH_OS_U_BOOT;
		spl_image.name = "U-Boot";
	}
}

/*
 * This function jumps to an image with argument. Normally an FDT or ATAGS
 * image.
 * arg: Pointer to paramter image in RAM
 */
#ifdef CONFIG_SPL_OS_BOOT
static void __noreturn jump_to_image_linux(void *arg)
{
	debug("Entering kernel arg pointer: 0x%p\n", arg);
	typedef void (*image_entry_arg_t)(int, int, void *)
		__attribute__ ((noreturn));
	image_entry_arg_t image_entry =
		(image_entry_arg_t) spl_image.entry_point;
	cleanup_before_linux();
	image_entry(0, CONFIG_MACH_TYPE, arg);
}
#endif

static void __noreturn jump_to_image_no_args(void)
{
	typedef void __noreturn (*image_entry_noargs_t)(u32 *);
	image_entry_noargs_t image_entry =
			(image_entry_noargs_t) spl_image.entry_point;

	debug("image entry point: 0x%X\n", spl_image.entry_point);
	/* Pass the saved boot_params from rom code */
#if defined(CONFIG_VIRTIO) || defined(CONFIG_ZEBU)
	image_entry = (image_entry_noargs_t)0x80100000;
#endif
	u32 boot_params_ptr_addr = (u32)&boot_params_ptr;
	image_entry((u32 *)boot_params_ptr_addr);
}

#define MMCSD_SECTOR_SIZE 512

static int mmc_load_image_raw(struct mmc *mmc)
{
	u32 image_size_sectors, err;
	const struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	/* read image header to find the image size & load address */
	err = mmc->block_dev.block_read(0,
			CONFIG_MMC_U_BOOT_SECTOR_START, 1,
			(void *)header);

	if (err <= 0)
		goto end;

	spl_parse_image_header(header);

	/* convert size to sectors - round up */
	image_size_sectors = (spl_image.size + MMCSD_SECTOR_SIZE - 1) /
				MMCSD_SECTOR_SIZE;

	/* Read the header too to avoid extra memcpy */
	err = mmc->block_dev.block_read(0,
			CONFIG_MMC_U_BOOT_SECTOR_START,
			image_size_sectors, (void *)spl_image.load_addr);

end:
	if (err <= 0) {
		printf("spl: mmc blk read err - %d\n", err);
		return err;
	}
	return 0;
}

#if defined(CONFIG_SPL_FAT_SUPPORT)
static int mmc_load_image_fat(struct mmc *mmc)
{
	s32 err;
	struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	err = fat_register_device(&mmc->block_dev,
				CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION);
	if (err) {
		printf("spl: fat register err - %d\n", err);
		hang();
	}

	err = file_fat_read(CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME,
				(u8 *)header, sizeof(struct image_header));
	if (err <= 0)
		goto end;

	spl_parse_image_header(header);

	err = file_fat_read(CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME,
				(u8 *)spl_image.load_addr, 0);

end:
	if (err <= 0) {
		printf("spl: error reading image %s, err - %d\n",
			CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME, err);
		return err;
	}
	return 0;
}
#endif

void board_init_r(gd_t *id, ulong dest_addr)
{
	__attribute__((noreturn)) void (*uboot)(void);
	struct mmc *mmc;
	s32 err;
	struct image_header *header;
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	gd = &gdata;
	gd->bd = &bdata;
	gd->flags |= GD_FLG_RELOC;
	gd->baudrate = CONFIG_BAUDRATE;

	timer_init();
	serial_init();

	printf("\nU-Boot SPL %s (%s - %s)\n", PLAIN_VERSION, U_BOOT_DATE,
		U_BOOT_TIME);

	puts("MMC:   ");
	mmc_initialize(gd->bd);
	/* We register only one device. So, the dev id is always 0 */
	mmc = find_mmc_device(0);
	if (!mmc) {
		puts("spl: mmc device not found!!\n");
		hang();
	}

	err = mmc_init(mmc);
	if (err) {
		printf("spl: mmc init failed: err - %d\n", err);
		hang();
	}

	puts("Loading U-Boot...   ");

#if defined(CONFIG_SPL_FAT_SUPPORT)
	err = mmc_load_image_fat(mmc);
	if (err < 0)
#endif
#if defined(CONFIG_MMC_U_BOOT_SECTOR_START)
		err = mmc_load_image_raw(mmc);
	if (err < 0)
#endif
	{
		printf("Failed to load u-boot: err - %d\n", err);
		hang();
	}

	puts("Jumping to U-Boot...\n");
	/* Jump to U-Boot image */
	switch (spl_image.os) {
	case IH_OS_U_BOOT:
		debug("Jumping to U-Boot\n");
		jump_to_image_no_args();
		break;
#ifdef CONFIG_SPL_OS_BOOT
	case IH_OS_LINUX:
		debug("Jumping to Linux\n");
		spl_board_prepare_for_linux();
		jump_to_image_linux((void *)CONFIG_SYS_SPL_ARGS_ADDR);
		break;
#endif
	default:
		puts("Unsupported OS image.. Jumping nevertheless..\n");
		jump_to_image_no_args();
	}
	/* Never returns Here */
}
#endif
