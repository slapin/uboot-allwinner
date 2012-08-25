/*
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Configuration settings for the Allwinner A10-evb board.
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

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 */
#define CONFIG_ALLWINNER				/* It's a Allwinner chip */
#define	CONFIG_SUNXI					/* which is sunxi family */
#define CONFIG_SUN4I					/* which is sun4i */
#define CONFIG_A10_EVB					/* working with A10-EVB board */

#include <asm/arch/cpu.h>				/* get chip and board defs */

#define BOARD_LATE_INIT					/* init the fastboot partitions */

#define CONFIG_SYS_TEXT_BASE		0x4A000000
#if 0
#define CONFIG_SKIP_LOWLEVEL_INIT			/* currently u-boot is loaded from ice */
#endif
/*
 * Display CPU and Board information
 */
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

/* Clock Defines */


/* Serial & console */
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)		/* ns16550 reg in the low bits of cpu reg */
#define CONFIG_SYS_NS16550_CLK		(24000000)
#define CONFIG_SYS_NS16550_COM1		SUNXI_UART0_BASE
#define CONFIG_SYS_NS16550_COM2		SUNXI_UART1_BASE
#define CONFIG_SYS_NS16550_COM3		SUNXI_UART2_BASE
#define CONFIG_SYS_NS16550_COM4		SUNXI_UART3_BASE

#define CONFIG_CONS_INDEX			1	/* which serial channel for console */

/* DRAM Base */
#define CONFIG_SYS_SDRAM_BASE		0x40000000
#define CONFIG_SYS_INIT_RAM_ADDR	(SUNXI_SRAM_A1_BASE)
#define CONFIG_SYS_INIT_RAM_SIZE	(SUNXI_SRAM_A1_SIZE)

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* A10-EVB has 1 banks of DRAM, we use only one in U-Boot */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM_1			CONFIG_SYS_SDRAM_BASE	/* SDRAM Bank #1 */
#define PHYS_SDRAM_1_SIZE		(512 << 20)		/* 0x20000000, 512 MB Bank #1 */

//#define CONFIG_SYS_MONITOR_BASE	0x00000000

/* Nand config */
#define CONFIG_NAND
#define CONFIG_NAND_SUNXI
#define CONFIG_CMD_NAND                         		/* NAND support */
#define CONFIG_SYS_MAX_NAND_DEVICE      1
#define CONFIG_SYS_NAND_BASE            0x00
#define CONFIG_CMD_MEMORY
#define CONFIG_SUNXI_DMA

/* mmc config */
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_CMD_MMC
#define CONFIG_MMC_SUNXI
#define CONFIG_MMC_SUNXI_SLOT		0	/* which mmc slot to use, could be 0,1,2,3 */
#define CONFIG_MMC_SUNXI_USE_DMA

#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_CMDLINE_TAG
#define CONFIG_INITRD_TAG
#define CONFIG_CMDLINE_EDITING
#define CONFIG_DOS_PARTITION

/*
 * Size of malloc() pool
 * 1MB = 0x100000, 0x100000 = 1024 * 1024
 */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (1 << 20))

#define CONFIG_FASTBOOT
#define CONFIG_STORAGE_NAND
#define FASTBOOT_TRANSFER_BUFFER	0x41000000
#define FASTBOOT_TRANSFER_BUFFER_SIZE	256 << 20 /* 256M */

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP			/* undef to save memory */
#define CONFIG_SYS_HUSH_PARSER			/* use "hush" command parser	*/
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_PROMPT		"sun4i#"
#define CONFIG_SYS_CBSIZE	256		/* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE	384		/* Print Buffer Size */
#define CONFIG_SYS_MAXARGS	16		/* max number of command args */

/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

/* memtest works on */
#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_SDRAM_BASE + 256<<20)	/* 256M */
#define CONFIG_SYS_LOAD_ADDR		0x50000000				/* default load address */

#define CONFIG_SYS_HZ			1000

/* valid baudrates */
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE		(256 << 10)	/* 256 KiB */

#define LOW_LEVEL_SRAM_STACK		0x00003FFC


/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
/*------------------------------------------------------------------------
 * we save the environment in a nand partition, the partition name is defined
 * in sysconfig.fex, which must be the same as CONFIG_SUNXI_NAND_ENV_PARTITION
 * if not, below CONFIG_ENV_ADDR and CONFIG_ENV_SIZE will be where to store env.
 * */
#define CONFIG_ENV_ADDR			(256 << 20)
#define CONFIG_ENV_SIZE			(128 << 10)	/* 128KB */
#define CONFIG_MACH_TYPE		4104
#define CONFIG_ENV_OFFSET		(544 << 10) 	/* env starts here */ /* (8 + 24 + 512)KB */

#define CONFIG_SYS_NO_FLASH

#define CONFIG_SYS_MONITOR_LEN		(256 << 10)		/* 256 KiB */
#define CONFIG_IDENT_STRING		" Allwinner Technology "

/*
 * Save the env in NAND
 */

#define CONFIG_ENV_IS_IN_NAND_SUNXI	    			/* we store env in one partition of our nand */
#define CONFIG_SUNXI_ENV_PARTITION	"env"			/* the partition name */

/*
 * Save the env on the MMC
 */
#if 0
#define CONFIG_ENV_IS_IN_MMC					/* we store env on the MMC */
#define CONFIG_SYS_MMC_ENV_DEV		CONFIG_MMC_SUNXI_SLOT	/* env in which mmc */
#endif
/*-----------------------------------------------------------------------
 * Environment default settings
 */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"boot_fastboot=fastboot\0" \
	"boot_initrd=fatload nand 0 0x44000000 linux/${kernel}; fatload nand 0 0x45000000 linux/${initrd}; bootm 0x44000000 0x45000000\0" \
	"boot_mmc=fatload mmc 0 0x48000000 ${kernel} || ext2load mmc 0 0x48000000 ${kernel}; bootm 0x48000000\0" \
	"boot_normal=nand read 40007800 boot;boota 40007800\0" \
	"boot_recovery=nand read 40007800 recovery;boota 40007800\0" \
	"bootargs=console=${console} root=${root} loglevel=${loglevel} panic=${panicarg}\0" \
	"bootcmd_bootscr=if run load_scr; then echo Jumping to ${bootscr}; source ${scriptaddr}; fi\0" \
	"bootcmd_env=if run load_env; then echo Loaded environment from ${bootenv}; env import -t ${scriptaddr} ${filesize}; fi\0" \
	"bootcmd_initrd=run shell_load; run reset_load; run set_modules; run set_root_dev; run set_bootargs; run load_hw; run boot_initrd\0" \
	"bootcmd_mmc=run setargs; run load_fathw; run boot_mmc\0" \
	"bootcmd_nand=run set_nand; if test ${root_dev} = ${nand_root}; then run setargs; run boot_normal; fi\0" \
        "bootcmd_nand_env=run setargs_nand; run boot_normal\0" \
	"bootcmd_scr=run bootcmd_bootscr; run bootcmd_env; run bootcmd_uenv\0" \
	"bootcmd_shell=run set_modules; run setargs_shell; run load_hw; run boot_initrd\0" \
	"bootcmd_uenv=if test -n ${uenvcmd}; then echo Running uenvcmd ...; run uenvcmd; fi\0" \
	"bootconf=linux/conf.d/uboot.conf\0" \
	"bootdelay=3\0" \
	"bootenv=linux/uEnv.txt\0" \
	"bootscr=linux/boot.scr\0" \
	"console=ttyS0,115200\0" \
	"device=tablet\0" \
	"env_reset=linux/conf.d/env_reset\0" \
	"ethaddr=""\0" \
	"hw_conf=linux/conf.d/script.bin\0" \
	"ifup_auto=ip=dhcp\0" \
	"init=/init\0" \
	"initrd=uInitrd\0" \
	"kernel=uImage\0" \
	"load_bootconf=if fatload nand 0 ${scriptaddr} ${bootconf}; then env import -t ${scriptaddr} ${filesize}; fi\0" \
	"load_env=fatload nand 0 ${scriptaddr} ${bootenv}\0" \
	"load_fathw=fatload mmc 0 0x43000000 script.bin || ext2load mmc 0 0x43000000 script.bin" \
	" || ext2load mmc 0 43000000 ${hw_conf}\0" \
	"load_hw=fatload mmc 0 43000000 ${hw_conf}\0" \
	"load_scr=fatload mmc 0 ${scriptaddr} ${bootscr}\0" \
	"loglevel=8\0" \
	"mac=""\0" \
	"mmc_root=/dev/mmcblk0p2\0" \
	"modules=${modules_tablet}\0" \
	"modules_mele=sw_ahci_platform,8192cu,mali,sun4i_wemac\0" \
	"modules_tablet=8192cu,mali\0" \
	"nand_root=/dev/nandd\0" \
	"net_root=""\0" \
	"panicarg=panic=10\0" \
	"params=\0" \
	"rescue=linux/conf.d/rescue_shell\0" \
	"rescue_shell=0\0" \
	"reset_env=run load_bootconf; setenv reset_flag 0; saveenv\0" \
	"reset_flag=0\0" \
	"reset_load=if fatload mmc 0 ${scriptaddr} ${env_reset}; then env import -t ${scriptaddr} ${filesize}; run reset_test; fi\0" \
	"reset_root=setenv root_dev ${root}; setenv root_flag 0\0" \
	"reset_test=if test ${reset_flag} -eq 1; then run reset_env; fi\0" \
	"root=/dev/nandd\0" \
	"root_dev=${root}\0" \
	"root_device=nand\0" \
	"root_flag=1\0" \
	"rootfstype=ext4\0" \
	"sata_root=""\0" \
	"scriptaddr=0x44000000\0" \
	"set_bootargs=setenv bootargs console=${console} root=${root_dev} loglevel=${loglevel}" \
	" mac_addr=${mac} rd.driver.post=${modules} ${params} rootwait\0" \
	"set_mmc=if test ${root_device} = mmc && test -n ${mmc_root}; then setenv root_dev ${mmc_root}; setenv root_flag 1; fi\0" \
	"set_modules=if test ${device} = mele; then setenv modules ${modules_mele}; else setenv modules ${modules_tablet}; fi\0" \
	"set_nand=if test ${root_device} = nand && test -n ${nand_root}; then setenv root_dev ${nand_root}; setenv root_flag 1;" \
	" else setenv root_dev 0; fi\0" \
	"set_net=if test ${root_device} = net && test -n ${net_root}; then setenv root_dev ${net_root}; setenv root_flag 1; fi\0" \
	"set_root=run set_nand; run set_mmc; run set_usb; run set_sata; run set_net\0" \
	"set_root_dev=run reset_root; run set_root; if test ${root_flag} -eq 0; then run reset_root; fi\0" \
	"set_sata=if test ${root_device} = sata && test -n ${sata_root}; then setenv root_dev ${sata_root}; setenv root_flag 1; fi\0" \
	"set_usb=if test ${root_device} = usb && test -n ${usb_root}; then setenv root_dev ${usb_root}; setenv root_flag 1; fi\0" \
	"setargs=setenv bootargs console=${console} root=${root} loglevel=${loglevel} ${panicarg}" \
	" ${extraargs} rootfstype=${rootfstype} rootwait\0" \
	"setargs_nand=setenv bootargs console=${console} root=${nand_root} init=${init} loglevel=${loglevel} mac_addr=${mac}\0" \
	"setargs_shell=setenv bootargs console=${console} root=${root_dev} loglevel=${loglevel}" \
	" rd.driver.post=${modules} mac_addr=${ethaddr} ${ifup_auto} ${params} rd.break\0" \
	"shell_flag=0\0" \
	"shell_load=if fatload mmc 0 ${scriptaddr} ${rescue}; then env import -t ${scriptaddr} ${filesize}; run shell_test; fi\0" \
	"shell_test=if test ${shell_flag} -eq 1; then setenv shell_flag 0; saveenv; run bootcmd_shell; fi\0" \
	"usb_root=""\0" \


#define CONFIG_BOOTDELAY	1
#define CONFIG_BOOTCOMMAND	"run bootcmd_scr; run bootcmd_nand; run bootcmd_initrd; run bootcmd_mmc; reset"
#define CONFIG_SYS_BOOT_GET_CMDLINE
#define CONFIG_AUTO_COMPLETE
/*
 * Basic commands
 */
#define CONFIG_CMD_BOOTA	/* boot android image */
#define CONFIG_CMD_BDI		/* bdinfo			*/
#define CONFIG_CMD_BOOTD	/* bootd boot the default command */
#define CONFIG_CMD_CONSOLE	/* coninfo			*/
#define CONFIG_CMD_DUMPASCII	/* save ascii data over serial */
#define CONFIG_CMD_ECHO		/* echo arguments		*/
#define CONFIG_CMD_ENV
#define CONFIG_CMD_EDITENV	/* editenv			*/
#define CONFIG_CMD_FPGA		/* FPGA configuration Support	*/
#define CONFIG_CMD_IMI		/* iminfo			*/
#define CONFIG_CMD_ITEST	/* Integer (and string) test	*/

#ifndef CONFIG_SYS_NO_FLASH
#define CONFIG_CMD_FLASH	/* flinfo, erase, protect	*/
#define CONFIG_CMD_IMLS		/* List all found images	*/
#endif
	
#define CONFIG_CMD_LOADB	/* loadb			*/
#define CONFIG_CMD_LOADS	/* loads			*/
#define CONFIG_CMD_MEMORY	/* md mm nm mw cp cmp crc base loop mtest */
#define CONFIG_CMD_MISC		/* Misc functions like sleep etc*/
#define CONFIG_CMD_RUN		/* run command in env variable	*/
#define CONFIG_CMD_SAVEENV	/* saveenv			*/
#define CONFIG_CMD_SAVES	/* save data over serial */

#define CONFIG_CMD_SETARCH	/* override board arch */
#define CONFIG_CMD_SETGETDCR	/* DCR support on 4xx		*/
#define CONFIG_CMD_SOURCE	/* "source" command support	*/
#define CONFIG_CMD_XIMG		/* Load part of Multi Image	*/

/*
 * File system commands
 */
#define CONFIG_CMD_EXT2		/* with this we can access ext2/3 filesystem */
#define CONFIG_CMD_FAT		/* with this we can access fat filesystem */
#define CONFIG_CMD_JFFS2	/* with this we can access jffs2 filesystem */
#define CONFIG_LZO

/*
 * Ethernet Driver configuration - not yet working - do not enable
 */
#if 0
#define CONFIG_PHY_BASE_ADR	0
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET				/* bootp, tftpboot, rarpboot	*/
#define CONFIG_CMD_NFS				/* NFS support			*/
#define CONFIG_CMD_PING
#endif

/*
 * SATA configuration - not yet working - do not enable
 */
#if 0
#define CONFIG_SYS_ATA_IDE0_OFFSET	SATA_PORT0_OFFSET
#define CONFIG_CMD_IDE
#endif
/*
 * USB configuration - not yet working - do not enable
 */
#if 0
#define CONFIG_CMD_USB
#endif	

/*
 * MTD configuration - not yet working - do not enable if using sunxi nand commands
 */
#if 0
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_RBTREE
#define CONFIG_MTD_DEVICE               	/* needed for mtdparts commands */
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_MTDPARTS
#endif

/*
 * SPL Loader for MMC boot - not needed for NAND - not yet working - use existing
 */
#if 0
#define CONFIG_SPL

#define CONFIG_SPL_BSS_START_ADDR	0x50000000
#define CONFIG_SPL_BSS_MAX_SIZE		0x80000		/* 512 KB */

#define CONFIG_SPL_TEXT_BASE       	0x0          	/* sram start */
#define CONFIG_SPL_MAX_SIZE        	0x8000       	/* 32 KB */

#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBDISK_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT

#define LOW_LEVEL_SRAM_STACK		0x00006000	/* end of 24KB in sram */
#define CONFIG_SPL_STACK         	LOW_LEVEL_SRAM_STACK
#define CONFIG_SPL_LDSCRIPT 		"arch/arm/cpu/armv7/sunxi/u-boot-spl.lds"

#define CONFIG_MMC_U_BOOT_SECTOR_START	(64)    	/* 32KB offset */
#define CONFIG_MMC_U_BOOT_SECTOR_COUNT	(1000)  	/* 512KB, enough for a full u-boot.bin */
#endif

#endif /* __CONFIG_H */

