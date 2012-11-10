#include <common.h>
#include <nand.h>
#include <asm/io.h>
#include <asm/gpio.h>

#define NFC_REG_CTL	0x01c03000
#define NFC_RESET	(1 << 1)

#define NFC_REG_ECC_CTL 0x01c03034
#define NFC_ECC_MODE	(0xf << 12)

int board_nand_init(struct nand_chip *nand)
{
	u32 ctl;
	/* NRB1 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(7), 2);
	/* NRB0 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(6), 2);
	/* NRE# */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(5), 2);
	/* NCE0 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(4), 2);
	/* NCE1 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(3), 2);
	/* NCLE */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(2), 2);
	/* NALE */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(1), 2);
	/* NWE */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(0), 2);
	/* NDQ7 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(15), 2);
	/* NDQ6 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(14), 2);
	/* NDQ5 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(13), 2);
	/* NDQ4 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(12), 2);
	/* NDQ3 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(11), 2);
	/* NDQ2 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(10), 2);
	/* NDQ1 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(9), 2);
	/* NDQ0 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(8), 2);
	/* NCE7 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(22), 2);
	/* NCE7 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(22), 2);
	/* NCE6 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(21), 2);
	/* NCE5 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(20), 2);
	/* NCE4 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(19), 2);
	/* NCE3 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(18), 2);
	/* NCE2 */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(17), 2);
	/* NWP */
	sunxi_gpio_set_cfgpin(SUNXI_GPC(16), 2);
	ctl = readl(NFC_REG_ECC_CTL);
	ctl &= ~NFC_ECC_MODE;
	writel(ctl, NFC_REG_ECC_CTL);
	ctl = readl(NFC_REG_CTL);
	ctl |= NFC_RESET;
	writel(ctl, NFC_REG_CTL);
	while(readl(NFC_REG_CTL) & NFC_RESET);
	return 1;
}

