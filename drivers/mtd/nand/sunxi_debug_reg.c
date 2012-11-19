#ifdef DEBUG
static void debug_reg(const char *op, const char *regname, void *reg, uint32_t value)
{
	uint32_t v = value;
	debug("%s %s=%08x", op, regname, value);
#if 0
#define BITS(name, mask, shift) if (mask == 1) debug(" %s%s",v&(mask<<shift)?"":"#", #name); else debug(" %s=%x", #name, (v>>shift)&mask); v&=~(mask<<shift)
	switch((unsigned long)reg) {
	case NFC_REG_CTL:
		BITS(EN, 1, 0);
		BITS(RESET, 1, 1);
		BITS(BUS_WIDTH, 1, 2);
		BITS(RB_SEL, 1, 3);
		BITS(CE_CTL, 1, 6);
		BITS(CE_CTL1, 1, 7);
		BITS(PAGE_SIZE, 0xf, 8);
		BITS(SAM, 1, 12);
		BITS(RAM_METHOD, 1, 14);
		BITS(CE_SEL, 7, 24);
		goto rest;
	case NFC_REG_ECC_CTL:
		BITS(ECC_EN, 1, 0);
		if (value & NFC_ECC_EN) {
			BITS(ECC_PIPELINE, 1, 3);
			BITS(ECC_EXCEPTION, 1, 4);
			BITS(ECC_BLOCK_SIZE, 1, 5);
			BITS(ECC_MODE, 0xf, 12);
		}
		BITS(RANDOM_EN, 1, 9);
		if (value & NFC_RANDOM_EN)  {
			BITS(RANDOM_DIRECTION, 1, 10);
			BITS(RANDOM_SEED, 0x7fff, 16);
		}
		goto rest;
	case NFC_REG_ST:
		BITS(RB_B2R, 1, 0);
		BITS(CMD_INT, 1, 1);
		BITS(DMA_INT, 1, 2);
		BITS(CMD_FIFO_STATUS, 1, 3);
		BITS(STA, 1, 4);
		BITS(NATCH_INT, 1, 5);
		BITS(CMD_RB_STATE0, 1, 8);
		BITS(CMD_RB_STATE1, 1, 9);
		BITS(CMD_RB_STATE2, 1, 10);
		BITS(CMD_RB_STATE3, 1, 11);
		goto rest;
	case NFC_REG_CMD:
		BITS(CMD1, 0xff, 0);
		BITS(CMD_HI, 0xff, 8);
		BITS(SEND_ADR, 1, 19);
		if (value & NFC_SEND_ADR) {
			BITS(ADR_NUM, 0x7, 16);
		}
		BITS(ACCESS_DIR, 1, 20);
		BITS(DATA_TRANS, 1, 21);
		BITS(SEND_CMD1, 1, 22);
		BITS(WAIT_FLAG, 1, 23);
		BITS(SEND_CMD2, 1, 24);
		BITS(SEQ, 1, 25);
		BITS(DATA_SWAP_METHOD, 1, 26);
		BITS(ROW_AUTO_INC, 1, 27);
		BITS(SEND_CMD3, 1, 28);
		BITS(SEND_CMD4, 1, 29);
		BITS(CMD_TYPE, 3, 30);
		goto rest;
	case NFC_REG_RCMD_SET:
		BITS(CMD2, 0xff, 0);
		BITS(CMDS1, 0xff, 8);
		BITS(CMDS2, 0xff, 16);
		goto rest;
	case NFC_REG_WCMD_SET:
		BITS(PROGRAM_CMD, 0xff, 0);
		BITS(RANDOM_WRITE_CMD, 0xff, 8);
		BITS(READ_CMD0, 0xff, 16);
		BITS(READ_CMD1, 0xff, 24);
	default:
		break;
	rest:
		if(v)
			debug(" OTHER=%x", v);
		break;
	}
#endif
	debug("\n");
}

static uint32_t debug_readl(const char *name, void *addr) {
	uint32_t r = readl(addr);
	debug_reg("READ", name, addr, r);
	return r;
}

static void debug_writel(const char *name, uint32_t r, void *addr) {
	debug_reg("WRITE", name, addr, r);
	writel(r, addr);
}
#undef readl
#define readl(addr)		debug_readl(#addr, (void *)addr)
#undef writel
#define writel(value, addr)	debug_writel(#addr, value, (void *)addr)
#endif
