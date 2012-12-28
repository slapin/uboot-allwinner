/*
 * Core registration and callback routines for MTD
 * drivers and users.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/mtd/mtd.h>
#include <linux/compat.h>
#include <linux/mtd/partitions.h>
#include <ubi_uboot.h>

struct mtd_info *mtd_table[MAX_MTD_DEVICES];

int add_mtd_device(struct mtd_info *mtd)
{
	int i;

	BUG_ON(mtd->writesize == 0);

	for (i = 0; i < MAX_MTD_DEVICES; i++)
		if (!mtd_table[i]) {
			mtd_table[i] = mtd;
			mtd->index = i;
			mtd->usecount = 0;

			/* No need to get a refcount on the module containing
			   the notifier, since we hold the mtd_table_mutex */

			/* We _know_ we aren't being removed, because
			   our caller is still holding us here. So none
			   of this try_ nonsense, and no bitching about it
			   either. :) */
			/* default value if not set by driver */
			if (mtd->bitflip_threshold == 0)
				mtd->bitflip_threshold = mtd->ecc_strength;

			if (is_power_of_2(mtd->erasesize))
				mtd->erasesize_shift = ffs(mtd->erasesize) - 1;
			else
				mtd->erasesize_shift = 0;

			if (is_power_of_2(mtd->writesize))
				mtd->writesize_shift = ffs(mtd->writesize) - 1;
			else
				mtd->writesize_shift = 0;

			mtd->erasesize_mask = (1 << mtd->erasesize_shift) - 1;
			mtd->writesize_mask = (1 << mtd->writesize_shift) - 1;
			if ((mtd->flags & MTD_WRITEABLE) && (mtd->flags & MTD_POWERUP_LOCK)) {
				int error;
				error = mtd_unlock(mtd, 0, mtd->size);
				if (error && error != -EOPNOTSUPP)
					printk(KERN_WARNING
			       			"%s: unlock failed, writes may not work\n",
			       			mtd->name);
			}
			return 0;
		}

	return 1;
}

/**
 *      del_mtd_device - unregister an MTD device
 *      @mtd: pointer to MTD device info structure
 *
 *      Remove a device from the list of MTD devices present in the system,
 *      and notify each currently active MTD 'user' of its departure.
 *      Returns zero on success or 1 on failure, which currently will happen
 *      if the requested device does not appear to be present in the list.
 */
int del_mtd_device(struct mtd_info *mtd)
{
	int ret;

	if (mtd_table[mtd->index] != mtd) {
		ret = -ENODEV;
	} else if (mtd->usecount) {
		printk(KERN_NOTICE "Removing MTD device #%d (%s)"
				" with use count %d\n",
				mtd->index, mtd->name, mtd->usecount);
		ret = -EBUSY;
	} else {
		/* No need to get a refcount on the module containing
		 * the notifier, since we hold the mtd_table_mutex */
		mtd_table[mtd->index] = NULL;

		ret = 0;
	}

	return ret;
}

/**
 * mtd_device_parse_register - parse partitions and register an MTD device.
 *
 * @mtd: the MTD device to register
 * @types: the list of MTD partition probes to try, see
 *         'parse_mtd_partitions()' for more information
 * @parser_data: MTD partition parser-specific data
 * @parts: fallback partition information to register, if parsing fails;
 *         only valid if %nr_parts > %0
 * @nr_parts: the number of partitions in parts, if zero then the full
 *            MTD device is registered if no partition info is found
 *
 * This function aggregates MTD partitions parsing (done by
 * 'parse_mtd_partitions()') and MTD device and partitions registering. It
 * basically follows the most common pattern found in many MTD drivers:
 *
 * * It first tries to probe partitions on MTD device @mtd using parsers
 *   specified in @types (if @types is %NULL, then the default list of parsers
 *   is used, see 'parse_mtd_partitions()' for more information). If none are
 *   found this functions tries to fallback to information specified in
 *   @parts/@nr_parts.
 * * If any partitioning info was found, this function registers the found
 *   partitions.
 * * If no partitions were found this function just registers the MTD device
 *   @mtd and exits.
 *
 * Returns zero in case of success and a negative error code in case of failure.
 */
int mtd_device_parse_register(struct mtd_info *mtd, const char **types,
			      struct mtd_part_parser_data *parser_data,
			      const struct mtd_partition *parts,
			      int nr_parts)
{
	int err;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *real_parts;

	err = parse_mtd_partitions(mtd, types, &real_parts, parser_data);
	if (err <= 0 && nr_parts && parts) {
		real_parts = kmemdup(parts, sizeof(*parts) * nr_parts,
				     GFP_KERNEL);
		if (!real_parts)
			err = -ENOMEM;
		else
			err = nr_parts;
	}

	if (err > 0) {
		err = add_mtd_partitions(mtd, real_parts, err);
		kfree(real_parts);
	} else if (err == 0) {
		err = add_mtd_device(mtd);
		if (err == 1)
			err = -ENODEV;
	}
#else
	err = add_mtd_device(mtd);
	if (err == 1)
		err = -ENODEV;
#endif

	return err;
}

/**
 * mtd_device_unregister - unregister an existing MTD device.
 *
 * @master: the MTD device to unregister.  This will unregister both the master
 *          and any partitions if registered.
 */
int mtd_device_unregister(struct mtd_info *master)
{
	int err;

#ifdef CONFIG_MTD_PARTITIONS
	err = del_mtd_partitions(master);
	if (err)
		return err;
#endif

	return del_mtd_device(master);
}
/**
 *	get_mtd_device - obtain a validated handle for an MTD device
 *	@mtd: last known address of the required MTD device
 *	@num: internal device number of the required MTD device
 *
 *	Given a number and NULL address, return the num'th entry in the device
 *      table, if any.  Given an address and num == -1, search the device table
 *      for a device with that address and return if it's still present. Given
 *      both, return the num'th driver only if its address matches. Return
 *      error code if not.
 */
struct mtd_info *get_mtd_device(struct mtd_info *mtd, int num)
{
	struct mtd_info *ret = NULL;
	int i, err = -ENODEV;

	if (num == -1) {
		for (i = 0; i < MAX_MTD_DEVICES; i++)
			if (mtd_table[i] == mtd)
				ret = mtd_table[i];
	} else if (num < MAX_MTD_DEVICES) {
		ret = mtd_table[num];
		if (mtd && mtd != ret)
			ret = NULL;
	}

	if (!ret)
		goto out_unlock;

	ret->usecount++;
	return ret;

out_unlock:
	return ERR_PTR(err);
}

/**
 *      get_mtd_device_nm - obtain a validated handle for an MTD device by
 *      device name
 *      @name: MTD device name to open
 *
 *      This function returns MTD device description structure in case of
 *      success and an error code in case of failure.
 */
struct mtd_info *get_mtd_device_nm(const char *name)
{
	int i, err = -ENODEV;
	struct mtd_info *mtd = NULL;

	for (i = 0; i < MAX_MTD_DEVICES; i++) {
		if (mtd_table[i] && !strcmp(name, mtd_table[i]->name)) {
			mtd = mtd_table[i];
			break;
		}
	}

	if (!mtd)
		goto out_unlock;

	mtd->usecount++;
	return mtd;

out_unlock:
	return ERR_PTR(err);
}

void put_mtd_device(struct mtd_info *mtd)
{
	int c;

	c = --mtd->usecount;
	BUG_ON(c < 0);
}
/*
 * Erase is an asynchronous operation.  Device drivers are supposed
 * to call instr->callback() whenever the operation completes, even
 * if it completes with a failure.
 * Callers are supposed to pass a callback function and wait for it
 * to be called before writing to the block.
 */
int mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	if (instr->addr > mtd->size || instr->len > mtd->size - instr->addr)
		return -EINVAL;
	if (!(mtd->flags & MTD_WRITEABLE))
		return -EROFS;
	instr->fail_addr = MTD_FAIL_ADDR_UNKNOWN;
	if (!instr->len) {
		instr->state = MTD_ERASE_DONE;
		mtd_erase_callback(instr);
		return 0;
	}
	return mtd->_erase(mtd, instr);
}

/*
 * This stuff for eXecute-In-Place. phys is optional and may be set to NULL.
 */
int mtd_point(struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen,
	      void **virt, resource_size_t *phys)
{
	*retlen = 0;
	*virt = NULL;
	if (phys)
		*phys = 0;
	if (!mtd->_point)
		return -EOPNOTSUPP;
	if (from < 0 || from > mtd->size || len > mtd->size - from)
		return -EINVAL;
	if (!len)
		return 0;
	return mtd->_point(mtd, from, len, retlen, virt, phys);
}
/* We probably shouldn't allow XIP if the unpoint isn't a NULL */
int mtd_unpoint(struct mtd_info *mtd, loff_t from, size_t len)
{
	if (!mtd->_point)
		return -EOPNOTSUPP;
	if (from < 0 || from > mtd->size || len > mtd->size - from)
		return -EINVAL;
	if (!len)
		return 0;
	return mtd->_unpoint(mtd, from, len);
}

/*
 * Allow NOMMU mmap() to directly map the device (if not NULL)
 * - return the address to which the offset maps
 * - return -ENOSYS to indicate refusal to do the mapping
 */
unsigned long mtd_get_unmapped_area(struct mtd_info *mtd, unsigned long len,
				    unsigned long offset, unsigned long flags)
{
	if (!mtd->_get_unmapped_area)
		return -EOPNOTSUPP;
	if (offset > mtd->size || len > mtd->size - offset)
		return -EINVAL;
	return mtd->_get_unmapped_area(mtd, len, offset, flags);
}

int mtd_read(struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen,
	     u_char *buf)
{
	int ret_code;
	*retlen = 0;
	if (from < 0 || from > mtd->size || len > mtd->size - from)
		return -EINVAL;
	if (!len)
		return 0;

	/*
	 * In the absence of an error, drivers return a non-negative integer
	 * representing the maximum number of bitflips that were corrected on
	 * any one ecc region (if applicable; zero otherwise).
	 */
	ret_code = mtd->_read(mtd, from, len, retlen, buf);
	if (unlikely(ret_code < 0))
		return ret_code;
	if (mtd->ecc_strength == 0)
		return 0;	/* device lacks ecc */
	return ret_code >= mtd->bitflip_threshold ? -EUCLEAN : 0;
}

int mtd_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen,
	      const u_char *buf)
{
	*retlen = 0;
	if (to < 0 || to > mtd->size || len > mtd->size - to)
		return -EINVAL;
	if (!mtd->_write || !(mtd->flags & MTD_WRITEABLE))
		return -EROFS;
	if (!len)
		return 0;
	return mtd->_write(mtd, to, len, retlen, buf);
}

/*
 * In blackbox flight recorder like scenarios we want to make successful writes
 * in interrupt context. panic_write() is only intended to be called when its
 * known the kernel is about to panic and we need the write to succeed. Since
 * the kernel is not going to be running for much longer, this function can
 * break locks and delay to ensure the write succeeds (but not sleep).
 */
int mtd_panic_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen,
		    const u_char *buf)
{
	*retlen = 0;
	if (!mtd->_panic_write)
		return -EOPNOTSUPP;
	if (to < 0 || to > mtd->size || len > mtd->size - to)
		return -EINVAL;
	if (!(mtd->flags & MTD_WRITEABLE))
		return -EROFS;
	if (!len)
		return 0;
	return mtd->_panic_write(mtd, to, len, retlen, buf);
}

int mtd_read_oob(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops)
{
	int ret_code;
	ops->retlen = ops->oobretlen = 0;
	if (!mtd->_read_oob)
		return -EOPNOTSUPP;
	/*
	 * In cases where ops->datbuf != NULL, mtd->_read_oob() has semantics
	 * similar to mtd->_read(), returning a non-negative integer
	 * representing max bitflips. In other cases, mtd->_read_oob() may
	 * return -EUCLEAN. In all cases, perform similar logic to mtd_read().
	 */
	ret_code = mtd->_read_oob(mtd, from, ops);
	if (unlikely(ret_code < 0))
		return ret_code;
	if (mtd->ecc_strength == 0)
		return 0;	/* device lacks ecc */
	return ret_code >= mtd->bitflip_threshold ? -EUCLEAN : 0;
}

/*
 * Method to access the protection register area, present in some flash
 * devices. The user data is one time programmable but the factory data is read
 * only.
 */
int mtd_get_fact_prot_info(struct mtd_info *mtd, struct otp_info *buf,
			   size_t len)
{
	if (!mtd->_get_fact_prot_info)
		return -EOPNOTSUPP;
	if (!len)
		return 0;
	return mtd->_get_fact_prot_info(mtd, buf, len);
}

int mtd_read_fact_prot_reg(struct mtd_info *mtd, loff_t from, size_t len,
			   size_t *retlen, u_char *buf)
{
	*retlen = 0;
	if (!mtd->_read_fact_prot_reg)
		return -EOPNOTSUPP;
	if (!len)
		return 0;
	return mtd->_read_fact_prot_reg(mtd, from, len, retlen, buf);
}

int mtd_get_user_prot_info(struct mtd_info *mtd, struct otp_info *buf,
			   size_t len)
{
	if (!mtd->_get_user_prot_info)
		return -EOPNOTSUPP;
	if (!len)
		return 0;
	return mtd->_get_user_prot_info(mtd, buf, len);
}

int mtd_read_user_prot_reg(struct mtd_info *mtd, loff_t from, size_t len,
			   size_t *retlen, u_char *buf)
{
	*retlen = 0;
	if (!mtd->_read_user_prot_reg)
		return -EOPNOTSUPP;
	if (!len)
		return 0;
	return mtd->_read_user_prot_reg(mtd, from, len, retlen, buf);
}

int mtd_write_user_prot_reg(struct mtd_info *mtd, loff_t to, size_t len,
			    size_t *retlen, u_char *buf)
{
	*retlen = 0;
	if (!mtd->_write_user_prot_reg)
		return -EOPNOTSUPP;
	if (!len)
		return 0;
	return mtd->_write_user_prot_reg(mtd, to, len, retlen, buf);
}

int mtd_lock_user_prot_reg(struct mtd_info *mtd, loff_t from, size_t len)
{
	if (!mtd->_lock_user_prot_reg)
		return -EOPNOTSUPP;
	if (!len)
		return 0;
	return mtd->_lock_user_prot_reg(mtd, from, len);
}

/* Chip-supported device locking */
int mtd_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	if (!mtd->_lock)
		return -EOPNOTSUPP;
	if (ofs < 0 || ofs > mtd->size || len > mtd->size - ofs)
		return -EINVAL;
	if (!len)
		return 0;
	return mtd->_lock(mtd, ofs, len);
}

int mtd_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	if (!mtd->_unlock)
		return -EOPNOTSUPP;
	if (ofs < 0 || ofs > mtd->size || len > mtd->size - ofs)
		return -EINVAL;
	if (!len)
		return 0;
	return mtd->_unlock(mtd, ofs, len);
}

int mtd_is_locked(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	if (!mtd->_is_locked)
		return -EOPNOTSUPP;
	if (ofs < 0 || ofs > mtd->size || len > mtd->size - ofs)
		return -EINVAL;
	if (!len)
		return 0;
	return mtd->_is_locked(mtd, ofs, len);
}

int mtd_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	if (!mtd->_block_isbad)
		return 0;
	if (ofs < 0 || ofs > mtd->size)
		return -EINVAL;
	return mtd->_block_isbad(mtd, ofs);
}

int mtd_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	if (!mtd->_block_markbad)
		return -EOPNOTSUPP;
	if (ofs < 0 || ofs > mtd->size)
		return -EINVAL;
	if (!(mtd->flags & MTD_WRITEABLE))
		return -EROFS;
	return mtd->_block_markbad(mtd, ofs);
}

#if defined(CONFIG_CMD_MTDPARTS_SPREAD)
/**
 * mtd_get_len_incl_bad
 *
 * Check if length including bad blocks fits into device.
 *
 * @param mtd an MTD device
 * @param offset offset in flash
 * @param length image length
 * @return image length including bad blocks in *len_incl_bad and whether or not
 *         the length returned was truncated in *truncated
 */
void mtd_get_len_incl_bad(struct mtd_info *mtd, uint64_t offset,
			  const uint64_t length, uint64_t *len_incl_bad,
			  int *truncated)
{
	*truncated = 0;
	*len_incl_bad = 0;

	if (!mtd->block_isbad) {
		*len_incl_bad = length;
		return;
	}

	uint64_t len_excl_bad = 0;
	uint64_t block_len;

	while (len_excl_bad < length) {
		if (offset >= mtd->size) {
			*truncated = 1;
			return;
		}

		block_len = mtd->erasesize - (offset & (mtd->erasesize - 1));

		if (!mtd->block_isbad(mtd, offset & ~(mtd->erasesize - 1)))
			len_excl_bad += block_len;

		*len_incl_bad += block_len;
		offset       += block_len;
	}
}
#endif /* defined(CONFIG_CMD_MTDPARTS_SPREAD) */
