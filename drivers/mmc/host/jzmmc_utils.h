#ifndef __JZMMC_UTILS_H__
#define __JZMMC_UTILS_H__
#include <linux/scatterlist.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/mmc/ioctl.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>

int mmc_trans_blocks(struct mmc_card * card,
        unsigned char * buffer, unsigned addr, unsigned blkcnt, int write);

int mmc_erase_blocks(struct mmc_card * card,
        unsigned addr, unsigned blkcnt);

int mmc_write(struct mmc_card * card,
        unsigned char * buffer, unsigned long addr, unsigned long size);

struct mmc_card * find_emmc_card(void);

#endif  //__JZMMC_UTILS_H__
