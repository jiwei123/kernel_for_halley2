#include "./jzmmc_utils.h"

static void mmc_prepare_mrq(struct mmc_card *card,
    struct mmc_request *mrq, struct scatterlist *sg, unsigned sg_len,
    unsigned dev_addr, unsigned blocks, unsigned blksz, int write)
{
    BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

    if (blocks > 1) {
        mrq->cmd->opcode = write ?
            MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
    } else {
        mrq->cmd->opcode = write ?
            MMC_WRITE_BLOCK : MMC_READ_SINGLE_BLOCK;
    }

    mrq->cmd->arg = dev_addr;
    if (!mmc_card_blockaddr(card))
        mrq->cmd->arg <<= 9;

    mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

    if (blocks == 1)
        mrq->stop = NULL;
    else {
        mrq->stop->opcode = MMC_STOP_TRANSMISSION;
        mrq->stop->arg = 0;
        mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
    }

    mrq->data->blksz = blksz;
    mrq->data->blocks = blocks;
    mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
    mrq->data->sg = sg;
    mrq->data->sg_len = sg_len;

    mmc_set_data_timeout(mrq->data, card);
}

static int mmc_card_busy(struct mmc_command *cmd)
{
    return !(cmd->resp[0] & R1_READY_FOR_DATA) ||
        (R1_CURRENT_STATE(cmd->resp[0]) == R1_STATE_PRG);
}

/*
 * Wait for the card to finish the busy state
 */
static int mmc_wait_busy(struct mmc_card *card)
{
    int ret, busy;
    struct mmc_command cmd = {0};

    busy = 0;
    do {
        memset(&cmd, 0, sizeof(struct mmc_command));

        cmd.opcode = MMC_SEND_STATUS;
        cmd.arg = card->rca << 16;
        cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

        ret = mmc_wait_for_cmd(card->host, &cmd, 0);
        if (ret)
            break;

        if (!busy && mmc_card_busy(&cmd)) {
            busy = 1;
            if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY)
                pr_info("%s: Warning: Host did not "
                    "wait for busy state to end.\n",
                    mmc_hostname(card->host));
        }
    } while (mmc_card_busy(&cmd));

    return ret;
}

static int mmc_map_sg(struct mmc_card * card, unsigned char * buffer,
        unsigned blkcnt, struct scatterlist *sglist, unsigned int *sg_len)
{
    struct scatterlist *sg = NULL;
    int cnt, len;
    unsigned char * buf = buffer;

    sg_init_table(sglist, card->host->max_segs);
    *sg_len = 0;

    while (blkcnt) {
        if (blkcnt > card->host->max_blk_count) {
            cnt = card->host->max_blk_count;
        }
        else {
            cnt = blkcnt;
        }
        len = cnt * card->host->max_blk_size;
        if (sg)
            sg = sg_next(sg);
        else
            sg = sglist;
        if (!sg)
            return -EINVAL;

        sg_set_buf(sg, buf, len);

        *sg_len += 1;
        buf += len;
        blkcnt -= cnt;
    }

    if (sg)
        sg_mark_end(sg);

    return 0;
}

int mmc_trans_blocks(struct mmc_card * card,
        unsigned char * buffer, unsigned addr, unsigned blkcnt, int write)
{
    int ret;
    int blksz;
    int cnt, i = 0;
    struct scatterlist * sg;
    unsigned int sg_len;

    blksz = card->host->max_blk_size;

    sg = kmalloc(sizeof(struct scatterlist) * card->host->max_segs, GFP_KERNEL);
    if (!sg) {
        return -ENOMEM;
    }
    mmc_map_sg(card, buffer, blkcnt, sg, &sg_len);
    mmc_claim_host(card->host);

    for (i = 0; i < sg_len; i++) {
        struct mmc_request mrq = {0};
        struct mmc_command cmd = {0};
        struct mmc_command stop = {0};
        struct mmc_data data = {0};
        mrq.cmd = &cmd;
        mrq.data = &data;
        mrq.stop = &stop;

        if (blkcnt > card->host->max_blk_count)
            cnt = card->host->max_blk_count;
        else
            cnt = blkcnt;

        mmc_prepare_mrq(card, &mrq, &sg[i], 1, addr, cnt, blksz, write);

        mmc_wait_for_req(card->host, &mrq);

        if (cmd.error)
            return cmd.error;
        if (data.error)
            return data.error;

        ret = mmc_wait_busy(card);
        if (ret)
            return ret;

        blkcnt -= cnt;
        addr += cnt;
    }
    mmc_release_host(card->host);

    return 0;
}
EXPORT_SYMBOL(mmc_trans_blocks);

int mmc_write(struct mmc_card * card,
        unsigned char * buffer, unsigned long addr, unsigned long size) {
    int ret;
    int sz;
    unsigned char * buf = NULL;
    int blksz;

    blksz = card->host->max_blk_size;
    buf = kzalloc(blksz, GFP_KERNEL);
    if (!buf) {
        return -ENOMEM;
    }

    if (addr % blksz) {
        sz = addr % blksz;

        ret = mmc_trans_blocks(card, buf, addr / blksz, 1, 0);
        if (ret)
            return ret;

        memcpy(buf + sz, buffer, blksz - sz);
        ret = mmc_trans_blocks(card, buf, addr / blksz, 1, 1);
        if (ret)
            return ret;

        addr += blksz - sz;
        buffer += sz;
        size -= sz;
    }

    ret = mmc_trans_blocks(card, (unsigned char *) (buffer), addr / blksz,
            size / blksz, 1);
    if (ret)
        return ret;

    if (size % blksz) {
        sz = size % blksz;
        addr += size - sz;
        buffer += size - sz;

        ret = mmc_trans_blocks(card, buf, addr / blksz, 1, 0);
        if (ret)
            return ret;

        memcpy(buf, buffer, sz);
        ret = mmc_trans_blocks(card, buf, addr / blksz, 1, 1);
        if (ret)
            return ret;
    }

    kfree(buf);
    return 0;
}
EXPORT_SYMBOL(mmc_write);

int mmc_erase_blocks(struct mmc_card * card,
        unsigned addr, unsigned blkcnt) {

    mmc_claim_host(card->host);

    if (!mmc_can_erase(card)) {
        printk("can not erase!!!\n");
        return 0;
    }

    mmc_erase(card, addr, blkcnt, MMC_ERASE_ARG);

    mmc_release_host(card->host);
    return 0;
}
EXPORT_SYMBOL(mmc_erase_blocks);

struct mmc_card * find_emmc_card(void) {
    struct class_dev_iter iter;
    struct device *dev;
    struct mmc_card * card = NULL;

    class_dev_iter_init(&iter, &block_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter))) {
        struct gendisk *disk = dev_to_disk(dev);
        struct disk_part_iter piter;

        if (!dev->type || strcmp(dev->type->name, "disk")) {
            continue;
        }

        if (get_capacity(disk) == 0
                || (disk->flags & GENHD_FL_SUPPRESS_PARTITION_INFO))
            continue;

        disk_part_iter_init(&piter, disk, DISK_PITER_INCL_PART0);
        while ((disk_part_iter_next(&piter))) {
            card = mmc_card_get(disk);
            if (card && mmc_card_mmc(card)) {
                break;
            }
        }
        disk_part_iter_exit(&piter);
    }
    class_dev_iter_exit(&iter);
    return card;
}
