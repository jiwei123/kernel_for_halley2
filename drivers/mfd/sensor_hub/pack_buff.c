
#include <linux/string.h>
#include <linux/printk.h>
#include "pack_buff.h"


int write_block_to_buff(char *buff, int blocks, BUFF_T * xbuff_t)
{
	int i = 0;
	if( if_buff_is_full(xbuff_t, blocks) == 0)
	{
		return -1;
	}

	for (i = 0; i < blocks; i++)
	{
		strncpy(xbuff_t->buff + xbuff_t->head, buff + i * EACH_BLOCK_SIZE, EACH_BLOCK_SIZE);
		xbuff_t->head = ( xbuff_t->head + EACH_BLOCK_SIZE ) % MAX_BUFF_SIZE;
	}
	xbuff_t->block_count += blocks;

	return 0;
}

int read_block_from_buff(char *buff, int blocks, BUFF_T *xbuff_t)
{
	int i = 0, temp_tail = 0;

	if( if_buff_hold_enough_block(xbuff_t, blocks) < 0)
	{
		return -1;
	}

	temp_tail = xbuff_t->tail;

	for (i = 0; i < blocks; i++)
	{
		strncpy(buff + i * EACH_BLOCK_SIZE, xbuff_t->buff + temp_tail, EACH_BLOCK_SIZE);
		temp_tail = ( temp_tail + EACH_BLOCK_SIZE ) % MAX_BUFF_SIZE;
	}

	return 0;
}

int read_pack_from_buff(char *buff, int Size ,BUFF_T *xbuff_t)
{
	int size = 0, blocks = 0, ret = 0;

	if(if_buff_is_empty(xbuff_t) == 0)
	{
		printk("%s: if_buff_is_empty\n", __func__);
		return -1;
	}

	blocks = if_buff_have_entire_pack(xbuff_t);

	if (blocks < 0)
	{
		printk("%s: buff not have entire pack!\n");
		return -1;
	}

	read_block_from_buff(buff, blocks, xbuff_t);

	return blocks;
}

int write_pack_to_buff(char *buff, int size, BUFF_T *xbuff_t)
{
	int length = 0, blocks = 0;

	length = ( (hub_format_header_t *)buff )->length + HEAD_SIZE ;
	if(size != length)
	{
		printk("%s: err param!!!\n", __func__);
		return -1;
	}

	if ( (size % EACH_BLOCK_SIZE) == 0)
	{
		blocks = size / EACH_BLOCK_SIZE;
	} else {
		blocks = size / EACH_BLOCK_SIZE + 1;
	}

	write_block_to_buff(buff, blocks, xbuff_t);

	return 0;
}

void cut_down_block_count(BUFF_T *xbuff_t, int blocks)
{
	int size;

	xbuff_t->tail = (xbuff_t->tail + blocks * EACH_BLOCK_SIZE) % MAX_BUFF_SIZE;
	xbuff_t->block_count -= blocks;
}

int if_buff_is_full(BUFF_T * xbuff_t, int blocks)
{
	if ( (MAX_BLOCK_COUNT - xbuff_t->block_count -1) < blocks)
	{
		return 0;
	}

	return -1;
}

int if_buff_is_empty(BUFF_T * xbuff_t)
{
	if(xbuff_t->block_count == 0)
	{
		return 0;
	}
	return -1;
}

int if_buff_hold_enough_block(BUFF_T * xbuff_t, int blocks)
{
	if( xbuff_t->block_count < blocks)
	{
		return -1;
	}

	return 0;
}

int if_buff_have_entire_pack(BUFF_T * xbuff_t)
{
	int length = 0, blocks = 0;

	 if((MAX_BUFF_SIZE - xbuff_t->tail) == 1)
	 {
		 length = xbuff_t->buff[0] + HEAD_SIZE;
	 } else {
		 length = ( (hub_format_header_t *)(xbuff_t->buff + xbuff_t->tail) )->length + HEAD_SIZE;
	 }

	if( (length % EACH_BLOCK_SIZE) == 0)
	{
		blocks = length / EACH_BLOCK_SIZE;
	} else {
		blocks = length / EACH_BLOCK_SIZE + 1;
	}

	if(xbuff_t->block_count < blocks)
	{
		return -1;
	}

	return blocks;
}
