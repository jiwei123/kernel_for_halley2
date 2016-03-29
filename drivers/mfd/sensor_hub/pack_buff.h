
#ifndef __PACK_BUFF_H
#define __PACK_BUFF_H

#include "sensorhub_common.h"

int write_block_to_buff(char *, int , BUFF_T * );

int read_block_from_buff(char *, int , BUFF_T *);

int read_pack_from_buff(char *, int  ,BUFF_T *);

int write_pack_to_buff(char *, int , BUFF_T *);

void cut_down_block_count(BUFF_T *, int );

int if_buff_is_full(BUFF_T * , int );

int if_buff_is_empty(BUFF_T * );

int if_buff_hold_enough_block(BUFF_T * , int );

int if_buff_have_entire_pack(BUFF_T * );

#endif /* __PACK_BUFF_H */
