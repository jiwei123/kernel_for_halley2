/*
 *  sound/soc/ingenic/asoc-dma.h
 *  ALSA Soc Audio Layer -- ingenic dma platform driver
 *
 *  Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *	cli <chen.li@ingenic.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef __ASOC_DMA_H__
#define __ASOC_DMA_H__

#include <linux/dmaengine.h>
struct jz_pcm_dma_params {
	dma_addr_t dma_addr;
	enum dma_slave_buswidth buswidth;
	int max_burst;
};

#endif
