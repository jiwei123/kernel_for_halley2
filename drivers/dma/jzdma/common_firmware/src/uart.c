/*
 * Copyright (C) 2016 Ingenic Semiconductor
 *
 * HuangLihong(Regen) <lihong.huang@ingenic.com>
 *
 */

#include <common.h>
#include <asm/jzsoc.h>
#include <tcsm.h>
#include <uart.h>
#include <intc.h>

void uart_init(void)
{
	REG32(TCSM_UART_TBUF_WP) = 0;
	REG32(TCSM_UART_TBUF_RP) = 0;

	REG32(TCSM_UART_RBUF_WP) = 0;
	REG32(TCSM_UART_RBUF_RP) = 0;
	REG32(TCSM_UART_DEVICE_NUM) = 1;
}

#define UART_REG32(addr)  REG32(0x10030000 + REG32(TCSM_UART_DEVICE_NUM) * 0x1000 + (addr))
#define UART_REG8(addr)   REG8(0x10030000 + REG32(TCSM_UART_DEVICE_NUM) * 0x1000 + (addr))
#define INTC_UART0_BIT 19
#define INTC_UART(n) (1 << (INTC_UART0_BIT - n))

#ifdef DEBUG
static void gpio_port_direction_output(int port, int pin, int value)
{
	REG_GPIO_PXINTC(port) = 1 << pin;
	REG_GPIO_PXMASKS(port) = 1 << pin;
	REG_GPIO_PXPAT1C(port) = 1 << pin;
	if (value == 1)
		REG_GPIO_PXPAT0S(port) = 1 << pin;
	else
		REG_GPIO_PXPAT0C(port) = 1 << pin;
}
#endif

static void start_tx()
{
	UART_REG8(UART_IER) |= IER_TDRIE;
}

static void stop_tx()
{
	UART_REG8(UART_IER) &= ~IER_TDRIE;
}

static int valid_bytes_in_buf(int bufLength, unsigned int waddr, unsigned int raddr)
{
	int count;

	if(waddr >= raddr)
		count = waddr - raddr;
	else
		count = bufLength + waddr - raddr;

	return count;
}

static int is_rxBuf_full(unsigned int waddr, unsigned int raddr)
{
	int isFull;
	if (waddr < raddr)
		isFull = (waddr == raddr - 2);
	else
		isFull = (waddr == raddr + TCSM_UART_RBUF_LEN - 2);

	return isFull;
}

static int isBufEmpty(unsigned int waddr, unsigned int raddr)
{
	return (waddr == raddr);
}

static int valid_bytes_in_txBuf(unsigned int waddr, unsigned int raddr)
{
	return valid_bytes_in_buf(TCSM_UART_TBUF_LEN, waddr, raddr);
}

static int needNotifyCpuToRead(unsigned int rxBufWaddr)
{
	int rxBufRaddr = REG32(TCSM_UART_RBUF_RP);
	if (!isBufEmpty(rxBufWaddr, rxBufRaddr))
		return 1;

	return 0;
}

static struct mailbox_pend_addr_s *mailbox_pend_addr;
static unsigned int receive_chars(unsigned int lsr)
{
	volatile unsigned char *buf = (volatile unsigned char *)TCSM_UART_RBUF_ADDR;
	unsigned int waddr = REG32(TCSM_UART_RBUF_WP);
	int raddr = REG32(TCSM_UART_RBUF_RP);

	int timeout = 100 * 100;
	do {
		while (lsr & LSR_DRY) {
			if (is_rxBuf_full(waddr, raddr)) {
				break;
			}

			buf[waddr] = UART_REG8(UART_RBR); // data
			buf[waddr + 1] = lsr; // status
			waddr = (waddr + 2) & (TCSM_UART_RBUF_LEN - 1);

			lsr = UART_REG8(UART_LSR);
		}

		raddr = REG32(TCSM_UART_RBUF_RP);
		if (is_rxBuf_full(waddr, raddr)) {
			break;
		}
		lsr = UART_REG8(UART_LSR);
	} while (timeout--);

	REG32(TCSM_UART_RBUF_WP) = waddr;

	return waddr;
}

static void transmit_chars()
{
	volatile unsigned char *buf = (volatile unsigned char *)TCSM_UART_TBUF_ADDR;
	unsigned int raddr = REG32(TCSM_UART_TBUF_RP);
	unsigned int waddr = REG32(TCSM_UART_TBUF_WP);
	int i = 0;

	while (!isBufEmpty(waddr, raddr) && i < 32 ) {
		UART_REG8(UART_THR) = buf[raddr];
		i++;
		raddr = (raddr + 1) & (TCSM_UART_TBUF_LEN - 1);
	}

	REG32(TCSM_UART_TBUF_RP) = raddr;

	if(isBufEmpty(waddr, raddr) || valid_bytes_in_txBuf(waddr, raddr) == 64) {
		mailbox_pend_addr->mcu_state |= TCSM_UART_NEED_WRITE; // data request int to cpu
	}
}

static void check_modem_status(int *isTxStoppedByCts)
{
	unsigned int msr = UART_REG32(UART_MSR);
	if (msr & MSR_ANY_DELTA) {
		mailbox_pend_addr->mcu_state |= msr << TCSM_UART_CTS_CHANGE_BIT;
		if (msr & MSR_CTS) {
			start_tx();
			*isTxStoppedByCts = 0;
		} else {
			*isTxStoppedByCts = 1;
			stop_tx();
		}
	}
}

int handle_uart_irq(void)
{
	mailbox_pend_addr = (struct mailbox_pend_addr_s *)MAILBOX_PEND_ADDR;
	unsigned int dpr1 = INTC_REG32(INTC_DPR1);

	if (dpr1 & INTC_UART(REG32(TCSM_UART_DEVICE_NUM))) {
		int rxBufWaddr = REG32(TCSM_UART_RBUF_WP);
		static int isTxStoppedByCts = 0;

		unsigned int iir = UART_REG32(UART_IIR);
		unsigned int lsr = UART_REG32(UART_LSR);

		if (iir & IIR_NO_INT)
			return 0;

		if (lsr & LSR_DRY) {
			rxBufWaddr = receive_chars(lsr);
		}

		check_modem_status(&isTxStoppedByCts);

		if (lsr & LSR_TDRQ) {
			if (isTxStoppedByCts) {
				// Tx may be started again by m200 after stopping by mcu
				// cts irq handler, so we stop tx here
				stop_tx();
			} else {
				transmit_chars();
			}
		}

		if (lsr & LSR_DRY) {
			if(needNotifyCpuToRead(rxBufWaddr)) {
				mailbox_pend_addr->mcu_state |= TCSM_UART_NEED_READ;
			}
		}
		if (mailbox_pend_addr->mcu_state)
			return 1;
	}

	return 0;
}
