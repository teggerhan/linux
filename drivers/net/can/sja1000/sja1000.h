/*
 * sja1000.h -  Philips SJA1000 network device driver
 *
 * Copyright (c) 2003 Matthias Brukner, Trajet Gmbh, Rebenring 33,
 * 38106 Braunschweig, GERMANY
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#ifndef SJA1000_DEV_H
#define SJA1000_DEV_H

#include <linux/irqreturn.h>
#include <linux/can/dev.h>
#include <linux/can/platform/sja1000.h>

#define SJA1000_ECHO_SKB_MAX	1 /* the SJA1000 has one TX buffer object */

#define SJA1000_MAX_IRQ 20	/* max. number of interrupts handled in ISR */

/* SJA1000 registers - manual section 6.4 (Pelican Mode) */
#define SJA1000_MOD		0x00  //模式
#define SJA1000_CMR		0x01  //命令
#define SJA1000_SR		0x02  //状态
#define SJA1000_IR		0x03  //中断
#define SJA1000_IER		0x04  //中断使能
#define SJA1000_ALC		0x0B  //arbitration lost capture
#define SJA1000_ECC		0x0C  //error code capture
#define SJA1000_EWL		0x0D  //error warning limit
#define SJA1000_RXERR		0x0E  //RX error counter
#define SJA1000_TXERR		0x0F  //TX error counter
#define SJA1000_ACCC0		0x10  //acceptance code 0
#define SJA1000_ACCC1		0x11  //acceptance code 1
#define SJA1000_ACCC2		0x12  //acceptance code 2
#define SJA1000_ACCC3		0x13  //acceptance code 3
#define SJA1000_ACCM0		0x14  //acceptance mask 0
#define SJA1000_ACCM1		0x15  //acceptance mask 1
#define SJA1000_ACCM2		0x16  //acceptance mask 2
#define SJA1000_ACCM3		0x17  //acceptance mask 3
#define SJA1000_RMC		0x1D  //RX message counter
#define SJA1000_RBSA		0x1E  //RX buffer start address

/* Common registers - manual section 6.5 */ //6.5
#define SJA1000_BTR0		0x06  //BUS TIMING REGISTER 0 (BTR0)
#define SJA1000_BTR1		0x07  //BUS TIMING REGISTER 1 (BTR1)
#define SJA1000_OCR		0x08  //OUTPUT CONTROL REGISTER (OCR)
#define SJA1000_CDR		0x1F  //CLOCK DIVIDER REGISTER (CDR)
//page 39 and page 47
#define SJA1000_FI		0x10  //
#define SJA1000_SFF_BUF		0x13  //
#define SJA1000_EFF_BUF		0x15  //
//Table 34 RX frame information (SFF); CAN address 16?
#define SJA1000_FI_FF		0x80 //bit 7
#define SJA1000_FI_RTR		0x40 //bit 6

#define SJA1000_ID1		0x11  //Table 29 TX identifier 1 (EFF); CAN address 17
#define SJA1000_ID2		0x12  //Table 30 TX identifier 2 (EFF); CAN address 18
#define SJA1000_ID3		0x13  //Table 31 TX identifier 3 (EFF); CAN address 19
#define SJA1000_ID4		0x14  //Table 32 TX identifier 4 (EFF); CAN address 20

#define SJA1000_CAN_RAM		0x20

/* mode register */         //6.4.2 RESET VALUES
#define MOD_RM		0x01  // Reset Mode
#define MOD_LOM		0x02  // Listen Only Mode
#define MOD_STM		0x04  // Self Test Mode
#define MOD_AFM		0x08  // Acceptance Filter Mode
#define MOD_SM		0x10  // Sleep Mode

/* commands */    //6.4.2 RESET VALUES
#define CMD_SRR		0x10    // Self Reception Request
#define CMD_CDO		0x08   //Clear Data Overrun
#define CMD_RRB		0x04   // Release Receive Buffer
#define CMD_AT		0x02  // Abort Transmission
#define CMD_TR		0x01  //Transmission Request

/* interrupt sources */   //6.4.2 RESET VALUES
#define IRQ_BEI		0x80  // Bus Error Interrupt
#define IRQ_ALI		0x40  //Arbitration Lost Interrupt
#define IRQ_EPI		0x20  //Error Passive Interrupt
#define IRQ_WUI		0x10  // Wake-Up Interrupt
#define IRQ_DOI		0x08  //Data Overrun Interrupt
#define IRQ_EI		0x04  //Error Interrupt	
#define IRQ_TI		0x02  //Transmit Interrupt
#define IRQ_RI		0x01  //Receive Interrupt
#define IRQ_ALL		0xFF  
#define IRQ_OFF		0x00

/* status register content */  //6.3.5 STATUS REGISTER (SR) or 6.4.2 RESET VALUES
#define SR_BS		0x80  //Bus Status
#define SR_ES		0x40  //Error Status
#define SR_TS		0x20  //Transmit Status
#define SR_RS		0x10  //Receive Status
#define SR_TCS		0x08  //Transmission Complete Status
#define SR_TBS		0x04  //Transmit Buffer Status
#define SR_DOS		0x02  //Data Overrun Status
#define SR_RBS		0x01  //Receive Buffer Status

#define SR_CRIT (SR_BS|SR_ES)

/* ECC register */  //ERROR CODE CAPTURE REGISTER (ECC)
#define ECC_SEG		0x1F  //ECC.0~ECC.4
#define ECC_DIR		0x20  //Direction 1 RX, 0 TX
#define ECC_ERR		6     //?
#define ECC_BIT		0x00 //bit error
#define ECC_FORM	0x40 //form error
#define ECC_STUFF	0x80 // stuff error
#define ECC_MASK	0xc0 // ?

/*
 * Flags for sja1000priv.flags
 */
#define SJA1000_CUSTOM_IRQ_HANDLER	BIT(0)
#define SJA1000_QUIRK_NO_CDR_REG	BIT(1)
#define SJA1000_QUIRK_RESET_ON_OVERRUN	BIT(2)

/*
 * SJA1000 private data structure
 */
struct sja1000_priv {
	struct can_priv can;	/* must be the first member */
	struct sk_buff *echo_skb;

	/* the lower-layer is responsible for appropriate locking */
	u8 (*read_reg) (const struct sja1000_priv *priv, int reg);
	void (*write_reg) (const struct sja1000_priv *priv, int reg, u8 val);
	void (*pre_irq) (const struct sja1000_priv *priv);
	void (*post_irq) (const struct sja1000_priv *priv);

	void *priv;		/* for board-specific data */
	struct net_device *dev;

	void __iomem *reg_base;	 /* ioremap'ed address to registers */
	unsigned long irq_flags; /* for request_irq() */
	spinlock_t cmdreg_lock;  /* lock for concurrent cmd register writes */

	u16 flags;		/* custom mode flags */
	u8 ocr;			/* output control register */
	u8 cdr;			/* clock divider register */
};

struct net_device *alloc_sja1000dev(int sizeof_priv);
void free_sja1000dev(struct net_device *dev);
int register_sja1000dev(struct net_device *dev);
void unregister_sja1000dev(struct net_device *dev);

irqreturn_t sja1000_interrupt(int irq, void *dev_id);

#endif /* SJA1000_DEV_H */
