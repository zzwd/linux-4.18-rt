/*
 * Driver for the i2c controller on the Allwinner SoC family).
 *
 * Author: Mark A. Greer <mgreer@mvista.com>
 *
 * 2005 (c) MontaVista, Software, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sunxi_i2c.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#define SUNXI_I2C_ADDR_ADDR(val)			((val & 0x7f) << 1)
#define SUNXI_I2C_BAUD_DIV_N(val)			(val & 0x7)
#define SUNXI_I2C_BAUD_DIV_M(val)			((val & 0xf) << 3)

#define	SUNXI_I2C_REG_CONTROL_ACK			BIT(2)
#define	SUNXI_I2C_REG_CONTROL_IFLG			BIT(3)
#define	SUNXI_I2C_REG_CONTROL_STOP			BIT(4)
#define	SUNXI_I2C_REG_CONTROL_START			BIT(5)
#define	SUNXI_I2C_REG_CONTROL_TWSIEN			BIT(6)
#define	SUNXI_I2C_REG_CONTROL_INTEN			BIT(7)

/* Ctlr status values */
#define	SUNXI_I2C_STATUS_BUS_ERR			0x00
#define	SUNXI_I2C_STATUS_MAST_START			0x08
#define	SUNXI_I2C_STATUS_MAST_REPEAT_START		0x10
#define	SUNXI_I2C_STATUS_MAST_WR_ADDR_ACK		0x18
#define	SUNXI_I2C_STATUS_MAST_WR_ADDR_NO_ACK		0x20
#define	SUNXI_I2C_STATUS_MAST_WR_ACK			0x28
#define	SUNXI_I2C_STATUS_MAST_WR_NO_ACK		0x30
#define	SUNXI_I2C_STATUS_MAST_LOST_ARB		0x38
#define	SUNXI_I2C_STATUS_MAST_RD_ADDR_ACK		0x40
#define	SUNXI_I2C_STATUS_MAST_RD_ADDR_NO_ACK		0x48
#define	SUNXI_I2C_STATUS_MAST_RD_DATA_ACK		0x50
#define	SUNXI_I2C_STATUS_MAST_RD_DATA_NO_ACK		0x58
#define	SUNXI_I2C_STATUS_MAST_WR_ADDR_2_ACK		0xd0
#define	SUNXI_I2C_STATUS_MAST_WR_ADDR_2_NO_ACK	0xd8
#define	SUNXI_I2C_STATUS_MAST_RD_ADDR_2_ACK		0xe0
#define	SUNXI_I2C_STATUS_MAST_RD_ADDR_2_NO_ACK	0xe8
#define	SUNXI_I2C_STATUS_NO_STATUS			0xf8

/* Register defines (I2C bridge) */
#define	SUNXI_I2C_REG_TX_DATA_LO			0xc0
#define	SUNXI_I2C_REG_TX_DATA_HI			0xc4
#define	SUNXI_I2C_REG_RX_DATA_LO			0xc8
#define	SUNXI_I2C_REG_RX_DATA_HI			0xcc
#define	SUNXI_I2C_REG_BRIDGE_CONTROL			0xd0
#define	SUNXI_I2C_REG_BRIDGE_STATUS			0xd4
#define	SUNXI_I2C_REG_BRIDGE_INTR_CAUSE		0xd8
#define	SUNXI_I2C_REG_BRIDGE_INTR_MASK		0xdC
#define	SUNXI_I2C_REG_BRIDGE_TIMING			0xe0

/* Bridge Control values */
#define	SUNXI_I2C_BRIDGE_CONTROL_WR			BIT(0)
#define	SUNXI_I2C_BRIDGE_CONTROL_RD			BIT(1)
#define	SUNXI_I2C_BRIDGE_CONTROL_ADDR_SHIFT		2
#define	SUNXI_I2C_BRIDGE_CONTROL_ADDR_EXT		BIT(12)
#define	SUNXI_I2C_BRIDGE_CONTROL_TX_SIZE_SHIFT	13
#define	SUNXI_I2C_BRIDGE_CONTROL_RX_SIZE_SHIFT	16
#define	SUNXI_I2C_BRIDGE_CONTROL_ENABLE		BIT(19)
#define	SUNXI_I2C_BRIDGE_CONTROL_REPEATED_START	BIT(20)

/* Bridge Status values */
#define	SUNXI_I2C_BRIDGE_STATUS_ERROR			BIT(0)

/* Driver states */
enum {
	SUNXI_I2C_STATE_INVALID,
	SUNXI_I2C_STATE_IDLE,
	SUNXI_I2C_STATE_WAITING_FOR_START_COND,
	SUNXI_I2C_STATE_WAITING_FOR_RESTART,
	SUNXI_I2C_STATE_WAITING_FOR_ADDR_1_ACK,
	SUNXI_I2C_STATE_WAITING_FOR_ADDR_2_ACK,
	SUNXI_I2C_STATE_WAITING_FOR_SLAVE_ACK,
	SUNXI_I2C_STATE_WAITING_FOR_SLAVE_DATA,
};

/* Driver actions */
enum {
	SUNXI_I2C_ACTION_INVALID,
	SUNXI_I2C_ACTION_CONTINUE,
	SUNXI_I2C_ACTION_SEND_RESTART,
	SUNXI_I2C_ACTION_SEND_ADDR_1,
	SUNXI_I2C_ACTION_SEND_ADDR_2,
	SUNXI_I2C_ACTION_SEND_DATA,
	SUNXI_I2C_ACTION_RCV_DATA,
	SUNXI_I2C_ACTION_RCV_DATA_STOP,
	SUNXI_I2C_ACTION_SEND_STOP,
};

struct sunxi_i2c_regs {
	u8	addr;
	u8	ext_addr;
	u8	data;
	u8	control;
	u8	status;
	u8	clock;
	u8	soft_reset;
};

struct sunxi_i2c_data {
	struct i2c_msg		*msgs;
	int			num_msgs;
	int			irq;
	u32			state;
	u32			action;
	u32			aborting;
	u32			cntl_bits;
	void __iomem		*reg_base;
	struct sunxi_i2c_regs	reg_offsets;
	u32			addr1;
	u32			addr2;
	u32			bytes_left;
	u32			byte_posn;
	u32			send_stop;
	u32			block;
	int			rc;
	u32			freq_m;
	u32			freq_n;
	struct clk              *clk;
	struct clk              *reg_clk;
	wait_queue_head_t	waitq;
	spinlock_t		lock;
	struct i2c_msg		*msg;
	struct i2c_adapter	adapter;
	bool			offload_enabled;
/* 5us delay in order to avoid repeated start timing violation */
	bool			errata_delay;
	struct reset_control	*rstc;
	bool			irq_clear_inverted;
	/* Clk div is 2 to the power n, not 2 to the power n + 1 */
	bool			clk_n_base_0;
};

static struct sunxi_i2c_regs sunxi_i2c_regs_mv64xxx = {
	.addr		= 0x00,
	.ext_addr	= 0x10,
	.data		= 0x04,
	.control	= 0x08,
	.status		= 0x0c,
	.clock		= 0x0c,
	.soft_reset	= 0x1c,
};

static struct sunxi_i2c_regs sunxi_i2c_regs_sun4i = {
	.addr		= 0x00,
	.ext_addr	= 0x04,
	.data		= 0x08,
	.control	= 0x0c,
	.status		= 0x10,
	.clock		= 0x14,
	.soft_reset	= 0x18,
};

static void
sunxi_i2c_prepare_for_io(struct sunxi_i2c_data *drv_data,
	struct i2c_msg *msg)
{
	u32	dir = 0;

	drv_data->cntl_bits = SUNXI_I2C_REG_CONTROL_ACK |
		SUNXI_I2C_REG_CONTROL_INTEN | SUNXI_I2C_REG_CONTROL_TWSIEN;

	if (msg->flags & I2C_M_RD)
		dir = 1;

	if (msg->flags & I2C_M_TEN) {
		drv_data->addr1 = 0xf0 | (((u32)msg->addr & 0x300) >> 7) | dir;
		drv_data->addr2 = (u32)msg->addr & 0xff;
	} else {
		drv_data->addr1 = SUNXI_I2C_ADDR_ADDR((u32)msg->addr) | dir;
		drv_data->addr2 = 0;
	}
}

/*
 *****************************************************************************
 *
 *	Finite State Machine & Interrupt Routines
 *
 *****************************************************************************
 */

/* Reset hardware and initialize FSM */
static void
sunxi_i2c_hw_init(struct sunxi_i2c_data *drv_data)
{
	if (drv_data->offload_enabled) {
		writel(0, drv_data->reg_base + SUNXI_I2C_REG_BRIDGE_CONTROL);
		writel(0, drv_data->reg_base + SUNXI_I2C_REG_BRIDGE_TIMING);
		writel(0, drv_data->reg_base +
			SUNXI_I2C_REG_BRIDGE_INTR_CAUSE);
		writel(0, drv_data->reg_base +
			SUNXI_I2C_REG_BRIDGE_INTR_MASK);
	}

	writel(0, drv_data->reg_base + drv_data->reg_offsets.soft_reset);
	writel(SUNXI_I2C_BAUD_DIV_M(drv_data->freq_m) | SUNXI_I2C_BAUD_DIV_N(drv_data->freq_n),
		drv_data->reg_base + drv_data->reg_offsets.clock);
	writel(0, drv_data->reg_base + drv_data->reg_offsets.addr);
	writel(0, drv_data->reg_base + drv_data->reg_offsets.ext_addr);
	writel(SUNXI_I2C_REG_CONTROL_TWSIEN | SUNXI_I2C_REG_CONTROL_STOP,
		drv_data->reg_base + drv_data->reg_offsets.control);
	drv_data->state = SUNXI_I2C_STATE_IDLE;
}

static void
sunxi_i2c_fsm(struct sunxi_i2c_data *drv_data, u32 status)
{
	/*
	 * If state is idle, then this is likely the remnants of an old
	 * operation that driver has given up on or the user has killed.
	 * If so, issue the stop condition and go to idle.
	 */
	if (drv_data->state == SUNXI_I2C_STATE_IDLE) {
		drv_data->action = SUNXI_I2C_ACTION_SEND_STOP;
		return;
	}

	/* The status from the ctlr [mostly] tells us what to do next */
	switch (status) {
	/* Start condition interrupt */
	case SUNXI_I2C_STATUS_MAST_START: /* 0x08 */
	case SUNXI_I2C_STATUS_MAST_REPEAT_START: /* 0x10 */
		drv_data->action = SUNXI_I2C_ACTION_SEND_ADDR_1;
		drv_data->state = SUNXI_I2C_STATE_WAITING_FOR_ADDR_1_ACK;
		break;

	/* Performing a write */
	case SUNXI_I2C_STATUS_MAST_WR_ADDR_ACK: /* 0x18 */
		if (drv_data->msg->flags & I2C_M_TEN) {
			drv_data->action = SUNXI_I2C_ACTION_SEND_ADDR_2;
			drv_data->state =
				SUNXI_I2C_STATE_WAITING_FOR_ADDR_2_ACK;
			break;
		}
		/* FALLTHRU */
	case SUNXI_I2C_STATUS_MAST_WR_ADDR_2_ACK: /* 0xd0 */
	case SUNXI_I2C_STATUS_MAST_WR_ACK: /* 0x28 */
		if ((drv_data->bytes_left == 0)
				|| (drv_data->aborting
					&& (drv_data->byte_posn != 0))) {
			if (drv_data->send_stop || drv_data->aborting) {
				drv_data->action = SUNXI_I2C_ACTION_SEND_STOP;
				drv_data->state = SUNXI_I2C_STATE_IDLE;
			} else {
				drv_data->action =
					SUNXI_I2C_ACTION_SEND_RESTART;
				drv_data->state =
					SUNXI_I2C_STATE_WAITING_FOR_RESTART;
			}
		} else {
			drv_data->action = SUNXI_I2C_ACTION_SEND_DATA;
			drv_data->state =
				SUNXI_I2C_STATE_WAITING_FOR_SLAVE_ACK;
			drv_data->bytes_left--;
		}
		break;

	/* Performing a read */
	case SUNXI_I2C_STATUS_MAST_RD_ADDR_ACK: /* 40 */
		if (drv_data->msg->flags & I2C_M_TEN) {
			drv_data->action = SUNXI_I2C_ACTION_SEND_ADDR_2;
			drv_data->state =
				SUNXI_I2C_STATE_WAITING_FOR_ADDR_2_ACK;
			break;
		}
		/* FALLTHRU */
	case SUNXI_I2C_STATUS_MAST_RD_ADDR_2_ACK: /* 0xe0 */
		if (drv_data->bytes_left == 0) {
			drv_data->action = SUNXI_I2C_ACTION_SEND_STOP;
			drv_data->state = SUNXI_I2C_STATE_IDLE;
			break;
		}
		/* FALLTHRU */
	case SUNXI_I2C_STATUS_MAST_RD_DATA_ACK: /* 0x50 */
		if (status != SUNXI_I2C_STATUS_MAST_RD_DATA_ACK)
			drv_data->action = SUNXI_I2C_ACTION_CONTINUE;
		else {
			drv_data->action = SUNXI_I2C_ACTION_RCV_DATA;
			drv_data->bytes_left--;
		}
		drv_data->state = SUNXI_I2C_STATE_WAITING_FOR_SLAVE_DATA;

		if ((drv_data->bytes_left == 1) || drv_data->aborting)
			drv_data->cntl_bits &= ~SUNXI_I2C_REG_CONTROL_ACK;
		break;

	case SUNXI_I2C_STATUS_MAST_RD_DATA_NO_ACK: /* 0x58 */
		drv_data->action = SUNXI_I2C_ACTION_RCV_DATA_STOP;
		drv_data->state = SUNXI_I2C_STATE_IDLE;
		break;

	case SUNXI_I2C_STATUS_MAST_WR_ADDR_NO_ACK: /* 0x20 */
	case SUNXI_I2C_STATUS_MAST_WR_NO_ACK: /* 30 */
	case SUNXI_I2C_STATUS_MAST_RD_ADDR_NO_ACK: /* 48 */
		/* Doesn't seem to be a device at other end */
		drv_data->action = SUNXI_I2C_ACTION_SEND_STOP;
		drv_data->state = SUNXI_I2C_STATE_IDLE;
		drv_data->rc = -ENXIO;
		break;

	default:
		dev_err(&drv_data->adapter.dev,
			"sunxi_i2c_fsm: Ctlr Error -- state: 0x%x, "
			"status: 0x%x, addr: 0x%x, flags: 0x%x\n",
			 drv_data->state, status, drv_data->msg->addr,
			 drv_data->msg->flags);
		drv_data->action = SUNXI_I2C_ACTION_SEND_STOP;
		sunxi_i2c_hw_init(drv_data);
		drv_data->rc = -EIO;
	}
}

static void sunxi_i2c_send_start(struct sunxi_i2c_data *drv_data)
{
	drv_data->msg = drv_data->msgs;
	drv_data->byte_posn = 0;
	drv_data->bytes_left = drv_data->msg->len;
	drv_data->aborting = 0;
	drv_data->rc = 0;

	sunxi_i2c_prepare_for_io(drv_data, drv_data->msgs);
	writel(drv_data->cntl_bits | SUNXI_I2C_REG_CONTROL_START,
	       drv_data->reg_base + drv_data->reg_offsets.control);
}

static void
sunxi_i2c_do_action(struct sunxi_i2c_data *drv_data)
{
	switch(drv_data->action) {
	case SUNXI_I2C_ACTION_SEND_RESTART:
		/* We should only get here if we have further messages */
		BUG_ON(drv_data->num_msgs == 0);

		drv_data->msgs++;
		drv_data->num_msgs--;
		sunxi_i2c_send_start(drv_data);

		if (drv_data->errata_delay)
			udelay(5);

		/*
		 * We're never at the start of the message here, and by this
		 * time it's already too late to do any protocol mangling.
		 * Thankfully, do not advertise support for that feature.
		 */
		drv_data->send_stop = drv_data->num_msgs == 1;
		break;

	case SUNXI_I2C_ACTION_CONTINUE:
		writel(drv_data->cntl_bits,
			drv_data->reg_base + drv_data->reg_offsets.control);
		break;

	case SUNXI_I2C_ACTION_SEND_ADDR_1:
		writel(drv_data->addr1,
			drv_data->reg_base + drv_data->reg_offsets.data);
		writel(drv_data->cntl_bits,
			drv_data->reg_base + drv_data->reg_offsets.control);
		break;

	case SUNXI_I2C_ACTION_SEND_ADDR_2:
		writel(drv_data->addr2,
			drv_data->reg_base + drv_data->reg_offsets.data);
		writel(drv_data->cntl_bits,
			drv_data->reg_base + drv_data->reg_offsets.control);
		break;

	case SUNXI_I2C_ACTION_SEND_DATA:
		writel(drv_data->msg->buf[drv_data->byte_posn++],
			drv_data->reg_base + drv_data->reg_offsets.data);
		writel(drv_data->cntl_bits,
			drv_data->reg_base + drv_data->reg_offsets.control);
		break;

	case SUNXI_I2C_ACTION_RCV_DATA:
		drv_data->msg->buf[drv_data->byte_posn++] =
			readl(drv_data->reg_base + drv_data->reg_offsets.data);
		writel(drv_data->cntl_bits,
			drv_data->reg_base + drv_data->reg_offsets.control);
		break;

	case SUNXI_I2C_ACTION_RCV_DATA_STOP:
		drv_data->msg->buf[drv_data->byte_posn++] =
			readl(drv_data->reg_base + drv_data->reg_offsets.data);
		drv_data->cntl_bits &= ~SUNXI_I2C_REG_CONTROL_INTEN;
		writel(drv_data->cntl_bits | SUNXI_I2C_REG_CONTROL_STOP,
			drv_data->reg_base + drv_data->reg_offsets.control);
		drv_data->block = 0;
		if (drv_data->errata_delay)
			udelay(5);

		wake_up(&drv_data->waitq);
		break;

	case SUNXI_I2C_ACTION_INVALID:
	default:
		dev_err(&drv_data->adapter.dev,
			"sunxi_i2c_do_action: Invalid action: %d\n",
			drv_data->action);
		drv_data->rc = -EIO;

		/* FALLTHRU */
	case SUNXI_I2C_ACTION_SEND_STOP:
		drv_data->cntl_bits &= ~SUNXI_I2C_REG_CONTROL_INTEN;
		writel(drv_data->cntl_bits | SUNXI_I2C_REG_CONTROL_STOP,
			drv_data->reg_base + drv_data->reg_offsets.control);
		drv_data->block = 0;
		wake_up(&drv_data->waitq);
		break;
	}
}

static void
sunxi_i2c_read_offload_rx_data(struct sunxi_i2c_data *drv_data,
				 struct i2c_msg *msg)
{
	u32 buf[2];

	buf[0] = readl(drv_data->reg_base + SUNXI_I2C_REG_RX_DATA_LO);
	buf[1] = readl(drv_data->reg_base + SUNXI_I2C_REG_RX_DATA_HI);

	memcpy(msg->buf, buf, msg->len);
}

static int
sunxi_i2c_intr_offload(struct sunxi_i2c_data *drv_data)
{
	u32 cause, status;

	cause = readl(drv_data->reg_base +
		      SUNXI_I2C_REG_BRIDGE_INTR_CAUSE);
	if (!cause)
		return IRQ_NONE;

	status = readl(drv_data->reg_base +
		       SUNXI_I2C_REG_BRIDGE_STATUS);

	if (status & SUNXI_I2C_BRIDGE_STATUS_ERROR) {
		drv_data->rc = -EIO;
		goto out;
	}

	drv_data->rc = 0;

	/*
	 * Transaction is a one message read transaction, read data
	 * for this message.
	 */
	if (drv_data->num_msgs == 1 && drv_data->msgs[0].flags & I2C_M_RD) {
		sunxi_i2c_read_offload_rx_data(drv_data, drv_data->msgs);
		drv_data->msgs++;
		drv_data->num_msgs--;
	}
	/*
	 * Transaction is a two messages write/read transaction, read
	 * data for the second (read) message.
	 */
	else if (drv_data->num_msgs == 2 &&
		 !(drv_data->msgs[0].flags & I2C_M_RD) &&
		 drv_data->msgs[1].flags & I2C_M_RD) {
		sunxi_i2c_read_offload_rx_data(drv_data, drv_data->msgs + 1);
		drv_data->msgs += 2;
		drv_data->num_msgs -= 2;
	}

out:
	writel(0, drv_data->reg_base +	SUNXI_I2C_REG_BRIDGE_CONTROL);
	writel(0, drv_data->reg_base +
	       SUNXI_I2C_REG_BRIDGE_INTR_CAUSE);
	drv_data->block = 0;

	wake_up(&drv_data->waitq);

	return IRQ_HANDLED;
}

static irqreturn_t
sunxi_i2c_intr(int irq, void *dev_id)
{
	struct sunxi_i2c_data	*drv_data = dev_id;
	unsigned long	flags;
	u32		status;
	irqreturn_t	rc = IRQ_NONE;

	spin_lock_irqsave(&drv_data->lock, flags);

	if (drv_data->offload_enabled)
		rc = sunxi_i2c_intr_offload(drv_data);

	while (readl(drv_data->reg_base + drv_data->reg_offsets.control) &
						SUNXI_I2C_REG_CONTROL_IFLG) {
		status = readl(drv_data->reg_base + drv_data->reg_offsets.status);
		sunxi_i2c_fsm(drv_data, status);
		sunxi_i2c_do_action(drv_data);

		if (drv_data->irq_clear_inverted)
			writel(drv_data->cntl_bits | SUNXI_I2C_REG_CONTROL_IFLG,
			       drv_data->reg_base + drv_data->reg_offsets.control);

		rc = IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&drv_data->lock, flags);

	return rc;
}

/*
 *****************************************************************************
 *
 *	I2C Msg Execution Routines
 *
 *****************************************************************************
 */
static void
sunxi_i2c_wait_for_completion(struct sunxi_i2c_data *drv_data)
{
	long		time_left;
	unsigned long	flags;
	char		abort = 0;

	time_left = wait_event_timeout(drv_data->waitq,
		!drv_data->block, drv_data->adapter.timeout);

	spin_lock_irqsave(&drv_data->lock, flags);
	if (!time_left) { /* Timed out */
		drv_data->rc = -ETIMEDOUT;
		abort = 1;
	} else if (time_left < 0) { /* Interrupted/Error */
		drv_data->rc = time_left; /* errno value */
		abort = 1;
	}

	if (abort && drv_data->block) {
		drv_data->aborting = 1;
		spin_unlock_irqrestore(&drv_data->lock, flags);

		time_left = wait_event_timeout(drv_data->waitq,
			!drv_data->block, drv_data->adapter.timeout);

		if ((time_left <= 0) && drv_data->block) {
			drv_data->state = SUNXI_I2C_STATE_IDLE;
			dev_err(&drv_data->adapter.dev,
				"mv64xxx: I2C bus locked, block: %d, "
				"time_left: %d\n", drv_data->block,
				(int)time_left);
			sunxi_i2c_hw_init(drv_data);
		}
	} else
		spin_unlock_irqrestore(&drv_data->lock, flags);
}

static int
sunxi_i2c_execute_msg(struct sunxi_i2c_data *drv_data, struct i2c_msg *msg,
				int is_last)
{
	unsigned long	flags;

	spin_lock_irqsave(&drv_data->lock, flags);

	drv_data->state = SUNXI_I2C_STATE_WAITING_FOR_START_COND;

	drv_data->send_stop = is_last;
	drv_data->block = 1;
	sunxi_i2c_send_start(drv_data);
	spin_unlock_irqrestore(&drv_data->lock, flags);

	sunxi_i2c_wait_for_completion(drv_data);
	return drv_data->rc;
}

static void
sunxi_i2c_prepare_tx(struct sunxi_i2c_data *drv_data)
{
	struct i2c_msg *msg = drv_data->msgs;
	u32 buf[2];

	memcpy(buf, msg->buf, msg->len);

	writel(buf[0], drv_data->reg_base + SUNXI_I2C_REG_TX_DATA_LO);
	writel(buf[1], drv_data->reg_base + SUNXI_I2C_REG_TX_DATA_HI);
}

static int
sunxi_i2c_offload_xfer(struct sunxi_i2c_data *drv_data)
{
	struct i2c_msg *msgs = drv_data->msgs;
	int num = drv_data->num_msgs;
	unsigned long ctrl_reg;
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);

	/* Build transaction */
	ctrl_reg = SUNXI_I2C_BRIDGE_CONTROL_ENABLE |
		(msgs[0].addr << SUNXI_I2C_BRIDGE_CONTROL_ADDR_SHIFT);

	if (msgs[0].flags & I2C_M_TEN)
		ctrl_reg |= SUNXI_I2C_BRIDGE_CONTROL_ADDR_EXT;

	/* Single write message transaction */
	if (num == 1 && !(msgs[0].flags & I2C_M_RD)) {
		size_t len = msgs[0].len - 1;

		ctrl_reg |= SUNXI_I2C_BRIDGE_CONTROL_WR |
			(len << SUNXI_I2C_BRIDGE_CONTROL_TX_SIZE_SHIFT);
		sunxi_i2c_prepare_tx(drv_data);
	}
	/* Single read message transaction */
	else if (num == 1 && msgs[0].flags & I2C_M_RD) {
		size_t len = msgs[0].len - 1;

		ctrl_reg |= SUNXI_I2C_BRIDGE_CONTROL_RD |
			(len << SUNXI_I2C_BRIDGE_CONTROL_RX_SIZE_SHIFT);
	}
	/*
	 * Transaction with one write and one read message. This is
	 * guaranteed by the mv64xx_i2c_can_offload() checks.
	 */
	else if (num == 2) {
		size_t lentx = msgs[0].len - 1;
		size_t lenrx = msgs[1].len - 1;

		ctrl_reg |=
			SUNXI_I2C_BRIDGE_CONTROL_RD |
			SUNXI_I2C_BRIDGE_CONTROL_WR |
			(lentx << SUNXI_I2C_BRIDGE_CONTROL_TX_SIZE_SHIFT) |
			(lenrx << SUNXI_I2C_BRIDGE_CONTROL_RX_SIZE_SHIFT) |
			SUNXI_I2C_BRIDGE_CONTROL_REPEATED_START;
		sunxi_i2c_prepare_tx(drv_data);
	}

	/* Execute transaction */
	drv_data->block = 1;
	writel(ctrl_reg, drv_data->reg_base + SUNXI_I2C_REG_BRIDGE_CONTROL);
	spin_unlock_irqrestore(&drv_data->lock, flags);

	sunxi_i2c_wait_for_completion(drv_data);

	return drv_data->rc;
}

static bool
sunxi_i2c_valid_offload_sz(struct i2c_msg *msg)
{
	return msg->len <= 8 && msg->len >= 1;
}

static bool
sunxi_i2c_can_offload(struct sunxi_i2c_data *drv_data)
{
	struct i2c_msg *msgs = drv_data->msgs;
	int num = drv_data->num_msgs;

	if (!drv_data->offload_enabled)
		return false;

	/*
	 * We can offload a transaction consisting of a single
	 * message, as long as the message has a length between 1 and
	 * 8 bytes.
	 */
	if (num == 1 && sunxi_i2c_valid_offload_sz(msgs))
		return true;

	/*
	 * We can offload a transaction consisting of two messages, if
	 * the first is a write and a second is a read, and both have
	 * a length between 1 and 8 bytes.
	 */
	if (num == 2 &&
	    sunxi_i2c_valid_offload_sz(msgs) &&
	    sunxi_i2c_valid_offload_sz(msgs + 1) &&
	    !(msgs[0].flags & I2C_M_RD) &&
	    msgs[1].flags & I2C_M_RD)
		return true;

	return false;
}

/*
 *****************************************************************************
 *
 *	I2C Core Support Routines (Interface to higher level I2C code)
 *
 *****************************************************************************
 */
static u32
sunxi_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL;
}

static int
sunxi_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct sunxi_i2c_data *drv_data = i2c_get_adapdata(adap);
	int rc, ret = num;

	BUG_ON(drv_data->msgs != NULL);
	drv_data->msgs = msgs;
	drv_data->num_msgs = num;

	if (sunxi_i2c_can_offload(drv_data))
		rc = sunxi_i2c_offload_xfer(drv_data);
	else
		rc = sunxi_i2c_execute_msg(drv_data, &msgs[0], num == 1);

	if (rc < 0)
		ret = rc;

	drv_data->num_msgs = 0;
	drv_data->msgs = NULL;

	return ret;
}

static const struct i2c_algorithm sunxi_i2c_algo = {
	.master_xfer = sunxi_i2c_xfer,
	.functionality = sunxi_i2c_functionality,
};

/*
 *****************************************************************************
 *
 *	Driver Interface & Early Init Routines
 *
 *****************************************************************************
 */
static const struct of_device_id sunxi_i2c_of_match_table[] = {
	{ .compatible = "allwinner,sun4i-a10-i2c", .data = &sunxi_i2c_regs_sun4i},
	{ .compatible = "allwinner,sun6i-a31-i2c", .data = &sunxi_i2c_regs_sun4i},
	{ .compatible = "marvell,mv64xxx-i2c", .data = &sunxi_i2c_regs_mv64xxx},
	{ .compatible = "marvell,mv78230-i2c", .data = &sunxi_i2c_regs_mv64xxx},
	{ .compatible = "marvell,mv78230-a0-i2c", .data = &sunxi_i2c_regs_mv64xxx},
	{}
};
MODULE_DEVICE_TABLE(of, sunxi_i2c_of_match_table);

#ifdef CONFIG_OF
static int
mv64xxx_calc_freq(struct sunxi_i2c_data *drv_data,
		  const int tclk, const int n, const int m)
{
	if (drv_data->clk_n_base_0)
		return tclk / (10 * (m + 1) * (1 << n));
	else
		return tclk / (10 * (m + 1) * (2 << n));
}

static bool
mv64xxx_find_baud_factors(struct sunxi_i2c_data *drv_data,
			  const u32 req_freq, const u32 tclk)
{
	int freq, delta, best_delta = INT_MAX;
	int m, n;

	for (n = 0; n <= 7; n++)
		for (m = 0; m <= 15; m++) {
			freq = mv64xxx_calc_freq(drv_data, tclk, n, m);
			delta = req_freq - freq;
			if (delta >= 0 && delta < best_delta) {
				drv_data->freq_m = m;
				drv_data->freq_n = n;
				best_delta = delta;
			}
			if (best_delta == 0)
				return true;
		}
	if (best_delta == INT_MAX)
		return false;
	return true;
}

static int
mv64xxx_of_config(struct sunxi_i2c_data *drv_data,
		  struct device *dev)
{
	const struct of_device_id *device;
	struct device_node *np = dev->of_node;
	u32 bus_freq, tclk;
	int rc = 0;

	/* CLK is mandatory when using DT to describe the i2c bus. We
	 * need to know tclk in order to calculate bus clock
	 * factors.
	 */
	if (IS_ERR(drv_data->clk)) {
		rc = -ENODEV;
		goto out;
	}
	tclk = clk_get_rate(drv_data->clk);

	if (of_property_read_u32(np, "clock-frequency", &bus_freq))
		bus_freq = 100000; /* 100kHz by default */

	if (of_device_is_compatible(np, "allwinner,sun4i-a10-i2c") ||
	    of_device_is_compatible(np, "allwinner,sun6i-a31-i2c"))
		drv_data->clk_n_base_0 = true;

	if (!mv64xxx_find_baud_factors(drv_data, bus_freq, tclk)) {
		rc = -EINVAL;
		goto out;
	}

	drv_data->rstc = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(drv_data->rstc)) {
		rc = PTR_ERR(drv_data->rstc);
		goto out;
	}
	reset_control_deassert(drv_data->rstc);

	/* Its not yet defined how timeouts will be specified in device tree.
	 * So hard code the value to 1 second.
	 */
	drv_data->adapter.timeout = HZ;

	device = of_match_device(sunxi_i2c_of_match_table, dev);
	if (!device)
		return -ENODEV;

	memcpy(&drv_data->reg_offsets, device->data, sizeof(drv_data->reg_offsets));

	/*
	 * For controllers embedded in new SoCs activate the
	 * Transaction Generator support and the errata fix.
	 */
	if (of_device_is_compatible(np, "marvell,mv78230-i2c")) {
		drv_data->offload_enabled = true;
		/* The delay is only needed in standard mode (100kHz) */
		if (bus_freq <= 100000)
			drv_data->errata_delay = true;
	}

	if (of_device_is_compatible(np, "marvell,mv78230-a0-i2c")) {
		drv_data->offload_enabled = false;
		/* The delay is only needed in standard mode (100kHz) */
		if (bus_freq <= 100000)
			drv_data->errata_delay = true;
	}

	if (of_device_is_compatible(np, "allwinner,sun6i-a31-i2c"))
		drv_data->irq_clear_inverted = true;

out:
	return rc;
}
#else /* CONFIG_OF */
static int
mv64xxx_of_config(struct sunxi_i2c_data *drv_data,
		  struct device *dev)
{
	return -ENODEV;
}
#endif /* CONFIG_OF */

static int
sunxi_i2c_probe(struct platform_device *pd)
{
	struct sunxi_i2c_data		*drv_data;
	struct sunxi_i2c_pdata	*pdata = dev_get_platdata(&pd->dev);
	struct resource	*r;
	int	rc;

	if ((!pdata && !pd->dev.of_node))
		return -ENODEV;

	drv_data = devm_kzalloc(&pd->dev, sizeof(struct sunxi_i2c_data),
				GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	r = platform_get_resource(pd, IORESOURCE_MEM, 0);
	drv_data->reg_base = devm_ioremap_resource(&pd->dev, r);
	if (IS_ERR(drv_data->reg_base))
		return PTR_ERR(drv_data->reg_base);

	strlcpy(drv_data->adapter.name, SUNXI_I2C_CTLR_NAME " adapter",
		sizeof(drv_data->adapter.name));

	init_waitqueue_head(&drv_data->waitq);
	spin_lock_init(&drv_data->lock);

	/* Not all platforms have clocks */
	drv_data->clk = devm_clk_get(&pd->dev, NULL);
	if (IS_ERR(drv_data->clk) && PTR_ERR(drv_data->clk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!IS_ERR(drv_data->clk))
		clk_prepare_enable(drv_data->clk);

	drv_data->reg_clk = devm_clk_get(&pd->dev, "reg");
	if (IS_ERR(drv_data->reg_clk) &&
	    PTR_ERR(drv_data->reg_clk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!IS_ERR(drv_data->reg_clk))
		clk_prepare_enable(drv_data->reg_clk);

	drv_data->irq = platform_get_irq(pd, 0);

	if (pdata) {
		drv_data->freq_m = pdata->freq_m;
		drv_data->freq_n = pdata->freq_n;
		drv_data->adapter.timeout = msecs_to_jiffies(pdata->timeout);
		drv_data->offload_enabled = false;
		memcpy(&drv_data->reg_offsets, &sunxi_i2c_regs_mv64xxx, sizeof(drv_data->reg_offsets));
	} else if (pd->dev.of_node) {
		rc = mv64xxx_of_config(drv_data, &pd->dev);
		if (rc)
			goto exit_clk;
	}
	if (drv_data->irq < 0) {
		rc = drv_data->irq;
		goto exit_reset;
	}

	drv_data->adapter.dev.parent = &pd->dev;
	drv_data->adapter.algo = &sunxi_i2c_algo;
	drv_data->adapter.owner = THIS_MODULE;
	drv_data->adapter.class = I2C_CLASS_DEPRECATED;
	drv_data->adapter.nr = pd->id;
	drv_data->adapter.dev.of_node = pd->dev.of_node;
	platform_set_drvdata(pd, drv_data);
	i2c_set_adapdata(&drv_data->adapter, drv_data);

	sunxi_i2c_hw_init(drv_data);

	rc = request_irq(drv_data->irq, sunxi_i2c_intr, 0,
			 SUNXI_I2C_CTLR_NAME, drv_data);
	if (rc) {
		dev_err(&drv_data->adapter.dev,
			"mv64xxx: Can't register intr handler irq%d: %d\n",
			drv_data->irq, rc);
		goto exit_reset;
	} else if ((rc = i2c_add_numbered_adapter(&drv_data->adapter)) != 0) {
		dev_err(&drv_data->adapter.dev,
			"mv64xxx: Can't add i2c adapter, rc: %d\n", -rc);
		goto exit_free_irq;
	}

	return 0;

exit_free_irq:
	free_irq(drv_data->irq, drv_data);
exit_reset:
	reset_control_assert(drv_data->rstc);
exit_clk:
	clk_disable_unprepare(drv_data->reg_clk);
	clk_disable_unprepare(drv_data->clk);

	return rc;
}

static int
sunxi_i2c_remove(struct platform_device *dev)
{
	struct sunxi_i2c_data		*drv_data = platform_get_drvdata(dev);

	i2c_del_adapter(&drv_data->adapter);
	free_irq(drv_data->irq, drv_data);
	reset_control_assert(drv_data->rstc);
	clk_disable_unprepare(drv_data->reg_clk);
	clk_disable_unprepare(drv_data->clk);

	return 0;
}

#ifdef CONFIG_PM
static int sunxi_i2c_resume(struct device *dev)
{
	struct sunxi_i2c_data *drv_data = dev_get_drvdata(dev);

	sunxi_i2c_hw_init(drv_data);

	return 0;
}

static const struct dev_pm_ops sunxi_i2c_pm = {
	.resume = sunxi_i2c_resume,
};

#define sunxi_i2c_pm_ops (&sunxi_i2c_pm)
#else
#define sunxi_i2c_pm_ops NULL
#endif

static struct platform_driver sunxi_i2c_driver = {
	.probe	= sunxi_i2c_probe,
	.remove	= sunxi_i2c_remove,
	.driver	= {
		.name	= SUNXI_I2C_CTLR_NAME,
		.pm     = sunxi_i2c_pm_ops,
		.of_match_table = sunxi_i2c_of_match_table,
	},
};

module_platform_driver(sunxi_i2c_driver);

MODULE_AUTHOR("ZZW <1600284146@qq.com>");
MODULE_DESCRIPTION("Allwinner host bridge i2c ctlr driver");
MODULE_LICENSE("GPL");
