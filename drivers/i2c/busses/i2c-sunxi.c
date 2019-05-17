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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/types.h>

#define SUNXI_I2C_CTLR_NAME	"sunxi_i2c"

#define SUNXI_I2C_ADDR_ADDR(val)			((val & 0x7f) << 1)
#define SUNXI_I2C_BAUD_DIV_N(val)			(val & 0x7)
#define SUNXI_I2C_BAUD_DIV_M(val)			((val & 0xf) << 3)

#define	SUNXI_I2C_REG_CONTROL_ACK			BIT(2)
#define	SUNXI_I2C_REG_CONTROL_IFLG			BIT(3)
#define	SUNXI_I2C_REG_CONTROL_STOP			BIT(4)
#define	SUNXI_I2C_REG_CONTROL_START			BIT(5)
#define	SUNXI_I2C_REG_CONTROL_TWSIEN		BIT(6)
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



/* i2c Platform Device, Driver Data */
struct sunxi_i2c_pdata {
	u32	freq_m;
	u32	freq_n;
	u32	timeout;	/* In milliseconds */
};

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



/*i2c相关寄存器*/
struct sunxi_i2c_regs {
	u8	addr;
	u8	ext_addr;
	u8	data;
	u8	control;
	u8	status;
	u8	clock;
	u8	soft_reset;
};

/*i2c相关寄存器偏移量*/
static struct sunxi_i2c_regs sunxi_i2c_regs_sun4i = {
	.addr		= 0x00,
	.ext_addr	= 0x04,
	.data		= 0x08,
	.control	= 0x0c,
	.status		= 0x10,
	.clock		= 0x14,
	.soft_reset	= 0x18,
};

struct sunxi_i2c_data {

	struct i2c_msg		*msgs; 		/*接收到的i2c消息数组*/
	int			num_msgs; 			/*接收到i2c消息的个数*/
	struct i2c_msg		*msg;  		/*表示单个i2c消息*/

	u32			addr1;				/*从设备地址，7位地址用addr1表示，10位地址用addr1和addr2表示*/
	u32			addr2;

	u32			state; 				/*I2C状态位*/	
	u32			action;				/*i2c需要执行的动作*/
	u32			send_stop; 			/*判断是否需要发送停止位*/
	u32			block;				/*阻塞标志位,i2c数据发送开始置1，发送完成会置0*/
	u32			aborting; 			/*异常标志位*/
	u32			bytes_left; 		/*需要发送的i2c消息的总字节数*/
	u32			byte_posn; 			/*记录发送i2c消息时发送到那个字节*/
	u32			cntl_bits; 			/*控制寄存器相关状态位*/
	int			rc; 				/*发送消息过程中的返回值*/
	
	int			irq; 				/*中断irq*/

	void __iomem		*reg_base;	/*保存i2c寄存器基地址*/
	struct sunxi_i2c_regs	reg_offsets; /*寄存器偏移量*/


	struct i2c_adapter	adapter;  	/*i2c设配器类，在/include/linux/i2c.h中定义*/


	struct clk   *clk; 				/*i2c时钟*/
	wait_queue_head_t	waitq;	    /*等待队列*/
	spinlock_t		lock; 			/*spinlock锁*/



	u32			freq_m; 			/*I2C时钟寄存器分频系数CLK_M*/
	u32			freq_n;				/*I2C时钟寄存器分频系数CLK_N*/
	
	bool			errata_delay; 	/* 5us延迟以避免重复启动时序违规 */
	struct reset_control	*rstc; 	/*i2c复位控制*/
};




/* Reset hardware and initialize FSM */
static void
sunxi_i2c_hw_init(struct sunxi_i2c_data *drv_data)
{
	/*软件复位寄存器设置0*/
	writel(0, drv_data->reg_base + drv_data->reg_offsets.soft_reset);

	/*设置i2c时钟寄存器，频率100khz*/
	writel(SUNXI_I2C_BAUD_DIV_M(drv_data->freq_m) | SUNXI_I2C_BAUD_DIV_N(drv_data->freq_n),
		drv_data->reg_base + drv_data->reg_offsets.clock);

	/*设置从设备地址寄存器为0*/
	writel(0, drv_data->reg_base + drv_data->reg_offsets.addr);
	writel(0, drv_data->reg_base + drv_data->reg_offsets.ext_addr);
	/*设置控制寄存器，设置i2C为主设备*/
	writel(SUNXI_I2C_REG_CONTROL_TWSIEN | SUNXI_I2C_REG_CONTROL_STOP,
		drv_data->reg_base + drv_data->reg_offsets.control);
	/*初始化i2c状态*/
	drv_data->state = SUNXI_I2C_STATE_IDLE;
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

	/*等待i2c发送完成或超时退出*/
	time_left = wait_event_timeout(drv_data->waitq,
		!drv_data->block, drv_data->adapter.timeout);

	/*上锁*/
	spin_lock_irqsave(&drv_data->lock, flags);
	if (!time_left) { /* time_left返回0，超时*/
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
				"sunxi_i2c: I2C bus locked, block: %d, "
				"time_left: %d\n", drv_data->block,
				(int)time_left);
			/*重新初始化i2c*/
			sunxi_i2c_hw_init(drv_data);
		}
	} else {
		spin_unlock_irqrestore(&drv_data->lock, flags);
	}
}


static void
sunxi_i2c_prepare_for_io(struct sunxi_i2c_data *drv_data,
	struct i2c_msg *msg)
{
	u32	dir = 0;

	/*设置应答，使能中断，设置主设备*/
	drv_data->cntl_bits = SUNXI_I2C_REG_CONTROL_ACK |
		SUNXI_I2C_REG_CONTROL_INTEN | SUNXI_I2C_REG_CONTROL_TWSIEN;

	if (msg->flags & I2C_M_RD) /*判断消息读写方向*/
		dir = 1;	/*是读消息*/

	if (msg->flags & I2C_M_TEN) { /*判断从设备地址是否是10位*/
		drv_data->addr1 = 0xf0 | (((u32)msg->addr & 0x300) >> 7) | dir;
		drv_data->addr2 = (u32)msg->addr & 0xff;
	} else {
		drv_data->addr1 = SUNXI_I2C_ADDR_ADDR((u32)msg->addr) | dir;
		drv_data->addr2 = 0;
	}
}

static void sunxi_i2c_send_start(struct sunxi_i2c_data *drv_data)
{
	drv_data->msg = drv_data->msgs;
	drv_data->byte_posn = 0;
	drv_data->bytes_left = drv_data->msg->len;
	drv_data->aborting = 0; 	
	drv_data->rc = 0;

	/*设置从设备地址和消息的发送方向*/
	sunxi_i2c_prepare_for_io(drv_data, drv_data->msgs);

	/*设置控制寄存器的值，设置SUNXI_I2C_REG_CONTROL_START即M_STA置1，根据控制寄存器该位的描述，
	  M_STA置1，控制寄存器进入master主设备模式，同时状态寄存器进入开始发送状态0x08或重复开始发送状态0x10,
	  状态寄存器进入这些状态会导致控制寄存器中断标志位INT_FLAG置1，进入中断处理函数，开始传输数据*/
	writel(drv_data->cntl_bits | SUNXI_I2C_REG_CONTROL_START,
	       drv_data->reg_base + drv_data->reg_offsets.control);
}

static int
sunxi_i2c_execute_msg(struct sunxi_i2c_data *drv_data, int is_last)
{
	unsigned long	flags;

	/*处理msg中，上锁保护该代码段*/
	spin_lock_irqsave(&drv_data->lock, flags);

	drv_data->state = SUNXI_I2C_STATE_WAITING_FOR_START_COND;

	drv_data->send_stop = is_last;	/*判断是否需要发送停止位*/
	drv_data->block = 1;

	/*开始发送i2c消息*/
	sunxi_i2c_send_start(drv_data);

	/*解锁*/
	spin_unlock_irqrestore(&drv_data->lock, flags);

	sunxi_i2c_wait_for_completion(drv_data);
	return drv_data->rc;
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
	/*返回I2C支持的通信协议*/
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL;
}

/*处理i2c_adapter传递过来的每个i2c_mgs*/
static int
sunxi_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct sunxi_i2c_data *drv_data = i2c_get_adapdata(adap);
	int rc, ret = num;

	BUG_ON(drv_data->msgs != NULL); /*判断接收到的msgs是否为空*/
	drv_data->msgs = msgs;
	drv_data->num_msgs = num;

	/*处理i2c 消息*/
	rc = sunxi_i2c_execute_msg(drv_data, num == 1);

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

		/* 传输完成，设置i2c控制寄存器，关闭中断使能、停止主设备模式 */
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


	/*根据状态寄存器判断需要执行的操作*/
	/* The status from the ctlr [mostly] tells us what to do next */
	switch (status) {
	/* Start condition interrupt */
	case SUNXI_I2C_STATUS_MAST_START: /* 0x08 */
	case SUNXI_I2C_STATUS_MAST_REPEAT_START: /* 0x10 */
		drv_data->action = SUNXI_I2C_ACTION_SEND_ADDR_1; 	/*发送从设备7位地址*/
		drv_data->state = SUNXI_I2C_STATE_WAITING_FOR_ADDR_1_ACK;
		break;

	/* Performing a write */
	case SUNXI_I2C_STATUS_MAST_WR_ADDR_ACK: /* 0x18 */
		if (drv_data->msg->flags & I2C_M_TEN) {
			drv_data->action = SUNXI_I2C_ACTION_SEND_ADDR_2; 	/*发送从设备10位地址*/
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


/*
 *****************************************************************************
 *
 *	Finite State Machine & Interrupt Routines
 *
 *****************************************************************************
 */

/*sunxi i2c中断处理函数*/
static irqreturn_t
sunxi_i2c_intr(int irq, void *dev_id)
{
	struct sunxi_i2c_data	*drv_data = dev_id; 	/*在request_irq是传递的最后一个参数*/
	unsigned long	flags;
	u32		status;
	irqreturn_t	rc = IRQ_NONE;

	spin_lock_irqsave(&drv_data->lock, flags);

	while (readl(drv_data->reg_base + drv_data->reg_offsets.control) &
						SUNXI_I2C_REG_CONTROL_IFLG) { /*判断控制寄存器中断标志位是否置1*/
		status = readl(drv_data->reg_base + drv_data->reg_offsets.status); /*读取状态寄存器的值*/
		sunxi_i2c_fsm(drv_data, status);
		sunxi_i2c_do_action(drv_data);

		/*重新置位SUNXI_I2C_REG_CONTROL_IFLG，触发中断继续发送i2c消息，
		当传输完成时会关闭中断使能，设置该标志位不会再触发中断*/
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
 *	Driver Interface & Early Init Routines
 *
 *****************************************************************************
 */
static const struct of_device_id sunxi_i2c_of_match_table[] = {
	{ .compatible = "allwinner,sun6i-a31-i2c" , .data = &sunxi_i2c_regs_sun4i},
	{}
};
MODULE_DEVICE_TABLE(of, sunxi_i2c_of_match_table);




#ifdef CONFIG_OF
static int
sunxi_calc_freq(struct sunxi_i2c_data *drv_data,
		  const int tclk, const int n, const int m)
{
		return tclk / (10 * (m + 1) * (1 << n));
}

static bool
sunxi_find_baud_factors(struct sunxi_i2c_data *drv_data,
			  const u32 req_freq, const u32 tclk)
{
	int freq, delta, best_delta = INT_MAX;
	int m, n;

	for (n = 0; n <= 7; n++)
		for (m = 0; m <= 15; m++) {
			freq = sunxi_calc_freq(drv_data, tclk, n, m);
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
sunxi_of_config(struct sunxi_i2c_data *drv_data,
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

	/*获取的tclk=24MHZ*/
	tclk = clk_get_rate(drv_data->clk);

	/*设备树i2c没有配置clock-frequency选项，获取的bus_freq=0*/
	if (of_property_read_u32(np, "clock-frequency", &bus_freq)) {
		/*设置默认总线频率为100khz*/
		bus_freq = 100000; /* 100kHz by default */
	}

	/*根据bus_freq的值计算出i2c时钟寄存器分频系数freq_m和freq_n，在硬件初始化时配置时钟寄存器*/
	if (!sunxi_find_baud_factors(drv_data, bus_freq, tclk)) {
		rc = -EINVAL;
		goto out;
	}

	/*复位控制，设备树配置resets = <&ccu RST_BUS_I2C1>;没有配置reset-names 所以第二个参数为空*/
	drv_data->rstc = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(drv_data->rstc)) {
		rc = PTR_ERR(drv_data->rstc);
		goto out;
	}
	reset_control_deassert(drv_data->rstc);

	/* Its not yet defined how timeouts will be specified in device tree.
	 * So hard code the value to 1 second.
	 */
	/*设置adapter超时时间默认为1s*/
	drv_data->adapter.timeout = HZ;

	device = of_match_device(sunxi_i2c_of_match_table, dev);
	if (!device)
		return -ENODEV;

	/*初始化寄存器偏移地址，device->data是sunxi_i2c_of_match_table的data成员*/
	memcpy(&drv_data->reg_offsets, device->data, sizeof(drv_data->reg_offsets));

	out:
		return rc;
}
#else /* CONFIG_OF */
static int
sunxi_of_config(struct sunxi_i2c_data *drv_data,
		  struct device *dev)
{
	return -ENODEV;
}
#endif /* CONFIG_OF */


static int
sunxi_i2c_probe(struct platform_device *pd)
{
	/* 
	*pdata = pdev->dev.platform_data
	在没有设备树时在arch/arm/mach-xx/目录下定义platform_device（即这里的pd->dev），
	定义platform_device设备时初始化dev.platform_data，在这里通过dev_get_platdata获取到这些私有数据，
	所以如何定义struct sunxi_i2c_pdata的成员，跟定义platform_device时初始化dev.platform_data成员时赋值的结构体有关，
	即需要传递什么数据，这里就怎么定义struct sunxi_i2c_pdata来获取数据。
	pd->dev.of_node
	有设备树时，platform_device的初始化在解析dtb文件时已经初始化，这时调用dev_get_platdata(&pd->dev);返回是NULL指针
	pd->dev.of_node)返回为true说明platform_device由设备树解析初始化，通过设备树node获取。其中初始化平台总线，将平台设备挂载
	到平台总线的代码在/drivers/of目录实现
	*/
	struct sunxi_i2c_pdata	*pdata = dev_get_platdata(&pd->dev);
	struct sunxi_i2c_data		*drv_data;
	struct resource	*r;
	int	rc;

	if ((!pdata && !pd->dev.of_node)) /*从pdata获取数据或从设备树获取数据，如果都没数据返回-ENODEV*/
		return -ENODEV;

	/*1、为drv_data分配内存*/
	drv_data = devm_kzalloc(&pd->dev, sizeof(struct sunxi_i2c_data),
				GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	/*2、映射I2C寄存器地址
	设备树配置了reg = <0x01c2b000 0x400>; 两个参数i2c1的起始地址和范围，跟A31芯片数据手册Memory Mapping对应*/
	r = platform_get_resource(pd, IORESOURCE_MEM, 0); 
	printk(KERN_EMERG  "name = %s start_addr = 0x%x , end_addr = 0x%x\n",r->name, (unsigned int)r->start, (unsigned int)r->end);
	drv_data->reg_base = devm_ioremap_resource(&pd->dev, r); 
	if (IS_ERR(drv_data->reg_base))
		return PTR_ERR(drv_data->reg_base);

	/*3、初始化队列和锁*/
	init_waitqueue_head(&drv_data->waitq); 		/*初始化等待队列*/
	spin_lock_init(&drv_data->lock);  			/*初始化锁*/


	/*4、配置I2C时钟
	设备树使用clocks = <&ccu CLK_BUS_I2C0>配置了时钟，没有配置时钟名称clock-names，所以第二个参数为空*/
	drv_data->clk = devm_clk_get(&pd->dev, NULL);
	if (IS_ERR(drv_data->clk) && PTR_ERR(drv_data->clk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!IS_ERR(drv_data->clk))
		clk_prepare_enable(drv_data->clk);

	if (pdata) { /*保存私有数据到drv_data*/
		drv_data->freq_m = pdata->freq_m; 	/*时钟分频系数clk_m*/
		drv_data->freq_n = pdata->freq_n; 	/*时钟分频系数clk_n*/
		drv_data->adapter.timeout = msecs_to_jiffies(pdata->timeout); 	/*发送消息等待超时时间*/
	} else if (pd->dev.of_node) {
		//配置时钟频率
		rc = sunxi_of_config(drv_data, &pd->dev);
		if (rc)
			goto exit_clk;
	}

	/*5、初始化i2c_adapter*/
	strlcpy(drv_data->adapter.name, SUNXI_I2C_CTLR_NAME " adapter", sizeof(drv_data->adapter.name)); /*设置i2c适配器名称*/
	drv_data->adapter.dev.parent = &pd->dev; 	/*设置I2C适配器的设备的父亲为pd->dev*/
	drv_data->adapter.algo = &sunxi_i2c_algo; 	/*设置I2C适配器的算法成员，这个是linux的I2C驱动的核心*/
	drv_data->adapter.owner = THIS_MODULE;
	drv_data->adapter.class = I2C_CLASS_DEPRECATED; 	/*提示用户i2c_adapter不再支持classes*/
	drv_data->adapter.nr = pd->id; 						/*设置i2c_adapter的次设备号，这里pd->id=-1，表示动态分配总线次设备号*/
	drv_data->adapter.dev.of_node = pd->dev.of_node;
	platform_set_drvdata(pd, drv_data);					/*将drv_data设置到pd->dev->driver_data*/
	i2c_set_adapdata(&drv_data->adapter, drv_data); 	/*将drv_data设置到adapter->dev->driver_data*/

	/*6、硬件初始化*/
	sunxi_i2c_hw_init(drv_data);

	
	/*7、中断配置*/
	drv_data->irq = platform_get_irq(pd, 0); 		/*获取中断irq*/
	printk(KERN_EMERG  "i2c get irq  = %d\n", drv_data->irq);
	if (drv_data->irq < 0) {
		rc = drv_data->irq;
		goto exit_reset;
	}
	rc = request_irq(drv_data->irq, sunxi_i2c_intr, 0,
			 SUNXI_I2C_CTLR_NAME, drv_data);
	if (rc) {
		dev_err(&drv_data->adapter.dev,
			"sunxi_i2c: Can't register interrupt handler irq%d: %d\n",
			drv_data->irq, rc);
		goto exit_reset;
	/*i2c_add_numbered_adapter在I2C总线下创建设备*/
	} else if ((rc = i2c_add_numbered_adapter(&drv_data->adapter)) != 0) {
		dev_err(&drv_data->adapter.dev,
			"sunxi_i2c: Can't add i2c adapter, rc: %d\n", -rc);
		goto exit_free_irq;
	}

	printk(KERN_EMERG  "sunxi i2c probe success\n");
	return 0;

exit_free_irq:
	free_irq(drv_data->irq, drv_data);
exit_reset:
	reset_control_assert(drv_data->rstc);
exit_clk:
	clk_disable_unprepare(drv_data->clk);

	return rc;
}

static int
sunxi_i2c_remove(struct platform_device *dev)
{
	struct sunxi_i2c_data		*drv_data = platform_get_drvdata(dev);

	/*注销适配器i2c_adapter*/
	i2c_del_adapter(&drv_data->adapter);

	/*注销中断资源*/
	free_irq(drv_data->irq, drv_data);

	/*注销复位控制器资源*/
	reset_control_assert(drv_data->rstc);

	/*关闭时钟*/
	clk_disable_unprepare(drv_data->clk);

	printk(KERN_EMERG "sunxi i2c remove success\n");
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
