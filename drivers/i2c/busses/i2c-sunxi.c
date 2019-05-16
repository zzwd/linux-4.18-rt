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

#include <linux/types.h>

#define SUNXI_I2C_CTLR_NAME	"sunxi_i2c"

#define SUNXI_I2C_ADDR_ADDR(val)			((val & 0x7f) << 1)
#define SUNXI_I2C_BAUD_DIV_N(val)			(val & 0x7)
#define SUNXI_I2C_BAUD_DIV_M(val)			((val & 0xf) << 3)

#define	SUNXI_I2C_REG_CONTROL_ACK			BIT(2)
#define	SUNXI_I2C_REG_CONTROL_IFLG			BIT(3)
#define	SUNXI_I2C_REG_CONTROL_STOP			BIT(4)
#define	SUNXI_I2C_REG_CONTROL_START			BIT(5)
#define	SUNXI_I2C_REG_CONTROL_TWSIEN			BIT(6)
#define	SUNXI_I2C_REG_CONTROL_INTEN			BIT(7)

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

	u32			state; 				/*I2C状态位*/	
	
	int			irq; 				/*中断irq*/

	void __iomem		*reg_base;	/*保存i2c寄存器基地址*/
	struct sunxi_i2c_regs	reg_offsets; /*寄存器偏移量*/


	struct i2c_adapter	adapter;  	/*i2c设配器类，在/include/linux/i2c.h中定义*/


	struct clk   *clk; 				/*时钟*/
	struct clk   *reg_clk; 			/*寄存器时钟*/
	wait_queue_head_t	waitq;	    /*等待队列*/
	spinlock_t		lock; 			/*spinlock锁*/



	u32			freq_m; 			/*与struct sunxi_i2c_pdata中的freq_m成员对应，用来保存platform_device传递过来的私有数据*/
	u32			freq_n;				/*与struct sunxi_i2c_pdata中的freq_n成员对应，用来保存platform_device传递过来的私有数据*/
	bool		offload_enabled;
	
	bool			errata_delay; 	/* 5us延迟以避免重复启动时序违规 */
	struct reset_control	*rstc;
	bool			irq_clear_inverted;
	/* Clk div is 2 to the power n, not 2 to the power n + 1 */
	bool			clk_n_base_0;
};



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

	return 0;
}

static const struct i2c_algorithm sunxi_i2c_algo = {
	.master_xfer = sunxi_i2c_xfer,
	.functionality = sunxi_i2c_functionality,
};


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
	writel(0, drv_data->reg_base + drv_data->reg_offsets.soft_reset);
	writel(SUNXI_I2C_BAUD_DIV_M(drv_data->freq_m) | SUNXI_I2C_BAUD_DIV_N(drv_data->freq_n),
		drv_data->reg_base + drv_data->reg_offsets.clock);
	writel(0, drv_data->reg_base + drv_data->reg_offsets.addr);
	writel(0, drv_data->reg_base + drv_data->reg_offsets.ext_addr);
	writel(SUNXI_I2C_REG_CONTROL_TWSIEN | SUNXI_I2C_REG_CONTROL_STOP,
		drv_data->reg_base + drv_data->reg_offsets.control);
	drv_data->state = SUNXI_I2C_STATE_IDLE;
}


/*sunxi i2c中断处理函数*/
static irqreturn_t
sunxi_i2c_intr(int irq, void *dev_id)
{
	return 0;
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
	if (drv_data->clk_n_base_0)
		return tclk / (10 * (m + 1) * (1 << n));
	else
		return tclk / (10 * (m + 1) * (2 << n));
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
	tclk = clk_get_rate(drv_data->clk);

	if (of_property_read_u32(np, "clock-frequency", &bus_freq))
		bus_freq = 100000; /* 100kHz by default */

	if (of_device_is_compatible(np, "allwinner,sun6i-a31-i2c"))
		drv_data->clk_n_base_0 = true;

	if (!sunxi_find_baud_factors(drv_data, bus_freq, tclk)) {
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
	if (of_device_is_compatible(np, "allwinner,sun6i-a31-i2c"))
		drv_data->irq_clear_inverted = true;

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
	1、*pdata = pdev->dev.platform_data
	在没有设备树时在arch/arm/mach-xx/目录下定义platform_device（即这里的pd->dev），
	定义platform_device设备时初始化dev.platform_data，在这里通过dev_get_platdata获取到这些私有数据，
	所以如何定义struct sunxi_i2c_pdata的成员，跟定义platform_device时初始化dev.platform_data成员时赋值的结构体有关，
	即需要传递什么数据，这里就怎么定义struct sunxi_i2c_pdata来获取数据。
	2、pd->dev.of_node
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

	/*为drv_data分配内存*/
	drv_data = devm_kzalloc(&pd->dev, sizeof(struct sunxi_i2c_data),
				GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	r = platform_get_resource(pd, IORESOURCE_MEM, 0); /*从设备树获取IORESOURCE_MEM设备资源*/
	printk(KERN_EMERG  "name = %s start_addr = 0x%x , end_addr = 0x%x\n",r->name, (unsigned int)r->start, (unsigned int)r->end);
	drv_data->reg_base = devm_ioremap_resource(&pd->dev, r); /*映射i2c寄存器地址空间*/
	if (IS_ERR(drv_data->reg_base))
		return PTR_ERR(drv_data->reg_base);

	/*初始化i2c_adapter.name成员*/
	strlcpy(drv_data->adapter.name, SUNXI_I2C_CTLR_NAME " adapter",
		sizeof(drv_data->adapter.name));
	printk(KERN_EMERG  "i2c_adapter name = %s\n", drv_data->adapter.name);


	/*初始化等待队列*/
	init_waitqueue_head(&drv_data->waitq);
	/*初始化锁*/
	spin_lock_init(&drv_data->lock);


	/* Not all platforms have clocks ,申请时钟，跟/driver/clk驱动有关*/
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

	/*获取中断irq*/
	drv_data->irq = platform_get_irq(pd, 0);
	printk(KERN_EMERG  "i2c get irq  = %d\n", drv_data->irq);
	if (drv_data->irq < 0) {
		rc = drv_data->irq;
		goto exit_reset;
	}

	if (pdata) { /*保存私有数据到drv_data*/
		drv_data->freq_m = pdata->freq_m;
		drv_data->freq_n = pdata->freq_n;
		drv_data->adapter.timeout = msecs_to_jiffies(pdata->timeout);
		drv_data->offload_enabled = false;
	} else if (pd->dev.of_node) {
		rc = sunxi_of_config(drv_data, &pd->dev);
		if (rc)
			goto exit_clk;
	}

	/*初始化i2c_adapter*/
	drv_data->adapter.dev.parent = &pd->dev; 	/*设置I2C适配器的设备的父亲为pd->dev*/
	drv_data->adapter.algo = &sunxi_i2c_algo; 	/*设置I2C适配器的算法成员，这个是linux的I2C驱动的核心*/
	drv_data->adapter.owner = THIS_MODULE;
	drv_data->adapter.class = I2C_CLASS_DEPRECATED; 	/*提示用户i2c_adapter不再支持classes*/
	drv_data->adapter.nr = pd->id; 						/*设置i2c_adapter的次设备号*/
	drv_data->adapter.dev.of_node = pd->dev.of_node;
	platform_set_drvdata(pd, drv_data);					/*将drv_data设置到pd->dev->driver_data*/
	i2c_set_adapdata(&drv_data->adapter, drv_data); 	/*将drv_data设置到adapter->dev->driver_data*/

	/*硬件初始化*/
	sunxi_i2c_hw_init(drv_data);


	/*申请中断*/
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
	clk_disable_unprepare(drv_data->reg_clk);
	clk_disable_unprepare(drv_data->clk);

	return rc;
}

static int
sunxi_i2c_remove(struct platform_device *dev)
{
	printk(KERN_EMERG "sunxi i2c remove success\n");
	return 0;
}

#ifdef CONFIG_PM
static int sunxi_i2c_resume(struct device *dev)
{
	printk(KERN_EMERG "sunxi i2c resume success\n");
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
