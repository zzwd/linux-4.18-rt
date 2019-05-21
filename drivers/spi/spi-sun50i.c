/*
 * Copyright (C) 2012 - 2014 Allwinner Tech
 * Pan Nan <pannan@allwinnertech.com>
 *
 * Copyright (C) 2014 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#define SUN6I_FIFO_DEPTH			128
#define SUN8I_FIFO_DEPTH			64

/*spi 数据全局控制寄存器*/
#define SUN50I_GBL_CTL_REG			0x04
#define SUN50I_GBL_CTL_BUS_ENABLE	BIT(0)
#define SUN50I_GBL_CTL_MASTER		BIT(1)
#define SUN50I_GBL_CTL_TP			BIT(7)
#define SUN50I_GBL_CTL_RST			BIT(31)

/*spi 数据传输控制寄存器*/
#define SUN50I_CTL_REG				0x08
#define SUN50I_CTL_ENABLE			BIT(0)
#define SUN50I_CTL_MASTER			BIT(1)
#define SUN50I_CTL_CPHA				BIT(2)
#define SUN50I_CTL_CPOL				BIT(3)
#define SUN50I_CTL_CS_ACTIVE_LOW	BIT(4)
#define SUN50I_CTL_LMTF				BIT(6)
#define SUN50I_CTL_TF_RST			BIT(8)
#define SUN50I_CTL_RF_RST			BIT(9)
#define SUN50I_CTL_XCH				BIT(10)
#define SUN50I_CTL_CS_MASK			0x3000
#define SUN50I_CTL_CS(cs)			(((cs) << 12) & SUN50I_CTL_CS_MASK)
#define SUN50I_CTL_DHB				BIT(15)
#define SUN50I_CTL_CS_MANUAL		BIT(16)
#define SUN50I_CTL_CS_LEVEL			BIT(17)
#define SUN50I_CTL_TP				BIT(18)


struct sun50i_spi {
	struct spi_master	*master;		/*指向初始化好的spi_master*/
	void __iomem		*base_addr; 	/*spi寄存器基地址*/
	struct clk		*hclk; 				/*SPI总线时钟*/
	struct clk		*mclk; 				/*spix时钟*/
	struct reset_control	*rstc;

	struct completion	done; 			/*等待队列，在include/linux/completion.h中定义*/

	const u8		*tx_buf;
	u8			*rx_buf;
	int			len;
	int 		irq; 					/*中断irq*/
	unsigned long		fifo_depth; 	/*FIFO位宽*/
};

static inline u32 sun50i_spi_read(struct sun50i_spi *sspi, u32 reg)
{
	return readl(sspi->base_addr + reg);
}

static inline void sun50i_spi_write(struct sun50i_spi *sspi, u32 reg, u32 value)
{
	writel(value, sspi->base_addr + reg);
}


static size_t sun50i_spi_max_transfer_size(struct spi_device *spi)
{
	return SUN8I_FIFO_DEPTH - 1;
}

static void sun50i_spi_set_cs(struct spi_device *spi, bool enable)
{

}

static int sun50i_spi_transfer_one(struct spi_master *master, struct spi_device *spi,
				  struct spi_transfer *tfr)
{
	return 0;
}

static irqreturn_t sun50i_spi_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}


static int sun50i_spi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sun50i_spi *sspi = spi_master_get_devdata(master);

	/*断言复位控制*/
	reset_control_assert(sspi->rstc);

	/*关闭时钟sspi->mclk和sspi->hclk,先关闭mclk*/
	clk_disable_unprepare(sspi->mclk);
	clk_disable_unprepare(sspi->hclk);

	return 0;
}

static int sun50i_spi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct sun50i_spi *sspi = spi_master_get_devdata(master);
	int ret;

	/*使能时钟sspi->hclk,先使能hclk*/
	ret = clk_prepare_enable(sspi->hclk);
	if (ret) {
		dev_err(dev, "Couldn't enable AHB clock\n");
		goto out;
	}
	/*然后使能时钟sspi->mclk*/
	ret = clk_prepare_enable(sspi->mclk);
	if (ret) {
		dev_err(dev, "Couldn't enable module clock\n");
		goto err;
	}

	/*解除复位控制*/
	ret = reset_control_deassert(sspi->rstc);
	if (ret) {
		dev_err(dev, "Couldn't deassert the device from reset\n");
		goto err2;
	}

	/*设置spi控制，使能SPI模块，设置为主模式，SUN50I_CTL_TP设置当FIFO满时停止传输*/
	sun50i_spi_write(sspi, SUN50I_GBL_CTL_REG,
			SUN50I_GBL_CTL_BUS_ENABLE | SUN50I_GBL_CTL_MASTER | SUN50I_GBL_CTL_TP);

	return 0;

err2:
	clk_disable_unprepare(sspi->mclk);
err:
	clk_disable_unprepare(sspi->hclk);
out:
	return ret;
}


static int sun50i_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct sun50i_spi *sspi;
	struct resource	*res;
	int ret = 0;

	/*1、内存分配*/
	/*spi_alloc_master同时分配了struct spi_master和struct sun50i_spi的内存空间，该函数直接返回
	struct spi_master内存空间的指针，struct sun50i_spi分配的内存空间指针由spi_master_get_devdata(master)获取*/
	master = spi_alloc_master(&pdev->dev, sizeof(struct sun50i_spi));
	if (!master) {
		dev_err(&pdev->dev, "Unable to allocate SPI Master\n");
		return -ENOMEM;
	}
	/*将初始化好的spi_master设置到pdev->dev->driver_data*/
	platform_set_drvdata(pdev, master);
	/*获取分配好内存空间的的struct sun50i_spi*/
	sspi = spi_master_get_devdata(master);

	/*2、寄存器映射*/
	/*设备树中的spi0定义了reg = <0x01c68000 0x1000>;即spi0的寄存器物理地址从0x01c68000开始，范围是0x1000（等于数据手册内存范围0x0FFF+1），
	  数据手册Memory Mapping中SPI0的内存空间分布是0x01C68000---0x01C68FFF,
	  platform_get_resource(pdev, IORESOURCE_MEM, 0);获取设备树中的IORESOURCE_MEM资源，通过printk打印出的地址查看是否正确
	*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	printk(KERN_WARNING  "name = %s start_addr = 0x%x , end_addr = 0x%x\n",res->name, (unsigned int)res->start, (unsigned int)res->end);
	/*将物理地址映射为虚拟地址*/
	sspi->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sspi->base_addr)) {
		ret = PTR_ERR(sspi->base_addr);
		goto err_free_master;
	}

	/*3、中断申请*/
	sspi->irq = platform_get_irq(pdev, 0);
	printk(KERN_WARNING  "spi get irq  = %d\n", sspi->irq);
	if (sspi->irq < 0) {
		dev_err(&pdev->dev, "No spi IRQ specified\n");
		ret = -ENXIO;
		goto err_free_master;
	}
	/*参数sun50i_spi_handler为中断处理函数，最后一个参数为sspi在中断处理函数中取出*/
	ret = devm_request_irq(&pdev->dev, sspi->irq, sun50i_spi_handler,
			       0, "sun50i-spi", sspi);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request IRQ\n");
		goto err_free_master;
	}

	/*4、初始化spi_master*/
	sspi->master = master; 								/*将spi_master保存到struct sun50i_spi中*/
	sspi->fifo_depth = (unsigned long)of_device_get_match_data(&pdev->dev); 	/*初始化spi的fifo位宽*/
	master->max_speed_hz = 100 * 1000 * 1000;			/*SPI最大频率100MHZ*/
	master->min_speed_hz = 3 * 1000; 					/*SPI最小频率3KHZ*/
	master->set_cs = sun50i_spi_set_cs; 				/*SPI设置片选函数*/
	master->transfer_one = sun50i_spi_transfer_one; 	/*SPI通信函数*/
	master->num_chipselect = 4;							/*SPI最大片选数*/
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST; 		/*SPI模式，默认为模式3，片选为高电平*/
	master->bits_per_word_mask = SPI_BPW_MASK(8); 		/*按每个字节8位传输*/
	master->dev.of_node = pdev->dev.of_node; 			/*设备树节点*/
	master->auto_runtime_pm = true;						/*自动电源管理*/
	master->max_transfer_size = sun50i_spi_max_transfer_size; 					/*设置spi最大传输字节*/

	/*5、时钟初始化*/
	/*在设备树sun50i-a64.dtsi中spi0定义了:
	 clocks = <&ccu CLK_BUS_SPI0>, <&ccu CLK_SPI0>;
	 clock-names = "ahb", "mod";
	 其中CLK_BUS_SPI0和CLK_SPI0在ccu-sun50i0a64.h中定义，在drivers/clk/sunxi-ng/ccu-sun50i-a64.c驱动中实现了时钟的初始化，
	 通过CLK_BUS_SPI0和CLK_SPI0数组下标即可获取到对应的时钟。
	*/

	/*函数devm_clk_get(&pdev->dev, "ahb")获取设备中clock-names="ahb"的时钟，即<&ccu CLK_BUS_SPI0>*/
	sspi->hclk = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(sspi->hclk)) {
		dev_err(&pdev->dev, "Unable to acquire AHB clock\n");
		ret = PTR_ERR(sspi->hclk);
		goto err_free_master;
	}

	sspi->mclk = devm_clk_get(&pdev->dev, "mod");
	/*函数devm_clk_get(&pdev->dev, "ahb")获取设备中clock-names="mod"的时钟，即<&ccu CLK_SPI0>*/
	if (IS_ERR(sspi->mclk)) {
		dev_err(&pdev->dev, "Unable to acquire module clock\n");
		ret = PTR_ERR(sspi->mclk);
		goto err_free_master;
	}
	/*sspi->hclk和sspi->mclk在调用sun50i_spi_transfer_one进行数据传输时被使用*/

	/*6、初始化等待队列*/
	init_completion(&sspi->done);


	/*7、硬件初始化（忽略）*/

	/*获取复位控制寄存器，设备树spi0配置中有：resets = <&ccu RST_BUS_SPI0>;没有配置reset-names 所以第二个参数为空*/
	sspi->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(sspi->rstc)) {
		dev_err(&pdev->dev, "Couldn't get reset controller\n");
		ret = PTR_ERR(sspi->rstc);
		goto err_free_master;
	}

	/*即时runtime_pm被关闭，这种唤醒/关闭模式也能唤醒设备*/
	ret = sun50i_spi_runtime_resume(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't resume the device\n");
		goto err_free_master;
	}
	/*初始化pm电源管理*/
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);


	/*8、注册spi_master，将spi_master注册进链表，同时扫描设备树在spi总线下创建spidev*/
	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "cannot register SPI master\n");
		goto err_pm_disable;
	}

	printk(KERN_WARNING "sun50i spi probe success\n");
	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	sun50i_spi_runtime_suspend(&pdev->dev);
err_free_master:
	spi_master_put(master);
	return ret;
}

static int sun50i_spi_remove(struct platform_device *pdev)
{
	pm_runtime_force_suspend(&pdev->dev);

	printk(KERN_WARNING "sun50i spi remove success\n");
	return 0;
}


static const struct of_device_id sun50i_spi_match[] = {
	{ .compatible = "allwinner,sun6i-a31-spi", .data = (void *)SUN6I_FIFO_DEPTH },
	{ .compatible = "allwinner,sun8i-h3-spi",  .data = (void *)SUN8I_FIFO_DEPTH },
	{}
};
MODULE_DEVICE_TABLE(of, sun50i_spi_match);

/*设备电源管理操作结构体*/
static const struct dev_pm_ops sun50i_spi_pm_ops = {
	.runtime_resume		= sun50i_spi_runtime_resume,
	.runtime_suspend	= sun50i_spi_runtime_suspend,
};

static struct platform_driver sun50i_spi_driver = {
	.probe	= sun50i_spi_probe,
	.remove	= sun50i_spi_remove,
	.driver	= {
		.name		= "sun50i-spi",
		.of_match_table	= sun50i_spi_match,
		.pm		= &sun50i_spi_pm_ops,
	},
};
module_platform_driver(sun50i_spi_driver);

MODULE_AUTHOR("zzw, email<1600284146@qq.com>");
MODULE_DESCRIPTION("Allwinner A64 SPI controller driver");
MODULE_LICENSE("GPL");
