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

#define SUN8I_FIFO_DEPTH		64

/*spi 数据全局控制寄存器*/
#define SUN50I_GBL_CTL_REG		0x04
#define SUN50I_GBL_CTL_BUS_ENABLE	BIT(0)
#define SUN50I_GBL_CTL_MASTER		BIT(1)
#define SUN50I_GBL_CTL_TP			BIT(7)
#define SUN50I_GBL_CTL_RST			BIT(31)

/*spi 数据传输控制寄存器*/
#define SUN50I_TFR_CTL_REG		0x08
#define SUN50I_TFR_CTL_CPHA			BIT(0)
#define SUN50I_TFR_CTL_CPOL			BIT(1)
#define SUN50I_TFR_CTL_SPOL			BIT(2)
#define SUN50I_TFR_CTL_CS_MASK			0x30
#define SUN50I_TFR_CTL_CS(cs)			(((cs) << 4) & SUN50I_TFR_CTL_CS_MASK)
#define SUN50I_TFR_CTL_CS_MANUAL			BIT(6)
#define SUN50I_TFR_CTL_CS_LEVEL			BIT(7)
#define SUN50I_TFR_CTL_DHB			BIT(8)
#define SUN50I_TFR_CTL_FBS			BIT(12)
#define SUN50I_TFR_CTL_XCH			BIT(31)

/*中断控制寄存器*/
#define SUN50I_INT_CTL_REG		0x10
#define SUN50I_INT_CTL_RF_RDY			BIT(0)
#define SUN50I_INT_CTL_TF_ERQ			BIT(4)
#define SUN50I_INT_CTL_RF_OVF			BIT(8)
#define SUN50I_INT_CTL_TC			BIT(12)


/*中断状态寄存器*/
#define SUN50I_INT_STA_REG		0x14

/*FIFO 控制寄存器*/
#define SUN50I_FIFO_CTL_REG		0x18
#define SUN50I_FIFO_CTL_RF_RDY_TRIG_LEVEL_MASK	0xff
#define SUN50I_FIFO_CTL_RF_RDY_TRIG_LEVEL_BITS	0
#define SUN50I_FIFO_CTL_RF_RST			BIT(15)
#define SUN50I_FIFO_CTL_TF_ERQ_TRIG_LEVEL_MASK	0xff
#define SUN50I_FIFO_CTL_TF_ERQ_TRIG_LEVEL_BITS	16
#define SUN50I_FIFO_CTL_TF_RST			BIT(31)

/*FIFO 状态寄存器*/
#define SUN50I_FIFO_STA_REG		0x1c
#define SUN50I_FIFO_STA_RF_CNT_MASK		0x7f
#define SUN50I_FIFO_STA_RF_CNT_BITS		0
#define SUN50I_FIFO_STA_TF_CNT_MASK		0x7f
#define SUN50I_FIFO_STA_TF_CNT_BITS		16


/*发送数据寄存器*/
#define SUN50I_TXDATA_REG		0x200
/*接收数据寄存器*/
#define SUN50I_RXDATA_REG		0x300


/*时钟控制寄存器*/
#define SUN50I_CLK_CTL_REG		0x24
#define SUN50I_CLK_CTL_CDR2_MASK			0xff
#define SUN50I_CLK_CTL_CDR2(div)			(((div) & SUN50I_CLK_CTL_CDR2_MASK) << 0)
#define SUN50I_CLK_CTL_CDR1_MASK			0xf
#define SUN50I_CLK_CTL_CDR1(div)			(((div) & SUN50I_CLK_CTL_CDR1_MASK) << 8)
#define SUN50I_CLK_CTL_DRS			BIT(12)

/*最大传输字节数*/
#define SUN50I_MAX_XFER_SIZE		0xffffff  	/*SUN50I_BURST_CNT_REG脉冲计数寄存器，0:23位，最大值0xffffff*/

/*脉冲计数寄存器*/
#define SUN50I_BURST_CNT_REG		0x30
#define SUN50I_BURST_CNT(cnt)			((cnt) & SUN50I_MAX_XFER_SIZE)


/*传输计数寄存器*/
#define SUN50I_XMIT_CNT_REG		0x34
#define SUN50I_XMIT_CNT(cnt)			((cnt) & SUN50I_MAX_XFER_SIZE)

/*脉冲控制计数寄存器*/
#define SUN50I_BURST_CTL_CNT_REG		0x38
#define SUN50I_BURST_CTL_CNT_STC(cnt)		((cnt) & SUN50I_MAX_XFER_SIZE)


struct sun50i_spi {
	struct spi_master	*master;		/*指向初始化好的spi_master*/
	void __iomem		*base_addr; 	/*spi寄存器基地址*/
	struct clk		*hclk; 				/*SPI总线时钟*/
	struct clk		*mclk; 				/*spix时钟*/
	struct reset_control	*rstc;		/*复位控制器*/

	struct completion	done; 			/*等待队列，在include/linux/completion.h中定义*/

	const u8		*tx_buf;			/*指向要发送的数据缓冲区*/
	u8				*rx_buf;			/*指向要接收的数据缓冲区*/
	int			len;					/*数据长度*/

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

static inline u32 sun50i_spi_get_tx_fifo_count(struct sun50i_spi *sspi)
{
	u32 reg = sun50i_spi_read(sspi, SUN50I_FIFO_STA_REG);

	reg >>= SUN50I_FIFO_STA_TF_CNT_BITS;

	return reg & SUN50I_FIFO_STA_TF_CNT_MASK;
}

static inline void sun50i_spi_enable_interrupt(struct sun50i_spi *sspi, u32 mask)
{
	u32 reg = sun50i_spi_read(sspi, SUN50I_INT_CTL_REG);

	reg |= mask;
	sun50i_spi_write(sspi, SUN50I_INT_CTL_REG, reg);
}

static inline void sun50i_spi_disable_interrupt(struct sun50i_spi *sspi, u32 mask)
{
	u32 reg = sun50i_spi_read(sspi, SUN50I_INT_CTL_REG);

	reg &= ~mask;
	sun50i_spi_write(sspi, SUN50I_INT_CTL_REG, reg);
}


static inline void sun50i_spi_drain_fifo(struct sun50i_spi *sspi, int len)
{
	u32 reg, cnt;
	u8 byte;

	/*读取状态寄存器*/
	reg = sun50i_spi_read(sspi, SUN50I_FIFO_STA_REG); 	
	/*判断有多少数据需要读取*/
	reg &= SUN50I_FIFO_STA_RF_CNT_MASK;	
	cnt = reg >> SUN50I_FIFO_STA_RF_CNT_BITS;

	if (len > cnt)
		len = cnt;

	while (len--) {
		/*通过接收寄存器读取相应长度数据，保存到sspi->rx_buf*/
		byte = readb(sspi->base_addr + SUN50I_RXDATA_REG);
		if (sspi->rx_buf)
			*sspi->rx_buf++ = byte;
	}
}

static inline void sun50i_spi_fill_fifo(struct sun50i_spi *sspi, int len)
{
	u32 cnt;
	u8 byte;

	/*判断有多少数据需要发送*/
	cnt = sspi->fifo_depth - sun50i_spi_get_tx_fifo_count(sspi);

	len = min3(len, (int)cnt, sspi->len);

	while (len--) {
		byte = sspi->tx_buf ? *sspi->tx_buf++ : 0;
		writeb(byte, sspi->base_addr + SUN50I_TXDATA_REG);
		sspi->len--;
	}
}


static void sun50i_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct sun50i_spi *sspi = spi_master_get_devdata(spi->master);
	u32 reg;

	/*配置spi使用的片选引脚*/
	reg = sun50i_spi_read(sspi, SUN50I_TFR_CTL_REG);
	reg &= ~SUN50I_TFR_CTL_CS_MASK;
	reg |= SUN50I_TFR_CTL_CS(spi->chip_select);


	/*配置spi片选电平*/
	if (enable)
		reg |= SUN50I_TFR_CTL_CS_LEVEL;
	else
		reg &= ~SUN50I_TFR_CTL_CS_LEVEL;

	sun50i_spi_write(sspi, SUN50I_TFR_CTL_REG, reg);
}

static size_t sun50i_spi_max_transfer_size(struct spi_device *spi)
{
	return SUN50I_MAX_XFER_SIZE - 1;
}


static int sun50i_spi_transfer_one(struct spi_master *master, struct spi_device *spi,
				  struct spi_transfer *tfr)
{
	struct sun50i_spi *sspi = spi_master_get_devdata(master);
	unsigned int mclk_rate, div, timeout;
	unsigned int start, end, tx_time;
	unsigned int trig_level;
	unsigned int tx_len = 0;
	int ret = 0;
	u32 reg;

	/*判断发送数据长度是否超过 SUN50I_MAX_XFER_SIZE*/
	if (tfr->len > SUN50I_MAX_XFER_SIZE)
		return -EINVAL;

	/*重新初始化等待队列*/
	reinit_completion(&sspi->done);

	/*1、保存数据到struct sun50i_spi*/

	/*保存本次传输的tx_buf,rx_buf和len*/
	sspi->tx_buf = tfr->tx_buf;
	sspi->rx_buf = tfr->rx_buf;
	sspi->len = tfr->len;


	/*2、复位与数据传输相关寄存器*/

	/*清除中断状态寄存器的所有状态位（写1清除）*/
	sun50i_spi_write(sspi, SUN50I_INT_STA_REG, ~0);

	/*复位RX/TX FIFO */
	sun50i_spi_write(sspi, SUN50I_FIFO_CTL_REG,
			SUN50I_FIFO_CTL_RF_RST | SUN50I_FIFO_CTL_TF_RST);

	/*
	 * Setup FIFO interrupt trigger level
	 * Here we choose 3/4 of the full fifo depth, as it's the hardcoded
	 * value used in old generation of Allwinner SPI controller.
	 * (See spi-sun4i.c)
	 */
	 /*设置RXFIFO/TXFIFO当FIFO容量达到fifo_depth /4*3时触发RXFIFO/TXFIFO就绪中断 */
	trig_level = sspi->fifo_depth / 4 * 3;
	sun50i_spi_write(sspi, SUN50I_FIFO_CTL_REG,
			(trig_level << SUN50I_FIFO_CTL_RF_RDY_TRIG_LEVEL_BITS) |
			(trig_level << SUN50I_FIFO_CTL_TF_ERQ_TRIG_LEVEL_BITS));


	/*3、设置数据传输控制寄存器*/

	/*
	 * Setup the transfer control register: Chip Select,
	 * polarities, etc.
	 */
	reg = sun50i_spi_read(sspi, SUN50I_TFR_CTL_REG);

	/*配置SPI模式*/
	if (spi->mode & SPI_CPOL)
		reg |= SUN50I_TFR_CTL_CPOL;
	else
		reg &= ~SUN50I_TFR_CTL_CPOL;

	if (spi->mode & SPI_CPHA)
		reg |= SUN50I_TFR_CTL_CPHA;
	else
		reg &= ~SUN50I_TFR_CTL_CPHA;

	if (spi->mode & SPI_LSB_FIRST)
		reg |= SUN50I_TFR_CTL_FBS;
	else
		reg &= ~SUN50I_TFR_CTL_FBS;

	/*
	 * If it's a TX only transfer, we don't want to fill the RX
	 * FIFO with bogus data
	 */
	/*判断本次传输是否需要接收数据*/
	if (sspi->rx_buf) {
		reg &= ~SUN50I_TFR_CTL_DHB;
	} else {
		/*设置SUN50I_TFR_CTL_DHB标志位为1，让RXFIFO丢弃无用的脉冲，免得让一些无用的数据填充到RXFIFO*/
		reg |= SUN50I_TFR_CTL_DHB;
	}

	/* We want to control the chip select manually ，设置片选信号由SPI控制*/
	reg |= SUN50I_TFR_CTL_CS_MANUAL;

	/*设置SPI传输控制寄存器*/
	sun50i_spi_write(sspi, SUN50I_TFR_CTL_REG, reg);



	/*4、配置本次传输的时钟*/

	/* Ensure that we have a parent clock fast enough */
	mclk_rate = clk_get_rate(sspi->mclk);
	if (mclk_rate < (2 * tfr->speed_hz)) {
		clk_set_rate(sspi->mclk, 2 * tfr->speed_hz);
		mclk_rate = clk_get_rate(sspi->mclk);
	}

	/*
	 * Setup clock divider.
	 *
	 * We have two choices there. Either we can use the clock
	 * divide rate 1, which is calculated thanks to this formula:
	 * SPI_CLK = MOD_CLK / (2 ^ cdr)
	 * Or we can use CDR2, which is calculated with the formula:
	 * SPI_CLK = MOD_CLK / (2 * (cdr + 1))
	 * Wether we use the former or the latter is set through the
	 * DRS bit.
	 *
	 * First try CDR2, and if we can't reach the expected
	 * frequency, fall back to CDR1.
	 */
	div = mclk_rate / (2 * tfr->speed_hz);
	if (div <= (SUN50I_CLK_CTL_CDR2_MASK + 1)) {
		if (div > 0)
			div--;

		reg = SUN50I_CLK_CTL_CDR2(div) | SUN50I_CLK_CTL_DRS;
	} else {
		div = ilog2(mclk_rate) - ilog2(tfr->speed_hz);
		reg = SUN50I_CLK_CTL_CDR1(div);
	}

	sun50i_spi_write(sspi, SUN50I_CLK_CTL_REG, reg);


	/*5、开始传输数据*/
	/* Setup the transfer now... */
	if (sspi->tx_buf)
		tx_len = tfr->len;

	/* Setup the counters */
	/*配置计数器*/
	sun50i_spi_write(sspi, SUN50I_BURST_CNT_REG, SUN50I_BURST_CNT(tfr->len)); 	/*配置脉冲计数器，本次传输需要的脉冲个数*/
	sun50i_spi_write(sspi, SUN50I_XMIT_CNT_REG, SUN50I_XMIT_CNT(tx_len));
	sun50i_spi_write(sspi, SUN50I_BURST_CTL_CNT_REG,
			SUN50I_BURST_CTL_CNT_STC(tx_len));

	/* Fill the TX FIFO */
	/*将tx数据填充到TX FIFO*/
	sun50i_spi_fill_fifo(sspi, sspi->fifo_depth);

	/* Enable the interrupts */
	/*使能传输完成中断和接收就绪中断*/
	sun50i_spi_write(sspi, SUN50I_INT_CTL_REG, SUN50I_INT_CTL_TC);
	sun50i_spi_enable_interrupt(sspi, SUN50I_INT_CTL_TC |
					 SUN50I_INT_CTL_RF_RDY);

	/*如果需要发送的数据一次发送不完，使能RX FIFO Empty Request中断*/
	if (tx_len > sspi->fifo_depth)
		sun50i_spi_enable_interrupt(sspi, SUN50I_INT_CTL_TF_ERQ);

	/* Start the transfer */
	/*开始传输*/
	reg = sun50i_spi_read(sspi, SUN50I_TFR_CTL_REG);
	/*使能SPI脉冲，开始数据传输*/
	sun50i_spi_write(sspi, SUN50I_TFR_CTL_REG, reg | SUN50I_TFR_CTL_XCH);

	/*计算发送需要的时间*/
	tx_time = max(tfr->len * 8 * 2 / (tfr->speed_hz / 1000), 100U);
	start = jiffies;

	/*将该进程挂起到等待队列*/
	timeout = wait_for_completion_timeout(&sspi->done,
					      msecs_to_jiffies(tx_time));
	end = jiffies;
	if (!timeout) { 	/*超时*/
		dev_warn(&master->dev,
			 "%s: timeout transferring %u bytes@%iHz for %i(%i)ms",
			 dev_name(&spi->dev), tfr->len, tfr->speed_hz,
			 jiffies_to_msecs(end - start), tx_time);
		ret = -ETIMEDOUT;
		goto out;
	}

out:
	/*关闭所有中断*/
	sun50i_spi_write(sspi, SUN50I_INT_CTL_REG, 0);

	return ret;
}



static irqreturn_t sun50i_spi_handler(int irq, void *dev_id)
{
	struct sun50i_spi *sspi = dev_id;

	/*读取状态寄存器*/
	u32 status = sun50i_spi_read(sspi, SUN50I_INT_STA_REG);

	if (status & SUN50I_INT_CTL_TC) {	/*判断传输完成标志位是否置1*/
		sun50i_spi_write(sspi, SUN50I_INT_STA_REG, SUN50I_INT_CTL_TC);	/*写入1清除该标志位*/
		sun50i_spi_drain_fifo(sspi, sspi->fifo_depth); 	/*读数据*/
		/*传输唤醒等待队列*/
		complete(&sspi->done);		
		return IRQ_HANDLED;
	}

	/* Receive FIFO 3/4 full ,当FIFO接收数据达到FIFO*3/4时，会触发SUN50I_INT_CTL_RF_RDY */
	if (status & SUN50I_INT_CTL_RF_RDY) { 	/*判断接收就绪标志位是否置1*/
		sun50i_spi_drain_fifo(sspi, SUN8I_FIFO_DEPTH); 	/*读数据*/
		/* Only clear the interrupt _after_ draining the FIFO */
		sun50i_spi_write(sspi, SUN50I_INT_STA_REG, SUN50I_INT_CTL_RF_RDY); 	/*写入1清除该标志位*/
		return IRQ_HANDLED;
	}

	/* Transmit FIFO 3/4 empty ，当FIFO发送到FIFO*3/4容量为空时，会触发SUN50I_INT_CTL_TF_ERQ*/
	if (status & SUN50I_INT_CTL_TF_ERQ) { 	/*判断发送就绪标志位是否置1 */
		sun50i_spi_fill_fifo(sspi, SUN8I_FIFO_DEPTH);	/*写数据*/

		if (!sspi->len) { 	/*sspi->len == 0，数据发送完毕*/
			/* 关闭FIFO Empty Request interrupt中断*/
			sun50i_spi_disable_interrupt(sspi, SUN50I_INT_CTL_TF_ERQ);
		}
		/* Only clear the interrupt _after_ re-seeding the FIFO */
		sun50i_spi_write(sspi, SUN50I_INT_STA_REG, SUN50I_INT_CTL_TF_ERQ); 	/*写入1清除该标志位*/

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
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


	/*3、初始化spi_master*/
	/*首先实现sun50i_spi_max_transfer_size回调函数，接着实现sun50i_spi_set_cs回调函数，最后实现sun50i_spi_transfer_one回调函数*/
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


	/*4、中断申请*/
	/*实现完spi_master的回调函数后，再实现中断处理函数*/
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
