/*
 * Broadcom bcm2835 SPI driver
 *
 * Author: A. Robinson <arobinson@cs.man.ac.uk>
 * Copyright (C) 2012 A Robinson University of Manchester.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/spi/bcm2835_spi.h>
#include <asm/unaligned.h>

#define DRIVER_NAME			"bcm2835_spi"


//TODO
#define BCM2835_NUM_CHIPSELECTS		3 
#define BCM2835_SPI_WAIT_RDY_MAX_LOOP	2000 /* in usec */

#define BCM2835_REG_BYTES_WIDE 4
//registers (these are offsets to the base)
#define SPI0_CNTLSTAT 0
#define SPI0_FIFO     0x1*BCM2835_REG_BYTES_WIDE
#define SPI0_CLKSPEED  0x2*BCM2835_REG_BYTES_WIDE
#define SPI0_DATA_LEN  0x3*BCM2835_REG_BYTES_WIDE
#define SPI0_LOSSI_MODE  0x4*BCM2835_REG_BYTES_WIDE
#define SPI0_DMA_CONTROLS  0x5*BCM2835_REG_BYTES_WIDE


// SPI0_CNTLSTAT register bits
//enable long data in lossi mode
//DMA enable Lossi mode
#define SPI0_CS_CS2ACTHIGH   0x00800000 // CS2 active high
#define SPI0_CS_CS2ACTHIGH   0x00800000 // CS2 active high
#define SPI0_CS_CS1ACTHIGH   0x00400000 // CS1 active high
#define SPI0_CS_CS0ACTHIGH   0x00200000 // CS0 active high
#define SPI0_CS_RXFIFOFULL   0x00100000 // Receive FIFO full
#define SPI0_CS_RXFIFO3_4    0x00080000 // Receive FIFO 3/4 full
#define SPI0_CS_TXFIFOSPCE   0x00040000 // Transmit FIFO has space
#define SPI0_CS_RXFIFODATA   0x00020000 // Receive FIFO has data
#define SPI0_CS_DONE         0x00010000 // SPI transfer done. WRT to TX FIFO to CLR!
//unused
//unused
//LEN lossI
#define SPI0_CS_MOSI_INPUT   0x00001000 // MOSI is input, read from MOSI (BI-dir mode)
#define SPI0_CS_DEASRT_CS    0x00000800 // De-assert CS at end
#define SPI0_CS_RX_IRQ       0x00000400 // irq enable on RX FIFO
#define SPI0_CS_DONE_IRQ     0x00000200 // irq when done
#define SPI0_CS_DMA_ENABLE   0x00000100 // Run in DMA mode
//DMA transfer active
#define SPI0_CS_ACTIVATE     0x00000080 // Activate: be high before starting
#define SPI0_CS_CS_POLARIT   0x00000040 // Chip selects active high
//fifo clears
#define SPI0_CS_CLRTXFIFO    0x00000020 // Clear TX FIFO    (auto clear bit)
#define SPI0_CS_CLRRXFIFO    0x00000010 // Clear RX FIFO    (auto clear bit)
#define SPI0_CS_CLRFIFOS     0x00000030 // Clear BOTH FIFOs (auto clear bit)
//clock polarity
#define SPI0_CS_CLK_IDLHI    0x00000008 // Clock pin is high when idle
//clock phase
#define SPI0_CS_CLKTRANS     0x00000004 // 0=first clock in middle of data bit
                                        // 1=first clock at begin of data bit
#define SPI0_CS_CHIPSEL0     0x00000000 // Use chip select 0
#define SPI0_CS_CHIPSEL1     0x00000001 // Use chip select 1
#define SPI0_CS_CHIPSEL2     0x00000002 // Use chip select 2
#define SPI0_CS_CHIPSELN     0x00000003 // No chip select (e.g. use GPIO pin)
 


#define SPI0_CS_CLRALL      (SPI0_CS_CLRFIFOS|SPI0_CS_DONE)


// SPI0_FIFO register bits


// SPI0_CLKSPEED  register bits




struct bcm2835_spi {
	struct work_struct	work;

	/* Lock access to transfer list.	*/
	spinlock_t		lock;

	struct list_head	msg_queue;
	struct spi_master	*master;
	void __iomem		*base;
	unsigned int		max_speed;
	unsigned int		min_speed;
	struct bcm2835_spi_info	*spi_info;
};

static struct workqueue_struct *bcm2835_spi_wq;

/* Register manipulation operations */
/* calc offsets, sets and clrs */
static inline void __iomem *spi_reg(struct bcm2835_spi *bcm2835_spi, u32 reg)
{
	return bcm2835_spi->base + reg;
}

static inline void
bcm2835_spi_setbits(struct bcm2835_spi *bcm2835_spi, u32 reg, u32 mask)
{
	void __iomem *reg_addr = spi_reg(bcm2835_spi, reg);
	u32 val;

	val = ioread32(reg_addr);
	val |= mask;
	iowrite32(val, reg_addr);
}

static inline void
bcm2835_spi_clrbits(struct bcm2835_spi *bcm2835_spi, u32 reg, u32 mask)
{
	void __iomem *reg_addr = spi_reg(bcm2835_spi, reg);
	u32 val;

	val = ioread32(reg_addr);
	val &= ~mask;
	iowrite32(val, reg_addr);
}

static inline void
bcm2835_spi_writeReg(struct bcm2835_spi *bcm2835_spi, u32 reg, u32 val)
{
	void __iomem *reg_addr = spi_reg(bcm2835_spi, reg);
	iowrite32(val, reg_addr);
	printk("bcm2835_spi wrote  0x%02lX to 0x%02lX \n", val, reg_addr);
}



//helper functions to manipulate specific registers

static int bcm2835_spi_set_transfer_size(struct bcm2835_spi *bcm2835_spi, int size)
{//TODO
	if (size != 8)
	{
		pr_debug("Bad bits per word value %d (only 8 bits are supported "
			 "allowed).\n", size);
		return -EINVAL;
	}
	return 0;
}

static int bcm2835_spi_baudrate_set(struct spi_device *spi, unsigned int speed)
{
	struct bcm2835_spi *bcm2835_spi;
	bcm2835_spi = spi_master_get_devdata(spi->master);

//TODO -- this is hard coded in to divide 250Mhz system clock to give 1Mhz SPI clk
	bcm2835_spi_writeReg(bcm2835_spi,SPI0_CLKSPEED, 250);
	return 0;
}



/*
 * called only when no transfer is active on the bus
 */
static int
bcm2835_spi_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	//TODO setup clock phase, clock polarity

	//TODO maybe set up chip select and chip select polarity
	//TODO maybe setup TA

	printk("in bcm2835_spi_setup_transfer\n");
	struct bcm2835_spi *bcm2835_spi;
	unsigned int speed = spi->max_speed_hz;
	unsigned int bits_per_word = spi->bits_per_word;
	int	rc;

	bcm2835_spi = spi_master_get_devdata(spi->master);

	if ((t != NULL) && t->speed_hz)
		speed = t->speed_hz;

	if ((t != NULL) && t->bits_per_word)
		bits_per_word = t->bits_per_word;

	rc = bcm2835_spi_baudrate_set(spi, speed);
	if (rc)
		return rc;

	return bcm2835_spi_set_transfer_size(bcm2835_spi, bits_per_word);
}

//set or clear the chip select line
static void bcm2835_spi_set_cs(struct bcm2835_spi *bcm2835_spi, int enable)
{
//TODO check which cs line we should use
/*
	if (enable)
		bcm2835_spi_setbits(bcm2835_spi, ORION_SPI_IF_CTRL_REG, 0x1);
	else
		bcm2835_spi_clrbits(bcm2835_spi, ORION_SPI_IF_CTRL_REG, 0x1);
*/
}

static inline int bcm2835_spi_wait_till_ready(struct bcm2835_spi *bcm2835_spi)
{
/*	int i;

	for (i = 0; i < BCM2835_SPI_WAIT_RDY_MAX_LOOP; i++) {
		if (readl(spi_reg(orion_spi, ORION_SPI_INT_CAUSE_REG)))
			return 1;
		else
			udelay(1);
	}

	return -1;*/
}

static inline int
bcm2835_spi_write_read_8bit(struct spi_device *spi,
			  const u8 **tx_buf, u8 **rx_buf)
{
	void __iomem *tx_reg, *rx_reg, *int_reg;
	struct bcm2835_spi *bcm2835_spi;

	bcm2835_spi = spi_master_get_devdata(spi->master);
	tx_reg = spi_reg(bcm2835_spi, SPI0_FIFO);
	rx_reg = spi_reg(bcm2835_spi, SPI0_FIFO);
	//int_reg = spi_reg(bcm2835_spi, ORION_SPI_INT_CAUSE_REG);

	/* clear the interrupt cause register 
	writel(0x0, int_reg);
*/

	//here we write one byte. This only works if DMAEN is clear and TA is set)
	if (tx_buf && *tx_buf)
		iowrite8(*(*tx_buf)++, tx_reg);
	else
		iowrite8(0, tx_reg);

	if (bcm2835_spi_wait_till_ready(bcm2835_spi) < 0) {
		dev_err(&spi->dev, "TXS timed out\n");
		return -1;
	}
	//here we read one byte. This only works if DMAEN is clear and TA is set)
	if (rx_buf && *rx_buf)
		*(*rx_buf)++ = ioread8(rx_reg);

	return 1;
}

static inline int
bcm2835_spi_write_read_16bit(struct spi_device *spi,
			   const u16 **tx_buf, u16 **rx_buf)
{
/*
	void __iomem *tx_reg, *rx_reg, *int_reg;
	struct bcm2835_spi *bcm2835_spi;

	bcm2835_spi = spi_master_get_devdata(spi->master);
	tx_reg = spi_reg(bcm2835_spi, ORION_SPI_DATA_OUT_REG);
	rx_reg = spi_reg(bcm2835_spi, ORION_SPI_DATA_IN_REG);
	int_reg = spi_reg(bcm2835, ORION_SPI_INT_CAUSE_REG);

	/* clear the interrupt cause register 
	writel(0x0, int_reg);

	if (tx_buf && *tx_buf)
		writel(__cpu_to_le16(get_unaligned((*tx_buf)++)), tx_reg);
	else
		writel(0, tx_reg);

	if (bcm2835_spi_wait_till_ready(orion_spi) < 0) {
		dev_err(&spi->dev, "TXS timed out\n");
		return -1;
	}

	if (rx_buf && *rx_buf)
		put_unaligned(__le16_to_cpu(readl(rx_reg)), (*rx_buf)++);

	return 1;
*/
}

static unsigned int
bcm2835_spi_write_read(struct spi_device *spi, struct spi_transfer *xfer)
{
	struct bcm2835_spi *bcm2835_spi;
	unsigned int count;
	int word_len;

	bcm2835_spi = spi_master_get_devdata(spi->master);
	word_len = spi->bits_per_word;
	count = xfer->len;

	if (word_len == 8) {
		const u8 *tx = xfer->tx_buf;
		u8 *rx = xfer->rx_buf;

		do {
			if (bcm2835_spi_write_read_8bit(spi, &tx, &rx) < 0)
				goto out;
			count--;
		} while (count);
	} /*else if (word_len == 16) {
		const u16 *tx = xfer->tx_buf;
		u16 *rx = xfer->rx_buf;

		do {
			if (bcm2835_spi_write_read_16bit(spi, &tx, &rx) < 0)
				goto out;
			count -= 2;
		} while (count);
	}*/

out:
	return xfer->len - count;
}


static void bcm2835_spi_work(struct work_struct *work)
{
	struct bcm2835_spi *bcm2835_spi =
		container_of(work, struct bcm2835_spi, work);

	printk("in bcm2835_spi_work\n");

	spin_lock_irq(&bcm2835_spi->lock);
	while (!list_empty(&bcm2835_spi->msg_queue)) {
		struct spi_message *m;
		struct spi_device *spi;
		struct spi_transfer *t = NULL;
		int par_override = 0;
		int status = 0;
		int cs_active = 0;

		m = container_of(bcm2835_spi->msg_queue.next, struct spi_message,
				 queue);

		list_del_init(&m->queue);
		spin_unlock_irq(&bcm2835_spi->lock);

		spi = m->spi;

		/* Load defaults */
		status = bcm2835_spi_setup_transfer(spi, NULL);

		if (status < 0)
			goto msg_done;

		list_for_each_entry(t, &m->transfers, transfer_list) {
			if (par_override || t->speed_hz || t->bits_per_word) {
				par_override = 1;
				status = bcm2835_spi_setup_transfer(spi, t);
				if (status < 0)
					break;
				if (!t->speed_hz && !t->bits_per_word)
					par_override = 0;
			}
			//TODO set transfer active
			if (!cs_active) {
				bcm2835_spi_set_cs(bcm2835_spi, 1);
				cs_active = 1;
			}

			if (t->len)
				m->actual_length +=
					bcm2835_spi_write_read(spi, t);

			if (t->delay_usecs)
				udelay(t->delay_usecs);

			//TODO set transfer inactive
			if (t->cs_change) {
				bcm2835_spi_set_cs(bcm2835_spi, 0);
				cs_active = 0;
			}
		}

msg_done:
		if (cs_active)
			bcm2835_spi_set_cs(bcm2835_spi, 0);

		m->status = status;
		m->complete(m->context);

		spin_lock_irq(&bcm2835_spi->lock);
	}

	spin_unlock_irq(&bcm2835_spi->lock);
}

static int __init bcm2835_spi_reset(struct bcm2835_spi *bcm2835_spi)
{
	/* Verify that the CS is deasserted */
	//bcm2835_spi_set_cs(bcm2835_spi, 0);

		
	//struct bcm2835_spi *bcm2835_spi;

	//bcm2835_spi = spi_master_get_devdata(spi->master);

	//TODO clr all the other registers

	//clear FIFOs (one shot op)
	bcm2835_spi_setbits(bcm2835_spi,SPI0_CNTLSTAT,SPI0_CS_CLRTXFIFO|SPI0_CS_CLRRXFIFO);

	//TODO datasheet says this is RO, but example code clrs it
	bcm2835_spi_clrbits(bcm2835_spi,SPI0_CNTLSTAT,SPI0_CS_DONE);
 
	return 0;
}

static int bcm2835_spi_setup(struct spi_device *spi)
{
	//TODO customise
	struct bcm2835_spi *bcm2835_spi;

	printk("in bcm2835_spi_setup\n");

	bcm2835_spi = spi_master_get_devdata(spi->master);

	/* Fix ac timing if required.   */
/*
	if (bcm2835_spi->spi_info->enable_clock_fix)
		bcm2835_spi_setbits(bcm2835_spi, ORION_SPI_IF_CONFIG_REG,
				  (1 << 14));
*/
	/*check speed is within limits */
	if ((spi->max_speed_hz == 0)
			|| (spi->max_speed_hz > bcm2835_spi->max_speed))
		spi->max_speed_hz = bcm2835_spi->max_speed;

	if (spi->max_speed_hz < bcm2835_spi->min_speed) {
		dev_err(&spi->dev, "setup: requested speed too low %d Hz\n",
			spi->max_speed_hz);
		return -EINVAL;
	}

	/*
	 * baudrate & width will be set bcm2835_spi_setup_transfer
	 */
	return 0;
}

static int bcm2835_spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct bcm2835_spi *bcm2835_spi;
	struct spi_transfer *t = NULL;
	unsigned long flags;

	m->actual_length = 0;
	m->status = 0;

	printk("in bcm2835_spi_transfer\n");

	/* reject invalid messages and transfers */
	if (list_empty(&m->transfers) || !m->complete)
		return -EINVAL;

	bcm2835_spi = spi_master_get_devdata(spi->master);

	list_for_each_entry(t, &m->transfers, transfer_list) {
		unsigned int bits_per_word = spi->bits_per_word;

		if (t->tx_buf == NULL && t->rx_buf == NULL && t->len) {
			dev_err(&spi->dev,
				"message rejected : "
				"invalid transfer data buffers\n");
			goto msg_rejected;
		}

		if (t->bits_per_word)
			bits_per_word = t->bits_per_word;

		if ((bits_per_word != 8) /*&& (bits_per_word != 16)*/) {
			dev_err(&spi->dev,
				"message rejected : "
				"invalid transfer bits_per_word (%d bits)\n",
				bits_per_word);
			goto msg_rejected;
		}
		/*make sure buffer length is even when working in 16 bit mode*/
/*		if ((t->bits_per_word == 16) && (t->len & 1)) {
			dev_err(&spi->dev,
				"message rejected : "
				"odd data length (%d) while in 16 bit mode\n",
				t->len);
			goto msg_rejected;
		}*/

		if (t->speed_hz && t->speed_hz < bcm2835_spi->min_speed) {
			dev_err(&spi->dev,
				"message rejected : "
				"device min speed (%d Hz) exceeds "
				"required transfer speed (%d Hz)\n",
				bcm2835_spi->min_speed, t->speed_hz);
			goto msg_rejected;
		}
	}


	spin_lock_irqsave(&bcm2835_spi->lock, flags);
	list_add_tail(&m->queue, &bcm2835_spi->msg_queue);
	queue_work(bcm2835_spi_wq, &bcm2835_spi->work);
	spin_unlock_irqrestore(&bcm2835_spi->lock, flags);

	return 0;
msg_rejected:
	/* Message rejected and not queued */
	m->status = -EINVAL;
	if (m->complete)
		m->complete(m->context);
	return -EINVAL;
}


static int __init bcm2835_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct bcm2835_spi *spi;
	struct resource *r;
	struct bcm2835_spi_info *spi_info;
	int status = 0;

	printk("in bcm2835_spi_probe\n");

	spi_info = pdev->dev.platform_data;

	master = spi_alloc_master(&pdev->dev, sizeof *spi);
	if (master == NULL) {
		dev_dbg(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	if (pdev->id != -1)
		master->bus_num = pdev->id;

	//TODO check mode_bits supported
	/* we support only mode 0, and no options */
	master->mode_bits = 0;


	//setup data structures and pointers

	master->setup = bcm2835_spi_setup;
	master->transfer = bcm2835_spi_transfer;
	master->num_chipselect = BCM2835_NUM_CHIPSELECTS;

	dev_set_drvdata(&pdev->dev, master);

	spi = spi_master_get_devdata(master);
	spi->master = master;
	spi->spi_info = spi_info;

	//calc the max and min speeds we can allow for transfers and store them
	//TODO check this
	spi->max_speed = DIV_ROUND_UP(spi_info->tclk, 4);
	spi->min_speed = DIV_ROUND_UP(spi_info->tclk, 30);

	//TODO check this -- but get mem defined by platform setup code specific to the SOC/board
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		status = -ENODEV;
		goto out;
	}

	//allocate a memory region that covers the SPI controller area of memory
	if (!request_mem_region(r->start, resource_size(r),
				dev_name(&pdev->dev))) {
		status = -EBUSY;
		goto out;
	}
	//get a  pointer to the base of the memory region we grabbed above. (Basically deals with virtual pages and other stuff)
	//TODO check size to remap
	spi->base = ioremap(r->start, SZ_1K);


	//initialise the work structure and add in our function that will be called 'in a bit' to do the work
	INIT_WORK(&spi->work, bcm2835_spi_work);

	//Initialise other structures e.g. spinlock and a linked list
	spin_lock_init(&spi->lock);
	INIT_LIST_HEAD(&spi->msg_queue);

	if (bcm2835_spi_reset(spi) < 0)
		goto out_rel_mem;

	status = spi_register_master(master);
	if (status < 0)
		goto out_rel_mem;

	return status;

	//actions to unwind if we hit an error somewhere in this function
out_rel_mem:
	release_mem_region(r->start, resource_size(r));

out:
	spi_master_put(master);
	return status;
}


static int __exit bcm2835_spi_remove(struct platform_device *pdev)
{
//TODO
	struct spi_master *master;
	struct bcm2835_spi *spi;
	struct resource *r;

	master = dev_get_drvdata(&pdev->dev);
	spi = spi_master_get_devdata(master);

	cancel_work_sync(&spi->work);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));

	spi_unregister_master(master);

	return 0;
}

MODULE_ALIAS("platform:" DRIVER_NAME);

static struct platform_driver bcm2835_spi_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.remove		= __exit_p(bcm2835_spi_remove),
};

static int __init bcm2835_spi_init(void)
{
	printk("in spi_init driver\n");
	bcm2835_spi_wq = create_singlethread_workqueue(
				bcm2835_spi_driver.driver.name);

	if (bcm2835_spi_wq == NULL)
		return -ENOMEM;

	return platform_driver_probe(&bcm2835_spi_driver, bcm2835_spi_probe);
}
module_init(bcm2835_spi_init);

static void __exit bcm2835_spi_exit(void)
{
	flush_workqueue(bcm2835_spi_wq);
	platform_driver_unregister(&bcm2835_spi_driver);

	destroy_workqueue(bcm2835_spi_wq);
}
module_exit(bcm2835_spi_exit);

MODULE_DESCRIPTION("bcm2835 SPI driver");
MODULE_AUTHOR("A Robinson<arobinson@cs.man.ac.uk>");
MODULE_LICENSE("GPL");

