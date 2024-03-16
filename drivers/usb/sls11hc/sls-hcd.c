/*
 * SLS USB1.1 Host Controller Driver
 *
 * Copyright (C) 2019 LG Electronics, Inc.
 * Author: Hansun Lee <hansun.lee@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <uapi/linux/sched/types.h>

#ifdef DEBUG
static unsigned int dump_epin = BIT(2);
#else
static unsigned int dump_epin;
#endif
module_param(dump_epin, uint, 0644);

#ifdef DEBUG
static unsigned int dump_epout = BIT(2);
#else
static unsigned int dump_epout;
#endif
module_param(dump_epout, uint, 0644);

static unsigned int spi_data_corruption_test;
module_param(spi_data_corruption_test, uint, 0644);

/* Port-change mask */
#define PORT_C_MASK \
	((USB_PORT_STAT_C_CONNECTION | \
	  USB_PORT_STAT_C_ENABLE | \
	  USB_PORT_STAT_C_SUSPEND | \
	  USB_PORT_STAT_C_OVERCURRENT | \
	  USB_PORT_STAT_C_RESET) << 16)

#define SLS_IP_VERSION			0x1013

#define SLS_FIFO_MEM_SIZE		2048
#define SLS_FIFO_SIZE			64

#define SLS_PAYLOAD_IN_SIZE		128
#define SLS_PAYLOAD_OUT_SIZE		1024

#define SLS_INT_MAX			3
#define SLS_INT_MAP_MASK		GENMASK(SLS_INT_MAX - 1, 0)

#define SLS_ATL_MAX			3
#define SLS_ATL_MAP_MASK		GENMASK(SLS_ATL_MAX - 1, 0)

#define SLS_FIFO_MAX			(SLS_FIFO_MEM_SIZE / SLS_FIFO_SIZE)
#define SLS_FIFO_MAP_MASK		GENMASK_ULL(SLS_FIFO_MAX - 1, 0)

/* SPI commands */
#define SLS_CTRL_REG_BASE		0x4004000

#define SLS_REG_VERSION			(SLS_CTRL_REG_BASE + 0x00)
#define SLS_REG_SCRATCH			(SLS_CTRL_REG_BASE + 0x04)

#define SLS_REG_USBCTRL			(SLS_CTRL_REG_BASE + 0x0C)
#define SLS_USBCTRL_REMOTE_WAKEUP	BIT(2)
#define SLS_USBCTRL_CONFIG		BIT(1)

#define SLS_REG_FRINDEX			(SLS_CTRL_REG_BASE + 0x10)

#define SLS_REG_INTR_SRC		(SLS_CTRL_REG_BASE + 0x20)
#define SLS_INTR_INT_TD_DONE		BIT(8)
#define SLS_INTR_ATL_TD_DONE		BIT(7)
#define SLS_INTR_SOF			BIT(1)
#define SLS_INTR_PE			BIT(0)

#define SLS_REG_INTR_EN			(SLS_CTRL_REG_BASE + 0x24)

#define SLS_REG_PORT_CMD		(SLS_CTRL_REG_BASE + 0x30)
#define SLS_PORT_CMD_RESUME_DONE	BIT(7)
#define SLS_PORT_CMD_RESUME		BIT(6)
#define SLS_PORT_CMD_DISABLE		BIT(5)
#define SLS_PORT_CMD_SUSPEND		BIT(4)
#define SLS_PORT_CMD_RESET_DONE		BIT(3)
#define SLS_PORT_CMD_RESET		BIT(2)
#define SLS_PORT_CMD_POWER_OFF		BIT(1)
#define SLS_PORT_CMD_POWER_ON		BIT(0)

#define SLS_REG_PORT_STS		(SLS_CTRL_REG_BASE + 0x34)
#define SLS_PORT_FULL_SPEED		BIT(6)
#define SLS_PORT_RESUME			BIT(5)
#define SLS_PORT_SUSPEND		BIT(4)
#define SLS_PORT_ENABLE			BIT(3)
#define SLS_PORT_RESET			BIT(2)
#define SLS_PORT_CONNECT		BIT(1)
#define SLS_PORT_POWER			BIT(0)

#define SLS_REG_PORT_INTR		(SLS_CTRL_REG_BASE + 0x38)
#define SLS_PORT_REMOTE_WAKEUP		BIT(6)

#define SLS_REG_INT_DONE_MAP		(SLS_CTRL_REG_BASE + 0x40)
#define SLS_REG_INT_SKIP_MAP		(SLS_CTRL_REG_BASE + 0x44)
#define SLS_REG_INT_LAST_MAP		(SLS_CTRL_REG_BASE + 0x48)
#define SLS_REG_INT_ABRT_MAP		(SLS_CTRL_REG_BASE + 0x4C)

#define SLS_REG_ATL_DONE_MAP		(SLS_CTRL_REG_BASE + 0x50)
#define SLS_REG_ATL_SKIP_MAP		(SLS_CTRL_REG_BASE + 0x54)
#define SLS_REG_ATL_LAST_MAP		(SLS_CTRL_REG_BASE + 0x58)
#define SLS_REG_ATL_ABRT_MAP		(SLS_CTRL_REG_BASE + 0x5C)

#define SLS_DATA_REG_BASE		0x4001000

#define SLS_REG_MEM_INT			(SLS_DATA_REG_BASE + 0x100)
#define SLS_REG_MEM_ATL			(SLS_DATA_REG_BASE + 0x200)
#define SLS_REG_MEM_FIFO		(SLS_DATA_REG_BASE + 0x400)

#define SLS_SPI_SOP			0x50
#define SLS_SPI_EOP			0xE0
#define SLS_SPI_ACK			0x01
#define SLS_SPI_ESC			0xD0
#define SLS_SPI_IDL			0x1D

#define SLS_SPI_SOP_OFFSET		0
#define SLS_SPI_ACK_OFFSET		1

#define SLS_SPI_HDR_SIZE		sizeof(struct sls_spi_hdr)
#define SLS_SPI_HDR_TYPE_WR		0x00 /* write register to sls avalon */
#define SLS_SPI_HDR_TYPE_RD		0x10 /* read register from sls avalon */
#define SLS_SPI_HDR_TYPE_CTRL		0x00
#define SLS_SPI_HDR_TYPE_DATA		0x01
#define SLS_SPI_HDR_TYPE_REG(reg)	((reg) >= SLS_CTRL_REG_BASE ? \
					 SLS_SPI_HDR_TYPE_CTRL : \
					 SLS_SPI_HDR_TYPE_DATA)

#define SLS_SPI_WR_META_SIZE		2
#define SLS_SPI_WR_PAYLOAD_OFFSET	(SLS_SPI_WR_META_SIZE + \
					 SLS_SPI_HDR_SIZE)

#define SLS_SPI_RD_META_SIZE		3
#define SLS_SPI_RD_PAYLOAD_OFFSET	2

#define SLS_SPI_META_SIZE		3 /* SOP, ACK, EOP */

#define SLS_XFER_DESC_SIZE		(sizeof(u32) * 4)

#define SLS_XFER_DESC0(toggle, pid) \
	(((toggle) << 31) | (3 << 10) | ((pid)  << 8) | (1 << 7))
#define SLS_XFER_DESC0_TOGGLE(dw0)	(((dw0) >> 31) & 1)
#define SLS_XFER_DESC0_PID(dw0)		(((dw0) >> 8) & 0x3)
#define SLS_XFER_DESC0_ACTIVE(dw0)	(((dw0) >> 7) & 1)
#define SLS_XFER_DESC0_HALT(dw0)	(((dw0) >> 6) & 1)
#define SLS_XFER_DESC0_BABBLE(dw0)	(((dw0) >> 4) & 1)
#define SLS_XFER_DESC0_TRA(dw0)		(((dw0) >> 3) & 1)
#define SLS_XFER_DESC0_ERROR(dw0)	(SLS_XFER_DESC0_HALT(dw0) || \
					 SLS_XFER_DESC0_BABBLE(dw0))

#define SLS_XFER_DESC1(urb) \
	(((usb_pipecontrol(urb->pipe) ? 0 : \
	   usb_pipeisoc(urb->pipe)    ? 1 : \
	   usb_pipebulk(urb->pipe)    ? 2 : 3) << 27) | \
	 (usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe)) << 16) | \
	 (((urb->dev->speed == USB_SPEED_LOW) ? 1 : 0) << 12) | \
	 (usb_pipeendpoint(urb->pipe) << 8) | usb_pipedevice(urb->pipe))
#define SLS_XFER_DESC1_EPNUM(dw1)	(((dw1) >> 8) & 0xF)

#define SLS_XFER_DESC2(urb)		(urb->interval << 16)

#define SLS_XFER_DESC3(reg, len)	((((reg) - SLS_DATA_REG_BASE) << 16) | \
					 (len))
#define SLS_XFER_DESC3_REG(dw3)		(((dw3) >> 16) + SLS_DATA_REG_BASE)
#define SLS_XFER_DESC3_LEN(dw3)		((dw3) & 0x7FFF)

#define SLS_XFER_DESC_INT_REG(slot)	(SLS_REG_MEM_INT + \
					 (SLS_XFER_DESC_SIZE * slot))
#define SLS_XFER_DESC_ATL_REG(slot)	(SLS_REG_MEM_ATL + \
					 (SLS_XFER_DESC_SIZE * slot))
#define SLS_XFER_DESC_XXX_REG(xfer, slot) (xfer ? \
					   SLS_XFER_DESC_INT_REG(slot) : \
					   SLS_XFER_DESC_ATL_REG(slot))

#define SLS_XFER_DESC_FIFO_REG(slot)	(SLS_REG_MEM_FIFO + \
					 (SLS_FIFO_SIZE * slot))

#define SLS_XFER_IS_INT(urb)		(usb_pipeint(urb->pipe) && \
					 usb_urb_dir_in(urb))

enum pkt_state {
	PKT_STATE_SETUP,	/* waiting to send setup packet to ctrl pipe */
	PKT_STATE_TRANSFER,	/* waiting to xfer transfer_buffer */
	PKT_STATE_TERMINATE	/* waiting to terminate control transfer */
};

enum scheduling_pass {
	SCHED_PASS_PERIODIC,
	SCHED_PASS_NON_PERIODIC,
	SCHED_PASS_DONE
};

enum sls_td_pid {
	XFR_PID_OUT,
	XFR_PID_IN,
	XFR_PID_SETUP,
};

/* Bit numbers for sls_hcd->todo */
enum {
	ENABLE_IRQ = 0,
	HCD_RESET,
	PORT_POWER,
	PORT_RESET,
	PORT_SUSPEND,
	URB_ENQUEUE,
	CHECK_UNLINK,
};

struct sls_hcd {
	struct device			*dev;
	struct spi_device		*spi;
	struct task_struct		*spi_thread;

	spinlock_t			lock;
	atomic_t			pm_suspended;

	/* lower 16 bits contain port status, upper 16 bits the change mask */
	u32				port_status;

	struct list_head		ep_list; /* list of EP's with work */

	u16				hirq;
	struct urb			*int_urb[SLS_INT_MAX];
	u16				int_skip_map;
	struct urb			*atl_urb[SLS_ATL_MAX];
	u16				atl_skip_map;
	u64				fifo_map;

	struct urb			*curr_urb;
	struct urb			*temp_urb;
	enum scheduling_pass		sched_pass;
	int				urb_done;
	size_t				curr_len;
	unsigned long			todo;
};

struct sls_ep {
	struct usb_host_endpoint	*ep;
	struct usb_device		*dev;
	struct list_head		ep_list;
	enum				pkt_state pkt_state;
	u32				xfer_desc[4];
	u16				skip_slot;
	u32				fifo_reg;
	u64				fifo_map;
	u16				fifo_slot;
};

struct sls_spi_hdr {
	u16 len;
	u8 type;
	u16 reg;
} __packed;

struct sls_dma_tx_buf {
	u8 data[SLS_SPI_META_SIZE + \
		((SLS_SPI_HDR_SIZE + SLS_PAYLOAD_OUT_SIZE) * 2)];
};

struct sls_dma_rx_buf {
	u8 data[SLS_SPI_META_SIZE + \
		((SLS_SPI_HDR_SIZE + SLS_PAYLOAD_IN_SIZE) * 2)];
};

static struct sls_dma_tx_buf *dma_tx_buf = NULL;
static struct sls_dma_rx_buf *dma_rx_buf = NULL;

static void sls_host_transfer_done(struct usb_hcd *hcd);
static int sls_urb_done(struct usb_hcd *hcd);

static inline struct sls_hcd *hcd_to_sls(struct usb_hcd *hcd)
{
	return (struct sls_hcd *)hcd->hcd_priv;
}

static inline struct usb_hcd *sls_to_hcd(struct sls_hcd *sls_hcd)
{
	return container_of((void *)sls_hcd, struct usb_hcd, hcd_priv);
}

static size_t spi_to_avalon(u8 *dst, const u8 *src, size_t len)
{
	u8 *p = dst;

	while (len--) {
		switch (*src) {
		case SLS_SPI_SOP:
		case SLS_SPI_EOP:
		case SLS_SPI_ESC:
		case SLS_SPI_IDL:
			*p = SLS_SPI_ESC;
			p++;
			*p = *src;
			break;
		default:
			*p = *src;
			break;
		}

		p++;
		src++;
	}

	return p - dst;
}

static size_t spi_from_avalon(u8 *dst, size_t dst_len,
		const u8 *src, size_t src_len)
{
	const u8 *p = src;
	size_t remain = dst_len;

	while (remain--) {
		switch (*p) {
		case SLS_SPI_ESC:
			p++;
			*dst = *p;
			break;
		default:
			*dst = *p;
			break;
		}

		p++;
		dst++;
	}

	remain = src_len - (p - src);
	if (*p != SLS_SPI_EOP &&
	    memchr(p, SLS_SPI_EOP, remain)) {
		print_hex_dump(KERN_DEBUG, "EOPERS", DUMP_PREFIX_OFFSET, 16, 1,
			       src, src_len, 0);
		print_hex_dump(KERN_DEBUG, "EOPERD", DUMP_PREFIX_OFFSET, 16, 1,
			       dst, dst_len, 0);

		BUG_ON(spi_data_corruption_test > 1);

		p -= dst_len;
		dst -= dst_len;
		memcpy((u8 *)p, dst, dst_len);
		p += spi_from_avalon(dst, dst_len, p, remain);
	}

	return p - src;
}

static void spi_rd_buf(struct usb_hcd *hcd, u32 reg,
		void *buf, size_t len)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct spi_device *spi = sls_hcd->spi;
	struct spi_transfer x[2];
	struct sls_spi_hdr hdr = {
		len,
		SLS_SPI_HDR_TYPE_RD | SLS_SPI_HDR_TYPE_REG(reg),
		reg & 0xFFFF
	};
	u8 *p;
	int ret;

	if (!buf || !len)
		return;

	dev_vdbg(sls_hcd->dev, "%s: reg:0x%08x len:%d\n", __func__, reg, len);

	p = dma_tx_buf->data;

	*p = SLS_SPI_SOP;
	p++;

	p += spi_to_avalon(p, (u8 *)&hdr, sizeof(hdr));

	*p = SLS_SPI_EOP;
	p++;

	memset(x, 0, sizeof(x));
	x[0].len = p - dma_tx_buf->data;
	x[0].tx_buf = dma_tx_buf->data;
	x[1].len = SLS_SPI_RD_META_SIZE + (len * 2);
	x[1].rx_buf = dma_rx_buf->data;

#ifdef VERBOSE_DEBUG
	print_hex_dump(KERN_DEBUG, "SPIWR", DUMP_PREFIX_OFFSET, 16, 1,
		       x[0].tx_buf, x[0].len, 0);
#endif

	ret = spi_sync_transfer(spi, x, ARRAY_SIZE(x));
	if (ret) {
		dev_err(sls_hcd->dev, "%s: spi_sync error %d\n", __func__, ret);
		goto err;
	}

#ifdef VERBOSE_DEBUG
	print_hex_dump(KERN_DEBUG, "SPIRD", DUMP_PREFIX_OFFSET, 16, 1,
		       x[1].rx_buf, x[1].len, 0);
#endif

	p = x[1].rx_buf;

	if (*p != SLS_SPI_SOP) {
		dev_err(sls_hcd->dev, "%s: bad SOP 0x%02x/%d\n", __func__,
			*p, SLS_SPI_SOP_OFFSET);
		ret = -EPROTO;
	}
	p++;

	if (*p != SLS_SPI_ACK) {
		dev_err(sls_hcd->dev, "%s: bad ACK 0x%02x/%d\n", __func__,
			*p, SLS_SPI_ACK_OFFSET);
		ret = -EPROTO;
	}
	p++;

	p += spi_from_avalon(buf, len,
			&x[1].rx_buf[SLS_SPI_RD_PAYLOAD_OFFSET], x[1].len);
	if (*p != SLS_SPI_EOP) {
		dev_err(sls_hcd->dev, "%s: bad EOP 0x%02x/%d\n", __func__,
			*p, (p - (u8 *)x[1].rx_buf));
		ret = -EPROTO;
	}

	if (ret) {
		dev_err(sls_hcd->dev, "%s: reg:0x%08x len:%d\n", __func__,
			reg, len);
		print_hex_dump(KERN_ERR, "SPIRD", DUMP_PREFIX_OFFSET, 16, 1,
			       x[1].rx_buf, x[1].len, 0);
		print_hex_dump(KERN_ERR, "BUFRD", DUMP_PREFIX_OFFSET, 16, 1,
			       buf, len, 0);

		BUG_ON(spi_data_corruption_test);
	}

err:
	if (ret)
		memset(buf, 0, len);
	return;
}

static void spi_wr_buf(struct usb_hcd *hcd, u32 reg,
		const void *buf, size_t len)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct spi_device *spi = sls_hcd->spi;
	struct spi_transfer x[2];
	struct sls_spi_hdr hdr = {
		len,
		SLS_SPI_HDR_TYPE_WR | SLS_SPI_HDR_TYPE_REG(reg),
		reg & 0xFFFF
	};
	u8 *p;
	int ret;

	if (!buf || !len)
		return;

	dev_vdbg(sls_hcd->dev, "%s: reg:0x%08x len:%d\n", __func__, reg, len);

	p = dma_tx_buf->data;

	*p = SLS_SPI_SOP;
	p++;

	p += spi_to_avalon(p, (u8 *)&hdr, sizeof(hdr));

	p += spi_to_avalon(p, buf, len);

	*p = SLS_SPI_EOP;
	p++;

	memset(x, 0, sizeof(x));
	x[0].len = p - dma_tx_buf->data;
	x[0].tx_buf = dma_tx_buf->data;
	x[1].len = SLS_SPI_RD_META_SIZE;
	x[1].rx_buf = dma_rx_buf->data;

#ifdef VERBOSE_DEBUG
	print_hex_dump(KERN_DEBUG, "SPIWR", DUMP_PREFIX_OFFSET, 16, 1,
		       x[0].tx_buf, x[0].len, 0);
#endif

	ret = spi_sync_transfer(spi, x, ARRAY_SIZE(x));
	if (ret) {
		dev_err(sls_hcd->dev, "%s: spi_sync error %d\n", __func__, ret);
		return;
	}

#ifdef VERBOSE_DEBUG
	print_hex_dump(KERN_DEBUG, "SPIRD", DUMP_PREFIX_OFFSET, 16, 1,
		       x[1].rx_buf, x[1].len, 0);
#endif

	p = x[1].rx_buf;

	if (p[SLS_SPI_SOP_OFFSET] != SLS_SPI_SOP) {
		dev_err(sls_hcd->dev, "%s: bad SOP 0x%02x/%d\n", __func__,
			p[SLS_SPI_SOP_OFFSET], SLS_SPI_SOP_OFFSET);
		ret = -EPROTO;
	}

	if (p[SLS_SPI_ACK_OFFSET] != SLS_SPI_ACK) {
		dev_err(sls_hcd->dev, "%s: bad ACK 0x%02x/%d\n", __func__,
			p[SLS_SPI_ACK_OFFSET], SLS_SPI_ACK_OFFSET);
		ret = -EPROTO;
	}

	if (p[SLS_SPI_RD_PAYLOAD_OFFSET] != SLS_SPI_EOP) {
		dev_err(sls_hcd->dev, "%s: bad EOP 0x%02x/%d\n", __func__,
			p[SLS_SPI_RD_PAYLOAD_OFFSET],
			SLS_SPI_RD_PAYLOAD_OFFSET);
		ret = -EPROTO;
	}

	if (ret) {
		dev_err(sls_hcd->dev, "%s: reg:0x%08x len:%d\n", __func__,
			reg, len);
		print_hex_dump(KERN_ERR, "SPIRD", DUMP_PREFIX_OFFSET, 16, 1,
			       x[1].rx_buf, x[1].len, 0);
		print_hex_dump(KERN_ERR, "BUFRD", DUMP_PREFIX_OFFSET, 16, 1,
			       buf, len, 0);

		BUG_ON(spi_data_corruption_test);
	}
}

static u16 spi_rd16(struct usb_hcd *hcd, u32 reg)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	u16 val;

	spi_rd_buf(hcd, reg, &val, 2);

	dev_dbg(sls_hcd->dev, "%s: reg:0x%08x val:0x%04x\n", __func__,
		reg, val);

	return val;
}

static void spi_wr16(struct usb_hcd *hcd, u32 reg, u16 val)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	spi_wr_buf(hcd, reg, &val, 2);

	dev_dbg(sls_hcd->dev, "%s: reg:0x%08x val:0x%04x\n", __func__,
		reg, val);
}

static void dump_sls(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct sls_ep *sls_ep;
	struct usb_host_endpoint *ep;
	struct urb *urb;
	int epnum;
	u32 desc_reg, desc[4];
	u16 hirq, skip_map, done_map;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&sls_hcd->lock, flags);

	list_for_each_entry(sls_ep, &sls_hcd->ep_list, ep_list) {
		ep = sls_ep->ep;

		epnum = usb_endpoint_num(&ep->desc);
		dev_info(sls_hcd->dev, "%s: EP%0u%s %u\n",
			 __func__,
			 epnum,
			 usb_endpoint_dir_in(&ep->desc) ? "IN " : "OUT",
			 sls_ep->pkt_state);

		list_for_each_entry(urb, &ep->urb_list, urb_list) {
			dev_info(sls_hcd->dev, "%s:  URB %p ep%d%s-%s %d/%d"
				 " status:%d unlinked:%d\n",
				 __func__,
				 urb,
				 epnum,
				 usb_urb_dir_in(urb) ? "in" : "out",
				 usb_pipeisoc(urb->pipe)    ? "isoc" :
				 usb_pipeint(urb->pipe)     ? "intr"  :
				 usb_pipecontrol(urb->pipe) ? "ctrl" : "bulk",
				 urb->actual_length,
				 urb->transfer_buffer_length,
				 urb->status,
				 urb->unlinked);
		}
	}

	spin_unlock_irqrestore(&sls_hcd->lock, flags);

	hirq = spi_rd16(hcd, SLS_REG_INTR_SRC);
	dev_info(sls_hcd->dev, "%s: hirq:0x%04x\n", __func__, hirq);

	skip_map = spi_rd16(hcd, SLS_REG_INT_SKIP_MAP);
	done_map = spi_rd16(hcd, SLS_REG_INT_DONE_MAP);
	dev_info(sls_hcd->dev, "%s: INT skip_map:0x%04x(0x%04x) done_map:0x%04x\n",
		 __func__, sls_hcd->int_skip_map, skip_map, done_map);
	for (i = 0; i < SLS_INT_MAX; i++) {
		desc_reg = SLS_XFER_DESC_XXX_REG(1, i);
		spi_rd_buf(hcd, desc_reg, desc, SLS_XFER_DESC_SIZE);
		dev_dbg(sls_hcd->dev, "%s:  INT[%d] xfer_desc[%08x, %08x, %08x, %08x]\n",
			__func__, i, desc[0], desc[1], desc[2], desc[3]);
	}
	for (i = 0; i < SLS_INT_MAX; i++) {
		urb = sls_hcd->int_urb[i];

		if (!urb)
			continue;

		sls_ep = urb->ep->hcpriv;

		dev_info(sls_hcd->dev, "%s:  URB[%d] %p\n",
			 __func__, i, urb);
		dev_info(sls_hcd->dev, "%s:   skip_slot:%d fifo_slot:%d(0x%016lx)\n",
			 __func__, i, sls_ep->fifo_slot, sls_ep->fifo_map);
		dev_info(sls_hcd->dev, "%s:   xfer_desc[%08x, %08x, %08x, %08x]\n",
			 __func__,
			 sls_ep->xfer_desc[0], sls_ep->xfer_desc[1],
			 sls_ep->xfer_desc[2], sls_ep->xfer_desc[3]);
	}

	skip_map = spi_rd16(hcd, SLS_REG_ATL_SKIP_MAP);
	done_map = spi_rd16(hcd, SLS_REG_ATL_DONE_MAP);
	dev_info(sls_hcd->dev, "%s: ATL skip_map:0x%04x(0x%04x) done_map:0x%04x\n",
		 __func__, sls_hcd->atl_skip_map, skip_map, done_map);
	for (i = 0; i < SLS_ATL_MAX; i++) {
		desc_reg = SLS_XFER_DESC_XXX_REG(0, i);
		spi_rd_buf(hcd, desc_reg, desc, SLS_XFER_DESC_SIZE);
		dev_dbg(sls_hcd->dev, "%s:  ATL[%d] xfer_desc[%08x, %08x, %08x, %08x]\n",
			__func__, i, desc[0], desc[1], desc[2], desc[3]);
	}
	for (i = 0; i < SLS_ATL_MAX; i++) {
		urb = sls_hcd->atl_urb[i];

		if (!urb)
			continue;

		sls_ep = urb->ep->hcpriv;

		dev_info(sls_hcd->dev, "%s:  URB[%d] %p\n",
			 __func__, i, urb);
		dev_info(sls_hcd->dev, "%s:   skip_slot:%d fifo_slot:%d(0x%016lx)\n",
			 __func__, i, sls_ep->fifo_slot, sls_ep->fifo_map);
		dev_info(sls_hcd->dev, "%s:   xfer_desc[%08x, %08x, %08x, %08x]\n",
			 __func__,
			 sls_ep->xfer_desc[0], sls_ep->xfer_desc[1],
			 sls_ep->xfer_desc[2], sls_ep->xfer_desc[3]);
	}

	dev_info(sls_hcd->dev, "%s: fifo_map:0x%016lx\n", __func__,
		 sls_hcd->fifo_map);
}

static u64 sls_alloc_fifo_map(struct usb_hcd *hcd, size_t len)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct {
		size_t min;
		size_t max;
		u64 map;
	} maps[] = {
		{ 512, 1024, 0x00000000FFFF0000 },
		{ 128, 512,  0x000000000000FF00 },
		{ 64,  128,  0x00000000000000C0 },
	};
	int i;

	if (unlikely(!sls_hcd->fifo_map))
		return 0;

	for (i = 0; i < ARRAY_SIZE(maps); i++) {
		if (len > maps[i].min && len <= maps[i].max) {
			if (sls_hcd->fifo_map & maps[i].map)
				return maps[i].map;
		}
	}

	return BIT(__ffs(sls_hcd->fifo_map));
}

static void sls_send_data(struct usb_hcd *hcd, struct urb *urb, u32 *desc,
		const void *buf, size_t len)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct sls_ep *sls_ep = urb->ep->hcpriv;
	bool int_xfer = SLS_XFER_IS_INT(urb);
	u32 skip_reg, desc_reg;
	u16 skip_slot;
	u16 *skip_map;
	struct urb **xxx_urb;
	u16 xxx_msk;

	if (int_xfer) {
		skip_reg = SLS_REG_INT_SKIP_MAP;
		skip_map = &sls_hcd->int_skip_map;
		xxx_urb = sls_hcd->int_urb;
		xxx_msk = SLS_INT_MAP_MASK;
	} else {
		skip_reg = SLS_REG_ATL_SKIP_MAP;
		skip_map = &sls_hcd->atl_skip_map;
		xxx_urb = sls_hcd->atl_urb;
		xxx_msk = SLS_ATL_MAP_MASK;
	}

	if (unlikely(!(*skip_map))) {
		dev_err(sls_hcd->dev, "%s: skip_map is empty (%s)\n",
			__func__, int_xfer ? "INT" : "ATL");
		goto err;
	}
	skip_slot = sls_ep->skip_slot = __ffs(*skip_map);
	dev_dbg(sls_hcd->dev, "%s: skip_map:0x%04x slot:%d (%s)\n", __func__,
		*skip_map, skip_slot, int_xfer ? "INT" : "ATL");

	if (unlikely(xxx_urb[skip_slot])) {
		dev_err(sls_hcd->dev, "%s: slot:%d is already in use\n",
			__func__, skip_slot);
		goto err;
	}
	xxx_urb[skip_slot] = urb;

	if (unlikely(!sls_hcd->fifo_map)) {
		dev_err(sls_hcd->dev, "%s: fifo_map is empty\n", __func__);
		xxx_urb[skip_slot] = NULL;
		goto err;
	}

	if (len) {
		sls_ep->fifo_map = sls_alloc_fifo_map(hcd, len);
		if (unlikely(!sls_ep->fifo_map)) {
			dev_err(sls_hcd->dev, "%s: fifo_map alloc failed\n",
				__func__);
			xxx_urb[skip_slot] = NULL;
			goto err;
		}
		sls_ep->fifo_slot = __ffs(sls_ep->fifo_map);
		sls_ep->fifo_reg = SLS_XFER_DESC_FIFO_REG(sls_ep->fifo_slot);
		dev_dbg(sls_hcd->dev, "%s: fifo_map:0x%016lx slot:%d(0x%016lx)\n",
			__func__, sls_hcd->fifo_map, sls_ep->fifo_slot,
			sls_ep->fifo_map);
		sls_hcd->fifo_map &= ~sls_ep->fifo_map;

		desc[3] = SLS_XFER_DESC3(sls_ep->fifo_reg, len);
	} else {
		sls_ep->fifo_map = 0;
		desc[3] = 0;
	}
	dev_dbg(sls_hcd->dev, "%s: xfer_desc[%08x, %08x, %08x, %08x]\n",
		__func__, desc[0], desc[1], desc[2], desc[3]);

	desc_reg = SLS_XFER_DESC_XXX_REG(int_xfer, skip_slot);
	spi_wr_buf(hcd, desc_reg, desc, SLS_XFER_DESC_SIZE);

	switch (SLS_XFER_DESC0_PID(desc[0])) {
	case XFR_PID_OUT:
	case XFR_PID_SETUP:
		if (!len)
			break;

		dev_dbg(sls_hcd->dev, "%s: reg:0x%08x len:%d\n", __func__,
			sls_ep->fifo_reg, len);
		if (dump_epout & BIT(usb_pipeendpoint(urb->pipe)))
			print_hex_dump_bytes("WR", DUMP_PREFIX_OFFSET,
					     buf, len);
		spi_wr_buf(hcd, sls_ep->fifo_reg, buf, len);
		break;

	case XFR_PID_IN:
		break;
	}

	*skip_map &= ~BIT(skip_slot);
	spi_wr16(hcd, skip_reg, ~BIT(skip_slot));
	return;

err:
	dump_sls(hcd);
	urb->status = -EINPROGRESS;
	sls_hcd->curr_urb = NULL;
	if (sls_hcd->temp_urb) {
		sls_hcd->curr_urb = sls_hcd->temp_urb;
		sls_hcd->temp_urb = NULL;
	}
	return;
}

static void sls_ctrl_setup(struct usb_hcd *hcd, struct urb *urb)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct sls_ep *sls_ep = urb->ep->hcpriv;
	u32 *desc = sls_ep->xfer_desc;

	dev_dbg(sls_hcd->dev, "SETUP ep%d%s ctrl: %02x %02x %02x %02x %02x %02x %02x %02x\n",
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb) ? "in" : "out",
		urb->setup_packet[0],
		urb->setup_packet[1],
		urb->setup_packet[2],
		urb->setup_packet[3],
		urb->setup_packet[4],
		urb->setup_packet[5],
		urb->setup_packet[6],
		urb->setup_packet[7]);

	memset(desc, 0, SLS_XFER_DESC_SIZE);
	desc[0] = SLS_XFER_DESC0(0, XFR_PID_SETUP);
	desc[1] = SLS_XFER_DESC1(urb);

	sls_send_data(hcd, urb, desc, urb->setup_packet, 8);
}

static void sls_transfer_in(struct usb_hcd *hcd, struct urb *urb)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct sls_ep *sls_ep = urb->ep->hcpriv;
	u32 max_packet;
	unsigned int toggle;
	u32 *desc = sls_ep->xfer_desc;
	void *src;
	size_t len;

	dev_dbg(sls_hcd->dev, "DATA ep%din %d/%d\n",
		usb_pipeendpoint(urb->pipe),
		urb->actual_length,
		urb->transfer_buffer_length);

	max_packet = usb_maxpacket(urb->dev, urb->pipe, 0);
	if (max_packet > SLS_FIFO_SIZE) {
		/* We do not support isochronous transfers */
		dev_err(sls_hcd->dev,
			"%s: packet-size of %u too big (limit is %u bytes)",
			__func__, max_packet, SLS_FIFO_SIZE);
		sls_hcd->urb_done = -EMSGSIZE;
		return;
	}

	toggle = usb_gettoggle(urb->dev, usb_pipeendpoint(urb->pipe), 0);

	src = urb->transfer_buffer + urb->actual_length;
	len = min((urb->transfer_buffer_length - urb->actual_length),
		  usb_pipeint(urb->pipe) ?
		  (u32)SLS_FIFO_SIZE : (u32)SLS_PAYLOAD_IN_SIZE);

	memset(desc, 0, SLS_XFER_DESC_SIZE);
	desc[0] = SLS_XFER_DESC0(toggle, XFR_PID_IN);
	desc[1] = SLS_XFER_DESC1(urb);
	if (usb_pipeint(urb->pipe))
		desc[2] = SLS_XFER_DESC2(urb);

	sls_send_data(hcd, urb, desc, src, len);
}

static void sls_transfer_out(struct usb_hcd *hcd, struct urb *urb)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct sls_ep *sls_ep = urb->ep->hcpriv;
	u32 max_packet;
	unsigned int toggle;
	u32 *desc = sls_ep->xfer_desc;
	void *src;
	size_t len;

	dev_dbg(sls_hcd->dev, "DATA ep%dout %d/%d\n",
		usb_pipeendpoint(urb->pipe),
		urb->actual_length,
		urb->transfer_buffer_length);

	max_packet = usb_maxpacket(urb->dev, urb->pipe, 1);
	if (max_packet > SLS_FIFO_SIZE) {
		/* We do not support isochronous transfers */
		dev_err(sls_hcd->dev,
			"%s: packet-size of %u too big (limit is %u bytes)",
			__func__, max_packet, SLS_FIFO_SIZE);
		sls_hcd->urb_done = -EMSGSIZE;
		return;
	}

	toggle = usb_gettoggle(urb->dev, usb_pipeendpoint(urb->pipe), 1);

	src = urb->transfer_buffer + urb->actual_length;
	len = min((urb->transfer_buffer_length - urb->actual_length),
		  (u32)SLS_PAYLOAD_OUT_SIZE);

	memset(desc, 0, SLS_XFER_DESC_SIZE);
	desc[0] = SLS_XFER_DESC0(toggle, XFR_PID_OUT);
	desc[1] = SLS_XFER_DESC1(urb);
	if (usb_pipeint(urb->pipe))
		desc[2] = SLS_XFER_DESC2(urb);

	sls_send_data(hcd, urb, desc, src, len);
}

static void sls_transfer_term(struct usb_hcd *hcd, struct urb *urb)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct sls_ep *sls_ep = urb->ep->hcpriv;
	u32 *desc = sls_ep->xfer_desc;

	dev_dbg(sls_hcd->dev, "TERM ep%d%s ctrl %d/%d\n",
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb) ? "in" : "out",
		urb->actual_length,
		urb->transfer_buffer_length);

	memset(desc, 0, SLS_XFER_DESC_SIZE);
	desc[0] = SLS_XFER_DESC0(1,
			usb_urb_dir_in(urb) ? XFR_PID_OUT : XFR_PID_IN);
	desc[1] = SLS_XFER_DESC1(urb);

	sls_send_data(hcd, urb, desc, NULL, 0);
}

static void sls_next_transfer(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct urb *urb = sls_hcd->curr_urb;
	struct sls_ep *sls_ep;

	if (!urb)
		return; /* nothing to do */

	sls_ep = urb->ep->hcpriv;

	switch (sls_ep->pkt_state) {
	case PKT_STATE_SETUP:
		sls_ctrl_setup(hcd, urb);

		if (urb->transfer_buffer_length && sls_hcd->atl_skip_map) {
			u16 done_map;
			u64 fifo_map;

			done_map = spi_rd16(hcd, SLS_REG_ATL_DONE_MAP);
			if (!(done_map & BIT(sls_ep->skip_slot)))
				break;

			fifo_map = sls_ep->fifo_map;
			sls_hcd->atl_urb[sls_ep->skip_slot] = NULL;
			sls_host_transfer_done(hcd);
			sls_ep->fifo_map |= fifo_map;
			sls_hcd->hirq |= SLS_INTR_ATL_TD_DONE;
		}
		break;

	case PKT_STATE_TRANSFER:
		/*
		 * IN transfers are terminated with HS_OUT token,
		 * OUT transfers with HS_IN:
		 */
		if (usb_urb_dir_in(urb))
			sls_transfer_in(hcd, urb);
		else
			sls_transfer_out(hcd, urb);
		break;

	case PKT_STATE_TERMINATE:
		/*
		 * IN transfers are terminated with HS_OUT token,
		 * OUT transfers with HS_IN:
		 */
		sls_transfer_term(hcd, urb);

		{
			u16 done_map;

			done_map = spi_rd16(hcd, SLS_REG_ATL_DONE_MAP);
			if (!(done_map & BIT(sls_ep->skip_slot)))
				break;

			sls_hcd->hirq |= SLS_INTR_ATL_TD_DONE;
		}
		break;
	}

	if (!usb_pipecontrol(urb->pipe)) {
		sls_hcd->curr_urb = NULL;
		if (sls_hcd->temp_urb) {
			sls_hcd->curr_urb = sls_hcd->temp_urb;
			sls_hcd->temp_urb = NULL;
		}
	}
}

static int sls_select_and_start_urb(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct urb *urb, *curr_urb = NULL;
	struct sls_ep *sls_ep;
	struct usb_host_endpoint *ep;
	struct list_head *pos;
	unsigned long flags;

	dev_dbg(sls_hcd->dev, "%s: %d\n", __func__, sls_hcd->sched_pass);

	spin_lock_irqsave(&sls_hcd->lock, flags);

	for (; sls_hcd->sched_pass < SCHED_PASS_DONE; sls_hcd->sched_pass++) {
		list_for_each(pos, &sls_hcd->ep_list) {
			urb = NULL;
			sls_ep = container_of(pos, struct sls_ep, ep_list);

			ep = sls_ep->ep;

			switch (usb_endpoint_type(&ep->desc)) {
			case USB_ENDPOINT_XFER_ISOC:
			case USB_ENDPOINT_XFER_INT:
				if (sls_hcd->sched_pass !=
				    SCHED_PASS_PERIODIC)
					continue;
				break;

			case USB_ENDPOINT_XFER_CONTROL:
			case USB_ENDPOINT_XFER_BULK:
				if (sls_hcd->sched_pass !=
				    SCHED_PASS_NON_PERIODIC)
					continue;
				break;
			}

			if (list_empty(&ep->urb_list))
				continue; /* nothing to do */

			urb = list_first_entry(&ep->urb_list, struct urb,
					       urb_list);
			if (urb->unlinked) {
				dev_dbg(sls_hcd->dev, "%s: URB %p ep%d%s unlinked=%d\n",
					__func__,
					urb,
					usb_pipeendpoint(urb->pipe),
					usb_urb_dir_in(urb) ? "in" : "out",
					urb->unlinked);
				sls_hcd->curr_urb = urb;
				sls_hcd->urb_done = 1;
				spin_unlock_irqrestore(&sls_hcd->lock, flags);
				return 1;
			}

			if (urb->status != -EINPROGRESS)
				continue;

			/* move current ep to tail */
			list_move_tail(pos, &sls_hcd->ep_list);
			curr_urb = urb;
			goto done;
		}
	}
done:
	if (!curr_urb) {
		spin_unlock_irqrestore(&sls_hcd->lock, flags);
		return 0;
	}

	urb = sls_hcd->curr_urb = curr_urb;
	urb->status = -EALREADY;

	dev_dbg(sls_hcd->dev, "%s: URB %p ep%d%s\n",
		__func__,
		urb,
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb) ? "in" : "out");

	/* start USB transaction */
	if (usb_endpoint_xfer_control(&ep->desc)) {
		/*
		 * See USB 2.0 spec section 8.6.1
		 * Initialization via SETUP Token
		 */
		usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe),
			      usb_pipeout(urb->pipe), 1);
		sls_ep->pkt_state = PKT_STATE_SETUP;
	} else {
		sls_ep->pkt_state = PKT_STATE_TRANSFER;
	}

	spin_unlock_irqrestore(&sls_hcd->lock, flags);

	sls_next_transfer(hcd);

	if (urb->status != -EALREADY) {
		dev_err(sls_hcd->dev, "%s: failed to transfer ep%d%s\n",
			__func__,
			usb_pipeendpoint(urb->pipe),
			usb_urb_dir_in(urb) ? "in" : "out");
		return 0;
	}

	return 1;
}

static void sls_purge_data(struct usb_hcd *hcd, struct urb *urb)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct urb **xxx_urb;
	unsigned int xxx_max;
	u32 abrt_reg, done_reg;
	u16 *skip_map;
	u16 slot;

	if (SLS_XFER_IS_INT(urb)) {
		xxx_urb = sls_hcd->int_urb;
		xxx_max = SLS_INT_MAX;
		abrt_reg = SLS_REG_INT_ABRT_MAP;
		done_reg = SLS_REG_INT_DONE_MAP;
		skip_map = &sls_hcd->int_skip_map;
	} else {
		xxx_urb = sls_hcd->atl_urb;
		xxx_max = SLS_ATL_MAX;
		abrt_reg = SLS_REG_ATL_ABRT_MAP;
		done_reg = SLS_REG_ATL_DONE_MAP;
		skip_map = &sls_hcd->atl_skip_map;
	}

	for (slot = 0; slot < xxx_max; slot++) {
		if (xxx_urb[slot] == urb) {
			struct sls_ep *sls_ep = urb->ep->hcpriv;
			u32 *desc = sls_ep->xfer_desc;
			int abrt_cnt = 3;
			u32 desc_reg;
			unsigned int toggle;
			int epnum, epout;

			dev_dbg(sls_hcd->dev, "%s: slot:%d (%s)\n", __func__,
				slot, usb_pipeint(urb->pipe) ? "INT" : "ATL");

			dev_dbg(sls_hcd->dev, "%s: skip_map:0x%04x fifo_map:0x%016lx fifo_slot:%d\n",
				__func__,
				*skip_map,
				sls_hcd->fifo_map,
				sls_ep->fifo_slot);
			*skip_map |= BIT(slot);
			sls_hcd->fifo_map |= sls_ep->fifo_map;

			dev_dbg(sls_hcd->dev, "%s: xfer_desc[%08x, %08x, %08x, %08x]\n",
				__func__, desc[0], desc[1], desc[2], desc[3]);

			xxx_urb[slot] = NULL;

			spi_wr16(hcd, abrt_reg, BIT(slot));
			do {
				u16 abrt_map = spi_rd16(hcd, abrt_reg);
				if (!(abrt_map & BIT(slot)))
					break;
			} while (abrt_cnt--);
			spi_wr16(hcd, done_reg, BIT(slot));

			desc_reg = SLS_XFER_DESC_XXX_REG(SLS_XFER_IS_INT(urb), slot);
			spi_rd_buf(hcd, desc_reg, desc, SLS_XFER_DESC_SIZE);
			dev_dbg(sls_hcd->dev, "%s: xfer_desc[%08x, %08x, %08x, %08x]\n",
				__func__, desc[0], desc[1], desc[2], desc[3]);

			epnum = usb_pipeendpoint(urb->pipe);
			epout = usb_pipeout(urb->pipe);
			toggle = SLS_XFER_DESC0_TOGGLE(desc[0]);
			usb_settoggle(urb->dev, epnum, epout, toggle);

			return;
		}
	}
}

static int sls_check_unlink(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct sls_ep *sls_ep, *sls_ep_n;
	struct usb_host_endpoint *ep;
	struct urb *urb, *urb_n;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&sls_hcd->lock, flags);

	list_for_each_entry_safe(sls_ep, sls_ep_n, &sls_hcd->ep_list, ep_list) {
		ep = sls_ep->ep;
		list_for_each_entry_safe(urb, urb_n, &ep->urb_list, urb_list) {
			if (!(sls_hcd->port_status & USB_PORT_STAT_ENABLE) &&
			    !urb->unlinked)
				urb->unlinked = -ESHUTDOWN;

			if (urb->unlinked) {
				ret = 1;
				dev_dbg(sls_hcd->dev, "%s: URB %p ep%d%s unlinked=%d\n",
					__func__,
					urb,
					usb_pipeendpoint(urb->pipe),
					usb_urb_dir_in(urb) ? "in" : "out",
					urb->unlinked);
				if (urb == sls_hcd->temp_urb)
					sls_hcd->temp_urb = NULL;
				if (urb == sls_hcd->curr_urb) {
					sls_hcd->urb_done = 0;
					sls_hcd->curr_urb = NULL;
					if (sls_hcd->temp_urb) {
						sls_hcd->curr_urb = sls_hcd->temp_urb;
						sls_hcd->temp_urb = NULL;
					}
				}
				usb_hcd_unlink_urb_from_ep(hcd, urb);
				spin_unlock_irqrestore(&sls_hcd->lock, flags);
				sls_purge_data(hcd, urb);
				urb->status = -EINPROGRESS;
				usb_hcd_giveback_urb(hcd, urb, 0);
				spin_lock_irqsave(&sls_hcd->lock, flags);
			}
		}
	}

	spin_unlock_irqrestore(&sls_hcd->lock, flags);

	return ret;
}

static void sls_recv_data_available(struct usb_hcd *hcd, bool int_xfer)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	u32 done_reg, desc_reg;
	u16 done_slot;
	u16 done_map, *skip_map;
	struct urb *urb;
	struct sls_ep *sls_ep;
	u32 desc[4];
	struct urb **xxx_urb;
	u16 xxx_msk;
	size_t transfer_size;

	if (int_xfer) {
		done_reg = SLS_REG_INT_DONE_MAP;
		skip_map = &sls_hcd->int_skip_map;
		xxx_urb = sls_hcd->int_urb;
		xxx_msk = SLS_INT_MAP_MASK;
	} else {
		done_reg = SLS_REG_ATL_DONE_MAP;
		skip_map = &sls_hcd->atl_skip_map;
		xxx_urb = sls_hcd->atl_urb;
		xxx_msk = SLS_ATL_MAP_MASK;
	}

	done_map = spi_rd16(hcd, done_reg);
	if (unlikely(!(done_map & xxx_msk))) {
		dev_dbg(sls_hcd->dev, "%s: done_map is empty (%s)\n",
			__func__, int_xfer ? "INT" : "ATL");
		return;
	}
	spi_wr16(hcd, done_reg, done_map);
	done_map &= xxx_msk;

repeat:
	done_slot = __ffs(done_map);
	dev_dbg(sls_hcd->dev, "%s: done_map:0x%04x slot:%d (%s)\n", __func__,
		done_map, done_slot, int_xfer ? "INT" : "ATL");
	done_map &= ~BIT(done_slot);
	*skip_map |= BIT(done_slot);

	urb = xxx_urb[done_slot];
	if (!urb)
		goto done;
	xxx_urb[done_slot] = NULL;

	sls_ep = urb->ep->hcpriv;

	desc_reg = SLS_XFER_DESC_XXX_REG(int_xfer, done_slot);
	spi_rd_buf(hcd, desc_reg, desc, SLS_XFER_DESC_SIZE);
	dev_dbg(sls_hcd->dev, "%s: xfer_desc[%08x, %08x, %08x, %08x]\n",
		__func__, desc[0], desc[1], desc[2], desc[3]);

	if (sls_ep->fifo_map) {
		sls_hcd->fifo_map |= sls_ep->fifo_map;
		dev_dbg(sls_hcd->dev, "%s: fifo_map:0x%016lx slot:%d(0x%016lx)\n",
			__func__, sls_hcd->fifo_map, sls_ep->fifo_slot,
			sls_ep->fifo_map);
	}

	transfer_size = SLS_XFER_DESC3_LEN(sls_ep->xfer_desc[3]);
	sls_ep->xfer_desc[0] = desc[0];
	sls_ep->xfer_desc[3] = desc[3];

	if (unlikely(SLS_XFER_DESC0_ERROR(desc[0])))
		goto err;

	if (unlikely(SLS_XFER_DESC0_ACTIVE(desc[0]))) {
		dev_err(sls_hcd->dev, "%s: ACTIVE is not cleared\n", __func__);
		dev_err(sls_hcd->dev, "%s:  URB %p ep%d%s-%s %d/%d"
			 " status:%d unlinked:%d\n",
			 __func__,
			 urb,
			 usb_pipeendpoint(urb->pipe),
			 usb_urb_dir_in(urb) ? "in" : "out",
			 usb_pipeisoc(urb->pipe)    ? "isoc" :
			 usb_pipeint(urb->pipe)     ? "intr"  :
			 usb_pipecontrol(urb->pipe) ? "ctrl" : "bulk",
			 urb->actual_length,
			 urb->transfer_buffer_length,
			 urb->status,
			 urb->unlinked);
#ifndef DEBUG
		dev_err(sls_hcd->dev, "%s: done_map:0x%04x slot:%d (%s)\n",
			__func__, done_map, done_slot,
			int_xfer ? "INT" : "ATL");
		dev_err(sls_hcd->dev, "%s: fifo_map:0x%016lx slot:%d(0x%016lx)\n",
			__func__, sls_hcd->fifo_map, sls_ep->fifo_slot,
			sls_ep->fifo_map);
		dev_err(sls_hcd->dev, "%s: xfer_desc[%08x, %08x, %08x, %08x]\n",
			__func__, desc[0], desc[1], desc[2], desc[3]);
#endif
		dump_sls(hcd);

		transfer_size = SLS_XFER_DESC3_LEN(desc[3]);
	}

	transfer_size -= SLS_XFER_DESC3_LEN(desc[3]);

	switch (SLS_XFER_DESC0_PID(desc[0])) {
	case XFR_PID_OUT:
		sls_hcd->curr_len = transfer_size;
		break;

	case XFR_PID_IN:
		if (transfer_size > 0) {
			void *dst = urb->transfer_buffer + urb->actual_length;

			dev_dbg(sls_hcd->dev, "%s: reg:0x%08x len:%d\n",
				__func__, sls_ep->fifo_reg, transfer_size);
			spi_rd_buf(hcd, sls_ep->fifo_reg, dst, transfer_size);
			if (dump_epin & BIT(usb_pipeendpoint(urb->pipe)))
				print_hex_dump_bytes("RD", DUMP_PREFIX_OFFSET,
						     dst, transfer_size);
			urb->actual_length += transfer_size;
			sls_hcd->curr_len = transfer_size;
		}
		break;

	case XFR_PID_SETUP:
		break;
	}

err:
	if (sls_hcd->curr_urb && sls_hcd->curr_urb != urb)
		sls_hcd->temp_urb = sls_hcd->curr_urb;
	sls_hcd->curr_urb = urb;
	sls_host_transfer_done(hcd);

done:
	if (done_map)
		goto repeat;
}

static void sls_handle_error(struct usb_hcd *hcd, u32 dw0)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s: %s%s%s%s\n", __func__,
		SLS_XFER_DESC0_HALT(dw0)   ? "H" : "",
		SLS_XFER_DESC0_BABBLE(dw0) ? "B" : "",
		SLS_XFER_DESC0_TRA(dw0)    ? "T" : "",
		SLS_XFER_DESC0_ACTIVE(dw0) ? "A" : "");

	if (SLS_XFER_DESC0_HALT(dw0))
		sls_hcd->urb_done = -EPIPE;
	else if (SLS_XFER_DESC0_BABBLE(dw0))
		sls_hcd->urb_done = -EPIPE;
	/* Trasaction error doesn't need to report as error */
	//else if (SLS_XFER_DESC0_TRA(dw0))
	//	sls_hcd->urb_done = -EPROTO;
	else
		sls_hcd->urb_done = -EINVAL;
}

static int sls_transfer_in_done(struct usb_hcd *hcd, struct urb *urb)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	u32 max_packet;

	dev_dbg(sls_hcd->dev, "%s: ep%din %d/%d\n", __func__,
		usb_pipeendpoint(urb->pipe),
		urb->actual_length,
		urb->transfer_buffer_length);

	if (urb->actual_length >= urb->transfer_buffer_length)
		return 1; /* read is complete, so we're done */

	/*
	 * USB 2.0 Section 5.3.2 Pipes: packets must be full size
	 * except for last one.
	 */
	max_packet = usb_maxpacket(urb->dev, urb->pipe, 0);
	if (max_packet > SLS_FIFO_SIZE) {
		/* We do not support isochronous transfers */
		dev_err(sls_hcd->dev,
			"%s: packet-size of %u too big (limit is %u bytes)",
			__func__, max_packet, SLS_FIFO_SIZE);
		return -EINVAL;
	}

	if (sls_hcd->curr_len && !(sls_hcd->curr_len % max_packet))
		return 0;

	if ((sls_hcd->curr_len % max_packet) < max_packet) {
		if (urb->transfer_flags & URB_SHORT_NOT_OK) {
			/*
			 * remaining > 0 and received an
			 * unexpected partial packet ->
			 * error
			 */
			return -EREMOTEIO;
		} else {
			/* short read, but it's OK */
			return 1;
		}
	}
	return 0; /* not done */
}

static int sls_transfer_out_done(struct usb_hcd *hcd, struct urb *urb)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	urb->actual_length += sls_hcd->curr_len;

	dev_dbg(sls_hcd->dev, "%s: ep%dout %d/%d\n", __func__,
		usb_pipeendpoint(urb->pipe),
		urb->actual_length,
		urb->transfer_buffer_length);

	if (urb->actual_length < urb->transfer_buffer_length)
		return 0;
	if (urb->transfer_flags & URB_ZERO_PACKET) {
		/*
		 * Some hardware needs a zero-size packet at the end
		 * of a bulk-out transfer if the last transfer was a
		 * full-sized packet (i.e., such hardware use <
		 * max_packet as an indicator that the end of the
		 * packet has been reached).
		 */
		u32 max_packet = usb_maxpacket(urb->dev, urb->pipe, 1);

		if (sls_hcd->curr_len && !(sls_hcd->curr_len % max_packet))
			return 0;
	}
	return 1;
}

static void sls_host_transfer_done(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct urb *urb = sls_hcd->curr_urb;
	struct sls_ep *sls_ep = urb->ep->hcpriv;
	u32 *desc = sls_ep->xfer_desc;
	unsigned int toggle;
	int epnum, epout;
	int urb_done = 0;

	if (!urb)
		return;

	epnum = usb_pipeendpoint(urb->pipe);
	epout = usb_pipeout(urb->pipe);

	if (unlikely(SLS_XFER_DESC0_ERROR(desc[0]))) {
		sls_handle_error(hcd, desc[0]);
		usb_settoggle(urb->dev, epnum, epout, 0);
		sls_urb_done(hcd);
		return;
	}

	switch (sls_ep->pkt_state) {
	case PKT_STATE_SETUP:
		if (urb->transfer_buffer_length > 0) {
			sls_ep->pkt_state = PKT_STATE_TRANSFER;
			usb_settoggle(urb->dev, epnum, epout, 1); /* DATA1 */
		} else {
			sls_ep->pkt_state = PKT_STATE_TERMINATE;
		}
		break;

	case PKT_STATE_TRANSFER:
		if (usb_urb_dir_in(urb))
			urb_done = sls_transfer_in_done(hcd, urb);
		else
			urb_done = sls_transfer_out_done(hcd, urb);
		if (urb_done > 0 && usb_pipecontrol(urb->pipe)) {
			/*
			 * We aren't really done - we still need to
			 * terminate the control transfer:
			 */
			sls_hcd->urb_done = urb_done = 0;
			sls_ep->pkt_state = PKT_STATE_TERMINATE;
			break;
		}
		toggle = SLS_XFER_DESC0_TOGGLE(desc[0]);
		usb_settoggle(urb->dev, epnum, epout, toggle);
		break;

	case PKT_STATE_TERMINATE:
		urb_done = 1;
		break;
	}

	if (urb_done) {
		sls_hcd->urb_done = urb_done;
		sls_urb_done(hcd);
	} else {
		sls_next_transfer(hcd);
	}
}

static void sls_detect_conn(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	u16 intr, sts;
	u32 old_port_status, chg;
	unsigned long flags;

	intr = spi_rd16(hcd, SLS_REG_PORT_INTR);
	spi_wr16(hcd, SLS_REG_PORT_INTR, intr);

	sts = spi_rd16(hcd, SLS_REG_PORT_STS);

	dev_dbg(sls_hcd->dev, "%s: intr:0x%04x sts:0x%04x\n", __func__,
		intr, sts);

	spin_lock_irqsave(&sls_hcd->lock, flags);

	old_port_status = sls_hcd->port_status;

	if (intr & SLS_PORT_REMOTE_WAKEUP)
		dev_dbg(sls_hcd->dev, "%s: REMOTE_WAKEUP\n", __func__);

	if (sts & SLS_PORT_POWER)
		dev_dbg(sls_hcd->dev, "%s: POWER\n", __func__);

	if (sts & SLS_PORT_CONNECT) {
		dev_dbg(sls_hcd->dev, "%s: CONNNECT\n", __func__);
		sls_hcd->port_status |=  USB_PORT_STAT_CONNECTION;
	} else {
		sls_hcd->port_status &= ~USB_PORT_STAT_CONNECTION;
	}

	if (sts & SLS_PORT_RESET) {
		dev_dbg(sls_hcd->dev, "%s: RESET\n", __func__);
		sls_hcd->port_status |=  USB_PORT_STAT_RESET;
	} else {
		sls_hcd->port_status &= ~USB_PORT_STAT_RESET;
	}

	if (sts & SLS_PORT_ENABLE) {
		dev_dbg(sls_hcd->dev, "%s: ENABLE\n", __func__);
		sls_hcd->port_status |=  USB_PORT_STAT_ENABLE;
	} else {
		if (sls_hcd->port_status & USB_PORT_STAT_ENABLE)
			set_bit(CHECK_UNLINK, &sls_hcd->todo);

		sls_hcd->port_status &= ~USB_PORT_STAT_ENABLE;
	}

	if (sts & SLS_PORT_SUSPEND)
		dev_dbg(sls_hcd->dev, "%s: SUSPEND\n", __func__);

	if (sts & SLS_PORT_RESUME) {
		dev_dbg(sls_hcd->dev, "%s: RESUME\n", __func__);
		sls_hcd->port_status &= ~USB_PORT_STAT_SUSPEND;
		set_bit(PORT_SUSPEND, &sls_hcd->todo);
	}

	if (!(sts & SLS_PORT_FULL_SPEED)) {
		dev_dbg(sls_hcd->dev, "%s: LOWSPEED\n", __func__);
		sls_hcd->port_status |=  USB_PORT_STAT_LOW_SPEED;
	} else {
		dev_dbg(sls_hcd->dev, "%s: FULLSPEED\n", __func__);
		sls_hcd->port_status &= ~USB_PORT_STAT_LOW_SPEED;
	}

	chg = (old_port_status ^ sls_hcd->port_status);
	sls_hcd->port_status |= chg << 16;

	dev_dbg(sls_hcd->dev, "%s: port_status:0x%08x\n", __func__,
		sls_hcd->port_status);

	spin_unlock_irqrestore(&sls_hcd->lock, flags);

	if (chg)
		usb_hcd_poll_rh_status(hcd);
}

static irqreturn_t sls_irq(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	if (hcd->state == HC_STATE_SUSPENDED)
		pm_wakeup_dev_event(sls_hcd->dev, 2000, true);

	if (atomic_read(&sls_hcd->pm_suspended)) {
		dev_dbg(sls_hcd->dev, "%s: device is suspended\n", __func__);
		goto skip_wakeup_spi_thread;
	}

	if (sls_hcd->spi_thread &&
	    sls_hcd->spi_thread->state != TASK_RUNNING)
		wake_up_process(sls_hcd->spi_thread);

skip_wakeup_spi_thread:
	if (!test_and_set_bit(ENABLE_IRQ, &sls_hcd->todo))
		disable_irq_nosync(hcd->irq);

	return IRQ_HANDLED;
}

#ifdef DEBUG
static void dump_eps(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct sls_ep *sls_ep;
	struct usb_host_endpoint *ep;
	unsigned long flags;
	struct urb *urb;
	int epnum;

	spin_lock_irqsave(&sls_hcd->lock, flags);

	list_for_each_entry(sls_ep, &sls_hcd->ep_list, ep_list) {
		ep = sls_ep->ep;

		epnum = usb_endpoint_num(&ep->desc);
		dev_info(sls_hcd->dev, "%s: EP%0u%s %u\n",
			 __func__,
			 epnum,
			 usb_endpoint_dir_in(&ep->desc) ? "IN " : "OUT",
			 sls_ep->pkt_state);

		list_for_each_entry(urb, &ep->urb_list, urb_list) {
			dev_info(sls_hcd->dev, "%s:  URB %p ep%d%s-%s %d/%d"
				 " status:%d unlinked:%d\n",
				 __func__,
				 urb,
				 epnum,
				 usb_urb_dir_in(urb) ? "in" : "out",
				 usb_pipeisoc(urb->pipe)    ? "isoc" :
				 usb_pipeint(urb->pipe)     ? "intr"  :
				 usb_pipecontrol(urb->pipe) ? "ctrl" : "bulk",
				 urb->actual_length,
				 urb->transfer_buffer_length,
				 urb->status,
				 urb->unlinked);
		}
	}

	spin_unlock_irqrestore(&sls_hcd->lock, flags);
}
#endif

static int sls_handle_irqs(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	u16 hirq;

	if (!sls_hcd->hirq) {
		hirq = spi_rd16(hcd, SLS_REG_INTR_SRC);
		dev_dbg(sls_hcd->dev, "%s: hirq:0x%04x\n", __func__, hirq);
		if (!hirq)
			return 0;
	} else {
		hirq = sls_hcd->hirq;
		sls_hcd->hirq = 0;
	}
	spi_wr16(hcd, SLS_REG_INTR_SRC, hirq);

	if (hirq & SLS_INTR_INT_TD_DONE)
		sls_recv_data_available(hcd, 1);

	if (hirq & SLS_INTR_ATL_TD_DONE)
		sls_recv_data_available(hcd, 0);

	if (hirq & SLS_INTR_PE)
		sls_detect_conn(hcd);

#ifdef DEBUG
	{
		static unsigned long last_time = 0;

		if (time_after(jiffies, last_time + 5*HZ)) {
			last_time = jiffies;
			dump_eps(hcd);
		}
	}
#endif
	return 1;
}

static int sls_hcd_reset(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	int i;

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	/* disable interrupts */
	spi_wr16(hcd, SLS_REG_INTR_EN, 0);

	/* clear configure */
	spi_wr16(hcd, SLS_REG_USBCTRL, 0);

	/* clear all interrupts */
	spi_wr16(hcd, SLS_REG_INTR_SRC, U16_MAX);
	spi_wr16(hcd, SLS_REG_PORT_INTR, U16_MAX);

	/* clear all done map */
	spi_wr16(hcd, SLS_REG_INT_DONE_MAP, SLS_INT_MAP_MASK);
	spi_wr16(hcd, SLS_REG_ATL_DONE_MAP, SLS_ATL_MAP_MASK);

	/* reset map */
	spi_wr16(hcd, SLS_REG_INT_LAST_MAP, BIT(SLS_INT_MAX - 1));
	for (i = 0; i < SLS_INT_MAX; i++)
		sls_hcd->int_urb[i] = NULL;
	sls_hcd->int_skip_map = SLS_INT_MAP_MASK;

	spi_wr16(hcd, SLS_REG_ATL_LAST_MAP, BIT(SLS_ATL_MAX - 1));
	for (i = 0; i < SLS_ATL_MAX; i++)
		sls_hcd->atl_urb[i] = NULL;
	sls_hcd->atl_skip_map = SLS_ATL_MAP_MASK;

	sls_hcd->fifo_map = SLS_FIFO_MAP_MASK;

	/* set configure */
	spi_wr16(hcd, SLS_REG_USBCTRL,
		 SLS_USBCTRL_CONFIG |
		 SLS_USBCTRL_REMOTE_WAKEUP);

	/* enable interrupts */
	spi_wr16(hcd, SLS_REG_INTR_EN,
		 SLS_INTR_INT_TD_DONE |
		 SLS_INTR_ATL_TD_DONE |
		 SLS_INTR_PE);

	return 1;
}

static int sls_urb_done(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct urb *urb;
	int status;
	unsigned long flags;

	status = sls_hcd->urb_done;
	sls_hcd->urb_done = 0;
	if (status > 0)
		status = 0;

	urb = sls_hcd->curr_urb;
	if (urb) {
		dev_dbg(sls_hcd->dev, "%s: URB %p ep%d%s status=%d\n",
			__func__,
			urb,
			usb_pipeendpoint(urb->pipe),
			usb_urb_dir_in(urb) ? "in" : "out",
			status);
		sls_hcd->curr_urb = NULL;
		if (sls_hcd->temp_urb) {
			sls_hcd->curr_urb = sls_hcd->temp_urb;
			sls_hcd->temp_urb = NULL;
		}
		spin_lock_irqsave(&sls_hcd->lock, flags);
		usb_hcd_unlink_urb_from_ep(hcd, urb);
		spin_unlock_irqrestore(&sls_hcd->lock, flags);
		/* must be called without the HCD spinlock */
		urb->status = -EINPROGRESS;
		usb_hcd_giveback_urb(hcd, urb, status);
	}

	return 1;
}

static int sls_spi_thread(void *dev_id)
{
	struct usb_hcd *hcd = dev_id;
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	int i_worked = 1;

repeat:
	if (kthread_should_stop())
		goto exit;

	i_worked |= sls_hcd->todo & ~BIT(ENABLE_IRQ);

	if (!i_worked) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (test_and_clear_bit(ENABLE_IRQ, &sls_hcd->todo))
			enable_irq(hcd->irq);
		schedule();
		__set_current_state(TASK_RUNNING);

		if (kthread_should_stop())
			goto exit;
	}

	i_worked = 0;

	dev_dbg(sls_hcd->dev, "%s: todo:%08x urb_done:%d curr_urb:%p temp_urb:%p\n",
		__func__, sls_hcd->todo, sls_hcd->urb_done,
		sls_hcd->curr_urb, sls_hcd->temp_urb);

	if (sls_hcd->urb_done)
		i_worked = sls_urb_done(hcd);
	else if (test_and_clear_bit(URB_ENQUEUE, &sls_hcd->todo) &&
		 !sls_hcd->curr_urb &&
		 sls_select_and_start_urb(hcd))
		i_worked = 1;
	else if (sls_handle_irqs(hcd))
		i_worked = 1;
	else if (!sls_hcd->curr_urb)
		i_worked = sls_select_and_start_urb(hcd);

	if (test_and_clear_bit(HCD_RESET, &sls_hcd->todo))
		i_worked |= sls_hcd_reset(hcd);

	if (test_and_clear_bit(PORT_POWER, &sls_hcd->todo)) {
		spi_wr16(hcd, SLS_REG_PORT_CMD,
			 (sls_hcd->port_status & USB_PORT_STAT_POWER) ?
			 SLS_PORT_CMD_POWER_ON : SLS_PORT_CMD_POWER_OFF);
		i_worked = 1;
	}

	if (test_and_clear_bit(PORT_RESET, &sls_hcd->todo)) {
		spi_wr16(hcd, SLS_REG_PORT_CMD, SLS_PORT_CMD_DISABLE);
		spi_wr16(hcd, SLS_REG_PORT_CMD, SLS_PORT_CMD_RESET);
		/* T_DRST 10-20ms */
		//msleep(15);
		spi_wr16(hcd, SLS_REG_PORT_CMD, SLS_PORT_CMD_RESET_DONE);
		sls_hcd->port_status &= ~USB_PORT_STAT_RESET;
		i_worked = 1;
	}

	if (test_and_clear_bit(PORT_SUSPEND, &sls_hcd->todo)) {
		if (sls_hcd->port_status & USB_PORT_STAT_SUSPEND) {
			spi_wr16(hcd, SLS_REG_PORT_CMD, SLS_PORT_CMD_SUSPEND);
		} else {
			spi_wr16(hcd, SLS_REG_PORT_CMD, SLS_PORT_CMD_RESUME);
			/* T_DRSMDN 20ms */
			//msleep(20);
			spi_wr16(hcd, SLS_REG_PORT_CMD, SLS_PORT_CMD_RESUME_DONE);
		}
		i_worked = 1;
	}

	if (test_and_clear_bit(CHECK_UNLINK, &sls_hcd->todo))
		i_worked |= sls_check_unlink(hcd);

	goto repeat;

exit:
	set_current_state(TASK_RUNNING);
	dev_dbg(sls_hcd->dev, "SPI thread exiting");
	return 0;
}

static int sls_port_power(struct usb_hcd *hcd, bool on)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s: %d\n", __func__, on);

	if (on)
		sls_hcd->port_status |=  USB_PORT_STAT_POWER;
	else
		sls_hcd->port_status &= ~USB_PORT_STAT_POWER;
	set_bit(PORT_POWER, &sls_hcd->todo);
	wake_up_process(sls_hcd->spi_thread);

	return 0;
}

static int sls_port_reset(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	sls_hcd->port_status &= ~(USB_PORT_STAT_ENABLE |
				  USB_PORT_STAT_LOW_SPEED);
	sls_hcd->port_status |=  USB_PORT_STAT_RESET;
	set_bit(PORT_RESET, &sls_hcd->todo);
	wake_up_process(sls_hcd->spi_thread);

	return 0;
}

static int sls_port_suspend(struct usb_hcd *hcd, bool on)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s: %d\n", __func__, on);

	if (on)
		sls_hcd->port_status |=  USB_PORT_STAT_SUSPEND;
	else
		sls_hcd->port_status &= ~USB_PORT_STAT_SUSPEND;
	set_bit(PORT_SUSPEND, &sls_hcd->todo);
	wake_up_process(sls_hcd->spi_thread);

	return 0;
}

static int sls_reset(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	hcd->self.sg_tablesize = 0;
	hcd->speed = HCD_USB11;
	hcd->self.root_hub->speed = USB_SPEED_FULL;
	set_bit(HCD_RESET, &sls_hcd->todo);
	wake_up_process(sls_hcd->spi_thread);

	return 0;
}

static int sls_start(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	spin_lock_init(&sls_hcd->lock);

	INIT_LIST_HEAD(&sls_hcd->ep_list);

	hcd->state = HC_STATE_RUNNING;
	hcd->uses_new_polling = 1;

	return 0;
}

static void sls_stop(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s\n", __func__);
}

static int sls_get_frame_number(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	u16 frame_number;

	frame_number = spi_rd16(hcd, SLS_REG_FRINDEX);
	dev_dbg(sls_hcd->dev, "%s: frame_number:%d\n", __func__, frame_number);

	return frame_number;
}

static int sls_urb_enqueue(struct usb_hcd *hcd,
		struct urb *urb, gfp_t mem_flags)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	struct sls_ep *sls_ep;
	unsigned long flags;
	int ret;

	if (!(sls_hcd->port_status & USB_PORT_STAT_ENABLE)) {
		dev_dbg(sls_hcd->dev, "%s: Can't queue urb, port error, link inactive\n",
			__func__);
		return -ENODEV;
	}

	dev_dbg(sls_hcd->dev, "%s: URB %p ep%d%s-%s %d\n", __func__,
		urb,
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb) ? "in" : "out",
		usb_pipeisoc(urb->pipe)    ? "isoc" :
		usb_pipeint(urb->pipe)     ? "intr"  :
		usb_pipecontrol(urb->pipe) ? "ctrl" : "bulk",
		urb->transfer_buffer_length);

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_INTERRUPT:
	case PIPE_ISOCHRONOUS:
		if (urb->interval < 0) {
			dev_err(sls_hcd->dev,
				"%s: interval=%d for intr-/iso-pipe; expected > 0\n",
				__func__, urb->interval);
			return -EINVAL;
		}
		break;

	default:
		break;
	}

	spin_lock_irqsave(&sls_hcd->lock, flags);

	sls_ep = urb->ep->hcpriv;
	if (!sls_ep) {
		/* gets freed in sls_endpoint_disable */
		sls_ep = kzalloc(sizeof(struct sls_ep), GFP_ATOMIC);
		if (!sls_ep) {
			ret = -ENOMEM;
			goto out;
		}
		sls_ep->ep = urb->ep;
		sls_ep->dev = urb->dev;
		urb->ep->hcpriv = sls_ep;

		list_add_tail(&sls_ep->ep_list, &sls_hcd->ep_list);
	}

	ret = usb_hcd_link_urb_to_ep(hcd, urb);
	if (!ret) {
		/* Since we added to the queue, restart scheduling */
		sls_hcd->sched_pass = SCHED_PASS_PERIODIC;
		set_bit(URB_ENQUEUE, &sls_hcd->todo);
		wake_up_process(sls_hcd->spi_thread);
	}

out:
	spin_unlock_irqrestore(&sls_hcd->lock, flags);

	return ret;
}

static int sls_urb_dequeue(struct usb_hcd *hcd,
		struct urb *urb, int status)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	unsigned long flags;
	int ret;

	dev_dbg(sls_hcd->dev, "%s: URB %p ep%d%s-%s status=%d\n", __func__,
		urb,
		usb_pipeendpoint(urb->pipe),
		usb_urb_dir_in(urb) ? "in" : "out",
		usb_pipeisoc(urb->pipe)    ? "isoc" :
		usb_pipeint(urb->pipe)     ? "intr"  :
		usb_pipecontrol(urb->pipe) ? "ctrl" : "bulk",
		status);

	spin_lock_irqsave(&sls_hcd->lock, flags);

	/*
	 * This will set urb->unlinked which in turn causes the entry
	 * to be dropped at the next opportunity.
	 */
	ret = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (!ret) {
		set_bit(CHECK_UNLINK, &sls_hcd->todo);
		wake_up_process(sls_hcd->spi_thread);
	}

	spin_unlock_irqrestore(&sls_hcd->lock, flags);

	return ret;
}

static void sls_endpoint_disable(struct usb_hcd *hcd,
		struct usb_host_endpoint *ep)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	unsigned long flags;

	dev_dbg(sls_hcd->dev, "%s: ep%d%s\n", __func__,
		usb_endpoint_num(&ep->desc),
		usb_endpoint_dir_in(&ep->desc) ? "in" : "out");

	spin_lock_irqsave(&sls_hcd->lock, flags);

	if (ep->hcpriv) {
		struct sls_ep *sls_ep = ep->hcpriv;

		/* remove myself from the ep_list */
		if (!list_empty(&sls_ep->ep_list))
			list_del(&sls_ep->ep_list);
		kfree(sls_ep);
		ep->hcpriv = NULL;
	}

	spin_unlock_irqrestore(&sls_hcd->lock, flags);
}

static int sls_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&sls_hcd->lock, flags);

	if (!HCD_HW_ACCESSIBLE(hcd)) {
		dev_dbg(sls_hcd->dev, "hw accessible flag not on?\n");
		goto done;
	}

	*buf = 0;
	if (sls_hcd->port_status & PORT_C_MASK) {
		*buf = BIT(1); /* a hub over-current condition exists */
		dev_dbg(sls_hcd->dev, "%s: port_status:0x%08x\n", __func__,
			sls_hcd->port_status);
		ret = 1;
		if (hcd->state == HC_STATE_SUSPENDED)
			usb_hcd_resume_root_hub(hcd);
	}
done:
	spin_unlock_irqrestore(&sls_hcd->lock, flags);

	return ret;
}

static inline void hub_descriptor(struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof(*desc));
	desc->bDescriptorType = USB_DT_HUB;
	desc->bDescLength = 9;
	desc->bNbrPorts = 1;
	desc->wHubCharacteristics = cpu_to_le16(HUB_CHAR_INDV_PORT_LPSM |
						HUB_CHAR_NO_LPSM);
	desc->bPwrOn2PwrGood = 10;
}

#ifdef DEBUG
static void print_port_feature(struct usb_hcd *hcd, u32 value)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	if (value & 0x0000FFFF) {
		if (value & BIT(USB_PORT_FEAT_CONNECTION))
			dev_dbg(sls_hcd->dev, "  CONNECT\n");
		if (value & BIT(USB_PORT_FEAT_ENABLE))
			dev_dbg(sls_hcd->dev, "  ENABLE\n");
		if (value & BIT(USB_PORT_FEAT_SUSPEND))
			dev_dbg(sls_hcd->dev, "  SUSPEND\n");
		if (value & BIT(USB_PORT_FEAT_OVER_CURRENT))
			dev_dbg(sls_hcd->dev, "  OVER_CURRENT\n");
		if (value & BIT(USB_PORT_FEAT_RESET))
			dev_dbg(sls_hcd->dev, "  RESET\n");
		if (value & BIT(USB_PORT_FEAT_L1))
			dev_dbg(sls_hcd->dev, "  L1\n");
		if (value & BIT(USB_PORT_FEAT_POWER))
			dev_dbg(sls_hcd->dev, "  POWER\n");
		if (value & BIT(USB_PORT_FEAT_LOWSPEED))
			dev_dbg(sls_hcd->dev, "  LOWSPEED\n");
	}
	if (value & 0xFFFF0000) {
		if (value & BIT(USB_PORT_FEAT_C_CONNECTION))
			dev_dbg(sls_hcd->dev, "  C_CONNECT\n");
		if (value & BIT(USB_PORT_FEAT_C_ENABLE))
			dev_dbg(sls_hcd->dev, "  C_ENABLE\n");
		if (value & BIT(USB_PORT_FEAT_C_SUSPEND))
			dev_dbg(sls_hcd->dev, "  C_SUSPEND\n");
		if (value & BIT(USB_PORT_FEAT_C_OVER_CURRENT))
			dev_dbg(sls_hcd->dev, "  C_OVER_CURRENT\n");
		if (value & BIT(USB_PORT_FEAT_C_RESET))
			dev_dbg(sls_hcd->dev, "  C_RESET\n");
		if (value & BIT(USB_PORT_FEAT_TEST))
			dev_dbg(sls_hcd->dev, "  TEST\n");
		if (value & BIT(USB_PORT_FEAT_INDICATOR))
			dev_dbg(sls_hcd->dev, "  INDICATOR\n");
		if (value & BIT(USB_PORT_FEAT_C_PORT_L1))
			dev_dbg(sls_hcd->dev, "  C_PORT_L1\n");
	}
}
#endif

static int sls_hub_control(struct usb_hcd *hcd, u16 type_req, u16 value,
		u16 index, char *buf, u16 length)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);
	unsigned long flags;
	int ret = 0;

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	spin_lock_irqsave(&sls_hcd->lock, flags);

	switch (type_req) {
	case ClearHubFeature:
		dev_dbg(sls_hcd->dev, "ClearHubFeature\n");
		break;

	case ClearPortFeature:
		dev_dbg(sls_hcd->dev, "ClearPortFeature v%04x\n", value);
#ifdef DEBUG
		print_port_feature(hcd, BIT(value));
#endif
		switch (value) {
		case USB_PORT_FEAT_SUSPEND:
			sls_port_suspend(hcd, false);
			break;
		case USB_PORT_FEAT_POWER:
			sls_port_power(hcd, false);
			break;
		default:
			sls_hcd->port_status &= ~BIT(value);
		}
		break;

	case GetHubDescriptor:
		dev_dbg(sls_hcd->dev, "GetHubDescriptor\n");
		hub_descriptor((struct usb_hub_descriptor *)buf);
		break;

	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
	case GetPortErrorCount:
	case SetHubDepth:
		/* USB3 only */
		goto err;
		break;

	case GetHubStatus:
		dev_dbg(sls_hcd->dev, "GetHubStatus\n");
		*(__le32 *)buf = cpu_to_le32(0);
		break;

	case GetPortStatus:
		dev_dbg(sls_hcd->dev, "GetPortStatus %d\n", index);
		if (index != 1) {
			ret = -EPIPE;
			goto err;
		}
#ifdef DEBUG
		print_port_feature(hcd, sls_hcd->port_status);
#endif

		((__le16 *)buf)[0] = cpu_to_le16(sls_hcd->port_status);
		((__le16 *)buf)[1] = cpu_to_le16(sls_hcd->port_status >> 16);
		break;

	case SetHubFeature:
		dev_dbg(sls_hcd->dev, "SetHubFeature\n");
		ret = -EPIPE;
		break;

	case SetPortFeature:
		dev_dbg(sls_hcd->dev, "SetPortFeature v%04x\n", value);
#ifdef DEBUG
		print_port_feature(hcd, BIT(value));
#endif
		switch (value) {
		case USB_PORT_FEAT_LINK_STATE:
		case USB_PORT_FEAT_U1_TIMEOUT:
		case USB_PORT_FEAT_U2_TIMEOUT:
		case USB_PORT_FEAT_BH_PORT_RESET:
			goto err;
			break;
		case USB_PORT_FEAT_SUSPEND:
			sls_port_suspend(hcd, true);
			break;
		case USB_PORT_FEAT_POWER:
			sls_port_power(hcd, true);
			break;
		case USB_PORT_FEAT_RESET:
			sls_port_reset(hcd);
			break;
		default:
			if ((sls_hcd->port_status & USB_PORT_STAT_POWER) != 0)
				sls_hcd->port_status |= BIT(value);
			break;
		}
		break;

	default:
		dev_dbg(sls_hcd->dev, "hub control req%04x v%04x i%04x l%d\n",
			type_req, value, index, length);
err:
		/* "protocol stall" on error */
		ret = -EPIPE;
	}

	spin_unlock_irqrestore(&sls_hcd->lock, flags);

	return ret;
}

static int sls_bus_suspend(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	pm_relax(sls_hcd->dev);

	return 0;
}

static int sls_bus_resume(struct usb_hcd *hcd)
{
	struct sls_hcd *sls_hcd = hcd_to_sls(hcd);

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	pm_stay_awake(sls_hcd->dev);

	return 0;
}

static const struct hc_driver sls_hcd_desc = {
	.description =		"sls-hcd",
	.product_desc =		"SLS Host Controller",
	.hcd_priv_size =        sizeof(struct sls_hcd),
	.flags =                HCD_USB11,
	.irq =			sls_irq,
	.reset =                sls_reset,
	.start =                sls_start,
	.stop =                 sls_stop,
	.get_frame_number =     sls_get_frame_number,
	.urb_enqueue =          sls_urb_enqueue,
	.urb_dequeue =          sls_urb_dequeue,
	.endpoint_disable =     sls_endpoint_disable,
	.hub_status_data =      sls_hub_status_data,
	.hub_control =          sls_hub_control,
	.bus_suspend =          sls_bus_suspend,
	.bus_resume =           sls_bus_resume,
};

static int sls_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_device *spi;
	struct sls_hcd *sls_hcd;
	struct usb_hcd *hcd;
	u16 ver;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	int ret = 0;

	dev_info(dev, "%s\n", __func__);

	spi = to_spi_device(dev->parent);

	if (!spi->irq) {
		dev_err(dev, "failed get SPI IRQ\n");
		return -EFAULT;
	}

	hcd = usb_create_hcd(&sls_hcd_desc, dev, dev_name(dev));
	if (!hcd) {
		dev_err(dev, "failed to create HCD structure\n");
		return -ENOMEM;
	}

	hcd_to_bus(hcd)->skip_resume = true;

	sls_hcd = hcd_to_sls(hcd);
	platform_set_drvdata(pdev, sls_hcd);
	sls_hcd->dev = dev;
	sls_hcd->spi = spi;
	INIT_LIST_HEAD(&sls_hcd->ep_list);

	ver = spi_rd16(hcd, SLS_REG_VERSION);
	if (ver != SLS_IP_VERSION) {
		dev_err(sls_hcd->dev, "bad ver 0x%04x\n", ver);
		ret = -EIO;
		goto err;
	}
	dev_info(dev, "ver 0x%04x, SPI clk %dHz, bpw %u, irq %d\n",
		 ver, spi->max_speed_hz, spi->bits_per_word, spi->irq);

	sls_hcd->spi_thread = kthread_run(sls_spi_thread, hcd,
					  "sls_spi_thread");
	if (sls_hcd->spi_thread == ERR_PTR(-ENOMEM)) {
		dev_err(dev, "failed to create SPI thread (out of memory)\n");
		ret = -ENOMEM;
		goto err;
	}
	sched_setscheduler(sls_hcd->spi_thread, SCHED_FIFO, &param);
	set_user_nice(sls_hcd->spi_thread, MIN_NICE);

	device_init_wakeup(dev, 1);
	irq_set_status_flags(spi->irq, IRQ_NOAUTOEN);

	ret = usb_add_hcd(hcd, spi->irq, 0);
	if (ret) {
		dev_err(dev, "failed to add HCD\n");
		goto err;
	}

	enable_irq_wake(hcd->irq);
	enable_irq(hcd->irq);

	return 0;

err:
	if (hcd) {
		if (sls_hcd->spi_thread)
			kthread_stop(sls_hcd->spi_thread);
		if (device_can_wakeup(dev))
		    device_init_wakeup(dev, 0);
		usb_put_hcd(hcd);
	}
	return ret;
}

static int sls_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sls_hcd *sls_hcd = dev_get_drvdata(dev);
	struct usb_hcd *hcd = sls_to_hcd(sls_hcd);

	dev_info(dev, "%s\n", __func__);

	device_init_wakeup(dev, 0);

	usb_remove_hcd(hcd);
	kthread_stop(sls_hcd->spi_thread);
	usb_put_hcd(hcd);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sls_pm_suspend(struct device *dev)
{
	struct sls_hcd *sls_hcd = dev_get_drvdata(dev);
	struct usb_hcd *hcd = sls_to_hcd(sls_hcd);

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	if (hcd->state != HC_STATE_SUSPENDED)
		return -EINVAL;

	if (sls_hcd->spi_thread &&
	    sls_hcd->spi_thread->state == TASK_RUNNING)
		return -EBUSY;

	atomic_set(&sls_hcd->pm_suspended, 1);

	return 0;
}

static int sls_pm_resume(struct device *dev)
{
	struct sls_hcd *sls_hcd = dev_get_drvdata(dev);

	dev_dbg(sls_hcd->dev, "%s\n", __func__);

	atomic_set(&sls_hcd->pm_suspended, 0);

	if (test_bit(ENABLE_IRQ, &sls_hcd->todo) &&
	    sls_hcd->spi_thread &&
	    sls_hcd->spi_thread->state != TASK_RUNNING)
		wake_up_process(sls_hcd->spi_thread);

	return 0;
}
#endif

static const struct dev_pm_ops sls_dev_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(sls_pm_suspend, sls_pm_resume)
};

static struct platform_driver sls_hcd_driver = {
	.probe		= sls_probe,
	.remove		= sls_remove,
	.driver		= {
		.name	= "sls-hcd",
		.pm	= &sls_dev_pm_ops,
		.owner	= THIS_MODULE,
	},
};

static int sls_init(void)
{
	int ret;

	pr_debug("%s\n", __func__);

	dma_tx_buf = kmalloc(sizeof(*dma_tx_buf), GFP_KERNEL);
	if (!dma_tx_buf) {
		pr_err("%s: failed to alloc tx buffer\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	dma_rx_buf = kmalloc(sizeof(*dma_rx_buf), GFP_KERNEL);
	if (!dma_rx_buf) {
		pr_err("%s: failed to alloc rx buffer\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	ret = platform_driver_register(&sls_hcd_driver);
	if (ret)
		goto err;

	return 0;

err:
	if (dma_tx_buf) {
		kfree(dma_tx_buf);
		dma_tx_buf = NULL;
	}
	if (dma_rx_buf) {
		kfree(dma_rx_buf);
		dma_rx_buf = NULL;
	}
	return ret;
}

static void sls_exit(void)
{
	pr_debug("%s\n", __func__);

	kfree(dma_tx_buf);
	dma_tx_buf = NULL;
	kfree(dma_rx_buf);
	dma_rx_buf = NULL;

	platform_driver_unregister(&sls_hcd_driver);
}

MODULE_AUTHOR("Hansun Lee <hansun.lee@lge.com>");
MODULE_DESCRIPTION("SLS USB1.1 Host Controller Driver");
MODULE_LICENSE("GPL v2");

module_init(sls_init);
module_exit(sls_exit);
