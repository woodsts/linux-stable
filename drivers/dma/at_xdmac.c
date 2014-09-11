/*
 * Driver for the Atmel Extensible DMA Controller (aka XDMAC on AT91 systems)
 *
 * Copyright (C) 2014 Atmel Corporation
 *
 * Author: Ludovic Desroches <ludovic.desroches@atmel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <dt-bindings/dma/at91.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "dmaengine.h"
#include "at_xdmac.h"

static unsigned int init_nr_desc_per_channel = 64;
module_param(init_nr_desc_per_channel, uint, 0644);
MODULE_PARM_DESC(init_nr_desc_per_channel,
		 "initial descriptors per channel (default: 64)");


static bool at_xdmac_chan_is_enabled(struct at_xdmac_chan *atchan)
{
	return at_xdmac_chan_read(atchan, AT_XDMAC_GS) & atchan->mask;
}

static void at_xdmac_off(struct at_xdmac *atxdmac)
{
	at_xdmac_write(atxdmac, AT_XDMAC_GD, -1L);

	/* Wait that all chans are disabled. */
	while (at_xdmac_read(atxdmac, AT_XDMAC_GS))
		cpu_relax();

	at_xdmac_write(atxdmac, AT_XDMAC_GID, -1L);
}

/* Call with lock hold. */
static void at_xdmac_start_xfer(struct at_xdmac_chan *atchan,
				struct at_xdmac_desc *first)
{
	struct at_xdmac	*atxdmac = to_at_xdmac(atchan->chan.device);
	u32		reg;

	dev_vdbg(chan2dev(&atchan->chan), "%s: desc 0x%p\n", __func__, first);

	if (at_xdmac_chan_is_enabled(atchan)) {
		dev_err(chan2dev(&atchan->chan),
			"BUG: Attempted to start a non-idle channel\n");
		return;
	}

	/* Set transfer as active to not try to start it again. */
	first->active_xfer = true;

	/* Tell xdmac where to get the first descriptor. */
	reg = AT_XDMAC_CNDA_NDA(first->tx_dma_desc.phys)
	      | AT_XDMAC_CNDA_NDAIF(atchan->memif);
	at_xdmac_chan_write(atchan, AT_XDMAC_CNDA, reg);

	/*
	 * When doing memory to memory transfer we need to use the next
	 * descriptor view 2 since some fields of the configuration register
	 * depend on transfer size and src/dest addresses.
	 */
	if (atchan->cfg & AT_XDMAC_CC_TYPE_PER_TRAN) {
		reg = AT_XDMAC_CNDC_NDVIEW_NDV1;
		at_xdmac_chan_write(atchan, AT_XDMAC_CC, atchan->cfg);
	} else
		reg = AT_XDMAC_CNDC_NDVIEW_NDV2;

	reg |= AT_XDMAC_CNDC_NDDUP
	       | AT_XDMAC_CNDC_NDSUP
	       | AT_XDMAC_CNDC_NDE;
	at_xdmac_chan_write(atchan, AT_XDMAC_CNDC, reg);

	dev_vdbg(chan2dev(&atchan->chan),
		 "%s: XDMAC_CC=0x%08x XDMAC_CNDA=0x%08x, XDMAC_CNDC=0x%08x, "
		 "XDMAC_CSA=0x%08x, XDMAC_CDA=0x%08x, XDMAC_CUBC=0x%08x\n",
		 __func__, at_xdmac_chan_read(atchan, AT_XDMAC_CC),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CNDA),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CNDC),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CSA),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CDA),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CUBC));

	/*
	 * There is no end of list when doing cyclic dma, we need to get
	 * an interrupt after each periods.
	 */
	if (at_xdmac_chan_is_cyclic(atchan))
		at_xdmac_chan_write(atchan, AT_XDMAC_CIE, AT_XDMAC_CIE_BIE);
	else
		at_xdmac_chan_write(atchan, AT_XDMAC_CIE, AT_XDMAC_CIE_LIE);
	at_xdmac_write(atxdmac, AT_XDMAC_GIE, atchan->mask);
	dev_vdbg(chan2dev(&atchan->chan),
		 "%s: enable channel (0x%08x)\n", __func__, atchan->mask);
	at_xdmac_write(atxdmac, AT_XDMAC_GE, atchan->mask);

	dev_vdbg(chan2dev(&atchan->chan),
		 "%s: XDMAC_CC=0x%08x XDMAC_CNDA=0x%08x, XDMAC_CNDC=0x%08x, "
		 "XDMAC_CSA=0x%08x, XDMAC_CDA=0x%08x, XDMAC_CUBC=0x%08x\n",
		 __func__, at_xdmac_chan_read(atchan, AT_XDMAC_CC),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CNDA),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CNDC),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CSA),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CDA),
		 at_xdmac_chan_read(atchan, AT_XDMAC_CUBC));

}

static dma_cookie_t at_xdmac_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct at_xdmac_desc	*desc = txd_to_at_desc(tx);
	struct at_xdmac_chan	*atchan = to_at_xdmac_chan(tx->chan);
	dma_cookie_t		cookie;
	unsigned long		flags;

	spin_lock_irqsave(&atchan->lock, flags);
	cookie = dma_cookie_assign(tx);

	dev_vdbg(chan2dev(tx->chan), "%s: atchan 0x%p, add desc 0x%p to xfers_list\n",
		 __func__, atchan, desc);
	list_add_tail(&desc->xfer_node, &atchan->xfers_list);
	if (list_is_singular(&atchan->xfers_list))
		at_xdmac_start_xfer(atchan, desc);

	spin_unlock_irqrestore(&atchan->lock, flags);
	return cookie;
}

static struct at_xdmac_desc *at_xdmac_alloc_desc(struct dma_chan *chan,
						 gfp_t gfp_flags)
{
	struct at_xdmac_desc	*desc;
	struct at_xdmac		*atxdmac = to_at_xdmac(chan->device);
	dma_addr_t		phys;

	desc = dma_pool_alloc(atxdmac->at_xdmac_desc_pool, gfp_flags, &phys);
	if (desc) {
		memset(desc, 0, sizeof(*desc));
		INIT_LIST_HEAD(&desc->descs_list);
		dma_async_tx_descriptor_init(&desc->tx_dma_desc, chan);
		desc->tx_dma_desc.tx_submit = at_xdmac_tx_submit;
		desc->tx_dma_desc.phys = phys;
	}

	return desc;
}

/* Call must be protected by lock. */
static struct at_xdmac_desc *at_xdmac_get_desc(struct at_xdmac_chan *atchan)
{
	struct at_xdmac_desc *desc;

	if (list_empty(&atchan->free_descs_list)) {
		desc = at_xdmac_alloc_desc(&atchan->chan, GFP_ATOMIC);
	} else {
		desc = list_first_entry(&atchan->free_descs_list,
					struct at_xdmac_desc, desc_node);
		list_del(&desc->desc_node);
	}

	return desc;
}

static bool at_xdmac_filter(struct dma_chan *chan, void *slave)
{
	return chan->device->dev == slave;
}

static struct dma_chan *at_xdmac_xlate(struct of_phandle_args *dma_spec,
				       struct of_dma *of_dma)
{
	struct at_xdmac		*atxdmac = of_dma->of_dma_data;
	struct at_xdmac_chan	*atchan;
	struct dma_chan		*chan;
	struct device		*dev = atxdmac->dma.dev;
	dma_cap_mask_t 		mask;

	if (dma_spec->args_count != 2) {
		dev_err(dev, "dma phandler args: bad number of args\n");
		return NULL;
	}

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	chan = dma_request_channel(mask, at_xdmac_filter, dev);
	if (!chan) {
		dev_err(dev, "can't get a dma channel\n");
		return NULL;
	}

	atchan = to_at_xdmac_chan(chan);
	atchan->memif = AT91_XDMAC_DT_GET_MEM_IF(dma_spec->args[0]);
	atchan->perif = AT91_XDMAC_DT_GET_PER_IF(dma_spec->args[0]);
	atchan->perid = AT91_XDMAC_DT_GET_PERID(dma_spec->args[1]);
	dev_info(dev, "chan dt cfg: memif=%u perif=%u perid=%u\n",
		 atchan->memif, atchan->perif, atchan->perid);

	return chan;
}

static int at_xdmac_set_slave_config(struct dma_chan *chan,
				      struct dma_slave_config *sconfig)
{
	struct at_xdmac_chan 	*atchan = to_at_xdmac_chan(chan);

	atchan->cfg = AT91_XDMAC_DT_PERID(atchan->perid)
		      |	AT_XDMAC_CC_SWREQ_HWR_CONNECTED
		      | AT_XDMAC_CC_MBSIZE_SIXTEEN
		      | AT_XDMAC_CC_TYPE_PER_TRAN;

	if (sconfig->direction == DMA_DEV_TO_MEM) {
		atchan->cfg |= AT_XDMAC_CC_DAM_INCREMENTED_AM
			       | AT_XDMAC_CC_SAM_FIXED_AM
			       | AT_XDMAC_CC_DIF(atchan->memif)
			       | AT_XDMAC_CC_SIF(atchan->perif)
			       | AT_XDMAC_CC_DSYNC_PER2MEM;
		atchan->dwidth = ffs(sconfig->src_addr_width) - 1;
		atchan->cfg |= AT_XDMAC_CC_DWIDTH(atchan->dwidth);
		atchan->cfg |= at_xdmac_csize(sconfig->src_maxburst);
	} else if (sconfig->direction == DMA_MEM_TO_DEV) {
		atchan->cfg |= AT_XDMAC_CC_DAM_FIXED_AM
			       | AT_XDMAC_CC_SAM_INCREMENTED_AM
			       | AT_XDMAC_CC_DIF(atchan->perif)
			       | AT_XDMAC_CC_SIF(atchan->memif)
			       | AT_XDMAC_CC_DSYNC_MEM2PER;
		atchan->dwidth = ffs(sconfig->dst_addr_width) - 1;
		atchan->cfg |= AT_XDMAC_CC_DWIDTH(atchan->dwidth);
		atchan->cfg |= at_xdmac_csize(sconfig->dst_maxburst);
	} else
		return -EINVAL;

	/*
	 * Src address and dest addr are needed to configure the link list
	 * descriptor so keep the slave configuration.
	 */
	memcpy(&atchan->dma_sconfig, sconfig, sizeof(struct dma_slave_config));

	dev_dbg(chan2dev(chan), "%s: atchan->cfg=0x%08x\n", __func__, atchan->cfg);

	return 0;
}

static struct dma_async_tx_descriptor *
at_xdmac_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		       unsigned int sg_len, enum dma_transfer_direction direction,
		       unsigned long flags, void *context)
{
	struct at_xdmac_chan 	*atchan = to_at_xdmac_chan(chan);
	struct dma_slave_config	*sconfig = &atchan->dma_sconfig;
	struct at_xdmac_desc	*first = NULL, *prev = NULL;
	struct scatterlist	*sg;
	int 			i;

	if (!sgl)
		return NULL;

	if (!is_slave_direction(direction)) {
		dev_err(chan2dev(chan), "invalid DMA direction\n");
		return NULL;
	}

	dev_dbg(chan2dev(chan), "%s: sg_len=%d, dir=%s, flags=0x%lx\n",
		 __func__, sg_len,
		 direction == DMA_MEM_TO_DEV ? "to device" : "from device",
		 flags);

	/* Protect dma_sconfig field that can be modified by set_slave_conf. */
	spin_lock(&atchan->lock);

	/* Prepare descriptors. */
	for_each_sg(sgl, sg, sg_len, i) {
		struct at_xdmac_desc	*desc = NULL;
		u32			len, mem;

		len = sg_dma_len(sg);
		mem = sg_dma_address(sg);
		if (unlikely(!len)) {
			dev_err(chan2dev(chan), "sg data length is zero\n");
			spin_unlock(&atchan->lock);
			return NULL;
		}
		dev_dbg(chan2dev(chan), "%s: * sg%d len=%u, mem=0x%08x\n",
			 __func__, i, len, mem);

		desc = at_xdmac_get_desc(atchan);
		if (!desc) {
			dev_err(chan2dev(chan), "can't get descriptor\n");
			if (first)
				list_splice_init(&first->descs_list, &atchan->free_descs_list);
			spin_unlock(&atchan->lock);
			return NULL;
		}

		/* Linked list descriptor setup. */
		if (direction == DMA_DEV_TO_MEM) {
			desc->lld.mbr_sa = sconfig->src_addr;
			desc->lld.mbr_da = mem;
		} else {
			desc->lld.mbr_sa = mem;
			desc->lld.mbr_da = sconfig->dst_addr;
		}
		desc->lld.mbr_ubc = AT_XDMAC_MBR_UBC_NDV1		/* next descriptor view */
			| AT_XDMAC_MBR_UBC_NDEN				/* next descriptor dst parameter update */
			| AT_XDMAC_MBR_UBC_NSEN				/* next descriptor src parameter update */
			| (i == sg_len - 1 ? 0 : AT_XDMAC_MBR_UBC_NDE)	/* descriptor fetch */
			| len / (1 << atchan->dwidth);			/* microblock length */
		dev_dbg(chan2dev(chan),
			 "%s: lld: mbr_sa=0x%08x, mbr_da=0x%08x, mbr_ubc=0x%08x\n",
			 __func__, desc->lld.mbr_sa, desc->lld.mbr_da, desc->lld.mbr_ubc);

		/* Chain lld. */
		if (prev) {
			prev->lld.mbr_nda = desc->tx_dma_desc.phys;
			dev_dbg(chan2dev(chan),
				 "%s: chain lld: prev=0x%p, mbr_nda=0x%08x\n",
				 __func__, prev, prev->lld.mbr_nda);
		}

		prev = desc;
		if (!first)
			first = desc;

		dev_dbg(chan2dev(chan), "%s: add desc 0x%p to descs_list 0x%p\n",
			 __func__, desc, first);
		list_add_tail(&desc->desc_node, &first->descs_list);
	}

	spin_unlock(&atchan->lock);

	first->tx_dma_desc.cookie = -EBUSY;
	first->tx_dma_desc.flags = flags;
	first->xfer_size = sg_len;

	return &first->tx_dma_desc;
}

static struct dma_async_tx_descriptor *
at_xdmac_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t buf_addr,
			 size_t buf_len, size_t period_len,
			 enum dma_transfer_direction direction,
			 unsigned long flags, void *context)
{
	struct at_xdmac_chan 	*atchan = to_at_xdmac_chan(chan);
	struct dma_slave_config	*sconfig = &atchan->dma_sconfig;
	struct at_xdmac_desc	*first = NULL, *prev = NULL;
	unsigned int		periods = buf_len / period_len;
	unsigned long		lock_flags;
	int 			i;

	dev_dbg(chan2dev(chan), "%s: buf_addr=0x%08x, buf_len=%d, period_len=%d, "
		"dir=%s, flags=0x%lx\n",
		__func__, buf_addr, buf_len, period_len,
		direction == DMA_MEM_TO_DEV ? "mem2per" : "per2mem", flags);

	if (!is_slave_direction(direction)) {
		dev_err(chan2dev(chan), "invalid DMA direction\n");
		return NULL;
	}

	if (test_and_set_bit(AT_XDMAC_CHAN_IS_CYCLIC, &atchan->status)) {
		dev_err(chan2dev(chan), "channel currently used\n");
		return NULL;
	}

	for (i = 0; i < periods; i++) {
		struct at_xdmac_desc	*desc = NULL;

		spin_lock_irqsave(&atchan->lock, lock_flags);
		desc = at_xdmac_get_desc(atchan);
		spin_unlock_irqrestore(&atchan->lock, lock_flags);
		if (!desc) {
			dev_err(chan2dev(chan), "can't get descriptor\n");
			if (first)
				list_splice_init(&first->descs_list, &atchan->free_descs_list);
			return NULL;
		}
		dev_dbg(chan2dev(chan),
			"%s: desc=0x%p, tx_dma_desc.phys=0x%08x\n",
			__func__, desc, desc->tx_dma_desc.phys);

		if (direction == DMA_DEV_TO_MEM) {
			desc->lld.mbr_sa = sconfig->src_addr;
			desc->lld.mbr_da = buf_addr + i * period_len;
		} else {
			desc->lld.mbr_sa = buf_addr + i * period_len;
			desc->lld.mbr_da = sconfig->dst_addr;
		};
		desc->lld.mbr_ubc = AT_XDMAC_MBR_UBC_NDV1
			| AT_XDMAC_MBR_UBC_NDEN
			| AT_XDMAC_MBR_UBC_NSEN
			| AT_XDMAC_MBR_UBC_NDE
			| period_len / (1 << atchan->dwidth);

		dev_dbg(chan2dev(chan),
			 "%s: lld: mbr_sa=0x%08x, mbr_da=0x%08x, mbr_ubc=0x%08x\n",
			 __func__, desc->lld.mbr_sa, desc->lld.mbr_da, desc->lld.mbr_ubc);

		/* Chain lld. */
		if (prev) {
			prev->lld.mbr_nda = desc->tx_dma_desc.phys;
			dev_dbg(chan2dev(chan),
				 "%s: chain lld: prev=0x%p, mbr_nda=0x%08x\n",
				 __func__, prev, prev->lld.mbr_nda);
		}

		prev = desc;
		if (!first)
			first = desc;

		dev_dbg(chan2dev(chan), "%s: add desc 0x%p to descs_list 0x%p\n",
			 __func__, desc, first);
		list_add_tail(&desc->desc_node, &first->descs_list);
	}

	prev->lld.mbr_nda = first->tx_dma_desc.phys;
	dev_dbg(chan2dev(chan),
		"%s: chain lld: prev=0x%p, mbr_nda=0x%08x\n",
		__func__, prev, prev->lld.mbr_nda);
	first->tx_dma_desc.cookie = -EBUSY;
	first->tx_dma_desc.flags = flags;
	first->xfer_size = buf_len;

	return &first->tx_dma_desc;
}

static struct dma_async_tx_descriptor *
at_xdmac_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
			 size_t len, unsigned long flags)
{
	struct at_xdmac_chan 	*atchan = to_at_xdmac_chan(chan);
	struct at_xdmac_desc	*first = NULL, *prev = NULL;
	size_t			remaining_size = len, xfer_size = 0, ublen;
	dma_addr_t		src_addr = src, dst_addr = dest;
	u32			dwidth;
	/*
	 * WARNING: The channel configuration is set here since there is no
	 * dmaengine_slave_config call in this case. Moreover we don't know the
	 * direction, it involves we can't dynamically set the source and dest
	 * interface so we have to use the same one. Only interface 0 allows EBI
	 * access. Hopefully we can access DDR through both ports (at least on
	 * SAMA5D4x), so we can use the same interface for source and dest,
	 * that solves the fact we don't know the direction.
	 */
	u32			chan_cc = AT_XDMAC_CC_DAM_INCREMENTED_AM
					| AT_XDMAC_CC_SAM_INCREMENTED_AM
					| AT_XDMAC_CC_DIF(0)
					| AT_XDMAC_CC_SIF(0)
					| AT_XDMAC_CC_MBSIZE_SIXTEEN
					| AT_XDMAC_CC_TYPE_MEM_TRAN;

	dev_dbg(chan2dev(chan), "%s: src=0x%08x, dest=0x%08x, len=%d, flags=0x%lx\n",
		__func__, src, dest, len, flags);

	if (unlikely(!len))
		return NULL;

	/* Check address alignment to select the greater data width we can use.
	 * Some XDMAC implementations don't provide dword transfer, in this
	 * case selecting dword has the same behavior as selecting word transfers.
	 */
	if (!((src_addr | dst_addr) & 7)) {
		dwidth = AT_XDMAC_CC_DWIDTH_DWORD;
		dev_dbg(chan2dev(chan), "%s: dwidth: double word\n", __func__);
	} else if (!((src_addr | dst_addr)  & 3)) {
		dwidth = AT_XDMAC_CC_DWIDTH_WORD;
		dev_dbg(chan2dev(chan), "%s: dwidth: word\n", __func__);
	} else if (!((src_addr | dst_addr) & 1)) {
		dwidth = AT_XDMAC_CC_DWIDTH_HALFWORD;
		dev_dbg(chan2dev(chan), "%s: dwidth: half word\n", __func__);
	} else {
		dwidth = AT_XDMAC_CC_DWIDTH_BYTE;
		dev_dbg(chan2dev(chan), "%s: dwidth: byte\n", __func__);
	}

	atchan->cfg = chan_cc | AT_XDMAC_CC_DWIDTH(dwidth);

	/* Prepare descriptors. */
	while (remaining_size) {
		struct at_xdmac_desc	*desc = NULL;

		dev_dbg(chan2dev(chan), "%s: remaining_size=%u\n", __func__, remaining_size);

		spin_lock_irqsave(&atchan->lock, flags);
		desc = at_xdmac_get_desc(atchan);
		spin_unlock_irqrestore(&atchan->lock, flags);
		if (!desc) {
			dev_err(chan2dev(chan), "can't get descriptor\n");
			if (first)
				list_splice_init(&first->descs_list, &atchan->free_descs_list);
			return NULL;
		}

		/* Update src and dest addresses. */
		src_addr += xfer_size;
		dst_addr += xfer_size;

		if (remaining_size >= AT_XDMAC_MBR_UBC_UBLEN_MAX << dwidth)
			xfer_size = AT_XDMAC_MBR_UBC_UBLEN_MAX << dwidth;
		else
			xfer_size = remaining_size;

		dev_dbg(chan2dev(chan), "%s: xfer_size=%u\n", __func__, xfer_size);

		/* Check remaining length and change data width if needed. */
		if (!((src_addr | dst_addr | xfer_size) & 7)) {
			dwidth = AT_XDMAC_CC_DWIDTH_DWORD;
			dev_dbg(chan2dev(chan), "%s: dwidth: double word\n", __func__);
		} else if (!((src_addr | dst_addr | xfer_size)  & 3)) {
			dwidth = AT_XDMAC_CC_DWIDTH_WORD;
			dev_dbg(chan2dev(chan), "%s: dwidth: word\n", __func__);
		} else if (!((src_addr | dst_addr | xfer_size) & 1)) {
			dwidth = AT_XDMAC_CC_DWIDTH_HALFWORD;
			dev_dbg(chan2dev(chan), "%s: dwidth: half word\n", __func__);
		} else if ((src_addr | dst_addr | xfer_size) & 1) {
			dwidth = AT_XDMAC_CC_DWIDTH_BYTE;
			dev_dbg(chan2dev(chan), "%s: dwidth: byte\n", __func__);
		}
		chan_cc |= AT_XDMAC_CC_DWIDTH(dwidth);

		ublen = xfer_size >> dwidth;
		remaining_size -= xfer_size;

		desc->lld.mbr_sa = src_addr;
		desc->lld.mbr_da = dst_addr;
		desc->lld.mbr_ubc = AT_XDMAC_MBR_UBC_NDV2
			| AT_XDMAC_MBR_UBC_NDEN
			| AT_XDMAC_MBR_UBC_NSEN
			| (remaining_size ? 0 : AT_XDMAC_MBR_UBC_NDE)
			| ublen;
		desc->lld.mbr_cfg = chan_cc;

		dev_dbg(chan2dev(chan),
			 "%s: lld: mbr_sa=0x%08x, mbr_da=0x%08x, mbr_ubc=0x%08x, mbr_cfg=0x%08x\n",
			 __func__, desc->lld.mbr_sa, desc->lld.mbr_da, desc->lld.mbr_ubc, desc->lld.mbr_cfg);

		/* Chain lld. */
		if (prev) {
			prev->lld.mbr_nda = desc->tx_dma_desc.phys;
			dev_dbg(chan2dev(chan),
				 "%s: chain lld: prev=0x%p, mbr_nda=0x%08x\n",
				 __func__, prev, prev->lld.mbr_nda);
		}

		prev = desc;
		if (!first)
			first = desc;

		dev_dbg(chan2dev(chan), "%s: add desc 0x%p to descs_list 0x%p\n",
			 __func__, desc, first);
		list_add_tail(&desc->desc_node, &first->descs_list);
	}

	first->tx_dma_desc.cookie = -EBUSY;
	first->tx_dma_desc.flags = flags;
	first->xfer_size = len;

	return &first->tx_dma_desc;
}

static enum dma_status
at_xdmac_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
		struct dma_tx_state *txstate)
{
	struct at_xdmac_chan 	*atchan = to_at_xdmac_chan(chan);
	struct at_xdmac		*atxdmac = to_at_xdmac(atchan->chan.device);
	struct at_xdmac_desc	*desc, *_desc;
	unsigned long		flags;
	enum dma_status		ret;
	int			residue;
	u32			cur_nda;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_SUCCESS)
		return ret;

	spin_lock_irqsave(&atchan->lock, flags);

	desc = list_first_entry(&atchan->xfers_list, struct at_xdmac_desc, xfer_node);

	if (!desc->active_xfer)
		dev_err(chan2dev(chan),
			"something goes wrong, there is no active transfer\n");

	residue = desc->xfer_size;

	/* Flush FIFO. */
	at_xdmac_write(atxdmac, AT_XDMAC_GSWF, atchan->mask);
	while (!(at_xdmac_chan_read(atchan, AT_XDMAC_CIS) & AT_XDMAC_CIS_FIS));
		cpu_relax();

	cur_nda = at_xdmac_chan_read(atchan, AT_XDMAC_CNDA) & 0xfffffffc;
	/*
	 * Remove size of all microblocks already transferred and the current
	 * one. Then add the remaining size to transfer of the current
	 * microblock.
	 */
	list_for_each_entry_safe(desc, _desc, &desc->descs_list, desc_node) {
		residue -= (desc->lld.mbr_ubc & 0xffffff) << atchan->dwidth;
		if ((desc->lld.mbr_nda & 0xfffffffc) == cur_nda)
			break;
	}
	residue += at_xdmac_chan_read(atchan, AT_XDMAC_CUBC) << atchan->dwidth;

	spin_unlock_irqrestore(&atchan->lock, flags);

	dma_set_residue(txstate, residue);

	dev_dbg(chan2dev(chan),
		 "%s: desc=0x%p, tx_dma_desc.phys=0x%08x, tx_status=%d, cookie=%d, residue=%d\n",
		 __func__, desc, desc->tx_dma_desc.phys, ret, cookie, residue);

	return ret;
}

static void at_xdmac_terminate_xfer(struct at_xdmac_chan *atchan,
				    struct at_xdmac_desc *desc)
{
	dev_dbg(chan2dev(&atchan->chan), "%s: desc 0x%p\n", __func__, desc);

	/*
	 * It's necessary to remove the transfer before calling the callback
	 * because some devices can call dma_engine_terminate_all causing to do
	 * dma_cookie_complete two times on the same cookie.
	 */
	list_del(&desc->xfer_node);
	list_splice_init(&desc->descs_list, &atchan->free_descs_list);
}

static void at_xdmac_advance_work(struct at_xdmac_chan *atchan)
{
	struct at_xdmac_desc	*desc;
	unsigned long		flags;

	spin_lock_irqsave(&atchan->lock, flags);

	/*
	 * If channel is enabled, do nothing, advance_work will be triggered
	 * after the interruption.
	 */
	if (at_xdmac_chan_is_enabled(atchan)) {
		dev_dbg(chan2dev(&atchan->chan), "%s: chan enabled\n",
			 __func__);
	} else if (!list_empty(&atchan->xfers_list)) {
		desc = list_first_entry(&atchan->xfers_list,
					struct at_xdmac_desc,
					xfer_node);
		dev_vdbg(chan2dev(&atchan->chan), "%s: desc 0x%p\n", __func__, desc);
		if (!desc->active_xfer)
			at_xdmac_start_xfer(atchan, desc);
	}

	spin_unlock_irqrestore(&atchan->lock, flags);
}

static void at_xdmac_handle_cyclic(struct at_xdmac_chan *atchan)
{
	struct at_xdmac_desc		*desc;
	struct dma_async_tx_descriptor	*txd;

	desc = list_first_entry(&atchan->xfers_list, struct at_xdmac_desc, xfer_node);
	txd = &desc->tx_dma_desc;

	if (txd->callback && (txd->flags & DMA_PREP_INTERRUPT))
		txd->callback(txd->callback_param);
}

static void at_xdmac_tasklet(unsigned long data)
{
	struct at_xdmac_chan	*atchan = (struct at_xdmac_chan *)data;
	struct at_xdmac_desc	*desc;
	u32			error_mask;

	dev_dbg(chan2dev(&atchan->chan), "%s: status=0x%08lx\n",
		 __func__, atchan->status);

	error_mask = AT_XDMAC_CIS_RBEIS
		     | AT_XDMAC_CIS_WBEIS
		     | AT_XDMAC_CIS_ROIS;

	if (at_xdmac_chan_is_cyclic(atchan)) {
		at_xdmac_handle_cyclic(atchan);
	} else if ((atchan->status & AT_XDMAC_CIS_LIS)
		   || (atchan->status & error_mask)) {
		struct dma_async_tx_descriptor  *txd;

		if (atchan->status & AT_XDMAC_CIS_RBEIS)
			dev_err(chan2dev(&atchan->chan), "read bus error!!!");
		else if (atchan->status & AT_XDMAC_CIS_WBEIS)
			dev_err(chan2dev(&atchan->chan), "write bus error!!!");
		else if (atchan->status & AT_XDMAC_CIS_ROIS)
			dev_err(chan2dev(&atchan->chan), "request overflow error!!!");

		desc = list_first_entry(&atchan->xfers_list,
					struct at_xdmac_desc,
					xfer_node);
		dev_vdbg(chan2dev(&atchan->chan), "%s: desc 0x%p\n", __func__, desc);
		BUG_ON(!desc->active_xfer);

		txd = &desc->tx_dma_desc;

		at_xdmac_terminate_xfer(atchan, desc);

		if (!at_xdmac_chan_is_cyclic(atchan)) {
			dma_cookie_complete(txd);
			if (txd->callback && (txd->flags & DMA_PREP_INTERRUPT))
				txd->callback(txd->callback_param);
		}

		dma_run_dependencies(txd);

		at_xdmac_advance_work(atchan);
	}
}

static irqreturn_t at_xdmac_interrupt(int irq, void *dev_id)
{
	struct at_xdmac		*atxdmac = (struct at_xdmac *)dev_id;
	struct at_xdmac_chan	*atchan;
	u32 			imr, status, pending;
	u32			chan_imr, chan_status;
	int			i, ret = IRQ_NONE;

	do {
		imr = at_xdmac_read(atxdmac, AT_XDMAC_GIM);
		status = at_xdmac_read(atxdmac, AT_XDMAC_GIS);
		pending = status & imr;

		dev_vdbg(atxdmac->dma.dev,
			 "%s: status=0x%08x, imr=0x%08x, pending=0x%08x\n",
			 __func__, status, imr, pending);

		if (!pending)
			break;

		/* We have to find which channel has generated the interrupt. */
		for (i = 0; i < atxdmac->dma.chancnt; i++) {
			if (!((1 << i) & pending))
				continue;

			atchan = &atxdmac->chan[i];
			chan_imr = at_xdmac_chan_read(atchan, AT_XDMAC_CIM);
			chan_status = at_xdmac_chan_read(atchan, AT_XDMAC_CIS);
			atchan->status = chan_status & chan_imr;
			dev_vdbg(atxdmac->dma.dev,
				 "%s: chan%d: imr=0x%x, status=0x%x\n",
				 __func__, i, chan_imr, chan_status);
			dev_vdbg(chan2dev(&atchan->chan),
				 "%s: XDMAC_CC=0x%08x XDMAC_CNDA=0x%08x, "
				 "XDMAC_CNDC=0x%08x, XDMAC_CSA=0x%08x, "
				 "XDMAC_CDA=0x%08x, XDMAC_CUBC=0x%08x\n",
				 __func__,
				 at_xdmac_chan_read(atchan, AT_XDMAC_CC),
				 at_xdmac_chan_read(atchan, AT_XDMAC_CNDA),
				 at_xdmac_chan_read(atchan, AT_XDMAC_CNDC),
				 at_xdmac_chan_read(atchan, AT_XDMAC_CSA),
				 at_xdmac_chan_read(atchan, AT_XDMAC_CDA),
				 at_xdmac_chan_read(atchan, AT_XDMAC_CUBC));

			if (atchan->status & (AT_XDMAC_CIS_RBEIS | AT_XDMAC_CIS_WBEIS))
				at_xdmac_write(atxdmac, AT_XDMAC_GD, atchan->mask);

			tasklet_schedule(&atchan->tasklet);
			ret = IRQ_HANDLED;
		}

	} while (pending);

	return ret;
}

static void at_xdmac_issue_pending(struct dma_chan *chan)
{
	struct at_xdmac_chan *atchan = to_at_xdmac_chan(chan);

	dev_dbg(chan2dev(&atchan->chan), "%s\n", __func__);

	if (!at_xdmac_chan_is_cyclic(atchan))
		at_xdmac_advance_work(atchan);

	return;
}

static int at_xdmac_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
			    unsigned long arg)
{
	struct at_xdmac_desc	*desc, *_desc;
	struct at_xdmac_chan	*atchan = to_at_xdmac_chan(chan);
	struct at_xdmac		*atxdmac = to_at_xdmac(atchan->chan.device);
	unsigned long		flags;
	int			ret = 0;

	dev_dbg(chan2dev(chan), "%s: cmd=%d\n", __func__, cmd);

	spin_lock_irqsave(&atchan->lock, flags);

	switch (cmd) {
	case DMA_PAUSE:
		at_xdmac_write(atxdmac, AT_XDMAC_GRWS, atchan->mask);
		set_bit(AT_XDMAC_CHAN_IS_PAUSED, &atchan->status);
		break;
	case DMA_RESUME:
		if (!at_xdmac_chan_is_paused(atchan))
			break;

		at_xdmac_write(atxdmac, AT_XDMAC_GRWR, atchan->mask);
		clear_bit(AT_XDMAC_CHAN_IS_PAUSED, &atchan->status);
		break;
	case DMA_TERMINATE_ALL:
		at_xdmac_write(atxdmac, AT_XDMAC_GIE, atchan->mask);
		at_xdmac_write(atxdmac, AT_XDMAC_GD, atchan->mask);
		while (at_xdmac_read(atxdmac, AT_XDMAC_GS) & atchan->mask)
			cpu_relax();

		/* Cancel all pending transfers. */
		list_for_each_entry_safe(desc, _desc, &atchan->xfers_list, xfer_node)
			at_xdmac_terminate_xfer(atchan, desc);

		clear_bit(AT_XDMAC_CHAN_IS_CYCLIC, &atchan->status);
		break;
	case DMA_SLAVE_CONFIG:
		ret = at_xdmac_set_slave_config(chan,
				(struct dma_slave_config *)arg);
		break;
	default:
		dev_err(chan2dev(chan),
			"unmanaged or unknown dma control cmd: %d\n", cmd);
		ret = -ENXIO;
	}

	spin_unlock_irqrestore(&atchan->lock, flags);

	return ret;
}

static int at_xdmac_alloc_chan_resources(struct dma_chan *chan)
{
	struct at_xdmac_chan	*atchan = to_at_xdmac_chan(chan);
	struct at_xdmac_desc	*desc;
	unsigned long		flags;
	int			i;

	spin_lock_irqsave(&atchan->lock, flags);

	if (at_xdmac_chan_is_enabled(atchan)) {
		dev_err(chan2dev(chan),
			"can't allocate channel resources (channel enabled)\n");
		i = -EIO;
		goto spin_unlock;
	}

	if (!list_empty(&atchan->free_descs_list)) {
		dev_err(chan2dev(chan),
			"can't allocate channel resources (channel not free from a previous use)\n");
		i = -EIO;
		goto spin_unlock;
	}

	for (i = 0; i < init_nr_desc_per_channel; i++) {
		desc = at_xdmac_alloc_desc(chan, GFP_ATOMIC);
		if (!desc) {
			dev_warn(chan2dev(chan),
				"only %d descriptors have been allocated\n", i);
			break;
		}
		list_add_tail(&desc->desc_node, &atchan->free_descs_list);
	}

	dma_cookie_init(chan);

	dev_dbg(chan2dev(chan), "%s: allocated %d descriptors\n", __func__, i);

spin_unlock:
	spin_unlock_irqrestore(&atchan->lock, flags);
	return i;
}

static void at_xdmac_free_chan_resources(struct dma_chan *chan)
{
	struct at_xdmac_chan	*atchan = to_at_xdmac_chan(chan);
	struct at_xdmac		*atxdmac = to_at_xdmac(chan->device);
	struct at_xdmac_desc	*desc, *_desc;

	list_for_each_entry_safe(desc, _desc, &atchan->free_descs_list, desc_node) {
		dev_dbg(chan2dev(chan), "%s: freeing descriptor %p\n", __func__, desc);
		list_del(&desc->desc_node);
		dma_pool_free(atxdmac->at_xdmac_desc_pool, desc, desc->tx_dma_desc.phys);
	}

	return;
}

static int atmel_xdmac_prepare(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct at_xdmac		*atxdmac = platform_get_drvdata(pdev);
	struct dma_chan		*chan, *_chan;

	list_for_each_entry_safe(chan, _chan, &atxdmac->dma.channels, device_node) {
		struct at_xdmac_chan	*atchan = to_at_xdmac_chan(chan);

		/* Wait for transfer completion, except in cyclic case. */
		if (at_xdmac_chan_is_enabled(atchan) && !at_xdmac_chan_is_cyclic(atchan))
			return -EAGAIN;
	}
	return 0;
}

static int atmel_xdmac_suspend_noirq(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct at_xdmac		*atxdmac = platform_get_drvdata(pdev);
	struct dma_chan		*chan, *_chan;

	list_for_each_entry_safe(chan, _chan, &atxdmac->dma.channels, device_node) {
		struct at_xdmac_chan	*atchan = to_at_xdmac_chan(chan);

		if (at_xdmac_chan_is_cyclic(atchan)) {
			if (!at_xdmac_chan_is_paused(atchan))
				at_xdmac_control(chan, DMA_PAUSE, 0);
			atchan->save_cim = at_xdmac_chan_read(atchan, AT_XDMAC_CIM);
			atchan->save_cnda = at_xdmac_chan_read(atchan, AT_XDMAC_CNDA);
			atchan->save_cndc = at_xdmac_chan_read(atchan, AT_XDMAC_CNDC);
		}
	}
	atxdmac->save_gim = at_xdmac_read(atxdmac, AT_XDMAC_GIM);

	at_xdmac_off(atxdmac);
	clk_disable_unprepare(atxdmac->clk);
	return 0;
}

static int atmel_xdmac_resume_noirq(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct at_xdmac		*atxdmac = platform_get_drvdata(pdev);
	struct at_xdmac_chan	*atchan;
	struct dma_chan		*chan, *_chan;
	int			i;

	clk_prepare_enable(atxdmac->clk);

	/* Clear pending interrupts. */
	for (i = 0; i < atxdmac->dma.chancnt; i++) {
		atchan = &atxdmac->chan[i];
		while (at_xdmac_chan_read(atchan, AT_XDMAC_CIS))
			cpu_relax();
	}

	at_xdmac_write(atxdmac, AT_XDMAC_GIE, atxdmac->save_gim);
	at_xdmac_write(atxdmac, AT_XDMAC_GE, atxdmac->save_gs);
	list_for_each_entry_safe(chan, _chan, &atxdmac->dma.channels, device_node) {
		atchan = to_at_xdmac_chan(chan);
		at_xdmac_chan_write(atchan, AT_XDMAC_CC, atchan->cfg);
		if (at_xdmac_chan_is_cyclic(atchan)) {
			at_xdmac_chan_write(atchan, AT_XDMAC_CNDA, atchan->save_cnda);
			at_xdmac_chan_write(atchan, AT_XDMAC_CNDC, atchan->save_cndc);
			at_xdmac_chan_write(atchan, AT_XDMAC_CIE, atchan->save_cim);
			at_xdmac_write(atxdmac, AT_XDMAC_GE, atchan->mask);
		}
	}
	return 0;
}

static int at_xdmac_probe(struct platform_device *pdev)
{
	struct resource	*res;
	struct at_xdmac	*atxdmac;
	int		irq, size, nr_channels, i, ret;
	void __iomem	*base;
	u32		reg;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	/*
	 * Read number of xdmac channels, read helper function can't be used
	 * since atxdmac is not yet allocated and we need to know the number
	 * of channels to do the allocation.
	 */
	reg = readl_relaxed(base + AT_XDMAC_GTYPE);
	nr_channels = AT_XDMAC_NB_CH(reg);
	if (nr_channels > AT_XDMAC_MAX_CHAN) {
		dev_err(&pdev->dev, "invalid number of channels (%u)\n",
			nr_channels);
		return -EINVAL;
	}

	size = sizeof(*atxdmac);
	size += nr_channels * sizeof(struct at_xdmac_chan);
	atxdmac = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!atxdmac) {
		dev_err(&pdev->dev, "can't allocate at_xdmac structure\n");
		return -ENOMEM;
	}

	atxdmac->regs = base;
	atxdmac->irq = irq;

	ret = devm_request_irq(&pdev->dev, atxdmac->irq, at_xdmac_interrupt, 0,
			       "at_xdmac", atxdmac);
	if (ret) {
		dev_err(&pdev->dev, "can't request irq\n");
		return ret;
	}

	atxdmac->clk = devm_clk_get(&pdev->dev, "dma_clk");
	if (IS_ERR(atxdmac->clk)) {
		dev_err(&pdev->dev, "can't get dma_clk\n");
		return PTR_ERR(atxdmac->clk);
	}

	ret = clk_prepare_enable(atxdmac->clk);
	if (ret) {
		dev_err(&pdev->dev, "can't prepare or enable clock\n");
		return ret;
	}

	atxdmac->at_xdmac_desc_pool =
		dmam_pool_create(dev_name(&pdev->dev), &pdev->dev,
				sizeof(struct at_xdmac_desc), 4, 0);
	if (!atxdmac->at_xdmac_desc_pool) {
		dev_err(&pdev->dev, "no memory for descriptors dma pool\n");
		ret = -ENOMEM;
		goto err_clk_disable;
	}

	dma_cap_set(DMA_CYCLIC, atxdmac->dma.cap_mask);
	dma_cap_set(DMA_MEMCPY, atxdmac->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, atxdmac->dma.cap_mask);
	atxdmac->dma.dev				= &pdev->dev;
	atxdmac->dma.device_alloc_chan_resources	= at_xdmac_alloc_chan_resources;
	atxdmac->dma.device_free_chan_resources		= at_xdmac_free_chan_resources;
	atxdmac->dma.device_tx_status			= at_xdmac_tx_status;
	atxdmac->dma.device_issue_pending		= at_xdmac_issue_pending;
	atxdmac->dma.device_prep_dma_cyclic		= at_xdmac_prep_dma_cyclic;
	atxdmac->dma.device_prep_dma_memcpy		= at_xdmac_prep_dma_memcpy;
	atxdmac->dma.device_prep_slave_sg		= at_xdmac_prep_slave_sg;
	atxdmac->dma.device_control			= at_xdmac_control;
	atxdmac->dma.chancnt				= nr_channels;

	/* Disable all chans and interrupts. */
	at_xdmac_off(atxdmac);

	/* Init channels. */
	INIT_LIST_HEAD(&atxdmac->dma.channels);
	for (i = 0; i < nr_channels; i++) {
		struct at_xdmac_chan *atchan = &atxdmac->chan[i];

		atchan->chan.device = &atxdmac->dma;
		list_add_tail(&atchan->chan.device_node,
			      &atxdmac->dma.channels);

		atchan->ch_regs = at_xdmac_chan_reg_base(atxdmac, i);
		atchan->mask = 1 << i;

		spin_lock_init(&atchan->lock);
		INIT_LIST_HEAD(&atchan->xfers_list);
		INIT_LIST_HEAD(&atchan->free_descs_list);
		tasklet_init(&atchan->tasklet, at_xdmac_tasklet,
			     (unsigned long)atchan);

		/* Clear pending interrupts. */
		while (at_xdmac_chan_read(atchan, AT_XDMAC_CIS))
			cpu_relax();
	}
	platform_set_drvdata(pdev, atxdmac);

	ret = dma_async_device_register(&atxdmac->dma);
	if (ret) {
		dev_err(&pdev->dev, "fail to register DMA engine device\n");
		goto err_clk_disable;
	}

	ret = of_dma_controller_register(pdev->dev.of_node,
					 at_xdmac_xlate, atxdmac);
	if (ret) {
		dev_err(&pdev->dev, "could not register of dma controller\n");
		goto err_dma_unregister;
	}

	dev_info(&pdev->dev, "%d channels, mapped at 0x%p\n",
		 nr_channels, atxdmac->regs);

	return 0;

err_dma_unregister:
	dma_async_device_unregister(&atxdmac->dma);
err_clk_disable:
	clk_disable_unprepare(atxdmac->clk);
	return ret;
}

static int at_xdmac_remove(struct platform_device *pdev)
{
	struct at_xdmac	*atxdmac = (struct at_xdmac*)platform_get_drvdata(pdev);
	int		i;

	at_xdmac_off(atxdmac);
	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&atxdmac->dma);
	clk_disable_unprepare(atxdmac->clk);

	synchronize_irq(atxdmac->irq);

	for (i = 0; i < atxdmac->dma.chancnt; i++) {
		struct at_xdmac_chan *atchan = &atxdmac->chan[i];

		tasklet_kill(&atchan->tasklet);
		at_xdmac_free_chan_resources(&atchan->chan);
	}

	return 0;
}

static const struct dev_pm_ops atmel_xdmac_dev_pm_ops = {
	.prepare	= atmel_xdmac_prepare,
	.suspend_noirq	= atmel_xdmac_suspend_noirq,
	.resume_noirq	= atmel_xdmac_resume_noirq,
};

static const struct of_device_id atmel_xdmac_dt_ids[] = {
	{
		.compatible = "atmel,sama5d4-dma",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, atmel_xdmac_dt_ids);

static struct platform_driver at_xdmac_driver = {
	.probe		= at_xdmac_probe,
	.remove		= at_xdmac_remove,
	.driver = {
		.name		= "at_xdmac",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(atmel_xdmac_dt_ids),
		.pm		= &atmel_xdmac_dev_pm_ops,
	}
};

static int __init at_xdmac_init(void)
{
	return platform_driver_probe(&at_xdmac_driver, at_xdmac_probe);
}
subsys_initcall(at_xdmac_init);

MODULE_DESCRIPTION("Atmel Extended DMA Controller driver");
MODULE_AUTHOR("Ludovic Desroches <ludovic.desroches@atmel.com>");
MODULE_LICENSE("GPL");
