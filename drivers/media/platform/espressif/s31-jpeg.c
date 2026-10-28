// SPDX-License-Identifier: GPL-2.0-only
/* Baseline JPEG codec with the standard V4L2 JPEG parser and reference
 * tables. The codec consumes SOS and entropy bytes; Linux validates/programs the
 * header and builds the encoder's complete, portable JPEG interchange file.
 */
#include <linux/unaligned.h>
#include <linux/soc/espressif/esp32s31-regfield.h>
#include <linux/soc/espressif/regs/jpeg-regs.h>
#include <media/v4l2-jpeg.h>
#include "s31-media.h"

struct s31_jpeg_run {
	struct s31_media_ctx *ctx;
	bool decode, gray, rgb;
	u8 sampling;
};
static void s31_jpeg_quant(jpeg_dev_t __iomem *h, unsigned int id,
			   const u8 *natural)
{
	volatile void __iomem *fifo[] = { &h->t0qnr.val, &h->t1qnr.val,
					  &h->t2qnr.val, &h->t3qnr.val };
	unsigned int i;

	for (i = 0; i < 64; i++)
		writel(natural[i], fifo[id]);
}
static int s31_jpeg_huffman(jpeg_dev_t __iomem *h, unsigned int table,
			    const u8 *data, size_t length)
{
	volatile void __iomem *totals[] = { &h->dht_totlen_dc0.val,
					    &h->dht_totlen_dc1.val,
					    &h->dht_totlen_ac0.val,
					    &h->dht_totlen_ac1.val };
	volatile void __iomem *minima[] = { &h->dht_codemin_dc0.val,
					    &h->dht_codemin_dc1.val,
					    &h->dht_codemin_ac0.val,
					    &h->dht_codemin_ac1.val };
	volatile void __iomem *values[] = { &h->dht_val_dc0.val,
					    &h->dht_val_dc1.val,
					    &h->dht_val_ac0.val,
					    &h->dht_val_ac1.val };
	unsigned int i, total = 0, code = 0, max = table < 2 ? 12 : 162;

	if (length < 16)
		return -EINVAL;
	for (i = 0; i < 16; i++) {
		if (code + data[i] > BIT(i + 1))
			return -EINVAL;
		total += data[i];
		writel(total, totals[table]);
		writel(data[i] ? code << (15 - i) : 0xffff, minima[table]);
		code = (code + data[i]) << 1;
	}
	if (total > max || total + 16 != length)
		return -EINVAL;
	for (i = 0; i < max; i++)
		writel(i < total ? data[16 + i] : 0, values[table]);
	return 0;
}
static const u8 *s31_jpeg_default_huffman(unsigned int table, size_t *size)
{
	const u8 *tables[] = { v4l2_jpeg_ref_table_luma_dc_ht,
			       v4l2_jpeg_ref_table_chroma_dc_ht,
			       v4l2_jpeg_ref_table_luma_ac_ht,
			       v4l2_jpeg_ref_table_chroma_ac_ht };
	*size = table < 2 ? V4L2_JPEG_REF_HT_DC_LEN : V4L2_JPEG_REF_HT_AC_LEN;
	return tables[table];
}
static void s31_jpeg_start(void *arg)
{
	struct s31_jpeg_run *run = arg;
	jpeg_dev_t __iomem *h = run->ctx->dev->regs;

	s31_reg_set(h, config, jpeg_start, 1);
}
static void s31_jpeg_stop(void *arg)
{
	struct s31_jpeg_run *run = arg;
	jpeg_dev_t __iomem *h = run->ctx->dev->regs;

	writel(0, &h->int_ena.val);
	s31_reg_set(h, config, soft_rst, 1);
	s31_reg_set(h, config, soft_rst, 0);
}
static void s31_jpeg_dma_configure(void *arg, dma2d_dev_t __iomem *dma)
{
	struct s31_jpeg_run *run = arg;

	if (run->decode && !run->gray) {
		/* JPEG produces subsampled YUV. Convert in DMA2D instead of exposing
		 * Espressif's private packed YUV420 layout as standard planar YU12.
		 */
		static const s32 table[3][4] = {
			/* JPEG YCbCr uses full-range luma, rather than video-range 16..235. */
			{ 256, 0, 359, -45952 },
			{ 256, -88, -183, 34688 },
			{ 256, 454, 0, -58112 }
		};
		dma2d_color_param_group_chn_reg_t params = {};
		volatile dma2d_color_param_reg_t *rows[] = { &params.param_h,
							     &params.param_m,
							     &params.param_l };
		unsigned int i;

		writel(((run->sampling == 0 ? 2 : 0) << 3) | BIT(2) | 1,
		       &dma->in_channel[0].in_color_convert.val);
		for (i = 0; i < 3; i++) {
			rows[i]->a = table[i][0];
			rows[i]->b = table[i][1];
			rows[i]->c = table[i][2];
			rows[i]->d = table[i][3];
		}
		memcpy_toio(&dma->in_channel[0].in_color_param_group, &params,
			    sizeof(params));
		if (run->rgb)
			s31_reg_set(dma, in_channel[0].in_scramble,
				    in_scramble_sel_post_chn, 5);
	}
}
/* Append bounded encoder headers. Caller reserves at least 1024 bytes. */
static void s31_jpeg_word(u8 **p, u16 value)
{
	put_unaligned_be16(value, *p);
	*p += 2;
}
static size_t s31_jpeg_encode_header(u8 *buffer, u32 width, u32 height,
				     bool gray, const u8 qt[2][64])
{
	u8 *p = buffer;
	unsigned int i, t, components = gray ? 1 : 3;

	s31_jpeg_word(&p, 0xffd8);
	for (t = 0; t < (gray ? 1 : 2); t++) {
		s31_jpeg_word(&p, 0xffdb);
		s31_jpeg_word(&p, 67);
		*p++ = t;
		for (i = 0; i < 64; i++)
			*p++ = qt[t][v4l2_jpeg_zigzag_scan_index[i]];
	}
	s31_jpeg_word(&p, 0xffc0);
	s31_jpeg_word(&p, 8 + components * 3);
	*p++ = 8;
	s31_jpeg_word(&p, height);
	s31_jpeg_word(&p, width);
	*p++ = components;
	for (i = 0; i < components; i++) {
		*p++ = i + 1;
		*p++ = !gray && i == 0 ? 0x22 : 0x11;
		*p++ = i ? 1 : 0;
	}
	for (t = 0; t < 4; t++) {
		size_t length;
		const u8 *table;

		if (gray && (t & 1))
			continue;
		table = s31_jpeg_default_huffman(t, &length);
		s31_jpeg_word(&p, 0xffc4);
		s31_jpeg_word(&p, length + 3);
		*p++ = (t & 1) | ((t >> 1) << 4);
		memcpy(p, table, length);
		p += length;
	}
	/* A COM segment aligns entropy data to 64 bytes, keeping DMA/CPU header
	 * ownership on separate cache lines, also for future imported buffers.
	 */
	i = ALIGN((p - buffer) + 4 + 8 + components * 2, 64) -
	    ((p - buffer) + 4 + 8 + components * 2);
	s31_jpeg_word(&p, 0xfffe);
	s31_jpeg_word(&p, i + 2);
	memset(p, 0, i);
	p += i;
	s31_jpeg_word(&p, 0xffda);
	s31_jpeg_word(&p, 6 + components * 2);
	*p++ = components;
	for (i = 0; i < components; i++) {
		*p++ = i + 1;
		*p++ = i ? 0x11 : 0;
	}
	*p++ = 0;
	*p++ = 63;
	*p++ = 0;
	return p - buffer;
}
static int s31_jpeg_decode_setup(struct s31_jpeg_run *run,
				 struct vb2_v4l2_buffer *src,
				 struct s31_dma2d_job *job)
{
	struct s31_media_ctx *ctx = run->ctx;
	jpeg_dev_t __iomem *h = ctx->dev->regs;
	struct v4l2_jpeg_reference qt[4] = {}, ht[4] = {};
	struct v4l2_jpeg_scan_header scan = {};
	struct v4l2_jpeg_header header = { .scan = &scan,
					   .quantization_tables = qt,
					   .huffman_tables = ht };
	u8 natural[64];
	u8 *input = vb2_plane_vaddr(&src->vb2_buf, 0);
	size_t length = vb2_get_plane_payload(&src->vb2_buf, 0), scan_offset;
	u32 i, n, component;
	int ret;

	if (!input)
		return -EFAULT;
	ret = v4l2_jpeg_parse_header(input, length, &header);
	if (ret)
		return ret;
	if (header.frame.precision != 8 ||
	    (header.frame.num_components != 1 &&
	     header.frame.num_components != 3) ||
	    header.frame.width != ctx->fmt[1].width ||
	    header.frame.height != ctx->fmt[1].height ||
	    header.ecs_offset >= length ||
	    scan.num_components != header.frame.num_components ||
	    !header.sof.start || header.sof.start < input + 2 ||
	    header.sof.start[-1] != 0xc0)
		return -EINVAL;
	/* The hardware handles one baseline scan, with fixed Y/chroma table lanes. */
	if (!header.sos.start || header.sos.length < 3 ||
	    header.sos.start[header.sos.length - 3] != 0 ||
	    header.sos.start[header.sos.length - 2] != 63 ||
	    header.sos.start[header.sos.length - 1] != 0)
		return -EINVAL;
	/* The receive CSC implements YCbCr, not Adobe RGB/YCCK. */
	if (header.frame.num_components == 3 &&
	    header.app14_tf != V4L2_JPEG_APP14_TF_UNKNOWN &&
	    header.app14_tf != V4L2_JPEG_APP14_TF_YCBCR)
		return -EINVAL;
	run->gray = header.frame.num_components == 1;
	if (run->gray != (ctx->fmt[1].pixelformat == V4L2_PIX_FMT_GREY))
		return -EINVAL;
	switch (header.frame.subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
		run->sampling = 3;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
		run->sampling = 0;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		run->sampling = 1;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		run->sampling = 2;
		break;
	default:
		return -EINVAL;
	}
	s31_reg_set(h, decode_conf, component_num, header.frame.num_components);
	s31_reg_set(h, decode_conf, restart_interval, header.restart_interval);
	for (i = 0; i < header.frame.num_components; i++) {
		const struct v4l2_jpeg_frame_component_spec *c =
			&header.frame.component[i];
		if (c->quantization_table_selector >= 4 ||
		    !qt[c->quantization_table_selector].start ||
		    scan.component[i].component_selector !=
			    c->component_identifier ||
		    scan.component[i].dc_entropy_coding_table_selector != !!i ||
		    scan.component[i].ac_entropy_coding_table_selector != !!i ||
		    (i && (c->horizontal_sampling_factor != 1 ||
			   c->vertical_sampling_factor != 1)))
			return -EINVAL;
		component = c->quantization_table_selector |
			    (c->vertical_sampling_factor << 8) |
			    (c->horizontal_sampling_factor << 12) |
			    (c->component_identifier << 16);
		writel(component, (void __iomem *)&h->c0 + i * 4);
	}
	for (i = 0; i < 4; i++) {
		if (!qt[i].start)
			continue;
		if (qt[i].length != 64)
			return -EINVAL;
		for (n = 0; n < 64; n++) {
			if (!qt[i].start[n])
				return -EINVAL;
			natural[v4l2_jpeg_zigzag_scan_index[n]] =
				qt[i].start[n];
		}
		s31_jpeg_quant(h, i, natural);
	}
	for (i = 0; i < 4; i++) {
		size_t size = ht[i].length;
		const u8 *table = ht[i].start;

		if (!table)
			table = s31_jpeg_default_huffman(i, &size);
		ret = s31_jpeg_huffman(h, i, table, size);
		if (ret)
			return ret;
	}
	/* Hardware parses the validated SOS marker itself, as in the native SDK. */
	scan_offset = header.sos.start - input - 2;
	s31_dma2d_desc_linear(&job->tx,
			      vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0) +
				      scan_offset,
			      length - scan_offset, true);
	job->rx_macroblock = run->sampling == 2 ? 2 :
			     run->sampling == 1 ? 1 :
						  0;
	job->rx_reorder = true;
	return 0;
}
int s31_jpeg_run(struct s31_media_ctx *ctx, struct vb2_v4l2_buffer *src,
		 struct vb2_v4l2_buffer *dst, size_t *bytes)
{
	struct s31_media *d = ctx->dev;
	jpeg_dev_t __iomem *h = d->regs;
	struct s31_jpeg_run run = { .ctx = ctx };
	struct s31_dma2d_job job = { .peripheral = 0,
				     .tx_macroblock = 3,
				     .rx_macroblock = 3,
				     .configure = s31_jpeg_dma_configure,
				     .start = s31_jpeg_start,
				     .stop = s31_jpeg_stop,
				     .arg = &run };
	jpeg_config_reg_t config = {};
	u8 qt[2][64];
	size_t header_bytes = 0;
	u32 w, height, hb, vb, i, t;
	int ret;

	run.decode = ctx->fmt[0].pixelformat == V4L2_PIX_FMT_JPEG;
	if (run.decode == (ctx->fmt[1].pixelformat == V4L2_PIX_FMT_JPEG))
		return -EINVAL;
	ret = reset_control_reset(d->reset);
	if (ret)
		return ret;
	config.mode = run.decode;
	config.qnr_fifo_en = 1;
	config.dht_fifo_en = 1;
	config.decode_timeout_thres = 32;
	config.cqnr_tbl_sel = 1;
	config.ff_check_en = 1;
	config.tailer_en = 1;
	writel(config.val | BIT(24), &h->config.val);
	writel(config.val, &h->config.val);
	writel(0x03020100, &h->dqt_info.val);
	writel(0x1010, &h->dht_info.val);
	if (run.decode) {
		/* The decoder requires zero dimensions before a new picture is set up. */
		writel(0, &h->pic_size.val);
		run.rgb = ctx->fmt[1].pixelformat == V4L2_PIX_FMT_RGB24;
		ret = s31_jpeg_decode_setup(&run, src, &job);
		if (ret)
			return ret;
		w = ctx->fmt[1].width;
		height = ctx->fmt[1].height;
		hb = run.gray	       ? 96 :
		     run.sampling == 0 ? 32 :
		     run.sampling == 1 ? 64 :
					 48;
		/* A receive block wider than the picture stalls the DMA reorder path.
		 * The preferred SDK sizes are upper bounds, including for small images.
		 */
		hb = min(hb, w);
		vb = run.sampling == 2 ? 16 : 8;
		s31_dma2d_desc_image(
			&job.rx,
			vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0), w,
			height, hb, vb, run.gray ? 1 : 4, true);
	} else {
		unsigned int quality = v4l2_ctrl_g_ctrl(ctx->quality);
		unsigned int factor = quality < 50 ? 5000 / quality :
						     200 - 2 * quality;
		u8 *output = vb2_plane_vaddr(&dst->vb2_buf, 0);

		if (!output || vb2_plane_size(&dst->vb2_buf, 0) < 2048)
			return -EINVAL;
		run.gray = ctx->fmt[0].pixelformat == V4L2_PIX_FMT_GREY;
		job.tx_macroblock = run.gray ? 0 : 2;
		job.tx_reorder = true;
		w = ctx->fmt[0].width;
		height = ctx->fmt[0].height;
		if (ctx->fmt[1].width != w || ctx->fmt[1].height != height)
			return -EINVAL;
		for (t = 0; t < 2; t++) {
			const u8 *base = t ? v4l2_jpeg_ref_table_chroma_qt :
					     v4l2_jpeg_ref_table_luma_qt;
			for (i = 0; i < 64; i++)
				qt[t][i] = clamp_t(
					u32, (base[i] * factor + 50) / 100, 1,
					255);
			s31_jpeg_quant(h, t, qt[t]);
			s31_jpeg_quant(h, t + 2, qt[t]);
		}
		config.color_space = run.gray ? 3 : 0;
		config.sample_sel = run.gray ? 0 : 2;
		config.pixel_rev = ctx->fmt[0].pixelformat ==
				   V4L2_PIX_FMT_RGB24;
		writel(config.val, &h->config.val);
		header_bytes =
			s31_jpeg_encode_header(output, w, height, run.gray, qt);
		if (header_bytes & 63)
			return -EINVAL;
		/* The CPU adds the aligned interchange header after QBUF. Publish it
		 * before DMA starts; only the separate entropy cache lines are written
		 * by the device. JPEG queues use bidirectional DMA mappings.
		 */
		dma_sync_single_for_device(
			s31_dma2d_device(d->dma),
			vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0),
			header_bytes, DMA_BIDIRECTIONAL);
		s31_dma2d_desc_linear(
			&job.rx,
			vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0) +
				header_bytes,
			vb2_plane_size(&dst->vb2_buf, 0) - header_bytes, false);
		s31_dma2d_desc_image(
			&job.tx,
			vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0), w,
			height, run.gray ? 128 : 32, run.gray ? 8 : 16,
			run.gray ? 1 : 4, true);
	}
	/* JPEG generates its frame EOF; descriptor exhaustion is not frame EOF. */
	job.tx.words[0] &= cpu_to_le32(~BIT(30));
	job.rx.words[0] &= cpu_to_le32(~BIT(30));
	writel(height | (w << 16), &h->pic_size.val);
	writel(0, &h->int_ena.val);
	writel(~0U, &h->int_clr.val);
	ret = s31_dma2d_transfer(d->dma, &job);
	if (!ret) {
		if (run.decode)
			*bytes = ctx->fmt[1].sizeimage;
		else if (!job.received)
			ret = -EIO;
		else if (job.received >
			 vb2_plane_size(&dst->vb2_buf, 0) - header_bytes)
			ret = -EOVERFLOW;
		else
			*bytes = header_bytes + job.received;
	}
	return ret;
}
