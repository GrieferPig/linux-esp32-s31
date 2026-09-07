// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S31 I2S/TDM/PDM ASoC driver through AHB GDMA. */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define S31_I2S_RX_CONF		0x20
#define S31_I2S_TX_CONF		0x24
#define S31_I2S_RX_CONF1		0x28
#define S31_I2S_TX_CONF1		0x2c
#define S31_I2S_RX_TDM_CTRL		0x50
#define S31_I2S_TX_TDM_CTRL		0x54
#define S31_I2S_RXEOF_NUM		0x64
#define S31_I2S_STATE			0x6c
#define S31_I2S_FIFO_CNT		0x78
#define S31_I2S_DESTINATION		0xf4

#define S31_I2S_RESET			BIT(0)
#define S31_I2S_FIFO_RESET		BIT(1)
#define S31_I2S_START			BIT(2)
#define S31_I2S_SLAVE			BIT(3)
#define S31_I2S_UPDATE			BIT(8)
#define S31_I2S_MSB_SHIFT		BIT(13)
#define S31_I2S_MONO			BIT(6)
#define S31_I2S_BIG_ENDIAN		BIT(7)
#define S31_I2S_PCM_BYPASS		BIT(12)
#define S31_I2S_LEFT_ALIGN		BIT(15)
#define S31_I2S_FILL_24			BIT(16)
#define S31_I2S_WS_IDLE_POL		BIT(17)
#define S31_I2S_BIT_ORDER		BIT(18)
#define S31_I2S_TDM_EN			BIT(19)
#define S31_I2S_PDM_EN			BIT(20)
#define S31_I2S_BCK_DIV			GENMASK(26, 21)
#define S31_I2S_SIG_LOOPBACK		BIT(30)
#define S31_I2S_RUNTIME_CONF_MASK	(S31_I2S_START | S31_I2S_SLAVE | \
					 S31_I2S_MSB_SHIFT | S31_I2S_TDM_EN | \
					 S31_I2S_PDM_EN | S31_I2S_BCK_DIV | \
					 S31_I2S_SIG_LOOPBACK)
#define S31_I2S_STD_FORMAT_MASK		(S31_I2S_MONO | S31_I2S_BIG_ENDIAN | \
					 S31_I2S_PCM_BYPASS | \
					 S31_I2S_LEFT_ALIGN | S31_I2S_FILL_24 | \
					 S31_I2S_WS_IDLE_POL | S31_I2S_BIT_ORDER)
#define S31_I2S_WS_WIDTH		GENMASK(8, 0)
#define S31_I2S_BITS_MOD			GENMASK(18, 14)
#define S31_I2S_HALF_SAMPLE_BITS	GENMASK(26, 19)
#define S31_I2S_CHAN_BITS		GENMASK(31, 27)
#define S31_I2S_TDM_CHAN_MASK		GENMASK(15, 0)
#define S31_I2S_TDM_CHAN_NUM		GENMASK(19, 16)

struct esp32s31_i2s {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_dmaengine_dai_dma_data capture_dma_data;
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu;
	struct snd_soc_dai_link_component codec;
	struct snd_soc_dai_link_component platform;
	struct pinctrl *pinctrl;
	struct pinctrl_state *playback_pins;
	struct pinctrl_state *capture_pins;
	bool tx_pdm;
	bool rx_pdm;
	bool tx_slave;
	bool rx_slave;
	bool external_card;
	u32 dai_format;
	u32 tx_mask;
	u32 rx_mask;
	u32 slots;
	u32 slot_width;
	u32 sysclk;
	u32 configured;
	u32 configured_mclk;
	bool loopback;
};

/* AHB GDMA stores a period in one 12-bit hardware descriptor. */
static const struct snd_pcm_hardware esp32s31_i2s_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |
		   SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_8000_192000,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 1,
	.channels_max = 16,
	.period_bytes_min = 256,
	.period_bytes_max = 4032,
	.periods_min = 2,
	.periods_max = 8,
	.buffer_bytes_max = 4032 * 8,
};

static const struct snd_dmaengine_pcm_config esp32s31_i2s_pcm_config = {
	.pcm_hardware = &esp32s31_i2s_pcm_hardware,
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.prealloc_buffer_size = 4032 * 8,
	/* Sv32 has no uncached PTE attribute. Use ALSA's streaming DMA
	 * synchronization instead of treating remapped PSRAM as coherent. */
	.buffer_type = SNDRV_DMA_TYPE_NONCOHERENT,
};

static struct esp32s31_i2s *esp32s31_i2s_from_dai(struct snd_soc_dai *dai)
{
	/* The built-in card shares the controller device. ASoC replaces its
	 * drvdata with the card before binding the DAI. An external machine
	 * card has a separate device and leaves the controller drvdata alone. */
	if (device_property_read_bool(dai->dev, "espressif,external-card"))
		return dev_get_drvdata(dai->dev);
	return container_of(dev_get_drvdata(dai->dev), struct esp32s31_i2s, card);
}

static bool esp32s31_i2s_running(struct esp32s31_i2s *i2s)
{
	return (readl(i2s->base + S31_I2S_TX_CONF) |
		readl(i2s->base + S31_I2S_RX_CONF)) & S31_I2S_START;
}

static int esp32s31_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct esp32s31_i2s *i2s = esp32s31_i2s_from_dai(dai);
	bool slave;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		break;
	default:
		return -EINVAL;
	}
	/* BCLK inversion belongs to the matrix route on this SoC.  Do not
	 * accept it here without actually changing the pin provider. */
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF &&
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_IF)
		return -EINVAL;
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BP_FP:
		slave = false;
		break;
	case SND_SOC_DAIFMT_BC_FC:
		slave = true;
		break;
	default:
		return -EINVAL;
	}
	if ((i2s->configured || esp32s31_i2s_running(i2s)) && i2s->dai_format != fmt)
		return -EBUSY;
	i2s->dai_format = fmt;
	i2s->tx_slave = slave;
	i2s->rx_slave = slave;
	return 0;
}

static int esp32s31_i2s_set_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask, int slots, int width)
{
	struct esp32s31_i2s *i2s = esp32s31_i2s_from_dai(dai);

	if (slots < 0 || slots > 16 ||
	    (slots && (width < 8 || width > 32)) ||
	    (!slots && (tx_mask || rx_mask)) ||
	    ((tx_mask | rx_mask) >> slots))
		return -EINVAL;
	if ((i2s->configured || esp32s31_i2s_running(i2s)) &&
	    (i2s->slots != slots || i2s->slot_width != (slots ? width : 0) ||
	     i2s->tx_mask != tx_mask || i2s->rx_mask != rx_mask))
		return -EBUSY;
	i2s->slots = slots;
	i2s->slot_width = slots ? width : 0;
	i2s->tx_mask = tx_mask;
	i2s->rx_mask = rx_mask;
	return 0;
}

static int esp32s31_i2s_set_sysclk(struct snd_soc_dai *dai, int id,
				 unsigned int rate, int direction)
{
	struct esp32s31_i2s *i2s = esp32s31_i2s_from_dai(dai);

	/* ID 0 is the provider-managed internal MCLK.  The driver has no
	 * external MCLK input selector; slave BCLK/WS do not require one. */
	if (id || direction != SND_SOC_CLOCK_OUT)
		return -EINVAL;
	if ((i2s->configured || esp32s31_i2s_running(i2s)) && rate != i2s->sysclk)
		return -EBUSY;
	i2s->sysclk = rate;
	return 0;
}

static void esp32s31_i2s_reset(struct esp32s31_i2s *i2s, u32 reg)
{
	u32 val = readl(i2s->base + reg);

	writel(val | S31_I2S_RESET | S31_I2S_FIFO_RESET, i2s->base + reg);
	writel(val & ~(S31_I2S_RESET | S31_I2S_FIFO_RESET), i2s->base + reg);
}

static int esp32s31_i2s_update(struct esp32s31_i2s *i2s, u32 reg)
{
	u32 val;
	int ret;

	writel(readl(i2s->base + reg) | S31_I2S_UPDATE, i2s->base + reg);
	ret = readl_poll_timeout(i2s->base + reg, val,
				 !(val & S31_I2S_UPDATE), 1, 1000);
	if (ret)
		dev_err(i2s->dev, "register %#x update stuck at %#08x\n",
			reg, val);
	return ret;
}

static int esp32s31_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct esp32s31_i2s *i2s = esp32s31_i2s_from_dai(dai);

	snd_soc_dai_init_dma_data(dai, &i2s->playback_dma_data,
				  &i2s->capture_dma_data);
	return 0;
}

static int esp32s31_i2s_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params,
				   struct snd_soc_dai *dai)
{
	struct esp32s31_i2s *i2s = esp32s31_i2s_from_dai(dai);
	u32 conf, conf1, tdm;
	u32 channels = params_channels(params);
	u32 width = params_width(params);
	u32 physical_width = params_physical_width(params);
	u32 slots = i2s->slots ?: channels;
	u32 slot_width = i2s->slot_width ?: physical_width;
	u32 mask = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		   i2s->tx_mask : i2s->rx_mask;
	u32 format = i2s->dai_format & SND_SOC_DAIFMT_FORMAT_MASK;
	u32 framing = S31_I2S_PCM_BYPASS | S31_I2S_LEFT_ALIGN;
	u32 ws_width = slot_width;
	u32 bit_clock = params_rate(params) * slots * slot_width;
	u32 clock_rate;
	bool slave = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		     i2s->tx_slave : i2s->rx_slave;
	/* The slave clock-domain synchronizer needs at least eight module
	 * clock cycles per external BCLK (the ESP-IDF slave clock minimum). */
	u32 mclk_multiple = max_t(u32, 128,
				 slots * slot_width * (slave ? 8 : 1));
	u32 mclk = i2s->sysclk ?: params_rate(params) * mclk_multiple;
	u32 bck_div;
	u64 bck_error, generated_bck;
	int ret;
	bool pdm = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		i2s->tx_pdm : i2s->rx_pdm;

	if (channels < 1 || channels > 16 || !width || width > 32 ||
	    !physical_width || physical_width > 32 || !bit_clock)
		return -EINVAL;
	if (pdm && channels > 2)
		return -EINVAL;
	if (slot_width < width || slots < channels || (slots * slot_width) % 2 ||
	    (mask && hweight32(mask) != channels))
		return -EINVAL;
	if (!mask)
		mask = GENMASK(channels - 1, 0);
	if (format == SND_SOC_DAIFMT_I2S || format == SND_SOC_DAIFMT_DSP_A)
		framing |= S31_I2S_MSB_SHIFT;
	if (format == SND_SOC_DAIFMT_DSP_A || format == SND_SOC_DAIFMT_DSP_B) {
		ws_width = 1;
		framing |= S31_I2S_WS_IDLE_POL;
	}
	if ((i2s->dai_format & SND_SOC_DAIFMT_INV_MASK) == SND_SOC_DAIFMT_NB_IF)
		framing ^= S31_I2S_WS_IDLE_POL;
	if (i2s->configured & ~BIT(substream->stream)) {
		if (mclk != i2s->configured_mclk)
			return -EBUSY;
	} else {
		ret = clk_set_rate(i2s->clk, mclk);
		if (ret)
			return ret;
	}
	clock_rate = clk_get_rate(i2s->clk);
	bck_div = DIV_ROUND_CLOSEST(clock_rate, bit_clock);
	if (!bck_div || bck_div > 64)
		return -EINVAL;
	if (slave && clock_rate < (u64)bit_clock * 8)
		return -EINVAL;
	generated_bck = (u64)bit_clock * bck_div;
	bck_error = clock_rate > generated_bck ? clock_rate - generated_bck :
		generated_bck - clock_rate;
	if (bck_error > bit_clock / 1000)
		return -ERANGE;

	conf1 = FIELD_PREP(S31_I2S_WS_WIDTH, ws_width - 1) |
		FIELD_PREP(S31_I2S_BITS_MOD, width - 1) |
		FIELD_PREP(S31_I2S_HALF_SAMPLE_BITS, slots * slot_width / 2 - 1) |
		FIELD_PREP(S31_I2S_CHAN_BITS, slot_width - 1);
	tdm = FIELD_PREP(S31_I2S_TDM_CHAN_MASK, mask) |
	      FIELD_PREP(S31_I2S_TDM_CHAN_NUM, slots - 1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		writel(conf1, i2s->base + S31_I2S_TX_CONF1);
		writel(tdm, i2s->base + S31_I2S_TX_TDM_CTRL);
		conf = readl(i2s->base + S31_I2S_TX_CONF);
		conf &= ~(S31_I2S_RUNTIME_CONF_MASK | S31_I2S_STD_FORMAT_MASK);
		conf |= framing |
			FIELD_PREP(S31_I2S_BCK_DIV, bck_div - 1);
		if (i2s->tx_slave)
			conf |= S31_I2S_SLAVE;
		if (!pdm)
			conf |= S31_I2S_TDM_EN;
		if (pdm)
			conf |= S31_I2S_PDM_EN;
		if (i2s->loopback)
			conf |= S31_I2S_SIG_LOOPBACK;
		writel(conf, i2s->base + S31_I2S_TX_CONF);
		ret = esp32s31_i2s_update(i2s, S31_I2S_TX_CONF);
		goto configured;
	}

	writel(conf1, i2s->base + S31_I2S_RX_CONF1);
	writel(tdm, i2s->base + S31_I2S_RX_TDM_CTRL);
	writel(params_period_bytes(params), i2s->base + S31_I2S_RXEOF_NUM);
	conf = readl(i2s->base + S31_I2S_RX_CONF);
	conf &= ~(S31_I2S_RUNTIME_CONF_MASK | S31_I2S_STD_FORMAT_MASK);
	conf |= framing |
		FIELD_PREP(S31_I2S_BCK_DIV, bck_div - 1);
	if (i2s->rx_slave)
		conf |= S31_I2S_SLAVE;
	if (!pdm)
		conf |= S31_I2S_TDM_EN;
	if (pdm)
		conf |= S31_I2S_PDM_EN;
	writel(conf, i2s->base + S31_I2S_RX_CONF);
	ret = esp32s31_i2s_update(i2s, S31_I2S_RX_CONF);
configured:
	if (!ret) {
		i2s->configured |= BIT(substream->stream);
		i2s->configured_mclk = mclk;
	}
	return ret;
}

static int esp32s31_i2s_hw_free(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct esp32s31_i2s *i2s = esp32s31_i2s_from_dai(dai);

	i2s->configured &= ~BIT(substream->stream);
	return 0;
}

static int esp32s31_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
				 struct snd_soc_dai *dai)
{
	struct esp32s31_i2s *i2s = esp32s31_i2s_from_dai(dai);
	u32 reg = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		  S31_I2S_TX_CONF : S31_I2S_RX_CONF;
	u32 val = readl(i2s->base + reg);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (cmd == SNDRV_PCM_TRIGGER_START) {
			dev_info(i2s->dev,
				 "%s start dma=%pad bytes=%zu head=%*ph\n",
				 substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
				 "playback" : "capture",
				 &substream->runtime->dma_addr,
				 substream->runtime->dma_bytes, 16,
				 substream->runtime->dma_area);
		}
		val |= S31_I2S_START;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev_info(i2s->dev,
			 "%s stop conf=%#x state=%#x fifo-count=%#x\n",
			 substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			 "playback" : "capture", val,
			 readl(i2s->base + S31_I2S_STATE),
			 readl(i2s->base + S31_I2S_FIFO_CNT));
		val &= ~S31_I2S_START;
		break;
	default:
		return -EINVAL;
	}
	writel(val, i2s->base + reg);
	return esp32s31_i2s_update(i2s, reg);
}

static int esp32s31_i2s_startup(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct esp32s31_i2s *i2s = esp32s31_i2s_from_dai(dai);
	struct pinctrl_state *state =
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		i2s->playback_pins : i2s->capture_pins;
	int ret;

	if (state) {
		ret = pinctrl_select_state(i2s->pinctrl, state);
		if (ret)
			return ret;
	}

	esp32s31_i2s_reset(i2s,
			  substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			  S31_I2S_TX_CONF : S31_I2S_RX_CONF);
	return 0;
}

static void esp32s31_i2s_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct esp32s31_i2s *i2s = esp32s31_i2s_from_dai(dai);

	/* The capture state contains input routes only and is therefore safe idle. */
	if (i2s->capture_pins)
		pinctrl_select_state(i2s->pinctrl, i2s->capture_pins);
}

static const struct snd_soc_dai_ops esp32s31_i2s_dai_ops = {
	.probe = esp32s31_i2s_dai_probe,
	.startup = esp32s31_i2s_startup,
	.shutdown = esp32s31_i2s_shutdown,
	.hw_params = esp32s31_i2s_hw_params,
	.hw_free = esp32s31_i2s_hw_free,
	.trigger = esp32s31_i2s_trigger,
	.set_fmt = esp32s31_i2s_set_fmt,
	.set_tdm_slot = esp32s31_i2s_set_tdm_slot,
	.set_sysclk = esp32s31_i2s_set_sysclk,
};

static struct snd_soc_dai_driver esp32s31_i2s_dai = {
	.name = "esp32s31-i2s",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.symmetric_rate = 1,
	.ops = &esp32s31_i2s_dai_ops,
};

static const struct snd_soc_component_driver esp32s31_i2s_component = {
	.name = "esp32s31-i2s",
};

static int esp32s31_i2s_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_i2s *i2s;
	struct reset_control *rst;
	struct resource *res;
	int ret;

	i2s = devm_kzalloc(dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;
	i2s->dev = dev;
	i2s->tx_pdm = device_property_read_bool(dev, "espressif,tx-pdm");
	i2s->rx_pdm = device_property_read_bool(dev, "espressif,rx-pdm");
	i2s->tx_slave = device_property_read_bool(dev, "espressif,tx-slave");
	i2s->rx_slave = true;
	i2s->dai_format = SND_SOC_DAIFMT_I2S;
	i2s->external_card = device_property_read_bool(dev, "espressif,external-card");
	i2s->loopback = device_property_read_bool(dev, "espressif,loopback");
	i2s->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(i2s->pinctrl)) {
		if (PTR_ERR(i2s->pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		i2s->pinctrl = NULL;
	} else {
		i2s->playback_pins = pinctrl_lookup_state(i2s->pinctrl,
							 "playback");
		if (IS_ERR(i2s->playback_pins))
			i2s->playback_pins = NULL;
		i2s->capture_pins = pinctrl_lookup_state(i2s->pinctrl,
							"capture");
		if (IS_ERR(i2s->capture_pins))
			i2s->capture_pins = NULL;
		if (!!i2s->playback_pins != !!i2s->capture_pins)
			return dev_err_probe(dev, -EINVAL,
				"both playback and capture pinctrl states are required\n");
	}
	platform_set_drvdata(pdev, i2s);
	i2s->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(i2s->base))
		return PTR_ERR(i2s->base);
	i2s->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(i2s->clk))
		return PTR_ERR(i2s->clk);
	rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "reset unavailable\n");
	ret = reset_control_reset(rst);
	if (ret)
		return dev_err_probe(dev, ret, "reset failed\n");

	writel(0, i2s->base + S31_I2S_DESTINATION);
	esp32s31_i2s_reset(i2s, S31_I2S_TX_CONF);
	esp32s31_i2s_reset(i2s, S31_I2S_RX_CONF);

	i2s->playback_dma_data.addr = res->start;
	i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	i2s->playback_dma_data.maxburst = 4;
	i2s->playback_dma_data.chan_name = "tx";
	i2s->capture_dma_data.addr = res->start;
	i2s->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	i2s->capture_dma_data.maxburst = 4;
	i2s->capture_dma_data.chan_name = "rx";

	ret = devm_snd_soc_register_component(dev, &esp32s31_i2s_component,
					      &esp32s31_i2s_dai, 1);
	if (ret)
		return ret;
	ret = devm_snd_dmaengine_pcm_register(dev, &esp32s31_i2s_pcm_config, 0);
	if (ret)
		return ret;

	/* A machine driver (including simple-audio-card) owns the link when an
	 * external codec is present.  Do not also bind it to the dummy card. */
	if (i2s->external_card)
		return 0;
	i2s->cpu.of_node = dev->of_node;
	i2s->platform.of_node = dev->of_node;
	i2s->codec.name = "snd-soc-dummy";
	i2s->codec.dai_name = "snd-soc-dummy-dai";
	i2s->link.name = "ESP32-S31 I2S loopback";
	i2s->link.stream_name = "I2S";
	i2s->link.cpus = &i2s->cpu;
	i2s->link.num_cpus = 1;
	i2s->link.codecs = &i2s->codec;
	i2s->link.num_codecs = 1;
	i2s->link.platforms = &i2s->platform;
	i2s->link.num_platforms = 1;
	/* Preserve the legacy overlay's separate TX/RX clock roles. */
	i2s->link.dai_fmt = 0;

	i2s->card.dev = dev;
	i2s->card.owner = THIS_MODULE;
	i2s->card.name = devm_kasprintf(dev, GFP_KERNEL, "%pOFn",
					dev->of_node);
	i2s->card.dai_link = &i2s->link;
	i2s->card.num_links = 1;

	return devm_snd_soc_register_card(dev, &i2s->card);
}

static const struct of_device_id esp32s31_i2s_of_match[] = {
	{ .compatible = "espressif,esp32s31-i2s" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_i2s_of_match);

static struct platform_driver esp32s31_i2s_driver = {
	.probe = esp32s31_i2s_probe,
	.driver = {
		.name = "esp32s31-i2s",
		.of_match_table = esp32s31_i2s_of_match,
	},
};
module_platform_driver(esp32s31_i2s_driver);

MODULE_DESCRIPTION("ESP32-S31 I2S ASoC driver");
MODULE_LICENSE("GPL");
