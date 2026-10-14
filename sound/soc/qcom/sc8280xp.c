// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/soundwire/sdw.h>
#include <sound/jack.h>
#include <linux/input-event-codes.h>
#include "qdsp6/q6afe.h"
#include "common.h"
#include "sdw.h"

struct sc8280xp_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct snd_soc_jack jack;
	struct snd_soc_jack dp_jack[8];
	bool jack_setup;
	/* optional DSP bit clocks of the MI2S ports, enabled before the codecs */
	struct clk *mi2s_clk[5];
	unsigned int mi2s_bclk[5];
	bool mi2s_clk_on[5];
};

static const char * const sc8280xp_mi2s_clk_names[] = {
	"primary-mi2s", "secondary-mi2s", "tertiary-mi2s",
	"quaternary-mi2s", "quinary-mi2s",
};

static int sc8280xp_mi2s_index(unsigned int dai_id)
{
	switch (dai_id) {
	case PRIMARY_MI2S_RX...QUATERNARY_MI2S_TX:
		return (dai_id - PRIMARY_MI2S_RX) / 2;
	case QUINARY_MI2S_RX...QUINARY_MI2S_TX:
		return 4;
	/* TDM on the same serial interface uses the same bit clock and codecs */
	case PRIMARY_TDM_RX_0...QUINARY_TDM_TX_7:
		return (dai_id - PRIMARY_TDM_RX_0) / 16;
	default:
		return -1;
	}
}

static int sc8280xp_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_jack *dp_jack  = NULL;
	int dp_pcm_id = 0;
	struct snd_soc_dai *codec_dai;
	int i;

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX...QUATERNARY_MI2S_TX:
	case QUINARY_MI2S_RX...QUINARY_MI2S_TX:
	/* TDM framed like I2S (from the topology) for the same codecs */
	case PRIMARY_TDM_RX_0...QUINARY_TDM_TX_7:
		snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP);
		/*
		 * The DSP provides bit clock and frame sync; codecs follow.
		 * With several codecs on one link (e.g. a pair of speaker
		 * amplifiers), codec N takes slot N: left, right, ...
		 *
		 * A stereo pair also gets the other channel as its second
		 * receive slot, as the vendor CS35L41 driver programs it
		 * (cirrus,right-channel-amp: RX1 = 1, RX2 = 0). The speaker
		 * protection firmware takes RX2 as the opposite channel; left
		 * at its reset default (slot 1) the right amplifier receives
		 * its own channel twice and the firmware keeps it silent.
		 */
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			unsigned int slots[2] = { i, !i };

			snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_CBC_CFC |
					    SND_SOC_DAIFMT_I2S |
					    SND_SOC_DAIFMT_NB_NF);
			if (rtd->dai_link->num_codecs == 2)
				snd_soc_dai_set_channel_map(codec_dai, 0, NULL, 2, slots);
			else if (rtd->dai_link->num_codecs > 2)
				snd_soc_dai_set_channel_map(codec_dai, 0, NULL, 1, slots);
		}
		break;
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_RX_1:
		/*
		 * Set limit of -3 dB on Digital Volume and 0 dB on PA Volume
		 * to reduce the risk of speaker damage until we have active
		 * speaker protection in place.
		 */
		snd_soc_limit_volume(card, "WSA_RX0 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA_RX1 Digital Volume", 81);
		snd_soc_limit_volume(card, "SpkrLeft PA Volume", 17);
		snd_soc_limit_volume(card, "SpkrRight PA Volume", 17);
		break;
	case DISPLAY_PORT_RX_0:
		/* DISPLAY_PORT dai ids are not contiguous */
		dp_pcm_id = 0;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	case DISPLAY_PORT_RX_1 ... DISPLAY_PORT_RX_7:
		dp_pcm_id = cpu_dai->id - DISPLAY_PORT_RX_1 + 1;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	default:
		break;
	}

	if (dp_jack)
		return qcom_snd_dp_jack_setup(rtd, dp_jack, dp_pcm_id);

	return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
}

static int sc8280xp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
	channels->min = 2;
	channels->max = 2;
	switch (cpu_dai->id) {
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
		channels->min = 1;
		break;
	/*
	 * Two microphones (e.g. Xiaomi Diting AMIC1 + AMIC5): keep both
	 * channels, the voice call's dual-mic noise suppression needs them.
	 */
	case TX_CODEC_DMA_TX_3:
		if (of_machine_is_compatible("xiaomi,diting"))
			break;
		channels->min = 1;
		break;
	default:
		break;
	}


	return 0;
}

/*
 * MI2S codecs that clock themselves from the bit clock (e.g. CS35L41 speaker
 * amplifiers) need its rate. Codecs without such a clock return -ENOTSUPP.
 */
static int sc8280xp_snd_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	int idx = sc8280xp_mi2s_index(cpu_dai->id);
	unsigned int bclk;
	int i, ret;

	if (idx < 0)
		return 0;

	bclk = params_rate(params) * params_channels(params) *
	       params_physical_width(params);
	data->mi2s_bclk[idx] = bclk;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_component_set_sysclk(codec_dai->component, 0, 0,
						   bclk, SND_SOC_CLOCK_IN);
		if (ret && ret != -ENOTSUPP)
			return ret;

		ret = snd_soc_dai_set_sysclk(codec_dai, 0, bclk, SND_SOC_CLOCK_IN);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	return 0;
}

static int sc8280xp_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	int idx = sc8280xp_mi2s_index(cpu_dai->id);
	int ret;

	/*
	 * The DSP starts the MI2S clocks only when the graph starts, after
	 * DAPM has powered up the codecs. Codecs that lock a PLL to the bit
	 * clock need it earlier: vote for it here.
	 */
	if (idx >= 0 && data->mi2s_clk[idx] && !data->mi2s_clk_on[idx]) {
		ret = clk_set_rate(data->mi2s_clk[idx], data->mi2s_bclk[idx]);
		if (ret)
			return ret;
		ret = clk_prepare_enable(data->mi2s_clk[idx]);
		if (ret)
			return ret;
		data->mi2s_clk_on[idx] = true;
	}

	return qcom_snd_sdw_prepare(substream, &data->stream_prepared[cpu_dai->id]);
}

static int sc8280xp_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int idx = sc8280xp_mi2s_index(cpu_dai->id);

	if (idx >= 0 && data->mi2s_clk_on[idx]) {
		clk_disable_unprepare(data->mi2s_clk[idx]);
		data->mi2s_clk_on[idx] = false;
	}

	return qcom_snd_sdw_hw_free(substream, &data->stream_prepared[cpu_dai->id]);
}

static const struct snd_soc_ops sc8280xp_be_ops = {
	.startup = qcom_snd_sdw_startup,
	.shutdown = qcom_snd_sdw_shutdown,
	.hw_params = sc8280xp_snd_hw_params,
	.hw_free = sc8280xp_snd_hw_free,
	.prepare = sc8280xp_snd_prepare,
};

static void sc8280xp_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->init = sc8280xp_snd_init;
			link->be_hw_params_fixup = sc8280xp_be_hw_params_fixup;
			link->ops = &sc8280xp_be_ops;
		}
	}
}

static int sc8280xp_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct sc8280xp_snd_data *data;
	struct device *dev = &pdev->dev;
	int i, ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	card->owner = THIS_MODULE;
	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(sc8280xp_mi2s_clk_names); i++) {
		data->mi2s_clk[i] = devm_clk_get_optional(dev, sc8280xp_mi2s_clk_names[i]);
		if (IS_ERR(data->mi2s_clk[i]))
			return dev_err_probe(dev, PTR_ERR(data->mi2s_clk[i]),
					     "failed to get %s clock\n",
					     sc8280xp_mi2s_clk_names[i]);
	}

	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->driver_name = of_device_get_match_data(dev);
	sc8280xp_add_be_ops(card);
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_sc8280xp_dt_match[] = {
	{.compatible = "qcom,eliza-sndcard", "eliza"},
	{.compatible = "qcom,kaanapali-sndcard", "kaanapali"},
	{.compatible = "qcom,qcm6490-idp-sndcard", "qcm6490"},
	{.compatible = "qcom,qcs615-sndcard", "qcs615"},
	{.compatible = "qcom,qcs6490-rb3gen2-sndcard", "qcs6490"},
	{.compatible = "qcom,qcs8275-sndcard", "qcs8300"},
	{.compatible = "qcom,qcs9075-sndcard", "sa8775p"},
	{.compatible = "qcom,qcs9100-sndcard", "sa8775p"},
	{.compatible = "qcom,sc8280xp-sndcard", "sc8280xp"},
	{.compatible = "qcom,sm8450-sndcard", "sm8450"},
	{.compatible = "qcom,sm8550-sndcard", "sm8550"},
	{.compatible = "qcom,sm8650-sndcard", "sm8650"},
	{.compatible = "qcom,sm8750-sndcard", "sm8750"},
	{}
};

MODULE_DEVICE_TABLE(of, snd_sc8280xp_dt_match);

static struct platform_driver snd_sc8280xp_driver = {
	.probe  = sc8280xp_platform_probe,
	.driver = {
		.name = "snd-sc8280xp",
		.of_match_table = snd_sc8280xp_dt_match,
	},
};
module_platform_driver(snd_sc8280xp_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("SC8280XP ASoC Machine Driver");
MODULE_LICENSE("GPL");
