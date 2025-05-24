#include "igb.h"
// #include <linux/init.h>
// #include <linux/pci.h>
// #include <linux/slab.h>
#include "avpdu.h"
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <asm/fpu/api.h>
#include "avb-config.h"

/* SNDRV_CARDS: maximum number of cards supported by this module */

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;

AVPDU_HEADER_NO_VLAN *cavpdus[NUMBER_OF_STREAMS];
AVPDU_HEADER *pavpdus[NUMBER_OF_STREAMS];

static int dev = 0;
u_int8_t channels = 8;

u32 mergeDatas[24 * 8 * 16 * 4];

// #define DMA_BUFFERSIZE (768*4)   /* 4*8*6*8*2 */
// #define DMA_PERIODSIZE (384*4)   /* 4*8*6*8 */

#define DMA_BUFFERSIZE (6 * 8 * 2 * 8)

#define MAXPACKETSIZE 1024
#define MAXDESCRIPTORS 256
#define LAUNCH_OFFSET 125000			/* nanoseconds */
#define PRESENTATION_TIME_OFFSET 300000 /* nanoseconds */

/* hardware definition */

static struct snd_pcm_hardware snd_avb_playback_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
			 SNDRV_PCM_INFO_INTERLEAVED |
			 SNDRV_PCM_INFO_BLOCK_TRANSFER |
			 SNDRV_PCM_INFO_MMAP_VALID), // |
										 // SNDRV_PCM_INFO_SYNC_START |
	// SNDRV_PCM_INFO_JOINT_DUPLEX),
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 |
			 SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000,
	.rate_min = 44100,
	.rate_max = 192000,
	.channels_min = 8,
	.channels_max = 128,
	.buffer_bytes_max = 1200 * 4 * 128 * 4,
	.period_bytes_min = 6 * 4 * 8,
	.period_bytes_max = 1200 * 4 * 128,
	.periods_min = 3,
	.periods_max = 3,
};
/* max period size = 1200 = 25ms at 48000 */
/* max periods = 4 */
/* max channels = 128 (16 streams) */
/* sample size = 4 bytes  (AM824) */
/* one for playback, one for capture */

/* hardware definition */
static struct snd_pcm_hardware snd_avb_capture_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
			 SNDRV_PCM_INFO_INTERLEAVED |
			 SNDRV_PCM_INFO_BLOCK_TRANSFER |
			 SNDRV_PCM_INFO_MMAP_VALID), // |
										 // SNDRV_PCM_INFO_SYNC_START |
	// SNDRV_PCM_INFO_JOINT_DUPLEX),
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 |
			 SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000,
	.rate_min = 44100,
	.rate_max = 192000,
	.channels_min = 8,
	.channels_max = 128,
	.buffer_bytes_max = 1200 * 4 * 128 * 4,
	.period_bytes_min = 6 * 4 * 8,
	.period_bytes_max = 1200 * 4 * 128,
	.periods_min = 3,
	.periods_max = 3,
};

/* some convertion routines */

void u64_to_array6(u64 src, u8 *dest)
{
	memset(dest, 0, 6);

	dest[0] = (src >> (8 * 5)) & 0xff;
	dest[1] = (src >> (8 * 4)) & 0xff;
	dest[2] = (src >> (8 * 3)) & 0xff;
	dest[3] = (src >> (8 * 2)) & 0xff;
	dest[4] = (src >> (8 * 1)) & 0xff;
	dest[5] = (src >> (8 * 0)) & 0xff;
}

void u64_to_array8(u64 src, u8 *dest)
{
	memset(dest, 0, 8);

	dest[0] = (src >> (8 * 7)) & 0xff;
	dest[1] = (src >> (8 * 6)) & 0xff;
	dest[2] = (src >> (8 * 5)) & 0xff;
	dest[3] = (src >> (8 * 4)) & 0xff;
	dest[4] = (src >> (8 * 3)) & 0xff;
	dest[5] = (src >> (8 * 2)) & 0xff;
	dest[6] = (src >> (8 * 1)) & 0xff;
	dest[7] = (src >> (8 * 0)) & 0xff;
}

/* open callback */
static int snd_avb_playback_open(struct snd_pcm_substream *substream)
{
	struct igb_adapter *adapter = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	//        printk(KERN_INFO "open playback\n");

	runtime->hw = snd_avb_playback_hw;
	adapter->playback = 0;

	//	runtime->dma_area = adapter->p_addr;
	//	runtime->dma_bytes = 8*4*6*8*2;

	//        snd_pcm_set_sync(substream);

	return 0;
}

/* close callback */
static int snd_avb_playback_close(struct snd_pcm_substream *substream)
{
	//        struct igb_adapter *adapter = snd_pcm_substream_chip(substream);

	//        printk(KERN_INFO "close playback\n");

	return 0;
}

/* open callback */
static int snd_avb_capture_open(struct snd_pcm_substream *substream)
{
	struct igb_adapter *adapter = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	//        printk(KERN_INFO "open capture\n");

	runtime->hw = snd_avb_capture_hw;
	adapter->capture = 0;

	return 0;
}

/* close callback */
static int snd_avb_capture_close(struct snd_pcm_substream *substream)
{
	//        struct igb_adapter *adapter = snd_pcm_substream_chip(substream);

	//        printk(KERN_INFO "close capture\n");
	return 0;
}

/* hw_params callback */
static int snd_avb_pcm_hw_params(struct snd_pcm_substream *substream,
								 struct snd_pcm_hw_params *hw_params)
{
	int ret;

	//        printk(KERN_INFO "hw params\n");

	ret = snd_pcm_lib_malloc_pages(substream,
								   params_buffer_bytes(hw_params));

	//        printk(KERN_INFO "ret = %d\n", ret);

	return ret;
}

/* hw_free callback */
static int snd_avb_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/* prepare playback callback */
static int snd_avb_pcm_playback_prepare(struct snd_pcm_substream *substream)
{
	struct igb_adapter *adapter = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (runtime->rate != adapter->rate)
	{
		printk(KERN_INFO "samplerate change not supported yet\n");
		return -1;
	}
	adapter->rate = runtime->rate;
	adapter->playback_channels = runtime->channels;
	adapter->playback_period_size = runtime->period_size;
	adapter->playback_buffer_size = runtime->buffer_size;
	adapter->hw_playback_pointer = 0;
	adapter->p_cnt = 0;

	switch (adapter->rate)
	{
	case 44100:
	case 48000:
		adapter->samples_per_packet = 6;
		break;
	case 88200:
	case 96000:
		adapter->samples_per_packet = 12;
		break;
	case 176400:
	case 192000:
		adapter->samples_per_packet = 24;
		break;
	}

	return 0;
}

/* prepare capture callback */
static int snd_avb_pcm_capture_prepare(struct snd_pcm_substream *substream)
{
	struct igb_adapter *adapter = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (runtime->rate != adapter->rate)
	{
		printk(KERN_INFO "samplerate change not supported yet\n");
		return -1;
	}

	adapter->rate = runtime->rate;
	adapter->capture_channels = runtime->channels;
	adapter->capture_period_size = runtime->period_size;
	adapter->capture_buffer_size = runtime->buffer_size;
	adapter->hw_capture_pointer = 0;
	adapter->c_cnt = 0;

	switch (adapter->rate)
	{
	case 44100:
	case 48000:
		adapter->samples_per_packet = 6;
		break;
	case 88200:
	case 96000:
		adapter->samples_per_packet = 12;
		break;
	case 176400:
	case 192000:
		adapter->samples_per_packet = 24;
		break;
	}

	return 0;
}

/* trigger playback callback */
static int snd_avb_pcm_playback_trigger(struct snd_pcm_substream *substream,
										int cmd)
{
	struct igb_adapter *adapter = snd_pcm_substream_chip(substream);

	// printk(KERN_INFO "playback trigger %d\n", cmd);

	switch (cmd)
	{
	case SNDRV_PCM_TRIGGER_START:
		adapter->playback = 1;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		adapter->playback = 0;
		udelay(300);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* trigger callback */
static int snd_avb_pcm_capture_trigger(struct snd_pcm_substream *substream,
									   int cmd)
{
	struct igb_adapter *adapter = snd_pcm_substream_chip(substream);

	switch (cmd)
	{
	case SNDRV_PCM_TRIGGER_START:
		adapter->capture = 1;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		adapter->capture = 0;
		udelay(300);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* pointer callback */
static snd_pcm_uframes_t
snd_avb_pcm_playback_pointer(struct snd_pcm_substream *substream)
{
	struct igb_adapter *adapter = snd_pcm_substream_chip(substream);

	return adapter->hw_playback_pointer / adapter->playback_channels;
}

/* pointer callback */
static snd_pcm_uframes_t
snd_avb_pcm_capture_pointer(struct snd_pcm_substream *substream)
{
	struct igb_adapter *adapter = snd_pcm_substream_chip(substream);

	return adapter->hw_capture_pointer / adapter->capture_channels;
}

/* operators */
static struct snd_pcm_ops snd_avb_playback_ops = {
	.open = snd_avb_playback_open,
	.close = snd_avb_playback_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_avb_pcm_hw_params,
	.hw_free = snd_avb_pcm_hw_free,
	.prepare = snd_avb_pcm_playback_prepare,
	.trigger = snd_avb_pcm_playback_trigger,
	.pointer = snd_avb_pcm_playback_pointer,
};

/* operators */
static struct snd_pcm_ops snd_avb_capture_ops = {
	.open = snd_avb_capture_open,
	.close = snd_avb_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_avb_pcm_hw_params,
	.hw_free = snd_avb_pcm_hw_free,
	.prepare = snd_avb_pcm_capture_prepare,
	.trigger = snd_avb_pcm_capture_trigger,
	.pointer = snd_avb_pcm_capture_pointer,
};

/* create a pcm device */
static int snd_avb_new_pcm(struct igb_adapter *adapter)
{
	struct snd_pcm *pcm;
	int err;
	int i;

	u8 dest[6];
	u8 src[6];
	u8 stream_id[8];

	u64_to_array6(OWN_TALKER_MAC_BASE, dest);
	u64_to_array6(OWN_MAC, src);
	u64_to_array8(OWN_MAC << 16, stream_id);

	printk(KERN_INFO "creating new pcm avb\n");
	err = snd_pcm_new(adapter->card, "AVB", 0, 1, 1, &pcm);
	printk(KERN_INFO "err %d\n", err);
	if (err < 0)
		return err;

	pcm->private_data = adapter;
	strcpy(pcm->name, "AVB");
	adapter->pcm = pcm;

	/* set operators */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
					&snd_avb_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
					&snd_avb_capture_ops);

	pcm->nonatomic = 0;

	printk(KERN_INFO "prealloc pages of new pcm avb\n");

	/* max period size = 1200 = 25ms at 48000 */
	/* max periods = 4 */
	/* max channels = 128 (16 streams) */
	/* sample size = 4 */
	/* one for playback, one for capture */

	/* ALSA BUG! there should be an err return value
			err = snd_pcm_lib_preallocate_pages_for_all
					(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
					 snd_dma_continuous_data(GFP_KERNEL),
					 64*1024, 64*1024);

			if (err < 0)
					return err;
	*/

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
										  NULL,
										  64 * 1024, 64 * 1024);

	printk(KERN_INFO "alloc tx new pcm avb\n");
	adapter->tx_addr = dma_alloc_coherent(&adapter->pdev->dev, 4096 * 16 * 4,
										  &adapter->tx_physaddr, GFP_KERNEL);
	if (adapter->tx_addr == NULL)
		return -1;

	printk(KERN_INFO "alloc rx new pcm avb\n");
	adapter->rx_addr = dma_alloc_coherent(&adapter->pdev->dev, 4096 * 16 * 4,
										  &adapter->rx_physaddr, GFP_KERNEL);
	if (adapter->rx_addr == NULL)
	{
		printk(KERN_INFO "alloc rx failed\n");
		if (adapter->tx_addr)
		{
			dma_free_coherent(&adapter->pdev->dev, 4096 * 16 * 4,
							  adapter->tx_addr, adapter->tx_physaddr);
			adapter->tx_addr = 0;
		}

		return -1;
	}

	/* initialize transmit buffers */

	memset(adapter->tx_addr, 0, 4096 * 16 * 4);

	int j = 0;

	for (i = 0; i < MAXDESCRIPTORS; i++)
	{
		j = i % NUMBER_OF_STREAMS;
		stream_id[7] = j;
		dest[5] = j;
		void *p = adapter->tx_addr;

		init_avpdu_header(p + i * 1024, dest, src, stream_id, channels, adapter->rate);
	}

	return 0;
}

int snd_avb_probe(struct igb_adapter *adapter, int samplerate)
{
	struct snd_card *card;

	int err;

	u8 filter[] =
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Destination MAC used by the input AVB stream */
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Source MAC (of AVB device) */
		 0x81, 0x00, 0x60, 0x02, 0x22, 0xf0,
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	u8 filter_mask[] = {0xcf, 0xff, 0x03};

	adapter->card = 0;
	adapter->capture = 0;
	adapter->playback = 0;

	snd_avb_capture_hw.channels_min = NUMBER_OF_STREAMS * 8;
	snd_avb_capture_hw.channels_max = NUMBER_OF_STREAMS * 8;
	snd_avb_playback_hw.channels_min = NUMBER_OF_STREAMS * 8;
	snd_avb_playback_hw.channels_max = NUMBER_OF_STREAMS * 8;

	switch (samplerate)
	{
	case 44100:
		adapter->samples_per_packet = 6;
		snd_avb_playback_hw.rates = SNDRV_PCM_RATE_44100;
		snd_avb_capture_hw.rates = SNDRV_PCM_RATE_44100;
		break;
	case 48000:
		snd_avb_playback_hw.rates = SNDRV_PCM_RATE_48000;
		snd_avb_capture_hw.rates = SNDRV_PCM_RATE_48000;
		adapter->samples_per_packet = 6;
		break;
	case 88200:
		snd_avb_playback_hw.rates = SNDRV_PCM_RATE_88200;
		snd_avb_capture_hw.rates = SNDRV_PCM_RATE_88200;
		adapter->samples_per_packet = 12;
		break;
	case 96000:
		snd_avb_playback_hw.rates = SNDRV_PCM_RATE_96000;
		snd_avb_capture_hw.rates = SNDRV_PCM_RATE_96000;
		adapter->samples_per_packet = 12;
		break;
	case 176400:
		snd_avb_playback_hw.rates = SNDRV_PCM_RATE_176400;
		snd_avb_capture_hw.rates = SNDRV_PCM_RATE_176400;
		adapter->samples_per_packet = 24;
		break;
	case 192000:
		snd_avb_playback_hw.rates = SNDRV_PCM_RATE_192000;
		snd_avb_capture_hw.rates = SNDRV_PCM_RATE_192000;
		adapter->samples_per_packet = 24;
		break;
	default:
		printk(KERN_ERR "Invalid sample rate %d\n", samplerate);
		return -1;
	}

	adapter->rate = samplerate;

	printk(KERN_INFO "probing avb dev\n");
	if (dev >= SNDRV_CARDS)
		return -ENODEV;

	printk(KERN_INFO "enabling avb dev\n");
	if (!enable[dev])
	{
		dev++;
		return -ENOENT;
	}

	printk(KERN_INFO "creating card avb dev\n");
	/* (2) */
	err = snd_card_new(&adapter->pdev->dev, index[dev], id[dev], THIS_MODULE, 0, &card);
	if (err < 0)
		return err;

	adapter->card = card;

	/* (3) */
	printk(KERN_INFO "creating pcm avb dev\n");
	err = snd_avb_new_pcm(adapter);
	if (err < 0)
		goto error;

	/* (4) */
	printk(KERN_INFO "setting names pcm avb dev\n");
	strcpy(card->driver, "AVB");
	strcpy(card->shortname, "AVB");
	sprintf(card->longname, "%s %d", card->shortname, dev);

	/* (5) */
	printk(KERN_INFO "register card pcm avb dev\n");
	err = snd_card_register(card);
	if (err < 0)
		goto error;

	/* (6) */
	printk(KERN_INFO "register card pcm avb dev done\n");
	dev++;

	/* setup rx queue */

	{
		union e1000_adv_rx_desc *desc = adapter->rx_ring[1]->desc;

		int i;

		for (i = 0; i < MAXDESCRIPTORS; i++)
		{
			desc[i].read.pkt_addr = adapter->rx_physaddr + MAXPACKETSIZE * i;
			desc[i].read.hdr_addr = 0;
		}

		wmb();
		writel(16, adapter->rx_ring[1]->tail);
	}

	/* setup flex filter */
	/* directs the AVB input stream to i210's rx-queue-1 */

	u64_to_array6(AVB_DEVICE_TALKER_MAC_BASE, &filter[0]);
	u64_to_array6(AVB_DEVICE_SOURCE_MAC, &filter[6]);

	igb_setup_flex_filter(adapter, 1, 0, sizeof(filter), filter, filter_mask);

	return err;

error:
	snd_card_free(card);
	return err;
}

void snd_avb_remove(struct igb_adapter *adapter)
{
	adapter->capture = 0;
	adapter->playback = 0;

	udelay(300);

	igb_clear_flex_filter(adapter, 0);

	if (adapter->tx_addr)
		dma_free_coherent(&adapter->pdev->dev, 4096 * 16 * 4,
						  adapter->tx_addr, adapter->tx_physaddr);
	adapter->tx_addr = 0;

	if (adapter->rx_addr)
		dma_free_coherent(&adapter->pdev->dev, 4096 * 16 * 4,
						  adapter->rx_addr, adapter->rx_physaddr);
	adapter->rx_addr = 0;

	if (adapter->card)
		snd_card_free(adapter->card);
	adapter->card = 0;
}

void handle_rx_packet(struct igb_adapter *adapter)
{
	int j;
	u_int32_t data;
	int oldcptr = adapter->hw_capture_pointer;
	int newcptr = oldcptr;
	int samples_per_packet = adapter->samples_per_packet;
	struct snd_pcm_substream *csubs = ((struct snd_pcm *)adapter->pcm)->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	int capture = adapter->capture;
	int c_cnt = adapter->c_cnt;
	u32 *caddr = 0;

	if (csubs && csubs->runtime && csubs->runtime->dma_area)
		caddr = ((u32 *)csubs->runtime->dma_area) + oldcptr;

	int tmp_idx = oldcptr / adapter->capture_channels;
	int count1 = samples_per_packet;
	int count2 = 0;

	if (tmp_idx + samples_per_packet > adapter->capture_buffer_size)
	{
		count2 = tmp_idx + samples_per_packet - adapter->capture_buffer_size;
		count1 -= count2;
	}

	for (j = 0; j < count1 * adapter->capture_channels; j++)
	{
		data = mergeDatas[j];
		data = data >> 8;
		data = ntohl(data);

		*caddr++ = data;
	}

	if (count2)
	{
		caddr = (u32 *)csubs->runtime->dma_area;

		for (j = 0; j < count2 * adapter->capture_channels; j++)
		{
			data = mergeDatas[j + count1 * adapter->capture_channels];
			data = data >> 8;
			data = ntohl(data);

			*caddr++ = data;
		}
	}

	c_cnt += samples_per_packet;
	newcptr = (newcptr + samples_per_packet * adapter->capture_channels) % (adapter->capture_buffer_size * adapter->capture_channels);
	adapter->hw_capture_pointer = newcptr;

	// pr_info("adapter->capture_channels  = %d c_cnt = %d\n", adapter->capture_channels, c_cnt);

	if (capture && c_cnt >= adapter->capture_period_size)
	{
		adapter->c_cnt = c_cnt - adapter->capture_period_size;
		snd_pcm_period_elapsed(csubs);
	}
	else
	{
		adapter->c_cnt = c_cnt;
	}
}

void handle_tx_packet(struct igb_adapter *adapter)
{
	int j;
	u32 data;
	int tmp = adapter->rx_ring[1]->next_to_clean;
	int oldpptr = adapter->hw_playback_pointer;
	int newpptr = oldpptr;
	int samples_per_packet = adapter->samples_per_packet;
	struct snd_pcm_substream *psubs = ((struct snd_pcm *)adapter->pcm)->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	int playback = adapter->playback;
	int p_cnt = adapter->p_cnt;
	u32 presentation_time = 0;
	u32 launch_time_l = 0;
	u64 launch_time = 0;
	u32 *paddr = 0;

	// if (!playback || !psubs || !psubs->runtime || !psubs->runtime->dma_area)
	// 	return;

	if (psubs)
	{
		if (psubs->runtime)
		{
			if (psubs->runtime->dma_area)
				paddr = ((u32 *)psubs->runtime->dma_area) + oldpptr; // 4 bytes per sample
		}
	}

	// paddr = ((u32 *)psubs->runtime->dma_area) + oldpptr;

	for (int s = 0; s < NUMBER_OF_STREAMS; s++)
	{
		pr_info("tx_idx = %d, tmp = %d\n", ((tmp < NUMBER_OF_STREAMS ? MAXDESCRIPTORS - tmp - NUMBER_OF_STREAMS : tmp - NUMBER_OF_STREAMS)+ s) % MAXDESCRIPTORS, tmp);
		int tx_idx = ((tmp < NUMBER_OF_STREAMS ? MAXDESCRIPTORS - tmp - NUMBER_OF_STREAMS : tmp - NUMBER_OF_STREAMS) + s) % MAXDESCRIPTORS;
		AVPDU_HEADER *pavpdu = (AVPDU_HEADER *)(adapter->tx_addr + MAXPACKETSIZE * tx_idx);
		AVPDU_HEADER_NO_VLAN *cavpdu = (AVPDU_HEADER_NO_VLAN *)(adapter->rx_addr + MAXPACKETSIZE * tx_idx + 16);
		u64 paylen = (50 + channels * samples_per_packet * 4);

		int base_channel = s * 8;

		int tmp1 = oldpptr / adapter->playback_channels;
		int count1 = samples_per_packet;
		int count2 = 0;

		// printk(KERN_INFO "tmp1 = %d, oldpptr = %d, pbuffersize %d\n", tmp1, oldpptr, adapter->playback_buffer_size);

		if (playback && (tmp1 + samples_per_packet > adapter->playback_buffer_size))
		{
			count2 = tmp1 + samples_per_packet - adapter->playback_buffer_size;
			count1 = count1 - count2;
		}

		// Timestamp du paquet de réception
		// u32 rx_ts = ntohl(cavpdu->timestamp);

		// Présentation : ts + offset (future lecture côté RX)
		presentation_time = ntohl(cavpdu->timestamp) + PRESENTATION_TIME_OFFSET;
		presentation_time = htonl(presentation_time);

		// Launch time = rx_ts + 125us
		launch_time_l = (*(u32 *)(adapter->rx_addr + MAXPACKETSIZE * tx_idx + 8)) + LAUNCH_OFFSET;

		if (launch_time_l >= 1000000000)
		{
			launch_time_l = launch_time_l - 1000000000;
		}
		launch_time_l = launch_time_l >> 5;

		launch_time = ((u64)(launch_time_l)) << 32;
		launch_time = launch_time & 0x1ffffff00000000;

		// pr_info("cavpdu->flags1 = %u, presentation_time = %u, cavpdu->sequence_number = %u, cavpdu->data_block_continuity = %u\n", cavpdu->flags1, presentation_time, cavpdu->sequence_number, cavpdu->data_block_continuity);
		//  Préparation AVTP
		pavpdu->flags1 = cavpdu->flags1;
		pavpdu->timestamp = presentation_time;
		pavpdu->sequence_number = cavpdu->sequence_number;
		pavpdu->data_block_continuity = cavpdu->data_block_continuity;

		for (j = 0; j < count1; j++)
		{
			for (int c = 0; c < channels; c++)
			{
				int chan_offset = j * adapter->playback_channels;
				int index = chan_offset + c;
				// pr_info("runtime->buffer_size * runtime->channels = %d, index = %d, paddr[0] = %u\n", adapter->playback_buffer_size * adapter->playback_channels, index, paddr);
				int chan_index = (s * adapter->playback_channels) + base_channel + c + chan_offset;
					if ((adapter->debug_count % 2000) == 0)
					{
						pr_info("chan_index = %d, chan_offset = %d, chan_offset + c = %d\n", chan_index, chan_offset, chan_offset + c);
					}
				
				// u32 sample = paddr[chan_offset + c];
				//  u32 am824 = 0x40000000 | ((sample >> 8) & 0x00FFFFFF);
				//  pavpdu->audio_data[s * 8 + c] = htonl(am824);
				if (likely(playback))
				{
					//pr_info("(likely(playback))\n");

					if (chan_index >= adapter->playback_buffer_size * adapter->playback_channels)
					{
						pr_err("Playback buffer overflow: index=%d\n", index);
						data = 0;
					}
					else
					{
						//pr_info("else\n");
						data = paddr[chan_index] >> 8;
						// data = paddr[chan_offset + c] >> 8;
					}
				}
				else
				{
					data = 0;
				}
				//data = 0;
				data = data | 0x40000000;

				// data = 0;
				// data = data | 0x40000000;
				// data = 0x40000000 >> 8;
				// data |= 0x40000000;
				pavpdu->audio_data[j * 8 + c] = htonl(data);

				if ((adapter->debug_count++ % 2000) == 0)
				{
					pr_info("index = %d, data = 0x%08x \n", index, data);
				}
			}
		}

		// pr_info("tx_idx = %d, tmp = %d\n", tx_idx, tmp);
		struct igb_avb_tx_desc *tx_desc = (struct igb_avb_tx_desc *)adapter->tx_ring[0]->desc;

		tx_desc[tx_idx % (MAXDESCRIPTORS >> 1)].launch_time = launch_time;

		tx_desc[tx_idx % (MAXDESCRIPTORS >> 1)].cmd1 =
			E1000_ADVTXD_DCMD_DEXT | E1000_ADVTXD_DTYP_CTXT;

		tx_desc[tx_idx % (MAXDESCRIPTORS >> 1)].addr =
			adapter->tx_physaddr + MAXPACKETSIZE * tx_idx;

		tx_desc[tx_idx % (MAXDESCRIPTORS >> 1)].cmd2 =
			(paylen << 46) |
			E1000_ADVTXD_DCMD_EOP |
			E1000_ADVTXD_DCMD_RS |
			E1000_ADVTXD_DTYP_DATA |
			E1000_ADVTXD_DCMD_IFCS |
			E1000_ADVTXD_DCMD_DEXT |
			// E1000_ADVTXD_MAC_TSTAMP | // Si MAC timestamp dispo
			paylen;

		//writel(((tx_idx) << 1) % MAXDESCRIPTORS, adapter->tx_ring[0]->tail);
	}

	// Mise à jour du pointeur ALSA
	p_cnt += samples_per_packet;
	newpptr = (newpptr + samples_per_packet * adapter->playback_channels) %
			  (adapter->playback_buffer_size * adapter->playback_channels);
	adapter->hw_playback_pointer = newpptr;

	// Générer l'interruption ALSA si nécessaire
	if (p_cnt >= adapter->playback_period_size)
	{
		adapter->p_cnt = p_cnt - adapter->playback_period_size;
		snd_pcm_period_elapsed(psubs);
	}
	else
	{
		adapter->p_cnt = p_cnt;
	}

	writel(((tmp) << 1) % MAXDESCRIPTORS, adapter->tx_ring[0]->tail);
	// pr_info("tmp = %d\n", tmp);
}

void snd_avb_receive(struct igb_adapter *adapter)
{
	int j = 0;
	u_int32_t data;
	static int i = 0;
	int tmp = adapter->rx_ring[1]->next_to_clean;
	union e1000_adv_rx_desc *rx_desc = adapter->rx_ring[1]->desc;
	struct igb_avb_tx_desc *tx_desc = (struct igb_avb_tx_desc *)adapter->tx_ring[0]->desc;
	int samples_per_packet = adapter->samples_per_packet;

	while (rx_desc[tmp].wb.upper.status_error & 1)
	{
		AVPDU_HEADER_NO_VLAN *cavpdu;
		// AVPDU_HEADER *pavpdu;

		cavpdu = (AVPDU_HEADER_NO_VLAN *)(adapter->rx_addr + MAXPACKETSIZE * tmp + 16);
		// pavpdu = (AVPDU_HEADER *)(adapter->tx_addr + MAXPACKETSIZE * tmp);

		int capture_uniqueId = cavpdu->stream_id[7];
		// int playback_uniqueId = pavpdu->stream_id[7];

		if (cavpdu->sequence_number != adapter->last_seq_num)
		{
			adapter->last_seq_num = cavpdu->sequence_number;
		}

		cavpdus[capture_uniqueId] = cavpdu;
		// pavpdus[playback_uniqueId] = pavpdu;
		adapter->numberAccum++;

		if (capture_uniqueId >= NUMBER_OF_STREAMS - 1 && adapter->numberAccum >= NUMBER_OF_STREAMS)
		{
			for (int frame = 0; frame < samples_per_packet; frame++)
			{
				for (int pkt = 0; pkt < NUMBER_OF_STREAMS; pkt++)
				{
					// Chaque paquet contient 8 canaux par frame, soit 8 u32
					u32 *src = &cavpdus[pkt]->audio_data[frame * 8];
					u32 *dst = &mergeDatas[frame * 24 + pkt * 8];
					memcpy(dst, src, 8 * sizeof(u32));
				}
			}

			int numberSample = NUMBER_OF_STREAMS * samples_per_packet * channels;
			// if ((adapter->debug_count++ % 1000) == 0)
			// {
			// 	for (int i = 0; i < numberSample; i++)
			// 	{ // Afficher 8 premiers échantillons du paquet
			// 		u32 raw = ntohl(mergeDatas[i]);
			// 		int sample = (int)(raw >> 8); // Supposé AM824, données sur 24 bits
			// 		bool valid = raw & 0x40000000;
			// 		// if (valid && sample != 0)
			// 		// 	pr_info("AVTP audio[%d] = 0x%08x -> %d\n", i, raw, sample);
			// 	}
			// }
			if (adapter->capture)
				handle_rx_packet(adapter);

			handle_tx_packet(adapter);
			adapter->numberAccum = 0;
		}

		if (unlikely(!i++))
		{
			printk(KERN_INFO "In interrupt = %ld\n", in_interrupt());
		}

		rx_desc[tmp].read.pkt_addr = adapter->rx_physaddr + MAXPACKETSIZE * tmp;
		rx_desc[tmp].read.hdr_addr = 0;
		tmp = (tmp + 1) % MAXDESCRIPTORS;
	}

	// printk(" tmp = %d next_to_clean = %d, next_to_use = %d\n", tmp, (int)adapter->rx_ring[1]->next_to_clean, (int)adapter->rx_ring[1]->next_to_use);
	if (adapter->rx_ring[1]->next_to_clean != tmp)
	{
		// printk("adapter->rx_ring[1]->next_to_clean != tmp %d\n", tmp);
		adapter->rx_ring[1]->next_to_clean = tmp;
		// adapter->tx_ring[0]->next_to_clean = tmp;
		wmb();

		// pr_info("tmp en sortie = %d, adapter->tx_ring[0]->next_to_clean %u, adapter->rx_ring[1]->next_to_clean %u\n", tmp, adapter->tx_ring[0]->next_to_clean, adapter->rx_ring[1]->next_to_clean);
		writel((tmp) % MAXDESCRIPTORS, adapter->rx_ring[1]->tail);
		//writel(((tmp) << 1) % MAXDESCRIPTORS, adapter->tx_ring[0]->tail);
	}
}

void snd_avb_receive1(struct igb_adapter *adapter)
{
	int j = 0;
	u_int32_t data;
	static int i = 0;
	int tmp = adapter->rx_ring[1]->next_to_clean;
	union e1000_adv_rx_desc *rx_desc = adapter->rx_ring[1]->desc;
	struct igb_avb_tx_desc *tx_desc = (struct igb_avb_tx_desc *)adapter->tx_ring[0]->desc;

	int oldcptr = adapter->hw_capture_pointer;
	int oldpptr = adapter->hw_playback_pointer;
	int newcptr = oldcptr;
	int newpptr = oldpptr;

	int samples_per_packet = adapter->samples_per_packet;
	u32 presentation_time = 0;
	u32 launch_time_l = 0;
	u64 launch_time = 0;

	struct snd_pcm_substream *csubs = ((struct snd_pcm *)adapter->pcm)->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	struct snd_pcm_substream *psubs = ((struct snd_pcm *)adapter->pcm)->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;

	int playback = adapter->playback;
	int p_cnt = adapter->p_cnt;

	int capture = adapter->capture;
	int c_cnt = adapter->c_cnt;

	u32 *caddr = 0;
	u32 *paddr = 0;

	if (csubs)
	{
		if (csubs->runtime)
		{
			if (csubs->runtime->dma_area)
				caddr = ((u32 *)csubs->runtime->dma_area) + oldcptr; // 4 bytes per sample
		}
	}

	if (psubs)
	{
		if (psubs->runtime)
		{
			if (psubs->runtime->dma_area)
				paddr = ((u32 *)psubs->runtime->dma_area) + oldpptr; // 4 bytes per sample
		}
	}

	//     kernel_fpu_begin(); // possibly later use SSE/AVX optimised conversion here

	// printk("adapter->rx_accum.count avant while %d\n", adapter->rx_accum.count);
	while (rx_desc[tmp].wb.upper.status_error & 1)
	{
		AVPDU_HEADER_NO_VLAN *cavpdu;
		AVPDU_HEADER *pavpdu;

		cavpdu = (AVPDU_HEADER_NO_VLAN *)(adapter->rx_addr + MAXPACKETSIZE * tmp + 16);
		pavpdu = (AVPDU_HEADER *)(adapter->tx_addr + MAXPACKETSIZE * tmp);
		int capture_uniqueId = cavpdu->stream_id[7];
		printk("uniqueid = %d\n", capture_uniqueId);
		if (cavpdu->sequence_number != adapter->last_seq_num)
		{
			adapter->last_seq_num = cavpdu->sequence_number;
		}

		/* presentation time = presentation time of received packet - 375 usec */

		presentation_time = ntohl(cavpdu->timestamp) + PRESENTATION_TIME_OFFSET;
		presentation_time = htonl(presentation_time);

		/* transmit time = reception time of received packet + 125 usec */

		/* we calculate the launch time for the outgoing packets here,
		   but we currently don't use it
		*/

		launch_time_l = (*(u32 *)(adapter->rx_addr + MAXPACKETSIZE * tmp + 8)) + LAUNCH_OFFSET;

		if (launch_time_l >= 1000000000)
		{
			launch_time_l = launch_time_l - 1000000000;
		}
		launch_time_l = launch_time_l >> 5;

		launch_time = ((u64)(launch_time_l)) << 32;
		launch_time = launch_time & 0x1ffffff00000000;

		// printk("adapter->rx_accum.count en entrant %d\n", adapter->rx_accum.count);
		if (capture_uniqueId >= NUMBER_OF_STREAMS - 1)
		{
			// int sample = 0;
			// int indexStream = 0;
			// int chan = 0;
			// int dataByte = 0;
			// for(sample = 0; sample < samples_per_packet;sample++)
			// {
			//         for(indexStream = 0; indexStream < NUMBER_OF_STREAMS; indexStream++)
			//         {

			//                 for(chan = 0; chan < channels; chan++)
			//                 {
			//                         //printk("(sample * indexStream * channels) %d ((chan + (indexStream * channels) ) * 4) %d (indexStream * channels) %d\n", (sample * indexStream * channels), ((chan + (indexStream * channels) ) * 4), (indexStream * channels) );
			//                         for(dataByte = 0; dataByte < 4; dataByte++)
			//                         {
			//                                 //printk("index mergedatas = %d\n", (sample * indexStream * channels) + (chan + dataByte + (indexStream * channels)));
			//                                 mergeDatas[(sample * NUMBER_OF_STREAMS * channels) + (chan * 4) + (indexStream * channels ) + dataByte ] =
			//                                 cavpdus[indexStream]->audio_data[(chan * 4) + (indexStream * channels) + dataByte ];
			//                         }
			//                 }
			//         }
			// }

			printk("capture_uniqueId = %d\n", capture_uniqueId);
			for (int frame = 0; frame < samples_per_packet; frame++)
			{
				for (int pkt = 0; pkt < NUMBER_OF_STREAMS; pkt++)
				{
					u32 *src = &cavpdus[pkt]->audio_data[frame * 8 * 4];
					u32 *dst = &mergeDatas[(frame * 24 + pkt * 8) * 4];
					memcpy(dst, src, 8 * 4);
				}
			}

			// Print data as hexadecimal
			for (i = 0; i < 576; i++)
			{
				printk(KERN_CONT "%02x ", mergeDatas[i] & 0xFF);
				if ((i + 1) % 32 == 0) // Print 16 bytes per line
					printk(KERN_CONT "\n");
			}
			printk(KERN_CONT "\n");
			adapter->numberAccum = 0;

			if (capture)
			{
				int tmpCapture = oldcptr / adapter->capture_channels;
				int count1 = samples_per_packet;
				int count2 = 0;

				// printk(KERN_INFO "tmp = %d, oldcptr = %d, cbuffersize %d\n", tmp, oldpptr, adapter->capture_buffer_size);
				if (tmpCapture + samples_per_packet > adapter->capture_buffer_size)
				{
					count2 = tmpCapture + samples_per_packet - adapter->capture_buffer_size;
					count1 = count1 - count2;
				}

				for (j = 0; j < count1 * adapter->capture_channels; j++)
				{
					data = mergeDatas[j];
					data = data >> 8;
					data = ntohl(data);

					*caddr++ = data;
				}

				if (count2)
				{
					caddr = (u32 *)csubs->runtime->dma_area;

					for (j = 0; j < count2 * adapter->capture_channels; j++)
					{
						data = mergeDatas[j + count1 * adapter->capture_channels];
						data = data >> 8;
						data = ntohl(data);

						*caddr++ = data;
					}
				}

				c_cnt += samples_per_packet;
				newcptr = (newcptr + samples_per_packet * adapter->capture_channels) %
						  (adapter->capture_buffer_size * adapter->capture_channels);
				adapter->hw_capture_pointer = newcptr;
			}
			// // printk(KERN_INFO "count1 = %d, count2 = %d\n", count1, count2);
			// for (int i = 0; i < count1; i++)
			// {
			//         for (int s = 0; s < NUMBER_OF_STREAMS; s++)
			//         {

			//                 AVPDU_HEADER_NO_VLAN *cavpdu = (AVPDU_HEADER_NO_VLAN *)(adapter->rx_addr + MAXPACKETSIZE * s + 16);

			//                 for (int ch = 0; ch < channels; ch++)
			//                 {
			//                         data = cavpdu->audio_data[i * channels + ch];
			//                         data = ntohl(data) >> 8;
			//                         *caddr++ = data;
			//                 }
			//         }
			// }

			// if (count2)
			// {
			//         caddr = (u32 *)csubs->runtime->dma_area;
			//         for (int i = 0; i < count2; i++)
			//         {
			//                 for (int s = 0; s < NUMBER_OF_STREAMS; s++)
			//                 {

			//                         AVPDU_HEADER_NO_VLAN *cavpdu = (AVPDU_HEADER_NO_VLAN *)(adapter->rx_addr + MAXPACKETSIZE * s + 16);

			//                         for (int ch = 0; ch < channels; ch++)
			//                         {
			//                                 data = cavpdu->audio_data[(count1 + i) * channels + ch];
			//                                 data = ntohl(data) >> 8;
			//                                 *caddr++ = data;
			//                         }
			//                 }
			//         }
			// }

			// c_cnt += samples_per_packet;
			// newcptr = (newcptr + samples_per_packet * adapter->capture_channels) % (adapter->capture_buffer_size * adapter->capture_channels);
			// adapter->hw_capture_pointer = newcptr;

			if (1)
			{
				u64 paylen = (50 + channels * samples_per_packet * 4);

				int tmp1 = oldpptr / adapter->playback_channels;
				int count1 = samples_per_packet;
				int count2 = 0;

				// printk(KERN_INFO "tmp1 = %d, oldpptr = %d, pbuffersize %d\n", tmp1, oldpptr, adapter->playback_buffer_size);

				if (playback && (tmp1 + samples_per_packet > adapter->playback_buffer_size))
				{
					count2 = tmp1 + samples_per_packet - adapter->playback_buffer_size;
					count1 = count1 - count2;
				}
				// printk(KERN_INFO "count1 = %d, count2 = %d\n", count1, count2);

				pavpdu->flags1 = cavpdu->flags1;
				pavpdu->timestamp = presentation_time;
				pavpdu->sequence_number = cavpdu->sequence_number;
				pavpdu->data_block_continuity = cavpdu->data_block_continuity;

				for (j = 0; j < count1 * adapter->playback_channels; j++)
				{
					if (likely(playback))
					{
						data = (*paddr++) >> 8;
					}
					else
					{
						data = 0;
					}
					data = data | 0x40000000;

					pavpdu->audio_data[j] = htonl(data);
				}

				if (count2)
				{
					paddr = (u32 *)psubs->runtime->dma_area;

					for (j = 0; j < count2 * adapter->playback_channels; j++)
					{
						if (likely(playback))
						{
							data = (*paddr++) >> 8;
						}
						else
						{
							data = 0;
						}
						data = data | 0x40000000;

						pavpdu->audio_data[j + count1 * adapter->playback_channels] = htonl(data);
					}
				}

				// fill context descriptor

				tx_desc[tmp % (MAXDESCRIPTORS >> 1)].launch_time = launch_time;
				tx_desc[tmp % (MAXDESCRIPTORS >> 1)].cmd1 = E1000_ADVTXD_DCMD_DEXT | E1000_ADVTXD_DTYP_CTXT;

				// fill data descriptor

				tx_desc[tmp % (MAXDESCRIPTORS >> 1)].addr = adapter->tx_physaddr + MAXPACKETSIZE * tmp;
				tx_desc[tmp % (MAXDESCRIPTORS >> 1)].cmd2 = (paylen << 46) |
															E1000_ADVTXD_DCMD_EOP |
															E1000_ADVTXD_DCMD_RS |
															E1000_ADVTXD_DTYP_DATA |
															E1000_ADVTXD_DCMD_IFCS |
															E1000_ADVTXD_DCMD_DEXT |
															paylen;

				if (playback)
				{
					p_cnt += samples_per_packet;
					newpptr = (newpptr + samples_per_packet * adapter->playback_channels) %
							  (adapter->playback_buffer_size * adapter->playback_channels);
					adapter->hw_playback_pointer = newpptr;
				}
			}
		}
		else
		{
			// printk("adapter->numberAccum %d\n", adapter->numberAccum);
			// printk("streamId %d\n", cavpdu->stream_id[7]);
			cavpdus[capture_uniqueId] = cavpdu;
			// adapter->numberAccum++;
		}
		if (unlikely(!i++))
		{
			printk(KERN_INFO "In interrupt = %ld\n", in_interrupt());
		}

		rx_desc[tmp].read.pkt_addr = adapter->rx_physaddr + MAXPACKETSIZE * tmp;
		rx_desc[tmp].read.hdr_addr = 0;
		tmp = (tmp + 1) % MAXDESCRIPTORS;

		// printk("adapter->rx_accum.count en sortant %d\n", adapter->rx_accum.count);
		// adapter->rx_accum.count++;
	}

	//     kernel_fpu_end();

	if (capture && c_cnt >= adapter->capture_period_size)
	{
		// printk("snd_pcm_period_elapsed c_cnt %d, adapter->capture_period_size %d\n", c_cnt, adapter->capture_period_size);
		adapter->c_cnt = c_cnt - adapter->capture_period_size;
		snd_pcm_period_elapsed(csubs);
	}
	else
	{
		// printk("assign c_cnt %d\n", c_cnt);
		adapter->c_cnt = c_cnt;
	}

	if (playback && p_cnt >= adapter->playback_period_size)
	{
		adapter->p_cnt = p_cnt - adapter->playback_period_size;
		snd_pcm_period_elapsed(psubs);
	}
	else
	{
		adapter->p_cnt = p_cnt;
	}

	if (adapter->rx_ring[1]->next_to_clean != tmp)
	{
		printk("adapter->rx_ring[1]->next_to_clean != tmp %d\n", tmp);
		adapter->rx_ring[1]->next_to_clean = tmp;
		wmb();

		writel((tmp) % MAXDESCRIPTORS, adapter->rx_ring[1]->tail);
		writel(((tmp) << 1) % MAXDESCRIPTORS, adapter->tx_ring[0]->tail);
	}
}