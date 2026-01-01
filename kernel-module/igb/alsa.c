
#include "igb.h"
#include "avpdu.h"
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <asm/fpu/api.h>
#include "avb-config.h"
#include <linux/timecounter.h>
#include <linux/ktime.h>

/* SNDRV_CARDS: maximum number of cards supported by this module */

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;

AVPDU_HEADER_NO_VLAN *cavpdus[NUMBER_OF_STREAMS];

static int dev = 0;
u_int8_t channels = 8;
int diag_counter = 0;

u32 mergeCaptureDatas[24 * 8 * NUMBER_OF_STREAMS * 4]; // MAX sample rate * channels per stream * number of streams * byte per sample

// #define DMA_BUFFERSIZE (768*4)   /* 4*8*6*8*2 */
// #define DMA_PERIODSIZE (384*4)   /* 4*8*6*8 */

#define DMA_BUFFERSIZE (6 * 8 * 2 * 8)

#define MAXPACKETSIZE 1024
#define MAXDESCRIPTORS 256
#define LAUNCH_OFFSET 125000            /* nanoseconds */
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
    // On écrit les 4 premiers octets d'un coup
    *(u32 *)(dest)     = htonl((u32)(src >> 16));
    // On écrit les 2 derniers octets
    *(u16 *)(dest + 4) = htons((u16)(src & 0xFFFF));
}

void u64_to_array8(u64 src, u8 *dest)
{
    // On convertit et on écrit 8 octets d'un seul coup
    // Le compilateur transforme cela en une seule instruction MOV
    *(u64 *)dest = __builtin_bswap64(src);
}

/* open callback */
static int snd_avb_playback_open(struct snd_pcm_substream *substream)
{
    struct igb_adapter *adapter = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;

    //        printk(KERN_INFO "open playback\n");

    runtime->hw = snd_avb_playback_hw;
    adapter->playback = 0;
    adapter->last_system_ns = 0;

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
        udelay(600);
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
        udelay(600);
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
    int j;

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

    // /* initialize transmit buffers for each streams after that no header change, just data*/

    memset(adapter->tx_addr, 0, 4096 * 16 * 4);

    for (i = 0; i < MAXDESCRIPTORS; i++)
    {
        j = i % NUMBER_OF_STREAMS;

        uint8_t sid[8];
        memcpy(sid, stream_id, 8);
        sid[7] = j;

        uint8_t dst[6];
        memcpy(dst, dest, 6);
        dst[5] = j;

        void *p = adapter->tx_addr;
        init_avpdu_header(p + i * 1024, dst, src, sid, channels, adapter->rate);
    }

    return 0;
}

int snd_avb_probe(struct igb_adapter *adapter, int samplerate)
{
    // Dans votre fonction de configuration ou au moment du start
    adapter->tx_seq = kcalloc(NUMBER_OF_STREAMS, sizeof(u8), GFP_KERNEL);
    adapter->tx_dbc = kcalloc(NUMBER_OF_STREAMS, sizeof(u8), GFP_KERNEL);

    if (!adapter->tx_seq || !adapter->tx_dbc)
    {
        // Gérer l'erreur d'allocation ici
        return -ENOMEM;
    }
    struct snd_card *card;

    int err;

    u8 filter[] =
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Destination MAC used by the input AVB stream */
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Source MAC (of AVB device) */
         0x81, 0x00, 0x60, 0x02, 0x22, 0xf0,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    u8 filter_mask[] = {0xcf, 0xff, 0x03}; // this filter pass all streams(uniqueId) from destination MAC (AVB_DEVICE_TALKER_MAC_BASE)

    adapter->card = 0;
    adapter->capture = 0;
    adapter->playback = 0;

    // force number of channels in app
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

    u64_to_array6(AVB_DEVICE_TALKER_MAC_BASE, &filter[0]); // 6 first bytes from AVTP packet
    u64_to_array6(AVB_DEVICE_SOURCE_MAC, &filter[6]);      // next 6 bytes

    // if other source, used multiple filters max of 8
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
    adapter->last_system_ns = 0;

    udelay(600);

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
    int oldcptr = adapter->hw_capture_pointer;
    int samples_per_packet = adapter->samples_per_packet;
    int capture_channels = adapter->capture_channels;
    struct snd_pcm_substream *csubs = ((struct snd_pcm *)adapter->pcm)->streams[SNDRV_PCM_STREAM_CAPTURE].substream;

    if (!adapter->capture || !csubs || !csubs->runtime || !csubs->runtime->dma_area)
        return;

    u32 *dma_base = (u32 *)csubs->runtime->dma_area;
    u32 *caddr = dma_base + oldcptr;
    u32 *src = mergeCaptureDatas;

    int tmp_idx = oldcptr / capture_channels;
    int count1 = samples_per_packet;
    int count2 = 0;

    // Calcul du wrap-around
    if (tmp_idx + samples_per_packet > adapter->capture_buffer_size)
    {
        count2 = tmp_idx + samples_per_packet - adapter->capture_buffer_size;
        count1 = samples_per_packet - count2;
    }

    // PARTIE 1 : Copie linéaire jusqu'à la fin du buffer
    // On multiplie par capture_channels une seule fois
    int total_samples1 = count1 * capture_channels;
    for (j = 0; j < total_samples1; j++)
    {
        // Optimisation : Shift puis BSWAP (ntohl)
        // Le compilateur remplacera cela par une instruction processeur unique (ROR/BSWAP)
        *caddr++ = be32_to_cpu(*src++ >> 8);
    }

    // PARTIE 2 : Wrap-around (retour au début du buffer DMA ALSA)
    if (unlikely(count2 > 0))
    {
        caddr = dma_base; // Retour au début de dma_area
        int total_samples2 = count2 * capture_channels;
        for (j = 0; j < total_samples2; j++)
        {
            *caddr++ = be32_to_cpu(*src++ >> 8);
        }
    }

    // Mise à jour des compteurs
    adapter->c_cnt += samples_per_packet;
    adapter->hw_capture_pointer = (oldcptr + samples_per_packet * capture_channels) %
                                  (adapter->capture_buffer_size * capture_channels);

    if (adapter->c_cnt >= adapter->capture_period_size)
    {
        adapter->c_cnt -= adapter->capture_period_size;
        snd_pcm_period_elapsed(csubs);
    }
}

u64 get_ptp_time_ns(struct igb_adapter *adapter)
{
    struct e1000_hw *hw = &adapter->hw;
    u32 lo, hi, hi2;
    u64 hw_time;

    // 1. Tentative de lecture Hardware (Direct Register)
    hi = rd32(E1000_SYSTIMH);
    lo = rd32(E1000_SYSTIML);
    hi2 = rd32(E1000_SYSTIMH);

    if (unlikely(hi != hi2))
    {
        lo = rd32(E1000_SYSTIML);
        hi = hi2;
    }

    hw_time = ((u64)hi << 32) | lo;

    // 2. Vérification du Fallback
    // Si le hardware renvoie 0 ou une valeur suspecte (ex: carte non initialisée)
    if (unlikely(hw_time == 0))
    {
        // Fallback sur l'horloge système (plus lent, mais sauve l'audio)
        return ktime_get_real_ns();
    }

    return hw_time;
}

u64 get_safe_avb_time(struct igb_adapter *adapter)
{
    u64 now_ptp = get_ptp_time_ns(adapter);
    u64 now_system = ktime_get_ns();
    u64 delta_system = now_system - adapter->last_system_ns;
    u64 delta_ptp = now_ptp - adapter->last_ptp_ns;
    u64 result_time;

    // Initialisation au premier appel
    if (unlikely(adapter->last_system_ns == 0)) {
        adapter->last_system_ns = now_system;
        adapter->last_ptp_ns = now_ptp;
        return now_ptp;
    }

    // Seuil de tolérance : le temps PTP doit avoir avancé d'au moins 20% 
    // et pas plus de 500% par rapport au temps système.
    // À 0.3ms, on est très sensible au jitter.
    if (unlikely(delta_ptp < (delta_system >> 2) || delta_ptp > (delta_system << 2))) {
        // --- MODE FREEWHEEL (Secours) ---
        result_time = adapter->last_ptp_ns + delta_system;
        adapter->freewheel_count++;
    } else {
        // --- MODE NORMAL ---
        result_time = now_ptp;
        adapter->freewheel_count = 0;
    }

    // Sauvegarde pour le prochain cycle
    adapter->last_ptp_ns = result_time;
    adapter->last_system_ns = now_system;

    return result_time;
}

void handle_tx_packet(struct igb_adapter *adapter)
{
    int i, j, s;
    u32 data;
    // Utilisation du pointeur TX interne pour la linéarité
    int next_to_use = adapter->tx_ring[0]->next_to_use;
    int oldpptr = adapter->hw_playback_pointer;
    int samples_per_packet = adapter->samples_per_packet;
    struct snd_pcm_substream *psubs = NULL;
    u32 *paddr = NULL;
    static u64 last_ptp_time = 0;

    if (adapter->pcm && adapter->playback)
    {
        psubs = ((struct snd_pcm *)adapter->pcm)->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
        if (psubs && psubs->runtime)
            paddr = (u32 *)psubs->runtime->dma_area;
    }

    u64 safe_now = get_safe_avb_time(adapter);

    u32 pres_time = cpu_to_be32((u32)((safe_now + 300000) & 0xFFFFFFFF));

    // On remplit le batch de streams de manière ordonnée (0, 1, 2, 3...)
    for (s = 0; s < NUMBER_OF_STREAMS; s++)
    {
        // L'index physique suit strictement l'ordre des streams
        int tx_idx = (next_to_use + s) % MAXDESCRIPTORS;

        AVPDU_HEADER *pavpdu = (AVPDU_HEADER *)(adapter->tx_addr + MAXPACKETSIZE * tx_idx);

        // --- GESTION DES COMPTEURS AVB ---
        pavpdu->sequence_number = adapter->tx_seq[s]++;
        pavpdu->data_block_continuity = adapter->tx_dbc[s];
        adapter->tx_dbc[s] = (adapter->tx_dbc[s] + samples_per_packet) % 256;

        pavpdu->timestamp = pres_time;
        pavpdu->flags1 = 0x02; // AVTP subtype audio
        pavpdu->flags2 = 0x01; // Gateway info / valid

        // Mapping ALSA : s=0 est Canal 1-8, s=1 est 9-16...
        int base_channel = s * 8;

        // --- GESTION AUDIO AVEC WRAP-AROUND ---
        int start_sample = oldpptr / adapter->playback_channels;
        int count1 = samples_per_packet;
        int count2 = 0;

        if (adapter->playback && (start_sample + samples_per_packet > adapter->playback_buffer_size))
        {
            count2 = start_sample + samples_per_packet - adapter->playback_buffer_size;
            count1 = samples_per_packet - count2;
        }

        // --- PRÉPARATION HORS BOUCLE ---
        u32 *dst = (u32 *)pavpdu->audio_data;
        int stride = adapter->playback_channels;
        u32 flags = 0x40000000;
        int skip = stride - 8;

        if (paddr && adapter->playback)
        {
            // PARTIE 1 : Jusqu'à la fin du buffer (count1)
            u32 *src1 = paddr + (start_sample * stride) + base_channel;
            for (j = 0; j < count1; j++)
            {
                dst[0] = htonl((*src1++ >> 8) | flags);
                dst[1] = htonl((*src1++ >> 8) | flags);
                dst[2] = htonl((*src1++ >> 8) | flags);
                dst[3] = htonl((*src1++ >> 8) | flags);
                dst[4] = htonl((*src1++ >> 8) | flags);
                dst[5] = htonl((*src1++ >> 8) | flags);
                dst[6] = htonl((*src1++ >> 8) | flags);
                dst[7] = htonl((*src1++ >> 8) | flags);
                src1 += skip;
                dst += 8;
            }

            // PARTIE 2 : Wrap-around depuis le début du buffer (count2)
            if (unlikely(count2 > 0))
            {
                u32 *src2 = paddr + base_channel; // Repart de l'index 0 du buffer + offset stream
                for (j = 0; j < count2; j++)
                {
                    dst[0] = htonl((*src2++ >> 8) | flags);
                    dst[1] = htonl((*src2++ >> 8) | flags);
                    dst[2] = htonl((*src2++ >> 8) | flags);
                    dst[3] = htonl((*src2++ >> 8) | flags);
                    dst[4] = htonl((*src2++ >> 8) | flags);
                    dst[5] = htonl((*src2++ >> 8) | flags);
                    dst[6] = htonl((*src2++ >> 8) | flags);
                    dst[7] = htonl((*src2++ >> 8) | flags);
                    src2 += skip;
                    dst += 8;
                }
            }
        }
        else
        {
            // Silence rapide si pas de playback
            memset(pavpdu->audio_data, 0, samples_per_packet * 8 * 4);
        }

        // --- DESCRIPTEUR MATÉRIEL ---
        struct igb_avb_tx_desc *tx_desc = &((struct igb_avb_tx_desc *)adapter->tx_ring[0]->desc)[tx_idx % (MAXDESCRIPTORS >> 1)];
        u64 paylen = (u64)(50 + 8 * samples_per_packet * 4);

        tx_desc->launch_time = 0; // Mode ASAP pour laisser le shaper lisser
        tx_desc->addr = cpu_to_le64(adapter->tx_physaddr + MAXPACKETSIZE * tx_idx);
        tx_desc->cmd1 = cpu_to_le64(E1000_ADVTXD_DCMD_DEXT | E1000_ADVTXD_DTYP_CTXT);
        tx_desc->cmd2 = cpu_to_le64((paylen << 46) | E1000_ADVTXD_DCMD_EOP | E1000_ADVTXD_DCMD_RS |
                                    E1000_ADVTXD_DTYP_DATA | E1000_ADVTXD_DCMD_IFCS | paylen);
    }

    // Mise à jour pointeur ALSA
    if (adapter->playback)
    {
        adapter->hw_playback_pointer = (oldpptr + samples_per_packet * adapter->playback_channels) %
                                       (adapter->playback_buffer_size * adapter->playback_channels);
        adapter->p_cnt += samples_per_packet;
        if (adapter->p_cnt >= adapter->playback_period_size)
        {
            adapter->p_cnt -= adapter->playback_period_size;
            snd_pcm_period_elapsed(psubs);
        }
    }

    // Avancer le pointeur de gestion TX
    adapter->tx_ring[0]->next_to_use = (next_to_use + NUMBER_OF_STREAMS) % MAXDESCRIPTORS;

    // Notification à la carte réseau (Tail Update)
    writel(((adapter->tx_ring[0]->next_to_use) << 1) % MAXDESCRIPTORS, adapter->tx_ring[0]->tail);
}

void snd_avb_receive(struct igb_adapter *adapter)
{
    int tmp = adapter->rx_ring[1]->next_to_clean;
    union e1000_adv_rx_desc *rx_desc = adapter->rx_ring[1]->desc;
    int samples_per_packet = adapter->samples_per_packet;

    // On traite tout ce qui est disponible dans le ring RX
    while (rx_desc[tmp].wb.upper.status_error & 1)
    {
        AVPDU_HEADER_NO_VLAN *cavpdu;
        cavpdu = (AVPDU_HEADER_NO_VLAN *)(adapter->rx_addr + MAXPACKETSIZE * tmp + 16);

        // On remplit le tableau cavpdus de manière séquentielle
        cavpdus[adapter->numberAccum] = cavpdu;
        adapter->numberAccum++;

        // Quand on a un batch complet (peu importe les seq num, on veut juste la cadence)
        if (adapter->numberAccum >= NUMBER_OF_STREAMS)
        {
            // --- CAPTURE ---
            if (adapter->capture)
            {
                int frame, pkt;
                for (frame = 0; frame < samples_per_packet; frame++)
                {
                    for (pkt = 0; pkt < NUMBER_OF_STREAMS; pkt++)
                    {
                        u32 *src = &cavpdus[pkt]->audio_data[frame * channels];
                        u32 *dst = &mergeCaptureDatas[frame * (NUMBER_OF_STREAMS * channels) + pkt * channels];
                        memcpy(dst, src, channels * sizeof(u32));
                    }
                }
                handle_rx_packet(adapter);
            }

            // --- TRANSMISSION ---
            // Le TX est déclenché par l'arrivée du batch RX (métronome)
            handle_tx_packet(adapter);

            // On reset l'accumulateur
            adapter->numberAccum = 0;
        }

        // Nettoyage et préparation du descripteur pour le prochain paquet
        rx_desc[tmp].wb.upper.status_error = 0; // Crucial pour le prochain tour
        rx_desc[tmp].read.pkt_addr = adapter->rx_physaddr + MAXPACKETSIZE * tmp;
        rx_desc[tmp].read.hdr_addr = 0;

        tmp = (tmp + 1) % MAXDESCRIPTORS;
    }

    if (adapter->rx_ring[1]->next_to_clean != tmp)
    {
        adapter->rx_ring[1]->next_to_clean = tmp;
        wmb();
        writel(tmp % MAXDESCRIPTORS, adapter->rx_ring[1]->tail);
    }
}