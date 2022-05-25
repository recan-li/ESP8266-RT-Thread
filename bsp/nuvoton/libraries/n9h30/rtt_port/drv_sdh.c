/**************************************************************************//**
*
* @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*
* SPDX-License-Identifier: Apache-2.0
*
* Change Logs:
* Date            Author           Notes
* 2020-12-12      Wayne            First version
*
******************************************************************************/

#include <rtconfig.h>

#if defined(BSP_USING_SDH)

#include <rtdevice.h>
#include <string.h>
#include "NuMicro.h"
#include <drv_sys.h>

#include <dfs_fs.h>
#include <dfs_file.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/statfs.h>

/* Private define ---------------------------------------------------------------*/

#if defined(NU_SDH_MOUNT_ON_ROOT)

    #if !defined(NU_SDH_MOUNTPOINT_EMMC)
        #define NU_SDH_MOUNTPOINT_EMMC  "/"
    #endif

    #if !defined(NU_SDH_MOUNTPOINT_SDH0)
        #define NU_SDH_MOUNTPOINT_SDH0  NU_SDH_MOUNTPOINT_SDH0"/sd0"
    #endif

    #if !defined(NU_SDH_MOUNTPOINT_SDH1)
        #define NU_SDH_MOUNTPOINT_SDH1  NU_SDH_MOUNTPOINT_SDH0"/sd1"
    #endif

#else

    #if !defined(NU_SDH_MOUNTPOINT_ROOT)
        #define NU_SDH_MOUNTPOINT_ROOT  "/mnt"
    #endif

#endif

#if !defined(NU_SDH_MOUNTPOINT_EMMC)
    #define NU_SDH_MOUNTPOINT_EMMC  NU_SDH_MOUNTPOINT_ROOT"/emmc"
#endif

#if !defined(NU_SDH_MOUNTPOINT_SDH0)
    #define NU_SDH_MOUNTPOINT_SDH0  NU_SDH_MOUNTPOINT_ROOT"/sd0"
#endif

#if !defined(NU_SDH_MOUNTPOINT_SDH1)
    #define NU_SDH_MOUNTPOINT_SDH1  NU_SDH_MOUNTPOINT_ROOT"/sd1"
#endif

#if defined(BSP_USING_SDH0) && defined(BSP_USING_SDH1)
    #define NU_SDH_SHARED 1
#endif

enum
{
    SDH_START = -1,
#if defined(BSP_USING_EMMC)
    EMMC_IDX,
#endif
#if defined(BSP_USING_SDH0)
    SDH0_IDX,
#endif
#if defined(BSP_USING_SDH1)
    SDH1_IDX,
#endif
    SDH_CNT
};

#define SDH_BLOCK_SIZE   512ul

#if defined(NU_SDH_HOTPLUG)
    #define NU_SDH_TID_STACK_SIZE  1024
#endif

#if defined(NU_SDH_HOTPLUG)
typedef enum
{
    NU_SDH_CARD_DETECTED_EMMC = (1 << 0),
    NU_SDH_CARD_DETECTED_SD0  = (1 << 1),
    NU_SDH_CARD_DETECTED_SD1  = (1 << 2),
    NU_SDH_CARD_EVENT_ALL = (NU_SDH_CARD_DETECTED_EMMC | NU_SDH_CARD_DETECTED_SD0 | NU_SDH_CARD_DETECTED_SD1)
} E_CARD_EVENT;
#endif

/* Private typedef --------------------------------------------------------------*/
struct nu_sdh
{
    struct rt_device      dev;
    char                 *name;
#if defined(NU_SDH_HOTPLUG)
    char                 *mounted_point;
#endif
    SDH_T                *base;
    IRQn_Type             irqn;
    E_SYS_IPRST           rstidx;
    E_SYS_IPCLK           clkidx;

#if defined(NU_SDH_HOTPLUG)
    E_CARD_EVENT          card_detected_event;
#endif
    uint32_t              card_num;

    uint32_t              is_card_inserted;
    SDH_INFO_T           *info;
    struct rt_semaphore   lock;
    uint8_t              *pbuf;
};
typedef struct nu_sdh *nu_sdh_t;

#if defined(NU_SDH_HOTPLUG)
    static struct rt_thread sdh_tid;
    static rt_uint8_t sdh_stack[NU_SDH_TID_STACK_SIZE];
#endif

/* Private functions ------------------------------------------------------------*/
static rt_err_t nu_sdh_init(rt_device_t dev);
static rt_err_t nu_sdh_open(rt_device_t dev, rt_uint16_t oflag);
static rt_err_t nu_sdh_close(rt_device_t dev);
static rt_size_t nu_sdh_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t blk_nb);
static rt_err_t nu_sdh_control(rt_device_t dev, int cmd, void *args);
static int rt_hw_sdh_init(void);

#if defined(NU_SDH_HOTPLUG)
    static rt_bool_t nu_sdh_hotplug_is_mounted(const char *mounting_path);
    static void sdh_hotplugger(void *param);
    static rt_err_t nu_sdh_hotplug_mount(nu_sdh_t sdh);
    static rt_err_t nu_sdh_hotplug_unmount(nu_sdh_t sdh);
#endif

/* Public functions -------------------------------------------------------------*/


/* Private variables ------------------------------------------------------------*/
#if defined(BSP_USING_EMMC)
    static SDH_INFO_T EMMC;
#endif

#if defined(BSP_USING_SDH0)
    static SDH_INFO_T SD0;
#endif

#if defined(BSP_USING_SDH1)
    static SDH_INFO_T SD1;
#endif

#if defined(NU_SDH_SHARED)
    static struct rt_mutex   g_shared_lock;
#endif

static struct nu_sdh nu_sdh_arr [] =
{
#if defined(BSP_USING_EMMC)
    {
        .name = "emmc",
#if defined(NU_SDH_HOTPLUG)
        .mounted_point = NU_SDH_MOUNTPOINT_EMMC,
#endif
        .irqn = IRQ_FMI,
        .base = SDH0,
        .card_num = SD_PORT0,
        .rstidx = FMIRST,
        .clkidx = EMMCCKEN,
        .info = &EMMC,
        .card_detected_event = NU_SDH_CARD_DETECTED_EMMC,
    },
#endif

#if defined(BSP_USING_SDH0)
    {
        .name = "sdh0",
#if defined(NU_SDH_HOTPLUG)
        .mounted_point = NU_SDH_MOUNTPOINT_SDH0,
#endif
        .irqn = IRQ_SDH,
        .base = SDH1,
        .card_num = SD_PORT0,
        .rstidx = SDIORST,
        .clkidx = SDHCKEN,
        .info = &SD0,
        .card_detected_event = NU_SDH_CARD_DETECTED_SD0,
    },
#endif

#if defined(BSP_USING_SDH1)
    {
        .name = "sdh1",
#if defined(NU_SDH_HOTPLUG)
        .mounted_point = NU_SDH_MOUNTPOINT_SDH1,
#endif
        .base = SDH1,
        .card_num = SD_PORT1,
#if defined(NU_SDH_SHARED)
        .irqn = (IRQn_Type)0,
        .rstidx = SYS_IPRST_NA,
        .clkidx = SYS_IPCLK_NA,
#else
        .irqn = IRQ_SDH,
        .rstidx = SDIORST,
        .clkidx = SDHCKEN,
#endif
        .info = &SD1,
        .card_detected_event = NU_SDH_CARD_DETECTED_SD1,
    },
#endif
}; /* struct nu_sdh nu_sdh_arr [] */
static struct rt_event sdh_event;

static void SDH_IRQHandler(int vector, void *param)
{
    nu_sdh_t sdh = (nu_sdh_t)param;
    SDH_T *sdh_base = sdh->base;
    unsigned int volatile isr;
    SDH_INFO_T *pSD = sdh->info;

#if defined(BSP_USING_SDH1)
    if (SDH_WhichCardIsSelected(sdh_base) == SD_PORT1)
        pSD = &SD1;
#endif

    // FMI data abort interrupt
    if (sdh_base->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        sdh_base->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    isr = sdh_base->INTSTS;
    if (isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        pSD->DataReadyFlag = TRUE;
        SDH_CLR_INT_FLAG(sdh_base, SDH_INTSTS_BLKDIF_Msk);
    }

    if (isr & SDH_INTSTS_CDIF_Msk)   // card number=0 detect
    {
#if defined(NU_SDH_HOTPLUG)
        rt_event_send(&sdh_event, sdh->card_detected_event);
#endif
        /* Clear CDIF interrupt flag */
        SDH_CLR_INT_FLAG(sdh_base, SDH_INTSTS_CDIF_Msk);
    }
    else if (isr & SDH_INTSTS_CDIF1_Msk)   // card number=1 detect
    {
#if defined(NU_SDH_HOTPLUG)
        rt_event_send(&sdh_event, NU_SDH_CARD_DETECTED_SD1);
#endif
        /* Clear CDIF1 interrupt flag */
        SDH_CLR_INT_FLAG(sdh_base, SDH_INTSTS_CDIF1_Msk);
    }

    // CRC error interrupt
    if (isr & SDH_INTSTS_CRCIF_Msk)
    {
        if (!(isr & SDH_INTSTS_CRC16_Msk))
        {
            /* CRC_16 error */
            // TODO: handle CRC 16 error
        }
        else if (!(isr & SDH_INTSTS_CRC7_Msk))
        {
            if (!pSD->R3Flag)
            {
                /* CRC_7 error */
                // TODO: handle CRC 7 error
            }
        }
        /* Clear CRCIF interrupt flag */
        SDH_CLR_INT_FLAG(sdh_base, SDH_INTSTS_CRCIF_Msk);
    }

    /* Data-in timeout */
    if (isr & SDH_INTSTS_DITOIF_Msk)
    {
        sdh_base->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    /* Response-in timeout interrupt */
    if (isr & SDH_INTSTS_RTOIF_Msk)
    {
        sdh_base->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

/* RT-Thread Device Driver Interface */
static rt_err_t nu_sdh_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t nu_sdh_open(rt_device_t dev, rt_uint16_t oflag)
{
    nu_sdh_t sdh = (nu_sdh_t)dev;
    rt_err_t result = RT_EOK;

    RT_ASSERT(dev != RT_NULL);

#if defined(NU_SDH_SHARED)
    if (sdh->base == SDH1)
    {
        result = rt_mutex_take(&g_shared_lock, RT_WAITING_FOREVER);
        RT_ASSERT(result == RT_EOK);
    }
    SDH_CardSelect(sdh->base, sdh->info, sdh->card_num);
#endif

    if (SDH_Probe(sdh->base, sdh->info, sdh->card_num) == 0)
    {
        result = RT_EOK;
    }
    else
    {
        result = -RT_ERROR;
    }

#if defined(NU_SDH_SHARED)
    if (sdh->base == SDH1)
    {
        rt_mutex_release(&g_shared_lock);
    }
#endif

    return result;
}

static rt_err_t nu_sdh_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t nu_sdh_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t blk_nb)
{
    rt_err_t result = RT_ERROR;
    rt_uint32_t ret = 0;
    nu_sdh_t sdh = (nu_sdh_t)dev;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

#if defined(NU_SDH_SHARED)
    if (sdh->base == SDH1)
    {
        result = rt_mutex_take(&g_shared_lock, RT_WAITING_FOREVER);
        RT_ASSERT(result == RT_EOK);
    }
    SDH_CardSelect(sdh->base, sdh->info, sdh->card_num);
#endif

    result = rt_sem_take(&sdh->lock, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);

    /* Check alignment. */
    if (((uint32_t)buffer & 0x03) != 0)
    {
        /* Non-aligned. */
        uint32_t i;
        uint8_t *copy_buffer = (uint8_t *)buffer;

        sdh->pbuf = rt_malloc(SDH_BLOCK_SIZE);
        if (sdh->pbuf == RT_NULL)
            goto exit_nu_sdh_read;

        for (i = 0; i < blk_nb; i++)
        {
            /* Read to temp buffer from specified sector. */
            ret = SDH_Read(sdh->base, sdh->info, (uint8_t *)((uint32_t)&sdh->pbuf[0] | NONCACHEABLE), pos, 1);
            if (ret != Successful)
                goto exit_nu_sdh_read;

            /* Move to user's buffer */
            memcpy((void *)copy_buffer, (void *)&sdh->pbuf[0], SDH_BLOCK_SIZE);

            pos ++;
            copy_buffer += SDH_BLOCK_SIZE;
        }
    }
    else
    {
#if defined(BSP_USING_MMU)
        mmu_clean_invalidated_dcache((rt_uint32_t)buffer, SDH_BLOCK_SIZE * blk_nb);
#endif

        /* Read to user's buffer from specified sector. */
        ret = SDH_Read(sdh->base, sdh->info, (uint8_t *)((uint32_t)buffer | NONCACHEABLE), pos, blk_nb);
    }

exit_nu_sdh_read:

    if (sdh->pbuf)
    {
        rt_free(sdh->pbuf);
        sdh->pbuf = RT_NULL;
    }

    result = rt_sem_release(&sdh->lock);
    RT_ASSERT(result == RT_EOK);

#if defined(NU_SDH_SHARED)
    if (sdh->base == SDH1)
    {
        rt_mutex_release(&g_shared_lock);
    }
#endif

    if (ret == Successful)
        return blk_nb;

    rt_kprintf("Read failed: %d, buffer 0x%08x\n", ret, buffer);
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t nu_sdh_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t blk_nb)
{
    rt_err_t result = RT_ERROR;
    rt_uint32_t ret = 0;
    nu_sdh_t sdh = (nu_sdh_t)dev;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

#if defined(NU_SDH_SHARED)
    if (sdh->base == SDH1)
    {
        result = rt_mutex_take(&g_shared_lock, RT_WAITING_FOREVER);
        RT_ASSERT(result == RT_EOK);
    }
    SDH_CardSelect(sdh->base, sdh->info, sdh->card_num);
#endif

    result = rt_sem_take(&sdh->lock, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);

    /* Check alignment. */
    if (((uint32_t)buffer & 0x03) != 0)
    {
        /* Non-aligned. */
        uint32_t i;
        uint8_t *copy_buffer = (uint8_t *)buffer;

        sdh->pbuf = rt_malloc(SDH_BLOCK_SIZE);
        if (sdh->pbuf == RT_NULL)
            goto exit_nu_sdh_write;

        for (i = 0; i < blk_nb; i++)
        {
#if defined(BSP_USING_MMU)
            mmu_clean_invalidated_dcache((rt_uint32_t)copy_buffer, SDH_BLOCK_SIZE);
#endif

            memcpy((void *)&sdh->pbuf[0], copy_buffer, SDH_BLOCK_SIZE);

            ret = SDH_Write(sdh->base, sdh->info, (uint8_t *)((uint32_t)&sdh->pbuf[0] | NONCACHEABLE), pos, 1);
            if (ret != Successful)
                goto exit_nu_sdh_write;

            pos++;
            copy_buffer += SDH_BLOCK_SIZE;
        }
    }
    else
    {
#if defined(BSP_USING_MMU)
        mmu_clean_invalidated_dcache((rt_uint32_t)buffer, SDH_BLOCK_SIZE * blk_nb);
#endif

        /* Write to device directly. */
        ret = SDH_Write(sdh->base, sdh->info, (uint8_t *)((uint32_t)buffer | NONCACHEABLE), pos, blk_nb);
    }

exit_nu_sdh_write:

    if (sdh->pbuf)
    {
        rt_free(sdh->pbuf);
        sdh->pbuf = RT_NULL;
    }

    result = rt_sem_release(&sdh->lock);
    RT_ASSERT(result == RT_EOK);

#if defined(NU_SDH_SHARED)
    if (sdh->base == SDH1)
    {
        rt_mutex_release(&g_shared_lock);
    }
#endif

    if (ret == Successful) return blk_nb;

    rt_kprintf("write failed: %d, buffer 0x%08x\n", ret, buffer);
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t nu_sdh_control(rt_device_t dev, int cmd, void *args)
{
    nu_sdh_t sdh = (nu_sdh_t)dev;

    RT_ASSERT(dev != RT_NULL);

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        SDH_INFO_T *sdh_info = sdh->info;

        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = sdh_info->sectorSize;
        geometry->block_size = sdh_info->sectorSize;
        geometry->sector_count = sdh_info->totalSectorN;
    }

    return RT_EOK;
}


static int rt_hw_sdh_init(void)
{
    int i;
    rt_err_t ret = RT_EOK;
    rt_uint32_t flags = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE;

    ret = rt_event_init(&sdh_event, "sdh_event", RT_IPC_FLAG_FIFO);
    RT_ASSERT(ret == RT_EOK);

#if defined(NU_SDH_SHARED)
    ret = rt_mutex_init(&g_shared_lock, "sdh_share_lock", RT_IPC_FLAG_PRIO);
    RT_ASSERT(ret == RT_EOK);
#endif

#if defined(BSP_USING_EMMC)
    nu_sys_ipclk_enable(FMICKEN);
    nu_sys_ipclk_enable(NANDCKEN);
#endif

    for (i = (SDH_START + 1); i < SDH_CNT; i++)
    {
        /* Register sdcard device */
        nu_sdh_arr[i].dev.type  = RT_Device_Class_Block;
        nu_sdh_arr[i].dev.init  = nu_sdh_init;
        nu_sdh_arr[i].dev.open  = nu_sdh_open;
        nu_sdh_arr[i].dev.close = nu_sdh_close;
        nu_sdh_arr[i].dev.read  = nu_sdh_read;
        nu_sdh_arr[i].dev.write = nu_sdh_write;
        nu_sdh_arr[i].dev.control = nu_sdh_control;

        /* Private */
        nu_sdh_arr[i].dev.user_data = (void *)&nu_sdh_arr[i];

        ret = rt_sem_init(&nu_sdh_arr[i].lock, "sdhlock", 1, RT_IPC_FLAG_FIFO);
        RT_ASSERT(ret == RT_EOK);

        if (nu_sdh_arr[i].irqn != 0)
        {
            rt_hw_interrupt_install(nu_sdh_arr[i].irqn, SDH_IRQHandler, (void *)&nu_sdh_arr[i], nu_sdh_arr[i].name);
            rt_hw_interrupt_umask(nu_sdh_arr[i].irqn);
        }

        if (nu_sdh_arr[i].clkidx != SYS_IPCLK_NA)
        {
            nu_sys_ipclk_enable(nu_sdh_arr[i].clkidx);
        }

        if (nu_sdh_arr[i].rstidx != SYS_IPRST_NA)
        {
            nu_sys_ip_reset(nu_sdh_arr[i].rstidx);
        }

        nu_sdh_arr[i].pbuf = RT_NULL;

        ret = rt_device_register(&nu_sdh_arr[i].dev, nu_sdh_arr[i].name, flags);
        RT_ASSERT(ret == RT_EOK);
    }

    return (int)ret;
}
INIT_BOARD_EXPORT(rt_hw_sdh_init);

#if defined(NU_SDH_HOTPLUG)
static rt_bool_t nu_sdh_hotplug_is_mounted(const char *mounting_path)
{
    rt_bool_t ret = RT_FALSE;

    struct dfs_filesystem *psFS = dfs_filesystem_lookup(mounting_path);
    if (psFS == RT_NULL)
    {
        goto exit_nu_sdh_hotplug_is_mounted;
    }
    else if (!rt_memcmp(psFS->path, mounting_path, rt_strlen(mounting_path)))
    {
        ret = RT_TRUE;
    }
    else
    {
        ret = RT_FALSE;
    }

exit_nu_sdh_hotplug_is_mounted:

    return ret;
}
static rt_err_t nu_sdh_hotplug_mount(nu_sdh_t sdh)
{
    rt_err_t ret = RT_ERROR;
    DIR *t;

    if (nu_sdh_hotplug_is_mounted(sdh->mounted_point) == RT_TRUE)
    {
        ret = RT_EOK;
        goto exit_nu_sdh_hotplug_mount;
    }

    /* Check the SD folder path is valid. */
    if ((t =  opendir(sdh->mounted_point)) != RT_NULL)
    {
        closedir(t);
    }
#if !defined(NU_SDH_MOUNT_ON_ROOT)
    else
    {

        /* Check the ROOT path is valid. */
        if ((t =  opendir(NU_SDH_MOUNTPOINT_ROOT)) != RT_NULL)
        {
            closedir(t);
        }
        else if ((ret = mkdir(NU_SDH_MOUNTPOINT_ROOT, 0)) != RT_EOK)
        {
            rt_kprintf("Failed to mkdir %s\n", NU_SDH_MOUNTPOINT_ROOT);
            goto exit_nu_sdh_hotplug_mount;
        }

        if ((ret = mkdir(sdh->mounted_point, 0)) != RT_EOK)
        {
            rt_kprintf("Failed to mkdir %s\n", sdh->mounted_point);
            goto exit_nu_sdh_hotplug_mount;
        }

    } //else
#endif

    if ((ret = dfs_mount(sdh->name, sdh->mounted_point, "elm", 0, 0)) == 0)
    {
        rt_kprintf("Mounted %s on %s\n", sdh->name, sdh->mounted_point);
    }
    else
    {
        rt_kprintf("Failed to mount %s on %s\n", sdh->name, sdh->mounted_point);
        ret = RT_ERROR;
    }

exit_nu_sdh_hotplug_mount:

    return -(ret);
}

static rt_err_t nu_sdh_hotplug_unmount(nu_sdh_t sdh)
{
    rt_err_t ret = RT_ERROR;

    if (nu_sdh_hotplug_is_mounted(sdh->mounted_point) == RT_FALSE)
    {
        ret = RT_EOK;
        goto exit_nu_sdh_hotplug_unmount;
    }

    ret = dfs_unmount(sdh->mounted_point);
    if (ret != RT_EOK)
    {
        rt_kprintf("Failed to unmount %s.\n", sdh->mounted_point);
    }
    else
    {
        rt_kprintf("Succeed to unmount %s.\n", sdh->mounted_point);
        ret = RT_EOK;
    }

exit_nu_sdh_hotplug_unmount:

    return -(ret);
}

static void nu_card_detector(nu_sdh_t sdh)
{
    SDH_T *sdh_base = sdh->base;
    uint32_t u32INTSTS_CDSTS_Msk = (sdh->card_num == SD_PORT0) ? SDH_INTSTS_CDSTS_Msk : SDH_INTSTS_CDSTS1_Msk;
    unsigned int volatile isr = sdh_base->INTSTS;

    if (isr & u32INTSTS_CDSTS_Msk)
    {
        /* Card removed */
        sdh->info->IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
        rt_memset((void *)sdh->info, 0, sizeof(SDH_INFO_T));
        nu_sdh_hotplug_unmount(sdh);
    }
    else
    {
#if defined(NU_SDH_SHARED)
        if (sdh_base == SDH1)
        {
            rt_err_t result = rt_mutex_take(&g_shared_lock, RT_WAITING_FOREVER);
            RT_ASSERT(result == RT_EOK);
        }
        SDH_CardSelect(sdh->base, sdh->info, sdh->card_num);
#endif

        SDH_Open(sdh_base, sdh->info, CardDetect_From_GPIO | sdh->card_num);
        if (!SDH_Probe(sdh_base, sdh->info, sdh->card_num))
        {
            /* Card inserted */
            nu_sdh_hotplug_mount(sdh);
        }
#if defined(NU_SDH_SHARED)
        if (sdh_base == SDH1)
        {
            rt_mutex_release(&g_shared_lock);
        }
#endif
    }
}

static void sdh_hotplugger(void *param)
{
    rt_uint32_t e;
    int i;

    for (i = (SDH_START + 1); i < SDH_CNT; i++)
    {
#if defined(NU_SDH_SHARED)
        if (nu_sdh_arr[i].base == SDH1)
        {
            rt_err_t result = rt_mutex_take(&g_shared_lock, RT_WAITING_FOREVER);
            RT_ASSERT(result == RT_EOK);
        }
        SDH_CardSelect(nu_sdh_arr[i].base, nu_sdh_arr[i].info, nu_sdh_arr[i].card_num);
#endif

        /* Try to detect SD card on selected port. */
        SDH_Open(nu_sdh_arr[i].base, nu_sdh_arr[i].info, CardDetect_From_GPIO | nu_sdh_arr[i].card_num);
        if (!SDH_Probe(nu_sdh_arr[i].base, nu_sdh_arr[i].info, nu_sdh_arr[i].card_num) &&
                nu_sdh_arr[i].info->IsCardInsert)
        {
            nu_sdh_hotplug_mount(&nu_sdh_arr[i]);
        }

#if defined(NU_SDH_SHARED)
        if (nu_sdh_arr[i].base == SDH1)
        {
            rt_mutex_release(&g_shared_lock);
        }
#endif
    }

    while (1)
    {
        if (rt_event_recv(&sdh_event, (NU_SDH_CARD_EVENT_ALL),
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &e) == RT_EOK)
        {
            /* Debounce */
            rt_thread_mdelay(500);
            switch (e)
            {
#if defined(BSP_USING_EMMC)
            case NU_SDH_CARD_DETECTED_EMMC:
                nu_card_detector(&nu_sdh_arr[EMMC_IDX]);
                break;
#endif
#if defined(BSP_USING_SDH0)
            case NU_SDH_CARD_DETECTED_SD0:
                nu_card_detector(&nu_sdh_arr[SDH0_IDX]);
                break;
#endif
#if defined(BSP_USING_SDH1)
            case NU_SDH_CARD_DETECTED_SD1:
                nu_card_detector(&nu_sdh_arr[SDH1_IDX]);
                break;
#endif
            default:
                break;

            } //switch(e)

        } //if

    } /* while(1) */
}

int mnt_init_sdcard_hotplug(void)
{
    rt_err_t ret = RT_EOK;

    ret = rt_thread_init(&sdh_tid, "hotplug", sdh_hotplugger, NULL, sdh_stack, sizeof(sdh_stack), RT_THREAD_PRIORITY_MAX - 2, 10);
    RT_ASSERT(ret == RT_EOK);

    ret = rt_thread_startup(&sdh_tid);
    RT_ASSERT(ret == RT_EOK);

    return 0;
}
INIT_ENV_EXPORT(mnt_init_sdcard_hotplug);
#endif

#endif //#if defined(BSP_USING_SDH)
