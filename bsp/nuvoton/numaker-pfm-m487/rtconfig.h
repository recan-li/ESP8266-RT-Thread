#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_HOOK_USING_FUNC_PTR
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 1024

/* kservice optimization */

#define RT_DEBUG
#define RT_DEBUG_COLOR

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE
#define RT_USING_SIGNALS

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_SMALL_MEM
#define RT_USING_SMALL_MEM_AS_HEAP
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 256
#define RT_CONSOLE_DEVICE_NAME "uart0"
#define RT_VER_NUM 0x40100
#define ARCH_ARM
#define RT_USING_CPU_FFS
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M4

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10
#define RT_USING_MSH
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 2048
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_CMD_SIZE 80
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_USING_DESCRIPTION
#define FINSH_ARG_MAX 10
#define RT_USING_DFS
#define DFS_USING_POSIX
#define DFS_USING_WORKDIR
#define DFS_FILESYSTEMS_MAX 8
#define DFS_FILESYSTEM_TYPES_MAX 4
#define DFS_FD_MAX 32
#define RT_USING_DFS_ELMFAT

/* elm-chan's FatFs, Generic FAT Filesystem Module */

#define RT_DFS_ELM_CODE_PAGE 437
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN_3
#define RT_DFS_ELM_USE_LFN 3
#define RT_DFS_ELM_LFN_UNICODE_0
#define RT_DFS_ELM_LFN_UNICODE 0
#define RT_DFS_ELM_MAX_LFN 255
#define RT_DFS_ELM_DRIVES 8
#define RT_DFS_ELM_MAX_SECTOR_SIZE 4096
#define RT_DFS_ELM_REENTRANT
#define RT_DFS_ELM_MUTEX_TIMEOUT 3000
#define RT_USING_DFS_DEVFS
#define RT_USING_FAL
#define FAL_DEBUG_CONFIG
#define FAL_DEBUG 1
#define FAL_PART_HAS_TABLE_CFG

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_USING_SYSTEM_WORKQUEUE
#define RT_SYSTEM_WORKQUEUE_STACKSIZE 2048
#define RT_SYSTEM_WORKQUEUE_PRIORITY 23
#define RT_USING_SERIAL
#define RT_USING_SERIAL_V1
#define RT_SERIAL_USING_DMA
#define RT_SERIAL_RB_BUFSZ 128
#define RT_USING_CAN
#define RT_USING_HWTIMER
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS
#define RT_USING_PIN
#define RT_USING_ADC
#define RT_USING_PWM
#define RT_USING_PM
#define PM_TICKLESS_THRESHOLD_TIME 2
#define RT_USING_RTC
#define RT_USING_SPI
#define RT_USING_QSPI
#define RT_USING_SFUD
#define RT_SFUD_USING_SFDP
#define RT_SFUD_USING_FLASH_INFO_TABLE
#define RT_SFUD_SPI_MAX_HZ 50000000
#define RT_USING_WDT
#define RT_USING_AUDIO
#define RT_AUDIO_REPLAY_MP_BLOCK_SIZE 4096
#define RT_AUDIO_REPLAY_MP_BLOCK_COUNT 2
#define RT_AUDIO_RECORD_PIPE_SIZE 2048
#define RT_USING_HWCRYPTO
#define RT_HWCRYPTO_DEFAULT_NAME "hwcryto"
#define RT_HWCRYPTO_IV_MAX_SIZE 16
#define RT_HWCRYPTO_KEYBIT_MAX_SIZE 256
#define RT_HWCRYPTO_USING_AES
#define RT_HWCRYPTO_USING_AES_ECB
#define RT_HWCRYPTO_USING_AES_CBC
#define RT_HWCRYPTO_USING_AES_CFB
#define RT_HWCRYPTO_USING_AES_CTR
#define RT_HWCRYPTO_USING_AES_OFB
#define RT_HWCRYPTO_USING_DES
#define RT_HWCRYPTO_USING_DES_ECB
#define RT_HWCRYPTO_USING_DES_CBC
#define RT_HWCRYPTO_USING_3DES
#define RT_HWCRYPTO_USING_3DES_ECB
#define RT_HWCRYPTO_USING_3DES_CBC
#define RT_HWCRYPTO_USING_SHA1
#define RT_HWCRYPTO_USING_SHA2
#define RT_HWCRYPTO_USING_SHA2_224
#define RT_HWCRYPTO_USING_SHA2_256
#define RT_HWCRYPTO_USING_SHA2_384
#define RT_HWCRYPTO_USING_SHA2_512
#define RT_HWCRYPTO_USING_RNG
#define RT_HWCRYPTO_USING_CRC
#define RT_HWCRYPTO_USING_CRC_07
#define RT_HWCRYPTO_USING_CRC_8005
#define RT_HWCRYPTO_USING_CRC_1021
#define RT_HWCRYPTO_USING_CRC_04C11DB7

/* Using USB */

#define RT_USING_USB
#define RT_USING_USB_HOST
#define RT_USBH_MSTORAGE
#define UDISK_MOUNTPOINT "/mnt/udisk/"
#define RT_USING_USB_DEVICE
#define RT_USBD_THREAD_STACK_SZ 4096
#define USB_VENDOR_ID 0x0FFE
#define USB_PRODUCT_ID 0x0001
#define _RT_USB_DEVICE_HID
#define RT_USB_DEVICE_HID
#define RT_USB_DEVICE_HID_MOUSE

/* C/C++ and POSIX layer */

#define RT_LIBC_DEFAULT_TIMEZONE 8

/* POSIX (Portable Operating System Interface) layer */

#define RT_USING_POSIX_FS
#define RT_USING_POSIX_DEVIO

/* Interprocess Communication (IPC) */


/* Socket is in the 'Network' category */


/* Network */

#define RT_USING_SAL
#define SAL_INTERNET_CHECK

/* protocol stack implement */

#define SAL_USING_LWIP
#define SAL_USING_POSIX
#define RT_USING_NETDEV
#define NETDEV_USING_IFCONFIG
#define NETDEV_USING_PING
#define NETDEV_USING_NETSTAT
#define NETDEV_USING_AUTO_DEFAULT
#define NETDEV_IPV4 1
#define NETDEV_IPV6 0
#define RT_USING_LWIP
#define RT_USING_LWIP203
#define RT_USING_LWIP_VER_NUM 0x20003
#define RT_LWIP_MEM_ALIGNMENT 4
#define RT_LWIP_IGMP
#define RT_LWIP_ICMP
#define RT_LWIP_DNS
#define RT_LWIP_DHCP
#define IP_SOF_BROADCAST 1
#define IP_SOF_BROADCAST_RECV 1

/* Static IPv4 Address */

#define RT_LWIP_IPADDR "192.168.1.30"
#define RT_LWIP_GWADDR "192.168.1.1"
#define RT_LWIP_MSKADDR "255.255.255.0"
#define RT_LWIP_UDP
#define RT_LWIP_TCP
#define RT_LWIP_RAW
#define RT_MEMP_NUM_NETCONN 8
#define RT_LWIP_PBUF_NUM 16
#define RT_LWIP_RAW_PCB_NUM 4
#define RT_LWIP_UDP_PCB_NUM 4
#define RT_LWIP_TCP_PCB_NUM 4
#define RT_LWIP_TCP_SEG_NUM 40
#define RT_LWIP_TCP_SND_BUF 8196
#define RT_LWIP_TCP_WND 8196
#define RT_LWIP_TCPTHREAD_PRIORITY 10
#define RT_LWIP_TCPTHREAD_MBOX_SIZE 8
#define RT_LWIP_TCPTHREAD_STACKSIZE 1024
#define RT_LWIP_ETHTHREAD_PRIORITY 12
#define RT_LWIP_ETHTHREAD_STACKSIZE 1024
#define RT_LWIP_ETHTHREAD_MBOX_SIZE 8
#define LWIP_NETIF_STATUS_CALLBACK 1
#define LWIP_NETIF_LINK_CALLBACK 1
#define SO_REUSE 1
#define LWIP_SO_RCVTIMEO 1
#define LWIP_SO_SNDTIMEO 1
#define LWIP_SO_RCVBUF 1
#define LWIP_SO_LINGER 0
#define LWIP_NETIF_LOOPBACK 0
#define RT_LWIP_USING_PING

/* Utilities */

#define RT_USING_UTEST
#define UTEST_THR_STACK_SIZE 4096
#define UTEST_THR_PRIORITY 20

/* RT-Thread Utestcases */


/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */

/* JSON: JavaScript Object Notation, a lightweight data-interchange format */


/* XML: Extensible Markup Language */


/* multimedia packages */

/* LVGL: powerful and easy-to-use embedded GUI library */


/* u8g2: a monochrome graphic library */


/* PainterEngine: A cross-platform graphics application framework written in C language */


/* tools packages */


/* system packages */

/* enhanced kernel services */


/* POSIX extension functions */


/* acceleration: Assembly language or algorithmic acceleration packages */


/* CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */


/* Micrium: Micrium software products porting for RT-Thread */


/* peripheral libraries and drivers */


/* AI packages */


/* miscellaneous packages */

/* project laboratory */

/* samples: kernel and components samples */


/* entertainment: terminal games and other interesting software packages */


/* Hardware Drivers Config */

/* On-chip Peripheral Drivers */

#define SOC_SERIES_M480
#define BSP_USE_STDDRIVER_SOURCE
#define BSP_USING_PDMA
#define NU_PDMA_MEMFUN_ACTOR_MAX 2
#define NU_PDMA_SGTBL_POOL_SIZE 16
#define BSP_USING_FMC
#define BSP_USING_GPIO
#define BSP_USING_CLK
#define NU_CLK_INVOKE_WKTMR
#define BSP_USING_EMAC
#define NU_EMAC_PDMA_MEMCOPY
#define NU_EMAC_PDMA_MEMCOPY_THRESHOLD 128
#define BSP_USING_RTC
#define NU_RTC_SUPPORT_MSH_CMD
#define BSP_USING_TMR
#define BSP_USING_UART
#define BSP_USING_UART0
#define BSP_USING_I2C
#define BSP_USING_I2C1
#define BSP_USING_I2C2
#define BSP_USING_SDH
#define BSP_USING_SDH0
#define NU_SDH_USING_PDMA
#define NU_SDH_HOTPLUG
#define BSP_USING_SPI
#define BSP_USING_SPI_PDMA
#define BSP_USING_SPI0_NONE
#define BSP_USING_SPI1_NONE
#define BSP_USING_SPI2_NONE
#define BSP_USING_SPI3
#define BSP_USING_I2S
#define NU_I2S_DMA_FIFO_SIZE 2048
#define BSP_USING_QSPI
#define BSP_USING_QSPI0
#define BSP_USING_QSPI0_PDMA
#define BSP_USING_CRYPTO
#define BSP_USING_CRC
#define NU_CRC_USE_PDMA
#define BSP_USING_WDT
#define BSP_USING_USBD
#define BSP_USING_HSUSBH
#define NU_USBHOST_HUB_POLLING_INTERVAL 100

/* On-board Peripheral Drivers */

#define BSP_USING_NULINKME
#define BOARD_USING_IP101GR
#define BOARD_USING_NAU88L25
#define BOARD_USING_STORAGE_SDCARD
#define BOARD_USING_STORAGE_SPIFLASH
#define BOARD_USING_USB_D_H
#define BOARD_USING_HSUSBH_USBD

/* Board extended module drivers */


/* Nuvoton Packages Config */

#define NU_PKG_USING_UTILS
#define NU_PKG_USING_DEMO
#define NU_PKG_USING_NAU88L25

#endif
