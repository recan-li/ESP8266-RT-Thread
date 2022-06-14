/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-01-13     weety      modified from mini2440
 */

#include <rthw.h>
#include <rtthread.h>
#include "at91sam9g45.h"

/**
 * @addtogroup AT91SAM926X
 */
/*@{*/

void machine_reset(void)
{
    AT91C_BASE_RSTC->RSTC_RCR = AT91C_RSTC_KEY | AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST;
}

void machine_shutdown(void)
{
    AT91C_BASE_SHDWC->SHDWC_SHCR = AT91C_SHDWC_KEY | AT91C_SHDWC_SHDW;
}

#ifdef RT_USING_FINSH

#include <finsh.h>

#ifdef FINSH_USING_MSH
int cmd_reset(int argc, char** argv)
{
    rt_hw_cpu_reset();
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_reset, reset, restart the system);

int cmd_shutdown(int argc, char** argv)
{
    rt_hw_cpu_shutdown();
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_shutdown, shutdown, shutdown the system);

#endif
#endif

/*@}*/
