/*
 * Copyright (c) 2006-2018, Synwit Technology Co.,Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-10     Zohar_Lee    first version
 * 2020-07-10     lik          format file
 */

#include "drv_rtc.h"
#include <sys/time.h>

#ifdef RT_USING_RTC
#ifdef BSP_USING_RTC

//#define DRV_DEBUG
#define LOG_TAG "drv.rtc"
#include <drv_log.h>

static uint32_t calcWeekDay(uint32_t year, uint32_t month, uint32_t date)
{
    uint32_t i, cnt = 0;
    const uint32_t daysOfMonth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    for (i = 1; i < month; i++)
        cnt += daysOfMonth[i];

    cnt += date;

    if ((year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0)) && (month >= 3))
        cnt += 1;

    cnt += (year - 1901) * 365;

    for (i = 1901; i < year; i++)
    {
        if ((i % 4 == 0) && ((i % 100 != 0) || (i % 400 == 0)))
            cnt += 1;
    }

    return (cnt + 1) % 7;
}

static time_t swm_get_rtc_time_stamp(void)
{
    RTC_DateTime get_datetime = {0};
    struct tm tm_new;

    RTC_GetDateTime(RTC, &get_datetime);

    tm_new.tm_sec = get_datetime.Second;
    tm_new.tm_min = get_datetime.Minute;
    tm_new.tm_hour = get_datetime.Hour;
    tm_new.tm_mday = get_datetime.Date;
    tm_new.tm_mon = get_datetime.Month;
    tm_new.tm_year = get_datetime.Year;

    LOG_D("get rtc time.");
    return mktime(&tm_new);
}
static rt_err_t swm_set_rtc_time_stamp(time_t time_stamp)
{
    RTC_DateTime set_datetime = {0};
    struct tm *p_tm;

    p_tm = gmtime(&time_stamp);

    set_datetime.Second = p_tm->tm_sec;
    set_datetime.Minute = p_tm->tm_min;
    set_datetime.Hour = p_tm->tm_hour;
    set_datetime.Date = p_tm->tm_mday;
    set_datetime.Month = p_tm->tm_mon;
    set_datetime.Year = p_tm->tm_year;
    // set_datetime.Day = p_tm->tm_wday;

    RTC_Stop(RTC);
    while (RTC->CFGABLE == 0)
        ;
    RTC->MINSEC = (set_datetime.Second << RTC_MINSEC_SEC_Pos) |
                  (set_datetime.Minute << RTC_MINSEC_MIN_Pos);
    RTC->DATHUR = (set_datetime.Hour << RTC_DATHUR_HOUR_Pos) |
                  ((set_datetime.Date) << RTC_DATHUR_DATE_Pos);
    RTC->MONDAY = (calcWeekDay(set_datetime.Year, set_datetime.Month, set_datetime.Date)
                   << RTC_MONDAY_DAY_Pos) |
                  ((set_datetime.Month) << RTC_MONDAY_MON_Pos);
    RTC->YEAR = set_datetime.Year;
    RTC->LOAD = 1 << RTC_LOAD_TIME_Pos;
    RTC_Start(RTC);

    LOG_D("set rtc time.");
    return RT_EOK;
}

static rt_err_t swm_rtc_init(void)
{
    RTC_InitStructure rtc_initstruct;

    rtc_initstruct.Year = 2020;
    rtc_initstruct.Month = 6;
    rtc_initstruct.Date = 8;
    rtc_initstruct.Hour = 12;
    rtc_initstruct.Minute = 0;
    rtc_initstruct.Second = 0;
    rtc_initstruct.SecondIEn = 0;
    rtc_initstruct.MinuteIEn = 0;
    RTC_Init(RTC, &rtc_initstruct);
    RTC_Start(RTC);

    return RT_EOK;
}

static rt_err_t swm_rtc_get_secs(void *args)
{
    *(rt_uint32_t *)args = swm_get_rtc_time_stamp();
    LOG_D("RTC: get rtc_time %x\n", *(rt_uint32_t *)args);

    return RT_EOK;
}

static rt_err_t swm_rtc_set_secs(void *args)
{
    rt_err_t result = RT_EOK;

    if (swm_set_rtc_time_stamp(*(rt_uint32_t *)args))
    {
        result = -RT_ERROR;
    }
    LOG_D("RTC: set rtc_time %x\n", *(rt_uint32_t *)args);

    return result;
}

static const struct rt_rtc_ops swm_rtc_ops =
{
    swm_rtc_init,
    swm_rtc_get_secs,
    swm_rtc_set_secs,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
};

static rt_rtc_dev_t swm_rtc_device;

int rt_hw_rtc_init(void)
{
    rt_err_t result;

    swm_rtc_device.ops = &swm_rtc_ops;
    result = rt_hw_rtc_register(&swm_rtc_device, "rtc", RT_DEVICE_FLAG_RDWR,RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("rtc register err code: %d", result);
        return result;
    }
    LOG_D("rtc init success");
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_rtc_init);

#endif /* BSP_USING_RTC */
#endif /* RT_USING_RTC */
