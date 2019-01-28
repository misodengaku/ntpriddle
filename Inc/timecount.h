
#ifndef __TIMECOUNT_H
#define __TIMECOUNT_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>

    typedef struct
    {
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
        uint16_t msec;
        uint64_t sec_count;
        // time_t last_locked;
    } TimeCount;

    void tick(TimeCount *t);

#ifdef __cplusplus
}
#endif

#endif