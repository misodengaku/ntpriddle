#ifndef __GPS_THREAD_H
#define __GPS_THREAD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lwip/udp.h"
#include "cmsis_os.h"

    void gps_loop();
    void nmea_dump_info();
    void sync_gps_rtc_time();
    time_t get_unixtime();

#ifdef __cplusplus
}
#endif

#endif /* __GPS_THREAD_H */
