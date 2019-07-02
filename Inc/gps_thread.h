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
    struct tm get_tm();
    time_t get_unixtime();
    void timer_autoreload_handler();
    void gps_pps_handler();

    enum ClockSyncMode {
        NO_SYNC,
        GNSS_SYNC
    };
#ifdef __cplusplus
}
#endif

#endif /* __GPS_THREAD_H */
