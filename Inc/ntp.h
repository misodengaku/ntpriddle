
#ifndef __NTP_H
#define __NTP_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <math.h>
#include "lwip/udp.h"
#include "lwip/api.h"
#include "lwip/ip_addr.h"
#include "lwip.h"
#include "gps_thread.h"

// NTP時刻とUNIX時刻の差分(秒)
// http://jjy.nict.go.jp/tsp/PubNtp/clients.html
#define NTP_TO_UNIX 2208988800

// RFC5905 "Figure 12: Reference Identifiers"あたりを参照
#define NTP_REFERENCE_IDENTIFIER "GPS\0"

// "Figure 11: Packet Stratum"
#define NTP_STRATUM 1

    typedef struct
    {
        uint8_t flags;
        uint8_t stratum;
        uint8_t poll;
        int8_t precision;
        uint32_t root_delay;
        uint32_t root_dispersion;
        char reference_identifier[4];
        uint64_t reference_timestamp;
        uint64_t originate_timestamp;
        uint64_t receive_timestamp;
        uint64_t transmit_timestamp;
    } __attribute__((packed)) ntp_msg;

    void ntp_start_listen();
    void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

#ifdef __cplusplus
}
#endif

#endif