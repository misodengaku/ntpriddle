#include "ntp.h"

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;

extern uint64_t sec_count;
extern time_t last_locked;

void ntp_start_listen()
{
    struct udp_pcb *pcb;
    const unsigned short src_port = 123;
    pcb = udp_new();
    udp_bind(pcb, IP_ADDR_ANY, src_port);
    udp_recv(pcb, ntp_recv, NULL);
}

void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    // uint8_t ntp_buf[128];
    ntp_msg nm = {0};
    if (p != NULL)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)"!!! NTP incoming !!!\ndump: ", 27, 1000);

        if ((*p).len > 128)
        {
            HAL_UART_Transmit(&huart3, (uint8_t *)"packet is too big\n", 18, 1000);
            pbuf_free(p);
            return;
        }

        __disable_irq();
        // htim3.Instance->CNT => 20us / 1count
        // 2208988800 = ntp time to unix time
        last_locked = get_unixtime();

        uint64_t int_part, fraction_part;

        // 整数部を作る
        int_part = last_locked + sec_count + NTP_TO_UNIX;
        // リトルエンディアンなのでビット順反転させる
        int_part = (uint64_t)htonl((uint32_t)int_part);

        // 小数部を作る
        fraction_part = round((htim3.Instance->CNT * 0.000020) * ((uint64_t)1 << 32));
        // リトルエンディアンなのでビット順反転させる
        fraction_part = (uint64_t)htonl((uint32_t)fraction_part);

        // パケット受信時刻
        // 整数部と小数部結合して固定小数点数とする
        // ビット順反転してあるので小数部を上に置く
        uint64_t receive_timestamp = int_part | (uint64_t)fraction_part << 32;
        __enable_irq();

        // NTP packet dump
        /*
        for (int i = 0; i < (*p).len; i++)
        {
            char b[4];
            sprintf(b, "%02X ", ((char *)(*p).payload)[i]);
            HAL_UART_Transmit(&huart3, (uint8_t *)b, 3, 1000);
        }
        HAL_UART_Transmit(&huart3, (uint8_t *)"\n", 1, 1000);
        */

        // 到着したパケットを送信メッセージとしてコピーしてくる
        // 想定外に大きければ切り詰める
        uint16_t len = (*p).len;
        if (len > sizeof(ntp_msg))
        {
            len = sizeof(ntp_msg);
        }
        memcpy((uint8_t *)(&nm), (*p).payload, (*p).len);

        printf("Leap %d\n", nm.flags >> 6);
        printf("NTPv%d\n", (nm.flags >> 3) & 0x7);
        printf("mode %d\n", nm.flags & 0x7);

        uint8_t leap = 0, version = (nm.flags >> 3) & 0x7, mode = 4;
        strcpy(nm.reference_identifier, NTP_REFERENCE_IDENTIFIER);

        // Total round-trip delay to the reference clock
        // 自分自身がreference clockということで無視して0
        nm.root_delay = 0;

        // Total dispersion to the reference clock
        // 知らないことにして0
        nm.root_dispersion = 0;

        // ポーリング間隔
        // nm.poll = 3;

        // 2^(-12) = 244usぐらい
        nm.precision = -12;

        // 言うだけタダ
        nm.stratum = 1;

        // leap, version, modeを1バイトに詰める
        nm.flags = (leap & 0x03) << 6 | (version & 0x07) << 3 | (mode & 0x07);

        // クライアントから送信されてきた時刻をoriginateとして取り込む
        nm.originate_timestamp = nm.transmit_timestamp;

        // パケット受信時刻
        nm.receive_timestamp = receive_timestamp;

        __disable_irq();

        int_part = last_locked + sec_count + NTP_TO_UNIX;
        fraction_part = round((htim3.Instance->CNT * 0.000020) * ((uint64_t)1 << 32));

        // little -> big
        int_part = (uint64_t)htonl((uint32_t)int_part);
        fraction_part = (uint64_t)htonl((uint32_t)fraction_part);

        // 最後に時刻同期を行った日時
        // この前のPPS受けた時刻ということにしておく
        nm.reference_timestamp = int_part;

        // パケット送出時刻
        uint64_t transmit_timestamp = int_part | (uint64_t)fraction_part << 32;
        nm.transmit_timestamp = transmit_timestamp;
        __enable_irq();

        // NTPの構造体を送信バッファに詰めて発射
        struct pbuf *np = pbuf_alloc(PBUF_TRANSPORT, sizeof(nm), PBUF_RAM);
        memcpy(np->payload, &nm, sizeof(nm));
        udp_sendto(pcb, np, addr, port);
        pbuf_free(np);
    }

    pbuf_free(p);
}