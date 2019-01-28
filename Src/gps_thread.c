#include "gps_thread.h"
#include "uart.h"
#include "timecount.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define NMEA_BUF_LEN 256
#define MAXTOKEN 32

extern osSemaphoreId GPSDataSemaphoreHandle;
extern TIM_HandleTypeDef htim3;
extern RTC_HandleTypeDef hrtc;
extern bool is_gps_fixed;

char nmea_line[NMEA_BUF_LEN];
char dump_info_buf[1024];

extern uint8_t satellite_count;
extern double latitude;
extern double longitude;
extern float speed;
extern float move_direction;
TimeCount tcount;
#if 0
extern uint16_t year;
extern uint8_t month;
extern uint8_t day;
extern uint8_t hour;
extern uint8_t min;
extern uint8_t sec;
extern uint16_t msec;
#endif

extern uint64_t sec_count;
extern time_t last_locked;

long str2int(char *src_ptr, uint8_t len);
int split(char *str, const char delim, char *token[], int max_item);
bool nmea_decode();
uint8_t nmea_validate();
time_t get_unixtime();

void gps_loop()
{
    char line_buff[256] = {0};
    char gps_buff[256];
    for (;;)
    {
        // HAL_UART_Transmit_DMA(&huart3, "test1\n", 6);
        // HAL_UART_Transmit_DMA(&huart3, "test2\n", 6);
        // HAL_UART_Transmit_DMA(&huart3, "test3\n", 6);
        while (1)
        {
            osDelay(10);
            // printf("fetch gps\n");
            int16_t ret = GPS_UART_ReadLine((uint8_t *)gps_buff, 256, true);
            if (strlen(line_buff) > 0)
            {
                strlcat(line_buff, gps_buff, sizeof(line_buff));
                int line_len = 0;
                for (; line_len < strlen(line_buff); line_len++)
                {
                    if (line_buff[line_len] == '\r' || line_buff[line_len] == '\n')
                    {
                        break;
                    }
                }
                strlcpy(gps_buff, line_buff, line_len);
                memset(line_buff, 0, sizeof(line_buff));
                memmove(line_buff, line_buff + line_len, sizeof(line_buff) - line_len);
            }
            else if (gps_buff[ret - 1] == '\n' || gps_buff[ret - 1] == '\r')
            {
                strcpy(line_buff, gps_buff);
                break;
            }
            // printf("%s\n", gps_buff);

            strcpy(nmea_line, gps_buff);
            if (nmea_validate() == 0)
            {
                nmea_decode();
                // last_locked = get_unixtime();
                sec_count = 0;
            }
            memset(gps_buff, 0, sizeof(gps_buff));
        }
    }
}

bool nmea_decode()
{
    char *nmea_lines[MAXTOKEN];
    if (nmea_validate() != 0)
    {
        //        printf("invalid message\n");
        printf("validation fail\n");
        return false;
    }

    // uint8_t nmea_line_index = split(nmea_line, ',', nmea_lines, MAXTOKEN);
    split(nmea_line, ',', nmea_lines, MAXTOKEN);

    if (strcmp((nmea_lines[0]) + 3, "GSV") == 0)
    {
        // 衛星数関連のデータ
        satellite_count = atoi(nmea_lines[3]);
        return false;
    }
    else if (strcmp((nmea_lines[0]) + 3, "RMC") == 0)
    {
        if (nmea_lines[2][0] == 'A')
        {
            //printf("RMC mode: Information\r\n");
        }
        else if (nmea_lines[2][0] == 'V')
        {
            //printf("RMC mode: Warning\r\n");
        }
        else
        {
            printf("RMC mode: invalid");
            return false;
        }

        // double decimal_part[2] = {0};

        /* 時刻のデコード */
        tcount.hour = (uint8_t)str2int(nmea_lines[1], 2);
        tcount.min = (uint8_t)str2int(nmea_lines[1] + 2, 2);
        tcount.sec = (uint8_t)str2int(nmea_lines[1] + 4, 2);
        // tcount.msec = (uint16_t)str2int(nmea_lines[1] + 7, 3);
        if (tcount.hour > 23 || tcount.hour < 0)
        {
            return false;
        }
        if (tcount.min > 59 || tcount.min < 0)
        {
            return false;
        }
        if (tcount.sec > 59 || tcount.sec < 0)
        {
            return false;
        }

        /* 日付のデコード */
        tcount.day = (uint8_t)str2int(nmea_lines[9], 2);
        tcount.month = (uint8_t)str2int(nmea_lines[9] + 2, 2);
        tcount.year = (uint16_t)str2int(nmea_lines[9] + 4, 2) + 2000;
        if (tcount.day > 31 || tcount.day < 1)
        {
            return false;
        }
        if (tcount.month > 12 || tcount.month < 1)
        {
            return false;
        }
        if (tcount.year < 2019)
        {
            return false;
        }

        /* 位置情報のデコード */
        // printf("line3: %s\n", nmea_lines[3]);
        // printf("line5: %s\n", nmea_lines[5]);

        char decimal_tokens[2][16];
        int8_t splitted_count = 0;

        // "3541.1234" => "3541", "1234"
        splitted_count = split(nmea_lines[3], '.', (char **)decimal_tokens, 2);

        if (splitted_count > 0)
        {
            // 3541 => 35.41
            latitude = atoi(decimal_tokens[0]) / 100.0f;
            // printf("%f", latitude);
            // decimal_part[1] = floor(())
        }
        else
        {
            latitude = (float)atoi(nmea_lines[1]) / 100;
        }

        /*
        // 整数部抜き出し
        // "3541.12346" => 3541.0
        decimal_part[0] = modf(strtof(nmea_lines[3], NULL), &latitude); // 秒

        // 3541 => 35.41
        latitude /= 100;

        // modf: 35.41 => latitude = 0.41, ret = 35.0
        // floor: (35
        decimal_part[1] = floor((modf(latitude, &latitude) * 100) + 0.5); // 分
        latitude += (decimal_part[1] + decimal_part[0]) / 60;

        if (nmea_lines[4][0] == 'S')
            latitude = -latitude;

        decimal_part[0] = modf(strtof(nmea_lines[5], NULL), &longitude); // 秒
        longitude /= 100;
        decimal_part[1] = floor((modf(longitude, &longitude) * 100) + 0.5); // 分
        longitude += (decimal_part[1] + decimal_part[0]) / 60;
        */

        if (nmea_lines[6][0] == 'W')
            longitude = -longitude;

        /* 移動速度のデコード */
        speed = strtof(nmea_lines[7], NULL) * 1.852f; // knot to km/h

        move_direction = strtof(nmea_lines[8], NULL);
    }
    else if (strcmp((nmea_lines[0]) + 3, "GGA") == 0)
    {
        double decimal_part[2] = {0};

        /* 時刻のデコード */
        tcount.hour = (uint8_t)str2int(nmea_lines[1], 2);
        tcount.min = (uint8_t)str2int(nmea_lines[1] + 2, 2);
        tcount.sec = (uint8_t)str2int(nmea_lines[1] + 4, 2);
        tcount.msec = (uint16_t)str2int(nmea_lines[1] + 7, 3);
        if (tcount.hour > 23 || tcount.hour < 0)
        {
            return false;
        }
        if (tcount.min > 59 || tcount.min < 0)
        {
            return false;
        }
        if (tcount.sec > 59 || tcount.sec < 0)
        {
            return false;
        }

        // GGAでは日付は不明

        /* 位置情報のデコード */
        decimal_part[0] = modf(strtof(nmea_lines[2], NULL), &latitude); // 秒
        latitude /= 100;
        decimal_part[1] = floor((modf(latitude, &latitude) * 100) + 0.5); // 分
        latitude += (decimal_part[1] + decimal_part[0]) / 60;

        if (nmea_lines[3][0] == 'S')
            latitude = -latitude;

        decimal_part[0] = modf(strtof(nmea_lines[4], NULL), &longitude); // 秒
        longitude /= 100;
        decimal_part[1] = floor((modf(longitude, &longitude) * 100) + 0.5); // 分
        longitude += (decimal_part[1] + decimal_part[0]) / 60;

        if (nmea_lines[5][0] == 'W')
            longitude = -longitude;

        // 移動速度と移動方向はGGAではわからない
    }
    else if (strcmp((nmea_lines[0]) + 3, "GLL") == 0)
    {
        // is_valid =

        if (nmea_lines[6][0] != 'A')
        {
            // printf("data is invalid.\n");
            return false;
        }

        // double decimal_part[2] = {0};

        /* 時刻のデコード */
        tcount.hour = (uint8_t)str2int(nmea_lines[5], 2);
        tcount.min = (uint8_t)str2int(nmea_lines[5] + 2, 2);
        tcount.sec = (uint8_t)str2int(nmea_lines[5] + 4, 2);
        // tcount.msec = (uint16_t)str2int(nmea_lines[5] + 7, 3);
        if (tcount.hour > 23 || tcount.hour < 0)
        {
            return false;
        }
        if (tcount.min > 59 || tcount.min < 0)
        {
            return false;
        }
        if (tcount.sec > 59 || tcount.sec < 0)
        {
            return false;
        }

        // GGAでは日付は不明

        /* 位置情報のデコード */
        char decimal_tokens[2][16];
        int8_t splitted_count = 0;
        splitted_count = split(nmea_lines[1], '.', (char **)decimal_tokens, 2);
        if (splitted_count == 0)
        {
            latitude = (float)atoi(nmea_lines[1]) / 100;
        }

        if (latitude < 0.0f || latitude > 90.0f)
        {
            return false;
        }

        if (nmea_lines[2][0] == 'S')
        {
            latitude = -latitude;
        }

        splitted_count = split(nmea_lines[3], '.', (char **)decimal_tokens, 2);
        if (splitted_count == 0)
        {
            longitude = (float)atoi(nmea_lines[3]) / 100;
        }

        if (longitude < 0.0f || longitude > 180.0f)
        {
            return false;
        }

        if (nmea_lines[4][0] == 'W')
        {
            longitude = -longitude;
        }

        // decimal_part[0] = modf(strtof(nmea_lines[1], NULL), &latitude); // 秒
        // latitude /= 100;
        // decimal_part[1] = floor((modf(latitude, &latitude) * 100) + 0.5); // 分
        // latitude += (decimal_part[1] + decimal_part[0]) / 60;

        // if (nmea_lines[2][0] == 'S')
        //     latitude = -latitude;

        // decimal_part[0] = modf(strtof(nmea_lines[3], NULL), &longitude); // 秒
        // longitude /= 100;
        // decimal_part[1] = floor((modf(longitude, &longitude) * 100) + 0.5); // 分
        // longitude += (decimal_part[1] + decimal_part[0]) / 60;

        // 移動速度と移動方向はGGAではわからない
    }
    else
    {
        return false;
    }
#if 0
    printf("%s\n", nmea_line);
    for (int i = 0; i < nmea_line_index; i++) {
        printf("%s,", nmea_lines[i]);
    }
    printf("\r\n");
#endif
    return true;
}

uint8_t nmea_validate()
{
    uint8_t checksum = 0, nmea_checksum = '\0', flag = 0;
    char checksum_str[3] = {0};

    if (nmea_line[0] != '$')
    {
        //        printf("invalid preamble %02X\n", nmea_line[0]);
        return 1;
    }

    for (int i = 1; i < strlen((char *)nmea_line); i++)
    {
        if (flag)
        {
            // 2byte checksum
            memcpy(checksum_str, (char *)nmea_line + i, 2);
            nmea_checksum = (uint8_t)strtol(checksum_str, NULL, 16);
            break;
        }
        else
        {
            if (nmea_line[i] == '*')
            {
                flag = 1;
                continue;
            }
            checksum = checksum ^ nmea_line[i];
        }
    }
    if (checksum != nmea_checksum)
    {
        printf("invalid checksum\n\t%s\n", nmea_line);
        return 2;
    }
    //    printf("validation passed\n");
    return 0;
}

long str2int(char *src_ptr, uint8_t len)
{
    char tmp[16] = {0};
    memcpy(tmp, src_ptr, len);
    return strtol(tmp, NULL, 10);
}

int split(char *str, const char delim, char *token[], int max_item)
{
    int cnt = 0, i;
    int len = strlen(str);

    token[cnt++] = str;
    for (i = 0; i < len; i++)
    {
        if (str[i] == delim)
        {
            str[i] = '\0';
            if (cnt == max_item)
                return cnt;
            token[cnt++] = str + i + 1;
        }
    }

    if (i == len)
    {
        // デリミタが分割対象に含まれていなかった場合
        return 0;
    }

    return cnt;
}

time_t get_unixtime()
{
    struct tm t;
    t.tm_sec = tcount.sec;
    t.tm_min = tcount.min;
    t.tm_hour = tcount.hour;
    t.tm_mday = tcount.day;
    t.tm_mon = tcount.month - 1;
    t.tm_year = tcount.year - 1900;

    return mktime(&t);
}

void nmea_dump_info()
{
    printf("Satellite count: %d\r\n", satellite_count);
    if (satellite_count > 4)
    {
        printf("tcount: %04d/%02d/%02d %02d:%02d:%02d (UTC)\r\n",
               tcount.year, tcount.month, tcount.day, tcount.hour, tcount.min, tcount.sec);
    }
    if (satellite_count > 3)
    {
        printf("latitude: %f\r\n", (float)latitude);
        printf("longitude: %f\r\n", (float)longitude);
    }
    // sprintf(buf, "%sspeed %f km/h (%f knot)\r\n", buf, speed, speed / 1.852f);
    // sprintf(buf, "%sdirection %f degree\r\n", buf, move_direction);
}

void sync_gps_rtc_time()
{
    RTC_TimeTypeDef rtcTime;
    RTC_DateTypeDef rtcDate;
    HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

    rtcDate.Year = (uint8_t)(tcount.year - 2000);
    rtcDate.Month = tcount.month;
    rtcDate.Date = tcount.day;
    rtcTime.Hours = tcount.hour;
    rtcTime.Minutes = tcount.min;
    rtcTime.Seconds = tcount.sec;
    HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
    is_gps_fixed = true;
}


// EXTIから呼ばれるやつ
void gps_pps_handler()
{
    // uint32_t msec_count;

    __disable_irq();
    // msec_count = htim3.Instance->CNT;
    HAL_TIM_Base_Stop(&htim3);
    htim3.Instance->CNT = 0;
    HAL_TIM_Base_Start_IT(&htim3);
    tick(&tcount);

    if (satellite_count > 4)
    {
        if (osSemaphoreWait(GPSDataSemaphoreHandle, 0) == osOK)
        {
            nmea_dump_info();
            osSemaphoreRelease(GPSDataSemaphoreHandle);
        }
        else
        {
            HAL_UART_Transmit(&huart3, (uint8_t *)"!!! PPS incoming !!!\n", 21, 1000);
        }
    }
    __enable_irq();
}
