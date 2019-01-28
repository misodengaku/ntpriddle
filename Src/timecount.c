#include "timecount.h"

void tick(TimeCount *t)
{
    t->sec++;
    if (t->sec >= 60)
    {
        t->min++;
        t->sec = 0;
    }
    if (t->min >= 60)
    {
        t->hour++;
        t->min = 0;
    }
    if (t->hour >= 24)
    {
        t->day++;
        t->hour = 0;
    }

    uint8_t day_max = 0;

    switch (t->month)
    {
    case 1:
    case 3:
    case 5:
    case 7:
    case 8:
    case 10:
    case 12:
        if (t->day > 31)
        {
            t->month++;
            t->day = 1;

            if (t->month == 13)
            {
                t->year++;
                t->month = 1;
            }
        }
        break;
    case 2:
        if ((t->year % 4 == 0))
        {
            if ((t->year % 100 == 0) && (t->year % 400 != 0))
            {
                day_max = 28;
            }
            else
            {
                // 4で割り切れる年
                // 100で割り切れる、かつ400で割り切れる年
                day_max = 29;
            }
        }
        else
        {
            day_max = 28;
        }
        if (t->day > day_max)
        {
            t->month++;
            t->day = 1;
        }
        break;
    default:
        if (t->day > 30)
        {
            t->month++;
            t->day = 1;
        }
        break;
    }
}

void dump(TimeCount *t)
{
    printf("%04d/%02d/%02d %02d:%02d:%02d\n", t->year, t->month, t->day, t->hour, t->min, t->sec);
}

#if 0
int main()
{

    TimeCount t;
    t.year = 2019;
    t.month = 12;
    t.day = 31;
    t.hour = 23;
    t.min = 59;
    t.sec = 55;

    for (int i = 0; i < 10; i++)
    {
        tick(&t); 
        dump(&t);
        // sleep(1);
    }
    printf("--------------\n");

    t.year = 2019;
    t.month = 2;
    t.day = 28;
    t.hour = 23;
    t.min = 59;
    t.sec = 55;

    for (int i = 0; i < 10; i++)
    {
        tick(&t);
        dump(&t);
        // sleep(1);
    }
    printf("--------------\n");

    t.year = 2020;
    t.month = 2;
    t.day = 28;
    t.hour = 23;
    t.min = 59;
    t.sec = 55;

    for (int i = 0; i < 10; i++)
    {
        tick(&t);
        dump(&t);
        // sleep(1);
    }
    printf("--------------\n");

    t.year = 2000;
    t.month = 2;
    t.day = 28;
    t.hour = 23;
    t.min = 59;
    t.sec = 55;

    for (int i = 0; i < 10; i++)
    {
        tick(&t);
        dump(&t);
        // sleep(1);
    }

    printf("--------------\n");

    t.year = 2100;
    t.month = 2;
    t.day = 28;
    t.hour = 23;
    t.min = 59;
    t.sec = 55;

    for (int i = 0; i < 10; i++)
    {
        tick(&t);
        dump(&t);
        // sleep(1);
    }

    return 0;
}
#endif
