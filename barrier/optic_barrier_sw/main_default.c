/*
 * Copyright (C) 2020 Tomas Prochazka <tomas.prochazka@cvut.cz>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/if_link.h>
#include <net/if.h>
#include <time.h>
#include <sys/time.h>
#include <limits.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>

#include <wiringPi.h>

#include "OLED_Config.h"
#include "OLED_Driver.h"
#include "OLED_GUI.h"
#include "Show_Lib.h"

#define BARRIER_SIGNAL 0     // 30
#define UNIVERSAL_BUTTON1 19 // 24
#define UNIVERSAL_BUTTON2 26 // 25
#define SHUTDOWN_BUTTON 21   // 29

#define MIN_TIME_US 1000000

pthread_mutex_t detect_mutex = PTHREAD_MUTEX_INITIALIZER;
bool object_detected = false;
struct timeval detect_time;

void interrupt_optic_barrier()
{
    struct timeval now;

    gettimeofday(&now, NULL);
    pthread_mutex_lock(&detect_mutex);
    if (!object_detected) {
        object_detected = true;
        detect_time = now;
    }
    pthread_mutex_unlock(&detect_mutex);
}

int get_ip_address(char *host)
{
    struct ifaddrs *ifa;
    int family, s; // n;

    if (getifaddrs(&ifa) == -1) {
        perror("getifaddrs");
        return EXIT_FAILURE;
    }

    for (; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL)
            continue;

        if ((strcmp("lo", ifa->ifa_name) == 0) || !(ifa->ifa_flags & (IFF_RUNNING)))
            continue;

        family = ifa->ifa_addr->sa_family;

        if (family == AF_INET) {
            s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0) {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                return EXIT_FAILURE;
            }

            // printf("%s%s\n",ip_address, host);
        }
    }

    freeifaddrs(ifa);

    return EXIT_SUCCESS;
}

typedef struct mytime_t {
    int minutes;
    int seconds;
    int miliseconds;
} mytime_t;

void convert_time(mytime_t *mytime, int64_t time_us)
{
    int64_t t = time_us;

    if (time_us < 0)
        t = -t;

    mytime->miliseconds = (t % 1000000) / 1000;
    t /= 1000000;

    mytime->seconds = t % 60;
    t /= 60;

    mytime->minutes = t % 99;

    if (time_us < 0)
        mytime->minutes *= -1;
}

char *us2str(char *dest, const char *prefix, int64_t time_us)
{
        mytime_t mytime;
        convert_time(&mytime, time_us);
        if (time_us != INT64_MAX)
            sprintf(dest, "%s%02d:%02d.%03d", prefix, mytime.minutes, mytime.seconds, mytime.miliseconds);
        else
            sprintf(dest, "%s--:--.---", prefix);
        return dest;
}

struct state {
    int64_t time_us;
    int64_t lap_time_us;
    int64_t best_time_us;
} state;

enum screen { EMPTY, TIME, SHUTDOWN };

void update_display(enum screen screen)
{
    GUI_Clear();
    switch (screen) {
    case EMPTY:
        break;
    case TIME: {
        char str[50];
        sFONT *font = &Font12;

        us2str(str, "      ", state.time_us);
        GUI_DisString_EN(0, 0, str, font, FONT_BACKGROUND, WHITE);                   // actual time

        us2str(str, "LAP:  ", state.lap_time_us);
        GUI_DisString_EN(0, font->Height - 2, str, font, FONT_BACKGROUND, WHITE);     // lap time

        us2str(str, "BEST: ", state.best_time_us);
        GUI_DisString_EN(0, 2 * font->Height - 4, str, font, FONT_BACKGROUND, WHITE); // best time
        break;
    }
    case SHUTDOWN: {
        GUI_DisString_EN(0, 0, "Hold 5s to pwroff", &Font12, FONT_BACKGROUND, WHITE);
        char host[NI_MAXHOST];
        get_ip_address(host);
        char str[100];
        sprintf(str, "IP: %s", host);
        GUI_DisString_EN(0, Font12.Height - 2, str, &Font12, FONT_BACKGROUND, WHITE);

        char name_user[32];
        char user_info[50];
        cuserid(name_user);
        sprintf(user_info, "User:%s Pass:user", name_user);
        GUI_DisString_EN(0, 2*Font12.Height - 4, user_info, &Font12, FONT_BACKGROUND, WHITE);

        break;
    }
    }
    GUI_Display();
}

int64_t usec_between(const struct timeval *start, const struct timeval *stop)
{
    return (stop->tv_sec - start->tv_sec) * 1000000LL + stop->tv_usec - start->tv_usec;
}

void shutdown_handler(bool btn_pressed)
{
    static struct timeval start;
    struct timeval now;

    if (!btn_pressed) {
        start = (struct timeval){0, 0};
        return;
    }

    if (start.tv_sec == 0)
        gettimeofday(&start, NULL);

    gettimeofday(&now, NULL);

    if (usec_between(&start, &now) > 5*1000000) {
        update_display(EMPTY);
        System_Exit();
        system("sudo shutdown -h now");
        exit(0);
    }
}

int main(int argc, char *argv[])
{
    // 1.System Initialization
    if (System_Init())
        exit(0);

    // 2.show
    OLED_Init();
    GUI_Init(OLED_WIDTH, OLED_HEIGHT);
    GUI_Clear();

    struct timeval start;
    // gettimeofday(&start, NULL);

    // This initialises the wiringPi system
    // and assumes that the calling program is going to be using the wiringPi pin numbering scheme.
    if (wiringPiSetup() < 0)
        return 1;
    // This function registers a function to received interrupts on the specified pin.
    if (wiringPiISR(BARRIER_SIGNAL, INT_EDGE_RISING, &interrupt_optic_barrier) < 0) {
        printf("Unable to setup ISR \n");
    }

    bool detect_in_progress = false;
    bool after_start = false;
    state.time_us = 0;
    state.lap_time_us = INT64_MAX;
    state.best_time_us = INT64_MAX;

    while (1) {
        /* Reset everything */
        if (digitalRead(UNIVERSAL_BUTTON1) == 1) {
            after_start = false;
            detect_in_progress = false;
            state.time_us = 0;
            state.lap_time_us = INT64_MAX;
            state.best_time_us = INT64_MAX;
        }

        if (digitalRead(SHUTDOWN_BUTTON) == 1) {
            update_display(SHUTDOWN);
            shutdown_handler(true);
        } else {
            update_display(TIME);
            shutdown_handler(false);
        }

        if (after_start) {
            struct timeval stop;
            gettimeofday(&stop, NULL);
            state.time_us = usec_between(&start, &stop);
        }

        {
            bool local_object_detected;
            struct timeval local_detect_time;

            pthread_mutex_lock(&detect_mutex);
            local_object_detected = object_detected;
            if (object_detected) {
                object_detected = false;
                local_detect_time = detect_time;
            }
            pthread_mutex_unlock(&detect_mutex);

            if (local_object_detected) {
                if (!detect_in_progress) {
                    detect_in_progress = true;
                    if (after_start) {
                        state.lap_time_us = usec_between(&start, &local_detect_time);;
                        if (state.best_time_us > state.lap_time_us)
                            state.best_time_us = state.lap_time_us;
                    }
                    start = local_detect_time;
                    after_start = true;

                    printf("{\"timestamp\":%lld}\n", local_detect_time.tv_sec * 1000000LL + local_detect_time.tv_usec);
                }
            } else {
                detect_in_progress = false;
            }
        }
        usleep(1000); // wait 1ms
    }
    System_Exit();
}

/* Local Variables: */
/* c-basic-offset: 4 */
/* End: */
