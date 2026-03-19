/*
 * systemTime.c
 *
 *  Created on: Feb 27, 2026
 *      Author: Atish.Singh
 */



 #include <stdbool.h>
#include <sys/time.h>
 #include <math.h>
 #include <stdint.h>
 #include <stdio.h>
 #include <string.h>
#include "datatypes.h"
#include "esp_timer.h"


bool system_time_valid = false;



bool is_system_time_valid()
{
    return system_time_valid;
}

time_t get_current_epoch()
{
    if (!system_time_valid)
        return 0;

    return time(NULL);
}

uint64_t get_uptime_ms()
{
    return esp_timer_get_time() / 1000ULL;   // ESP-IDF
}


void update_system_time(time_t epoch_from_cloud)
{
    // Reject obviously invalid timestamps
    if (epoch_from_cloud < 1700000000)   // ~Year 2023 safety check
    {
        system_time_valid = false;
        return;
    }

    struct timeval tv = {
        .tv_sec  = epoch_from_cloud,
        .tv_usec = 0
    };

    settimeofday(&tv, NULL);

    system_time_valid = true;
}


bool get_time_from_cloud()
{
    time_t cloud_epoch = 0;
	
	/*fetch from cloud here [using water method is needed */
    if (cloud_epoch < 1700000000)
    {
        system_time_valid = false;
        return false;
    }

    update_system_time(cloud_epoch);
    return true;
}



void periodic_time_sync_task()
{
    static uint32_t last_attempt = 0;

    if (get_uptime_ms() - last_attempt > 60000) // every 60s
    {
        last_attempt = get_uptime_ms();
        get_time_from_cloud();
    }
}