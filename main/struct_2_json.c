/*
 * struct_2_json.c
 *
 *  Created on: Dec 24, 2025
 *      Author: Dheeraj.Jain
 */

#include "struct_2_json.h"
#include <string.h>

cJSON * create_json_packet(const s_json_packet_t *data)
{
    cJSON *j = cJSON_CreateObject();
    if (!j) {
        return NULL;
    }
 
    cJSON_AddNumberToObject(j, "pkt_ind", data->pkt_ind);
    cJSON_AddStringToObject(j, "id", data->ID);
    cJSON_AddNumberToObject(j, "TS", data->TS);
    cJSON_AddNumberToObject(j, "bat", data->BAT);

    cJSON_AddNumberToObject(j, "HR", data->HR);
    cJSON_AddNumberToObject(j, "HRV", data->HRV);
    cJSON_AddNumberToObject(j, "RR", data->RR);
    cJSON_AddNumberToObject(j, "TEMP", data->TEMP);
    cJSON_AddNumberToObject(j, "a_x", data->a_x);
    cJSON_AddNumberToObject(j, "a_y", data->a_y);
    cJSON_AddNumberToObject(j, "a_z", data->a_z);
    cJSON_AddNumberToObject(j, "g_x", data->g_x);
    cJSON_AddNumberToObject(j, "g_y", data->g_y);
    cJSON_AddNumberToObject(j, "g_z", data->g_z);
    cJSON_AddStringToObject(j, "beh", data->BEH);
    cJSON_AddNumberToObject(j, "ACT", data->ACT);
    cJSON_AddNumberToObject(j, "LAT", data->LAT);
    cJSON_AddNumberToObject(j, "LONG", data->LONG);
    cJSON_AddNumberToObject(j, "dist", data->DIST);
    cJSON_AddNumberToObject(j, "cal", data->CAL);
    cJSON_AddNumberToObject(j, "alert_val", data->AL_VAL);
    cJSON_AddNumberToObject(j, "alert", data->ALERT);
    cJSON_AddNumberToObject(j, "alert_sev", data->AL_SEV);
    
    return j;
}

char *handle_json_packet(StateMachine *sm, s_json_packet_t* json_data)
{
    static uint16_t packet_index = 0;
    // Populate json_data from processed_signals and other sources
    json_data->pkt_ind = packet_index++; // Increment packet index
    strcpy(json_data->ID, "GUP_123"); // Replace with actual device ID
    json_data->TS = 123456; // Replace with actual timestamp
    json_data->BAT = 80; // Replace with actual battery level
    json_data->HR = sm->last_heart_rate; // Replace with actual heart rate
    json_data->HRV = 110 ; // Replace with actual HRV
    json_data->RR = 35;
    json_data->TEMP = 33;
    json_data->a_x = sm->sensor_data.accel_x; // Replace with actual accelerometer x-axis
    json_data->a_y = sm->sensor_data.accel_y; // Replace with actual accelerometer y-axis
    json_data->a_z = sm->sensor_data.accel_z; // Replace with actual accelerometer z-axis
    json_data->g_x = sm->sensor_data.gyro_x; // Replace with actual gyroscope x-axis
    json_data->g_y = sm->sensor_data.gyro_y; // Replace with actual gyroscope y-axis
    json_data->g_z = sm->sensor_data.gyro_z; // Replace with actual gyroscope z-axis
    json_data->ACT = sm->current_state; // Replace with actual activity level
    strcpy(json_data->BEH, "NONE"); // Replace with actual behavior
    json_data->LAT = 28.7041; // Replace with actual latitude
    json_data->LONG = 70.234; // Replace with actual longitude
    json_data->DIST = 30; // Replace with actual distance traveled
    json_data->CAL = 200; // Replace with actual calories burned
    json_data->AL_VAL = 250; // Replace with actual alert value
    json_data->ALERT = 2; // Replace with actual alert status
    json_data->AL_SEV = 2; // Replace with actual alert severity

    cJSON *j = create_json_packet(json_data);
    if (!j) {
        return NULL;
    }

    char *json_string = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);

    return json_string;
}