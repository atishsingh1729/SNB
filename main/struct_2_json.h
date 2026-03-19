/*
 * struct_2_json.h
 *
 *  Created on: Dec 24, 2025
 *      Author: Dheeraj.Jain
 */

#ifndef MAIN_STRUCT_2_JSON_H_
#define MAIN_STRUCT_2_JSON_H_

#include <stdint.h>
#include <cJSON.h>
#include "Feature_processing/feature_processing.h"

typedef struct json_packet{

float a_x; //accelerometer x-axis
float a_y; //accelerometer y-axis
float a_z; //accelerometer z-axis
float g_x; //gyroscope x-axis
float g_y; //gyroscope y-axis
float g_z; //gyroscope z-axis

float LAT; //latitude
float LONG; //longitude

uint32_t TS; //timestamp
float HR; //heart rate
uint16_t HRV; //heart rate variability
uint16_t RR; //respiratory rate 
uint16_t TEMP; //temperature
uint16_t DIST; //distance traveled
uint16_t CAL; //calories burned
uint16_t AL_VAL; //alert value (e.g., heart rate value that triggered alert)
uint16_t pkt_ind; //packet index for tracking multiple packets

uint8_t ALERT; //alert code (e.g., 1=high heart rate, 2=low heart rate, etc.)
uint8_t AL_SEV; //alert severity (e.g. 1=concern, 2=warning, 3=critical)
uint8_t BAT; //battery level

uint8_t ACT; //activity level   
char BEH[30]; //behavior
char ID[10]; //device ID
}s_json_packet_t;

cJSON * create_json_packet(const s_json_packet_t *data);
char *handle_json_packet(StateMachine *sm, s_json_packet_t* pkt);

#endif /* MAIN_STRUCT_2_JSON_H_ */
