/*
 * datatypes.h
 *
 *  Created on: Feb 23, 2026
 *      Author: Atish.Singh
 */

#ifndef MAIN_DATATYPES_H_
#define MAIN_DATATYPES_H_


#include "stdio.h"
#include "stdint.h"
#include "config.h"
#include <time.h>
 
 typedef struct {
	 float raw_volt_body,raw_volt_amb,raw_volt_battery;
	 uint8_t batt_index;
	 uint8_t batt_count;
	 float batt_percent_buffer[BATT_AVG_SAMPLES];
	 float Batt_percentage;	
	 float amb_temp;
	 float body_temp;	 
 }ADC_Variables;

 typedef struct {
	time_t cloud_epoch_time;
	uint64_t curr_system_uptime_us;
	uint64_t last_reboot_time_us;	
 }systemTime;



#endif /* MAIN_DATATYPES_H_ */
