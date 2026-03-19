/*
 * wifi.h
 *
 *  Created on: Jan 27, 2026
 *      Author: Sanjay.Chauhan
 */

#ifndef COMPONENTS_WIFI_WIFI_H_
#define COMPONENTS_WIFI_WIFI_H_



#include <stdint.h>
#include <stdbool.h>


//extern char globl_buff[];

void wifi_init(void);

void wifi_init_softap(void);
void wifi_hello(void);





#endif /* COMPONENTS_WIFI_WIFI_H_ */
