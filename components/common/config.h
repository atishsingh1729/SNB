/*
 * main.h
 *
 *  Created on: Dec 10, 2025
 *      Author: Dheeraj.Jain
 */

#ifndef CONFIG_H_
#define CONFIG_H_



#define I2C_SDA_GPIO 		GPIO_NUM_8
#define I2C_SCL_GPIO 		GPIO_NUM_9
#define I2C_SPEED_HZ      	100000

#define PIN_NUM_MISO  		GPIO_NUM_11
#define PIN_NUM_MOSI  		GPIO_NUM_13
#define PIN_NUM_CLK   		GPIO_NUM_12
#define PIN_NUM_CS    		GPIO_NUM_10
#define PIN_NUM_WP			GPIO_NUM_14
#define PIN_NUM_F_HLD		GPIO_NUM_21

#define MAX30102_ADDR		0x57
#define BNO085_ADDR			0x4B

#define UART_PORT 			UART_NUM_0
#define UART_BUF  			512

#define LED_GPIO 			GPIO_NUM_16


// Set this to true to see AC waveform plot, false for BPM detection
#define PLOT_AC_SIGNAL 				false
#define DEBUG__LOGS 				false

//NTC based Temp 
#define ADC_CHANNEL_BODY			0
#define ADC_CHANNEL_AMB				6
#define ADC_CHANNEL_BATT			5

#define BATT_AVG_SAMPLES  		    16   // 16 cycles × 3 sec = 48 sec smoothing

//Static buffer for CPU usage
#define STATS_BUFFER_SIZE 1024  // Usually 300-1000 bytes

//
#define BNO085_RST_PIN			GPIO_NUM_38
#define BNO085_INT_PIN			GPIO_NUM_48

//
#define STEP_FREQ_UPDATE_WIN_SEC		2.0f
#define VAR_WINDOW_SIZE 				100   // 1 second @100Hz

#endif /* CONFIG_H_ */