//C Headers
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>


//Components

#include "SRA18.h"
#include "TUNING.h"

// #define LS_LEFT 32
// #define LS_RIGHT 33

//Declare the the channel array consisting of the ADC channel inputs

adc1_channel_t channel[4] = {ADC_CHANNEL_7,ADC_CHANNEL_6,ADC_CHANNEL_0,ADC_CHANNEL_3};

uint32_t adc_reading[4];
float sensor_value[4];

static void calc_sensor_values()
{
    for(int i = 0; i < 4; i++)
    {
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
    }

}

static void read_sensors()
{
  	for(int i = 0; i < 4; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
    }
}

void sensor_task(void *arg)
{

	while(1)
	{
		//mapped
		// read_sensors();
		// calc_sensor_values();
		// for(int i=0;i<4;i++)
		// printf("RAW %d: %f\t",i,sensor_value[i]);		
		// printf("\n");

		//raw
		// read_sensors();
		// for(int i=0;i<4;i++)
		// printf("RAW %d: %d\t",i,adc_reading[i]);		
		// printf("\n");

		//ir sensor
		//grey-black-white
		//brown-red-yellow
		// gpio_set_direction(LS_LEFT, GPIO_MODE_INPUT);
		// gpio_set_direction(LS_RIGHT, GPIO_MODE_INPUT);
		// printf("Left: %d\tRight: %d\n", gpio_get_level(LS_LEFT), gpio_get_level(LS_RIGHT));	

		//all-mapped
		read_sensors();
		calc_sensor_values();
		// gpio_set_direction(LS_LEFT, GPIO_MODE_INPUT);
		// gpio_set_direction(LS_RIGHT, GPIO_MODE_INPUT);
		// int left = gpio_get_level(LS_LEFT);
		// int right = gpio_get_level(LS_RIGHT);
		for(int i=0;i<4;i++)
		printf("RAW %d: %f\t",i,sensor_value[i]);
		printf("\n");

		// printf("LEFT: %d\tRIGHT: %d\n", left, right);		
	}
	
}

void app_main()
{
	/*
		Basic Function for task creation
	*/

    xTaskCreate(&sensor_task,"turn task",4096,NULL,1,NULL);
}
