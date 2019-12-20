//C Headers
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>


//Components

#include "SRA18.h"
#include "TUNING.h"

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
		read_sensors();
		calc_sensor_values();
		for(int i=0;i<4;i++)
		printf("RAW %d: %f\t",i,sensor_value[i]);		
		printf("\n");
	}
	
}

void app_main()
{
	/*
		Basic Function for task creation
	*/

    xTaskCreate(&sensor_task,"blink task",4096,NULL,1,NULL);
}
