//#include <bits/stdc++.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "SRA18.h"
#include "TUNING.h"
//using namespace std;


adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};

int weights[4] = {3,1,-1,-3};

/*
 * Line Following PID Constants
 */
#define kP 1
#define kI 0
#define kD 1.5

/*
 * Motor value constraints
 */
float opt = 75;
float lower_pwm_constrain = 65;
float higher_pwm_constrain = 85;
float left_pwm = 0, right_pwm = 0;
int x,y,fx,node;

/*
 * Line Following PID Variables
 */
float error=0, prev_error, difference, cumulative_error, correction;

uint32_t adc_reading[4];
float sensor_value[4];


static void read_sensors()
{
  for(int i = 0; i < 4; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
    }
}

// static void extra_sensors()
// {

//      gpio_set_direction(GPIO_NUM4,GPIO_MODE_INPUT);
//      gpio_set_direction(GPIO_NUM5,GPIO_MODE_INPUT);
//      gpio_set_direction(GPIO_NUM6,GPIO_MODE_INPUT);

//      fx=gpio_get_level(GPIO_NUM4);  //left32
//      y=gpio_get_level(GPIO_NUM5);  //right19
//      x=gpio_get_level(GPIO_NUM6);  //front18

// }

static void calc_sensor_values()
{
    for(int i = 0; i < 4; i++)
    {
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
    }

}

static void calculate_error()
{
    int all_black_flag = 1;
    float weighted_sum = 0, sum = 0, pos = 0;

    for(int i = 0; i < 4; i++)
    {
        if(sensor_value[i] > 400)
        {
            all_black_flag = 0;
        }

        weighted_sum += (float)(sensor_value[i]) * (weights[i]);
        sum += sensor_value[i];

    }

    if(sum != 0)
    {
        pos = weighted_sum / sum;
    }

    if(all_black_flag == 1)
    {
        if(error > 0)
            pos = 2.5;
        else
            pos = -2.5;
    }

    error = pos;

}

static void calculate_correction()
{
    error *= 10;
    difference = (error - prev_error);
    cumulative_error += error;

    if(cumulative_error > 30)
    {
        cumulative_error = 30;
    }

    else if(cumulative_error < -30)
    {
        cumulative_error = -30;
    }

    correction = kP*error + kI*cumulative_error + kD*difference;
    prev_error = error;
}


void line_follow_task(void *arg)
{
	enable_buttons();
	mcpwm_initialize();

  	while(1)
	{
	    read_sensors();

	    calc_sensor_values();
	    calculate_error();
	    calculate_correction();
	    left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
	    right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
	    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
	}
	while(1)
	{
	    read_sensors();

	    calc_sensor_values();
	    calculate_error();
	    calculate_correction();
	    left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
	    right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
	    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
	}

}

void app_main()
{
	xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
}
