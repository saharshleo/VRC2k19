#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>

#include "SRA18.h"
#include "TUNING.h"
//using namespace std;

adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};

int weights[4] = {3,1,-1,-3};

int white = 150;
int black = 400;
int count = 0;
/*
 * Line Following PID Constants
 */
float kP = 1;    //1
float kI = 0;    //0
float kD = 1.5;  //1.5

//Self Balancing Tuning Parameters(no use)
float pitch_kP =  15.1;//5.85;       
float pitch_kI =  0.075;//95;          
float pitch_kD =  9;

float setpoint = -3.5;
float initial_acce_angle = 0;
float forward_angle = 0;

float forward_offset = 2.251;
float forward_buffer = 3;

/*
 * Motor value constraints
 */
float opt = 75;
float lower_pwm_constrain = 65;
float higher_pwm_constrain = 85;
float left_pwm = 0, right_pwm = 0;

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
        if(sensor_value[i] > black)
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

void turnleft90(){
	while(1){
		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, opt, opt);
		read_sensors();
    	calc_sensor_values();
    	if(sensor_value[0]>black && sensor_value[3]>black) break;
	}
    while(1){
        bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 77, 77);
        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 75,0);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]>black && sensor_value[2]>black){
            break;
        }
    }
    while(1){
        bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 77,77);
        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 75,0);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]<white && sensor_value[2]<white){
            break;
        }
    }
}

void turnright90(){
	while(1){
		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, opt, opt);
		read_sensors();
    	calc_sensor_values();
    	if(sensor_value[0]>black && sensor_value[3]>black) break;
	}
    while(1){
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 77, 77);
        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0,75);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]>black && sensor_value[2]>black){
            break;
        }
    }
    while(1){
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 77, 77);
        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0,75);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]<white && sensor_value[2]<white){
            break;
        }
    }
}

void slopeturnright(){
	while(1){
		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, opt, opt);
		read_sensors();
    	calc_sensor_values();
    	if(sensor_value[0]>black) break;
	}
    while(1){
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, opt, opt);
        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0,75);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]>black && sensor_value[2]>black){
            break;
        }
    }
    while(1){
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, opt, opt);
        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0,75);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]<white && sensor_value[2]<white){
            break;
        }
    }
} 

void slope_follow_task(void *arg)
{
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

	    if((count==0 || count==3) && sensor_value[0]<white && (sensor_value[1]<white && sensor_value[2]<white)){
	    	count++;
	    	while(1){
	    		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, opt, opt);
	    		read_sensors();
	    		calc_sensor_values();
	    		if(sensor_value[0]>black) break;
	    	}
	    }
	    else if(count==1 && sensor_value[0]<white && (sensor_value[1]<white && sensor_value[2]<white)){
	    	count++;
	    	turnleft90();
	    }
	    else if(count==2 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
	    	count++;
	    	while(1){
	    		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, opt, opt);
	    		read_sensors();
	    		calc_sensor_values();
	    		if(sensor_value[0]>black && sensor_value[3]>black) break;
	    	}
	    	// opt = 85;
	    }
	    else if(count==4 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
	    	count++;
	    	turnright90();
	    	// slopeturnright();
	    	// opt = 70;
	    }
	    else if(count==5 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
	    	count++;
	    	while(1){
	    		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, opt, opt);
	    		read_sensors();
	    		calc_sensor_values();
	    		if(sensor_value[3]>black) break;
	    	}
	    }
	    else if(count==6 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
	    	count++;
	    	turnright90();
	    	// while(1){
	    	// 	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 77, 77);
	    	// 	read_sensors();
	    	// 	calc_sensor_values();
	    	// 	if(sensor_value[0]>black && sensor_value[3]>black) break;
	    	// }
	    	// opt = 75;
	    }
	    else if(count==6 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
	    	count++;
	    	turnleft90();
	    }
	}
}


void app_main()
{
    xTaskCreate(&slope_follow_task,"slope_follow_task",100000,NULL,1,NULL);
}
