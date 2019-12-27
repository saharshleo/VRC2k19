//#include <bits/stdc++.h>
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

#define LR 15     
#define P1 2       
#define P2A 4
#define P2B 18  
#define P3A 19  
#define P3B 21  
#define P4A 22  
#define P4B 5
#define LS_LEFT 32
#define LS_RIGHT 33

int lr, p1, p2a, p2b, p3a, p3b, p4a, p4b;

int white = 150;
int black = 400;
int count = 0;
int count2 = 0;

adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};

int weights[4] = {3,1,-1,-3};

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
float forward_speed = 75;
float turning_speed = 80;
float slope_speed_up = 90;
float slope_speed_down = 73;
float slow_speed = 73;

float opt = 75;
float lower_pwm_constrain = 60;
float higher_pwm_constrain = 90;
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
    gpio_set_direction(LS_LEFT, GPIO_MODE_INPUT);
    gpio_set_direction(LS_RIGHT, GPIO_MODE_INPUT);
}

void button_init(){
	gpio_set_direction(LR,GPIO_MODE_INPUT);
	gpio_set_direction(P1,GPIO_MODE_INPUT);
	gpio_set_direction(P2A,GPIO_MODE_INPUT);
	gpio_set_direction(P2B,GPIO_MODE_INPUT);
	gpio_set_direction(P3A,GPIO_MODE_INPUT);
	gpio_set_direction(P3B,GPIO_MODE_INPUT);
	gpio_set_direction(P4A,GPIO_MODE_INPUT);
	gpio_set_direction(P4B,GPIO_MODE_INPUT);    
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
		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
		read_sensors();
    	calc_sensor_values();
    	if(sensor_value[0]>black && sensor_value[3]>black) break;
	}
    while(1){
        bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, turning_speed, turning_speed);
        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 75,0);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]>black && sensor_value[2]>black){
            break;
        }
    }
    while(1){
        bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, turning_speed,turning_speed);
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
		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
		read_sensors();
    	calc_sensor_values();
    	if(sensor_value[0]>black && sensor_value[3]>black) break;
	}
    while(1){
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, turning_speed, turning_speed);
        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0,75);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]>black && sensor_value[2]>black){
            break;
        }
    }
    while(1){
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, turning_speed, turning_speed);
        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0,75);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]<white && sensor_value[2]<white){
            break;
        }
    }
}

void path1(){
	read_sensors();
	calc_sensor_values();
	if(count==0 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    	count++;
		    	while(1){
		    		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
		    		read_sensors();
		    		calc_sensor_values();
		    		if(sensor_value[0]>black && sensor_value[3]>black) break;
		    	}
		    }
		    else if(count==1 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
		    	count++;
		    	turnleft90();
		    }
		    else if(count==2 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    	count++;
		    	turnright90();
		    }
		    else if(count==3 && sensor_value[0]>black && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]>black){
		    	count++;
		    	//turnleft90();
		    	while(1){
			        bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, turning_speed, turning_speed);
			        //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0,75);
			        read_sensors();
			        calc_sensor_values();
			        if(sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]>black && sensor_value[3]>black){
			            break;
			        }
    			}
		    }
		    else if(count==4 && sensor_value[0]<white && sensor_value[3]<white){
		    	count++;
		    	turnright90();
		    }
}

void path2(){
	read_sensors();
	calc_sensor_values();
	if(count2==0 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
    	count2++;
    	while(1){
    		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
    		read_sensors();
    		calc_sensor_values();
    		if(sensor_value[0]>black && sensor_value[3]>black) break;
    	}
    }
	else if((count2==1 || count2==2) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
		count2++;
		turnleft90();
	}
	else if((count2==3 || count2==4) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
		count2++;
		turnright90();
	}
	else if(count2==5 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
		count2++;
		turnright90();
	}

}

void line_follow_task(void *arg)
{
	//enable_buttons();
	mcpwm_initialize();
	button_init();
	lr = gpio_get_level(LR);
	p1 = gpio_get_level(P1);
	p2a = gpio_get_level(P2A);
	p2b = gpio_get_level(P2B);
	p3a = gpio_get_level(P3A);
	p3b = gpio_get_level(P3B);
	p4a = gpio_get_level(P4A);
	p4b = gpio_get_level(P4B);

	// if(lr==1 && p1==1 && p2a==1) {
	  	while(1)
		{
			// gpio_set_direction(LS_LEFT, GPIO_MODE_INPUT);
			// gpio_set_direction(LS_RIGHT, GPIO_MODE_INPUT);
		    read_sensors();
		    int left = gpio_get_level(LS_LEFT);
		    int right = gpio_get_level(LS_RIGHT);
		    calc_sensor_values();
		    calculate_error();
		    calculate_correction();
		    left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
		    right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
		    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);

		    if(p4b==1) path1();
		    else if(p1==1) path2();

		    /* **** */
		    // if(sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	//turnright90();
		    // 	 turnleft90();
		    // }
		    // else if(sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && left==0){
		    // 	//turnleft90();
		    // 	turnright90();

		    // }
		    // else if(sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white && right==0){
		    // 	//turnright90();
		    // 	turnleft90();
		    // }

		    /* **** */
		    // if(sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	while(1){
			   //  	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
			   //  	read_sensors();
			   //  	calc_sensor_values();
			   //  	if(sensor_value[0]>black && sensor_value[3]>black) break;
			   //  }
		    // }

		    /* **** */
		    // if(count==0 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	printf("1st if: %d\n", count);
		    // 	turnleft90();
		    // }
		    // else if((count==1 || count==4 || count==5) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	printf("2nd if: %d\n", count);
		    // 	turnright90();
		    // }	    
		    // else if(count==2 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	printf("3rd if: %d\n", count);
		    // 	while(1){
		    // 		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
		    // 		read_sensors();
		    // 		calc_sensor_values();
		    // 		if(sensor_value[3]>black) break;
		    // 	}
		    // }
		    // else if(count==3 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	printf("4th if: %d\n", count);
		    // 	while(1){
		    // 		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
		    // 		read_sensors();
		    // 		calc_sensor_values();
		    // 		if(sensor_value[0]>black && sensor_value[3]>black) break;
		    // 	}	
		    // }
		    // else if(count==6 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	turnleft90();
		    // }
		    /* END OF PATH1*/


		    /* PATH2 */
		    // if(count==0 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	turnleft90();
		    // }
		    // else if((count==1 || count==2) && (sensor_value[1]<white && sensor_value[2]<white) && sensor_value[3]<white){
		    // 	count++;
		    // 	turnright90();
		    // }	
		    // else if(count==3 && sensor_value[0]<white && sensor_value[2]<white && sensor_value[1]<white){
		    // 	count++;
		    // 	turnleft90();
		    // }
		    // else if(count ==4 && sensor_value[0]<white && sensor_value[3]<white)
		    // {
		    // 	count++;
		    // 	turnleft90();
		    // }


		    /*PATH BLIND */
		    // if(count==0 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	turnright90();
		    // }
		    // else if(count==1 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
		    // 	count++;
		    // 	turnleft90();
		    // } 
		    // else if(count==2 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	while(1){
		    // 		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
		    // 		read_sensors();
		    // 		calc_sensor_values();
		    // 		if(sensor_value[0]>black && sensor_value[3]>black) break;
		    // 	}
		    // }
		    // else if((count==4) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	while(1){
		    // 		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
		    // 		read_sensors();
		    // 		calc_sensor_values();
		    // 		if(sensor_value[3]>black) break;
		    // 	}
		    // }
		    // else if(count==3 && sensor_value[0]>black && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]>black){
		    // 	count++;
		    // 	while(1){
		    // 		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 73, 73);
		    // 		read_sensors();
		    // 		calc_sensor_values();
		    // 		if(sensor_value[0]<white || sensor_value[1]<white || sensor_value[2]<white || sensor_value[3]<white) break;
		    // 	}
		    // }
		    // else if(count==5 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	turnright90();
		    // }

		    /* CROSS JUNCTION */
		    // if(count==0 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	while(1){
		    // 		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
		    // 		read_sensors();
		    // 		calc_sensor_values();
		    // 		if(sensor_value[0]>black && sensor_value[3]>black) break;
		    // 	}
		    // }
		    // else if(count==1 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
		    // 	count++;
		    // 	turnleft90();
		    // }
		    // else if(count==2 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	turnright90();
		    // }
		    // else if(count==3 && sensor_value[0]>black && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]>black){
		    // 	count++;
		    // 	//turnleft90();
		    // 	while(1){
			   //      bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, turning_speed, turning_speed);
			   //      //bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 0,75);
			   //      read_sensors();
			   //      calc_sensor_values();
			   //      if(sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]>black && sensor_value[3]>black){
			   //          break;
			   //      }
    		// 	}
		    // }
		    // else if(count==4 && sensor_value[0]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	turnright90();
		    // }

		    /* CROSS STRAIGHT */
		    // if(count==0 && sensor_value[0]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	while(1){
		    // 		bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
		    // 		read_sensors();
		    // 		calc_sensor_values();
		    // 		if(sensor_value[0]>black || sensor_value[3]>black) break;
		    // 	}
		    // }
		    // else if(count==1 && sensor_value[0]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	turnright90();
		    // }

		    /* SEE-SAW */
		    // if(count==0 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
		    // 	count++;
		    // 	turnright90();
		    // }
		}
	// }
}


void app_main()
{
    xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
}