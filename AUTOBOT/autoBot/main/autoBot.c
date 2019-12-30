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

//esp pins
#define LR 15     
#define P1 2       
#define P2A 33
#define P2B 18  
#define P3A 19  
#define P3B 21  
#define P4A 22  
#define P4B 32
// #define LS_LEFT 32
// #define LS_RIGHT 33

int lr, p1, p2a, p2b, p3a, p3b, p4a, p4b;
int left, right;

int white = 120;
int black = 350;

//node counting variables
int count1 = 0, count2A = 0, count2B = 0, count3A = 0, count3B = 0, count4A = 0, count4B = 0, endcount = 0;
long int lastcount = 50000000;

// char L = 'l', R = 'r';


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
float forward_speed = 75;
float turning_speed = 77;
float slope_speed_up = 83;
float slope_speed_down = 72;
float opt_backup = 75;
float slow_speed = 73;
float see_saw_speed = 83;

float opt = 75;
float lower_pwm_constrain = 60;
float higher_pwm_constrain = 95;
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

 //    gpio_set_direction(LS_LEFT, GPIO_MODE_INPUT);
	// gpio_set_direction(LS_RIGHT, GPIO_MODE_INPUT);

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


void path_1(){
    read_sensors();
    calc_sensor_values();
    if(count1==1 && p4b==1) opt = see_saw_speed + 5.0;
    else if(count1==1 && p4a==1) opt = see_saw_speed - 3.0;
    else if(count1==1) opt = see_saw_speed;

    if(lr==0){
        if((count1==0 || count1==1 || count1==6 || count1==8) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count1++;
            //add delay if needed
            if(count1==1) opt = opt_backup;
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if((count1==2 || count1==7) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count1++;
            turnleft90();
        }
        else if(count1==3 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count1++;
            turnright90();
        }
        else if(count1==4 && sensor_value[0]>black && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]>black){
            count1++;
        	while(1){
		        bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, turning_speed, turning_speed);
		        read_sensors();
		        calc_sensor_values();
		        if(sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]>black && sensor_value[3]>black){
		            break;
		        }
			}
        }
        else if(count1==5 && (sensor_value[0]<white && sensor_value[3]<white)){
            //ye condition check karna padega
            count1++;
            turnright90();
        }
        //green k liye last condition
        else if(count1==9 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p1==1);
        }
    }

    else if(lr==1){
        if((count1==0 || count1==1 || count1==6 || count1==8) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count1++;
            //add delay if needed
            opt = opt_backup;
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if((count1==2 || count1==7) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count1++;
            turnright90();
        }
        else if(count1==3 && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count1++;
            turnleft90();
        }
        else if(count1==4 && sensor_value[0]>black && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]>black){
            count1++;
        	while(1){
		        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, turning_speed, turning_speed);
		        read_sensors();
		        calc_sensor_values();
		        if(sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]>black && sensor_value[3]>black){
		            break;
		        }
			}
        }
        else if(count1==5 && (sensor_value[0]<white && sensor_value[3]<white)){
            //ye condition check karna padega
            count1++;
            turnleft90();
        }
        //green k liye last condition
        else if(count1==9 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p1==1);
        }
    }
}


void path_2A(){
    read_sensors();
    calc_sensor_values();
    if(count2A==12 && p4b==1) opt = slope_speed_up + 3.0;
    else if(count2A==12 && p4a==1) opt = slope_speed_up - 3.0;
    else if(count2A==12) opt = slope_speed_up;

    if(lr==0){
        if((count2A==0 || count2A==2 || count2A==6 || count2A==11) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count2A++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if(count2A==1 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count2A++;
            turnright90();
        }
        else if(count2A==3 && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2A++;
            turnleft90();
        }
        else if(count2A==4 && sensor_value[0]<white && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]<white){
            count2A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black && sensor_value[3]>black){
                    break;
                }
            }
            opt = slow_speed;
        }
        else if(count2A==5 && (sensor_value[0]<white && sensor_value[3]<white)){
            //ye condition check karna padega
            count2A++;
            opt = opt_backup;
            turnright90();
        }
        else if((count2A==8 || count2A==12) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2A++;
            opt = opt_backup;
            turnleft90();
        }
        else if((count2A==7 || count2A==9 || count2A==10) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2A++;
            //bot forward till single line
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black){
                    break;
                }
            }
        }
        //last k liye condition
        else if(count2A==13 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p2a==1 || (p1==1 && p2b==1 && p3a==1));
        }
    }

    else if(lr==1){
        if((count2A=0 || count2A==2 || count2A==6 || count2A==11) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count2A++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if(count2A==1 && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
            count2A++;
            turnleft90();
        }
        else if(count2A==3 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2A++;
            turnright90();
        }
        else if(count2A==4 && sensor_value[0]<white && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]<white){
            count2A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black && sensor_value[3]>black){
                    break;
                }
            }
            opt = slow_speed;
        }
        else if(count2A==5 && (sensor_value[0]<white && sensor_value[3]<white)){
            //ye condition check karna padega
            count2A++;
            opt = opt_backup;
            turnleft90();
        }
        else if((count2A==8 || count2A==12) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2A++;
            opt = opt_backup;
            turnright90();
        }
        else if((count2A==7 || count2A==9 || count2A==10) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2A++;
            //bot forward till single line
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>black){
                    break;
                }
            }
        }
        //last k liye condition
        else if(count2A==13 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p2a==1 || (p1==1 && p2b==1 && p3a==1));
        }
    }
}


void path_2B(){
    read_sensors();
    calc_sensor_values();
    if(lr==0){
        if((count2B==0 || count2B==2 || count2B==10) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count2B++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if(count2B==1 && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2B++;
            turnright90();
        }
        else if(count2B==3 && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2B++;
            turnleft90();
        }
        else if(count2B==4 && sensor_value[0]<white && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]<white){
            count2B++;
            opt = slow_speed;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black && sensor_value[3]>black){
                    break;
                }
            }
        }
        else if(count2B==5 && (sensor_value[0]<white && sensor_value[3]<white)){
            //ye condition check karna padega
            count2B++;
            opt = opt_backup;
            turnright90();
        }
        else if((count2B==6 || count2B==7) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2B++;
            turnleft90();
        }
        else if((count2B==8 || count2B==9) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2B++;
            //bot forward till single line
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black){
                    break;
                }
            }
        }
        else if(count2B==11 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count2B++;
            turnright90();
        }
        //last k liye condition
        else if(count2B==12 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p2b==1);
        }
    }

    else if(lr==1){
        if((count2B==0 || count2B==2 || count2B==10) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count2B++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if(count2B==1 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2B++;
            turnleft90();
        }
        else if(count2B==3 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2B++;
            turnright90();
        }
        else if(count2B==4 && sensor_value[0]<white && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]<white){
            count2B++;
            opt = slow_speed;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black && sensor_value[3]>black){
                    break;
                }
            }
        }
        else if(count2B==5 && (sensor_value[0]<white && sensor_value[3]<white)){
            //ye condition check karna padega
            count2B++;
            opt = opt_backup;
            turnleft90();
        }
        else if((count2B==6 || count2B==7) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2B++;
            turnright90();
        }
        else if((count2B==8 || count2B==9) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count2B++;
            //bot forward till single line
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>black){
                    break;
                }
            }
        }
        else if(count2B==11 && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count2B++;
            turnleft90();
        }
        //last k liye condition
        else if(count2B==12 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p2b==1);
        }
    }
}

//from RA to OD
void path_3A(){
    read_sensors();
    calc_sensor_values();
    if(count3A==1 && p1==1) opt = slope_speed_down - 3.0;
    else if(count3A==1) opt = slope_speed_down;
 
    if(lr==0){
        if((count3A==0 || count3A==10) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3A++;
            turnright90();
        }
        else if((count3A==1 || count3A==9) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3A++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
            opt = opt_backup;
        }
        else if((count3A==2 || count3A==6) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3A++;
            turnright90();
        }
        else if((count3A==3 || count3A==5 || count3A==7 || count3A==8) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count3A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black){
                    break;
                }
            }
        }
        else if((count3A==4) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count3A++;
            turnleft90();
        }
        //last k liye condition
        else if(count3A==11 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p3a==1);
        }
    }
    else if(lr==1){
        if((count3A==0 || count3A==10) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3A++;
            turnleft90();
        }
        else if((count3A==1 || count3A==9) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3A++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
            opt = opt_backup;
        }
        else if((count3A==2 || count3A==6) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
            count3A++;
            turnleft90();
        }
        else if((count3A==3 || count3A==5 || count3A==7 || count3A==8) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count3A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>black){
                    break;
                }
            }
        }
        else if((count3A==4) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count3A++;
            turnright90();
        }
        //last k liye condition
        else if(count3A==11 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p3a==1);
        }
    }
}

//from OD to RA
void path_3B(){
    read_sensors();
    calc_sensor_values();
    if(count3B==10 && p1==1) opt = slope_speed_up + 3.0;
    else if(count3B==10) opt = slope_speed_up;

    if(lr==0){
        if((count3B==0 || count3B==4 || count3B==8 || count3B==10) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3B++;
            if(count3B==11) opt = opt_backup;
            turnleft90();
        }
        else if((count3B==1 || count3B==9) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3B++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if((count3B==2 || count3B==3 || count3B==5 || count3B==7) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>black){
                    break;
                }
            }
        }
        else if((count3B==6) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3B++;
            turnright90();
        }
        //last k liye condition
        else if(count3B==11 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p3b==1);
        }
    }
    else if(lr==1){
        if((count3B==0 || count3B==4 || count3B==8 || count3B==10) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3B++;
            if(count3B==11) opt = opt_backup;
            turnright90();
        }
        else if((count3B==1 || count3B==9) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count3B++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if((count3B==2 || count3B==3 || count3B==5 || count3B==7) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
            count3B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black){
                    break;
                }
            }
        }
        else if((count3B==6) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
            count3B++;
            turnleft90();
        }
        //last k liye condition
        else if(count3B==11 && sensor_value[1]>white && sensor_value[2]>white){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(p3b==1);
        }
    }
}


void path_4A(){
    read_sensors();
    calc_sensor_values();
    if(count4A==1) opt = slope_speed_down;
    if(lr==0){
        if((count4A==0) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4A++;
            turnright90();
        }
        else if((count4A==1 || count4A==4 || count4A==8 || count4A==12 || count4A==14) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4A++;
            //add delay if needed
            if(count4A==2) opt = opt_backup;
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if((count4A==2 || count4A==6 || count4A==13) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4A++;
            turnright90();
        }
        else if((count4A==3 || count4A==5 || count4A==15) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count4A++;
            turnleft90();
        }
        else if((count4A==7 || count4A==9 || count4A==11) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4A++;
            opt = opt_backup;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>black){
                    break;
                }
            }
        }
        else if((count4A==10) && sensor_value[0]>black && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]>black){
            count4A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[1]<white || sensor_value[2]<white || sensor_value[0]<white || sensor_value[3]<white){
                    break;
                }
            }
            opt = slow_speed; //ye dekhna padega
        }
        //last k liye condition
        else if(count4A==16){
            do{
                // bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                // vTaskDelay(10000/10);
                endcount++;
            }while(endcount!=lastcount);
            do{
            	bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
            	vTaskDelay(10000/10);
            }while(p4a==1);

        }
    }
    else if(lr==1){
        if((count4A==0) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4A++;
            turnleft90();
        }
        else if((count4A==1 || count4A==4 || count4A==8 || count4A==12 || count4A==14) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4A++;
            //add delay if needed
            if(count4A==2) opt = opt_backup;
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if((count4A==2 || count4A==6 || count4A==13) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
            count4A++;
            turnleft90();
        }
        else if((count4A==3 || count4A==5 || count4A==15) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count4A++;
            turnright90();
        }
        else if((count4A==7 || count4A==9 || count4A==11) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
            count4A++;
            opt = opt_backup;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black){
                    break;
                }
            }
        }
        else if((count4A==10) && sensor_value[0]>black && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]>black){
            count4A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[1]<white || sensor_value[2]<white || sensor_value[0]<white || sensor_value[3]<white){
                    break;
                }
            }
            opt = slow_speed; //ye dekhna padega
        }
        //last k liye condition
        else if(count4A==16){
            do{
                // bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                // vTaskDelay(10000/10);
            }while(endcount!=lastcount);
            do{
               bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
            	vTaskDelay(10000/10);
            }while(p4a==1);
        }
    }
}


void path_4B(){
    read_sensors();
    calc_sensor_values();
    if(lr==0){
        if((count4B==0 || count4B==5 || count4B==16) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4B++;
            turnleft90();
        }
        else if((count4B==1 || count4B==9 || count4B==13 || count4B==15) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4B++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if((count4B==2 || count4B==3 || count4B==8 || count4B==10 || count4B==12) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4B++;
            opt = opt_backup;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>black){
                    break;
                }
            }
        }
        else if((count4B==4 || count4B==7) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4B++;
            turnright90();
        }
        else if((count4B==6) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black){
                    break;
                }
            }
        }
        else if((count4B==11) && sensor_value[0]>black && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]>black){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[1]<white || sensor_value[2]<white || sensor_value[0]<white || sensor_value[3]<white){
                    break;
                }
            }
            opt = slow_speed;
        }
        else if((count4B==14) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4B++;
            turnright90();
        }
        //last k liye condition
        else if(count4B==17){
            do{
                // bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                // vTaskDelay(10000/10);
                endcount++;
            }while(endcount!=lastcount);
            do{
            	bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
            	vTaskDelay(10000/10);
            }while(p4b==1);
        }
    }
    else if(lr==1){
        if((count4B==0 || count4B==5 || count4B==16) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4B++;
            turnright90();
        }
        else if((count4B==1 || count4B==9 || count4B==13 || count4B==15) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4B++;
            //add delay if needed
            while(1){
            	bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
            	read_sensors();
            	calc_sensor_values();
            	if(sensor_value[0]>black && sensor_value[3]>black) break;
            }
        }
        else if((count4B==2 || count4B==3 || count4B==8 || count4B==10 || count4B==12) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
            count4B++;
            opt = opt_backup;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>black){
                    break;
                }
            }
        }
        else if((count4B==4 || count4B==7) && sensor_value[0]<white && sensor_value[1]<white && sensor_value[2]<white && sensor_value[3]<white){
            count4B++;
            turnleft90();
        }
        else if((count4B==6) && sensor_value[3]<white && sensor_value[1]<white && sensor_value[2]<white){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>black){
                    break;
                }
            }
        }
        else if((count4B==11) && sensor_value[0]>black && sensor_value[1]>black && sensor_value[2]>black && sensor_value[3]>black){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, forward_speed, forward_speed);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[1]<white || sensor_value[2]<white || sensor_value[3]<white || sensor_value[0]<white){
                    break;
                }
            }
            opt = slow_speed;
        }
        else if((count4B==14) && sensor_value[1]<white && sensor_value[2]<white && sensor_value[0]<white){
            count4B++;
            turnleft90();
        }
        //last k liye condition
        else if(count4B==17){
            do{
                // bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                // vTaskDelay(10000/10);
                endcount++;
            }while(endcount!=lastcount);
            do{
            	bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
            	vTaskDelay(10000/10);
            }while(p4b==1);
        }
    }
}  


void line_follow_task(void *arg)
{
	mcpwm_initialize();

  	while(1)
	{
		//setting up buttons
		button_init();
		lr = gpio_get_level(LR);
		p1 = gpio_get_level(P1);
		p2a = gpio_get_level(P2A);
		p2b = gpio_get_level(P2B);
		p3a = gpio_get_level(P3A);
		p3b = gpio_get_level(P3B);
		p4a = gpio_get_level(P4A);
		p4b = gpio_get_level(P4B);

		//setting up sensors
		// left = gpio_get_level(LS_LEFT);
		// right = gpio_get_level(LS_RIGHT);
	    read_sensors();
	    calc_sensor_values();

	    //pid
	    calculate_error();
	    calculate_correction();
	    left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
	    right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
	    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);

	    //conditions
        if(p1==1 && (p4a==1 || p4b==1)) path_1();

        else if(((p2a==1) || (p2b==1 && p3a==1)) && (p4a==1 || p4b==1)) path_2A();
        else if(p1==1) path_1();
        else if(p3a==1 && p1==1) path_3A();
        else if(p3b==1 && p1==1) path_3B();

        else if((p2a==1) || (p2b==1 && p3a==1)) path_2A();
        else if(p2b==1) path_2B();
        else if(p3a==1) path_3A();
        else if(p3b==1) path_3B();
        else if(p4a==1) path_4A();
        else if(p4b==1) path_4B();
	}

}

void app_main()
{
	xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
}
