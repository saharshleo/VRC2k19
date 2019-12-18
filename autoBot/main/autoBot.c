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
#define button1 GPIO_NUM4     //Right/left
#define button2 GPIO_NUM5     //Path1
#define button3 0     //Path2A
#define button4     //Path2B
#define button5     //Path3A
#define button6     //Path3B
#define button7     //Path4A
#define button8     //Path4B
#define LS_LEFT
#define LS_RIGHT
// #define button9     //extra

//node counting variables
int count1 = 0, count2A = 0, count2B = 0, count3A = 0, count3B = 0, count4A = 0, count4B = 0;

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

void turnleft90(){
    while(1){
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]>400 && sensor_value[2]>400){
            break;
        }
    }
    while(1){
        bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]<150 && sensor_value[2]<150){
            break;
        }
    }
}

void turnright90(){
    while(1){
        bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]>400 && sensor_value[2]>400){
            break;
        }
    }
    while(1){
        bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]<150 && sensor_value[2]<150){
            break;
        }
    }
}

void turnright45(){

}

void turnleft45(){

}

void crosspeturn(){
    //isme left right ka condition daalna h
}


void path_1(){
    read_sensors();
    calc_sensor_values();
    if(button1==0){
        if((count1==0 || count1==1 || count1==6 || count1==8) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count1++;
            //add delay if needed
        }
        else if((count1==2 || count1==7) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count1++;
            turnleft90();
        }
        else if(count1==3 && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count1++;
            turnright45();
        }
        else if(count1==4 && sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400){
            count1++;
            turnleft45();
        }
        else if(count1==5 && (sensor[0]<150 || sensor_value[3]<150) && sensor_value[1]<150 && sensor_value[2]<150){
            //ye condition check karna padega
            count1++;
            turnright45();
        }
        //green k liye last condition
        else if(count1==9 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button2==1);
        }
    }

    else if(button1==1){
        if((count1==0 || count1==1 || count1==6 || count1==8) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count1++;
            //add delay if needed
        }
        else if((count1==2 || count1==7) && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count1++;
            turnright90();
        }
        else if(count1==3 && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count1++;
            turnleft45();
        }
        else if(count1==4 && sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400){
            count1++;
            turnright45();
        }
        else if(count1==5 && (sensor[0]<150 || sensor_value[3]<150) && sensor_value[1]<150 && sensor_value[2]<150){
            //ye condition check karna padega
            count1++;
            turnleft45();
        }
        //green k liye last condition
        else if(count1==9 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button2==1);
        }
    }
}


void path_2A(){
    read_sensors();
    calc_sensor_values();
    if(button1==0){
        if((count2A==0 || count2A==2 || count2A==6 || count2A==11) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count2A++;
            //add delay if needed
        }
        else if(count2A==1 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count2A++;
            turnright90();
        }
        else if(count2A==3 && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2A++;
            turnleft45();
        }
        else if(count2A==4 && sensor_value[0]<150 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]<150){
            count2A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400 && sensor_value[3]>400){
                    break;
                }
            }
        }
        else if(count2A==5 && (sensor[0]<150 || sensor_value[3]<150) && sensor_value[1]<150 && sensor_value[2]<150){
            //ye condition check karna padega
            count2A++;
            turnright45();
        }
        else if((count2A==8 || count2A==12) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2A++;
            turnleft90();
        }
        else if((count2A==7 || count2A==9 || count2A==10) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2A++;
            //bot forward till single line
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400){
                    break;
                }
            }
        }
        //last k liye condition
        else if(count2A==13 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button3==1);
        }
    }

    else if(button1==1){
        if((count2A=0 || count2A==2 || count2A==6 || count2A==11) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count2A++;
            //add delay if needed
        }
        else if(count2A==1 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[0]<150){
            count2A++;
            turnleft90();
        }
        else if(count2A==3 && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2A++;
            turnright45();
        }
        else if(count2A==4 && sensor_value[0]<150 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]<150){
            count2A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400 && sensor_value[3]>400){
                    break;
                }
            }
        }
        else if(count2A==5 && (sensor[0]<150 || sensor_value[3]<150) && sensor_value[1]<150 && sensor_value[2]<150){
            //ye condition check karna padega
            count2A++;
            turnleft45();
        }
        else if((count2A==8 || count2A==12) && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2A++;
            turnright90();
        }
        else if((count2A==7 || count2A==9 || count2A==10) && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2A++;
            //bot forward till single line
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400){
                    break;
                }
            }
        }
        //last k liye condition
        else if(count2A==13 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button3==1);
        }
    }
}


void path_2B(){
    read_sensors();
    calc_sensor_values();
    if(button1==0){
        if((count2B==0 || count2B==2 || count2B==10) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count2B++;
            //add delay if needed
        }
        else if(count2B==1 && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2B++;
            turnright90();
        }
        else if(count2B==3 && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2B++;
            turnleft45();
        }
        else if(count2B==4 && sensor_value[0]<150 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]<150){
            count2B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400 && sensor_value[3]>400){
                    break;
                }
            }
        }
        else if(count2B==5 && (sensor[0]<150 || sensor_value[3]<150) && sensor_value[1]<150 && sensor_value[2]<150){
            //ye condition check karna padega
            count2B++;
            turnright45();
        }
        else if((count2B==6 || count2B==7) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2B++;
            turnleft90();
        }
        else if((count2B==8 || count2B==9) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2B++;
            //bot forward till single line
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400){
                    break;
                }
            }
        }
        else if(count2B==11 && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count2B++;
            turnright90();
        }
        //last k liye condition
        else if(count2B==12 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button4==1);
        }
    }

    else if(button1==1){
        if((count2B==0 || count2B==2 || count2B==10) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count2B++;
            //add delay if needed
        }
        else if(count2B==1 && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2B++;
            turnleft90();
        }
        else if(count2B==3 && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2B++;
            turnright45();
        }
        else if(count2B==4 && sensor_value[0]<150 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]<150){
            count2B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400 && sensor_value[3]>400){
                    break;
                }
            }
        }
        else if(count2B==5 && (sensor[0]<150 || sensor_value[3]<150) && sensor_value[1]<150 && sensor_value[2]<150){
            //ye condition check karna padega
            count2B++;
            turnleft45();
        }
        else if((count2B==6 || count2B==7) && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2B++;
            turnright90();
        }
        else if((count2B==8 || count2B==9) && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count2B++;
            //bot forward till single line
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400){
                    break;
                }
            }
        }
        else if(count2B==11 && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count2B++;
            turnleft90();
        }
        //last k liye condition
        else if(count2B==12 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button4==1);
        }
    }
}

//from RA to OD
void path_3A(){
    read_sensors();
    calc_sensor_values();
    if(button1==0){
        if((count3A==0 || count3A==10) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3A++;
            turnright90();
        }
        else if((count3A==1 || count3A==9) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3A++;
            //add delay if needed
        }
        else if((count3A==2 || count3A==6) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3A++;
            turnright90();
        }
        else if((count3A==3 || count3A==5 || count3A==7 || count3A==8) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count3A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400){
                    break;
                }
            }
        }
        else if((count3A==4) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count3A++;
            turnleft90();
        }
        //last k liye condition
        else if(count3A==11 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button5==1);
        }
    }
    else if(button1==1){
        if((count3A==0 || count3A==10) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3A++;
            turnleft90();
        }
        else if((count3A==1 || count3A==9) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3A++;
            //add delay if needed
        }
        else if((count3A==2 || count3A==6) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[0]<150){
            count3A++;
            turnleft90();
        }
        else if((count3A==3 || count3A==5 || count3A==7 || count3A==8) && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count3A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>400){
                    break;
                }
            }
        }
        else if((count3A==4) && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count3A++;
            turnright90();
        }
        //last k liye condition
        else if(count3A==11 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button5==1);
        }
    }
}

//from OD to RA
void path_3B(){
    read_sensors();
    calc_sensor_values();
    if(button1==0){
        if((count3B==0 || count3B==4 || count3B==8 || count3B==10) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3B++;
            turnleft90();
        }
        else if((count3B==1 || count3B==9) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3B++;
            //add delay if needed
        }
        else if((count3B==2 || count3B==3 || count3B==5 || count3B==7) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>400){
                    break;
                }
            }
        }
        else if((count3B==6) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3B++;
            turnright90();
        }
        //last k liye condition
        else if(count3B==11 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button6==1);
        }
    }
    else if(button1==1){
        if((count3B==0 || count3B==4 || count3B==8 || count3B==10) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3B++;
            turnright90();
        }
        else if((count3B==1 || count3B==9) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count3B++;
            //add delay if needed
        }
        else if((count3B==2 || count3B==3 || count3B==5 || count3B==7) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[0]<150){
            count3B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400){
                    break;
                }
            }
        }
        else if((count3B==6) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[0]<150){
            count3B++;
            turnleft90();
        }
        //last k liye condition
        else if(count3B==11 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button6==1);
        }
    }
}


void path_4A(){
    read_sensors();
    calc_sensor_values();
    if(button1==0){
        if((count4A==0) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4A++;
            turnright90();
        }
        else if((count4A==1 || count4A==4 || count4A==8 || count4A==12 || count4A==14) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4A++;
            //add delay if needed
        }
        else if((count4A==2 || count4A==6 || count4A==13) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4A++;
            turnright90();
        }
        else if((count4A==3 || count4A==5 || count4A==15) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count4A++;
            turnleft90();
        }
        else if((count4A==7 || count4A==9 || count4A==11) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>400){
                    break;
                }
            }
        }
        else if((count4A==10) && sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400){
            count4A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[1]<150 || sensor_value[2]<150 || sensor_value[0]<150 || sensor_value[3]<150){
                    break;
                }
            }
        }
        //last k liye condition
        else if(count4A==16 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button7==1);
        }
    }
    else if(button1==1){
        if((count4A==0) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4A++;
            turnleft90();
        }
        else if((count4A==1 || count4A==4 || count4A==8 || count4A==12 || count4A==14) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4A++;
            //add delay if needed
        }
        else if((count4A==2 || count4A==6 || count4A==13) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[0]<150){
            count4A++;
            turnleft90();
        }
        else if((count4A==3 || count4A==5 || count4A==15) && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count4A++;
            turnright90();
        }
        else if((count4A==7 || count4A==9 || count4A==11) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[0]<150){
            count4A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400){
                    break;
                }
            }
        }
        else if((count4A==10) && sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400){
            count4A++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[1]<150 || sensor_value[2]<150){
                    break;
                }
            }
        }
        //last k liye condition
        else if(count4A==16 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button7==1);
        }
    }
}


void path_4B(){
    read_sensors();
    calc_sensor_values();
    if(button1==0){
        if((count4B==0 || count4B==5 || count4B==16) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4B++;
            turnleft90();
        }
        else if((count4B==1 || count4B==9 || count4B==13 || count4B==15) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4B++;
            //add delay if needed
        }
        else if((count4B==2 || count4B==3 || count4B==8 || count4B==10 || count4B==12) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>400){
                    break;
                }
            }
        }
        else if((count4B==4 || count4B==7) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4B++;
            turnright90();
        }
        else if((count4B==6) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400){
                    break;
                }
            }
        }
        else if((count4B==11) && sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[1]<150 || sensor_value[2]<150){
                    break;
                }
            }
        }
        else if((count4B==14) && sensor[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4B++;
            turnright90();
        }
        //last k liye condition
        else if(count4B==17 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button8==1);
        }
    }
    else if(button1==1){
        if((count4B==0 || count4B==5 || count4B==16) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4B++;
            turnright90();
        }
        else if((count4B==1 || count4B==9 || count4B==13 || count4B==15) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4B++;
            //add delay if needed
        }
        else if((count4B==2 || count4B==3 || count4B==8 || count4B==10 || count4B==12) && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[0]<150){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[0]>400){
                    break;
                }
            }
        }
        else if((count4B==4 || count4B==7) && sensor_value[0]<150 && sensor_value[1]<150 && sensor_value[2]<150 && sensor_value[3]<150){
            count4B++;
            turnleft90();
        }
        else if((count4B==6) && sensor_value[3]<150 && sensor_value[1]<150 && sensor_value[2]<150){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[3]>400){
                    break;
                }
            }
        }
        else if((count4B==11) && sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400){
            count4B++;
            while(1){
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                read_sensors();
                calc_sensor_values();
                if(sensor_value[1]<150 || sensor_value[2]<150){
                    break;
                }
            }
        }
        else if((count4B==14) && ssensor[1]<150 && sensor_value[2]<150 && sensor_value[0]<150){
            count4B++;
            turnleft90();
        }
        //last k liye condition
        else if(count4B==17 && sensor_value[1]>150 && sensor_value[2]>150){
            do{
                bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                vTaskDelay(10000/10);
            }while(button8==1);
        }
    }
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
        if(button2==1) path_1();
        else if(button3==1) path_2A();
        else if(button4==1) path_2B();
        else if(button5==1) path_3A();
        else if(button6==1) path_3B();
        else if(button7==1) path_4A();
        else if(button8==1) path_4B();
	}

}

void app_main()
{
	xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
}
