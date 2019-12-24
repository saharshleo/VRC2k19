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

adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};

int weights[4] = {3,1,-1,-3};

int black = 400;
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
float opt = 80;
float lower_pwm_constrain = 65;
float higher_pwm_constrain = 95; //****************** |:D |
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

void line_follow_task(void *arg)
{
	//enable_buttons();
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
}

//Create an HTTP server to tune variables wirelessly 
// void http_server(void *arg)
// {
//     printf("%s\n", "http task");
//     struct netconn *conn, *newconn;
//     err_t err;
//     conn = netconn_new(NETCONN_TCP);
//     netconn_bind(conn, NULL, 80);
//     netconn_listen(conn);
//     do {
//      err = netconn_accept(conn, &newconn);
//      if (err == ERR_OK) {
//        http_server_netconn_serve(newconn,&setpoint,&pitch_kP,&pitch_kD,&pitch_kI,&kP,&kD,&kI, &forward_offset, &forward_buffer);
//        netconn_delete(newconn);
//      }
//     } while(err == ERR_OK);
//     netconn_close(conn);
//     netconn_delete(conn);
// }


void app_main()
{
    // initialise_wifi();
    xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
    // xTaskCreate(&http_server, "http_server", 10000, NULL, 2, NULL);
}
