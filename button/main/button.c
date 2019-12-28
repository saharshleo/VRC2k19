//C Headers
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>


//Components

#include "SRA18.h"
#include "TUNING.h"

//32, 4, 19, 2, 21, *0, 23
//33, 18, *5, 15, 22

// #define LS_LEFT 32
// #define LS_RIGHT 33
#define LR 15	//fix	
#define P1 2	//fix	
#define P2A 4
#define P2B 18	//fix
#define P3A 19	//fix
#define P3B 21	//fix
#define P4A 22	//fix
#define P4B 32

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

void button_task(void *arg)
{
	button_init();
	while(1)
	{
		int lr = gpio_get_level(LR);
		int p1 = gpio_get_level(P1);
		int p2a = gpio_get_level(P2A);
		int p2b = gpio_get_level(P2B);
		int p3a = gpio_get_level(P3A);
		int p3b = gpio_get_level(P3B);
		int p4a = gpio_get_level(P4A);
		int p4b = gpio_get_level(P4B);
		printf("LR: %d\tP1: %d\tP2A: %d\tP2B: %d\tP3A: %d\tP3B: %d\tP4A: %d\tP4B: %d\n", lr, p1, p2a, p2b, p3a, p3b, p4a, p4b);
		// printf("LR: %d\tP1: %d\n", lr, p1);
		vTaskDelay(10 / 10);
	}
	
}

void app_main()
{
	/*
		Basic Function for task creation
	*/

    xTaskCreate(&button_task,"buton task",4096,NULL,1,NULL);
}
