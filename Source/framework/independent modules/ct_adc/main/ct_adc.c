
#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <driver/adc.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/timer.h"
#include "math.h"
// #include "esp_intr_alloc.h"

static intr_handle_t s_timer_handle;

#define TIMER_DIVIDER   80

#define TIMER_INTR_SEL TIMER_INTR_LEVEL

xQueueHandle adc_queue;

int timer_group = TIMER_GROUP_0;
int timer_idx = TIMER_0;

int val;
int i=0;

float adc_val[50];

int flag = 0;

bool status = false;


void IRAM_ATTR timer_group0_isr(void *para)
{
	//val = adc1_get_voltage(ADC1_CHANNEL_0);

	//xQueueSendFromISR(adc_queue, &val, NULL);

	TIMERG0.int_clr_timers.t0 = 1;

	TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;

	status = !status;
	gpio_set_level(12, status);
		flag=1;


}


void adc_task(void* arg)
{
	// xQueueReceive(adc_queue, &val, portMAX_DELAY);
	while(1){
    	if (flag == 1){
    		//val = adc1_get_voltage(ADC1_CHANNEL_0);
    		adc_val[i] = adc1_get_voltage(ADC1_CHANNEL_0) * 0.00087;
    		adc_val[i] = adc_val[i]*adc_val[i];
    		i++;

    			//printf("%d\n", val );

    		// xQueueSend(adc_queue, &val, portMAX_DELAY);
    		flag = 0;

    	}
    	if(i== 50){
    		int sum = 0;
    		for(int j=0; j<50; j++){
    			sum+= adc_val[j];
    		}
    		float curr = sqrt(sum/50.0);
    		printf("%2.2f\n", curr);
    	}
    }



}

void init_timer(int timer_period_us)
{
	timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;

    timer_init(timer_group, timer_idx, &config);

    timer_set_alarm_value(timer_group, timer_idx, timer_period_us);

    

    timer_enable_intr(timer_group, timer_idx);

    timer_isr_register(timer_group, timer_idx, &timer_group0_isr, NULL, 0, &s_timer_handle);

    timer_start(timer_group, timer_idx);
}

void app_main()
{

	
	//adc_queue = xQueueCreate(1, sizeof(val));

	adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_11db);

    gpio_pad_select_gpio(12);

    gpio_set_direction(12, GPIO_MODE_OUTPUT);

    //init_timer(4000);

    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = false;

    timer_init(timer_group, timer_idx, &config);

    timer_set_alarm_value(timer_group, timer_idx, 320);

    

    timer_enable_intr(timer_group, timer_idx);

    timer_isr_register(timer_group, timer_idx, &timer_group0_isr, NULL, 0, &s_timer_handle);

    timer_start(timer_group, timer_idx);

    // while(1){
    // 	if (flag == 1){
    // 		val = adc1_get_voltage(ADC1_CHANNEL_0);
    // 		xQueueSend(adc_queue, &val, portMAX_DELAY);
    // 		flag = 0;

    // 	}
    // }

    //printf("hi\n");

    xTaskCreate(adc_task, "adc_task", 2048, NULL, 10, NULL);


    

}