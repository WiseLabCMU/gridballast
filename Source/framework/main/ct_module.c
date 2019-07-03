#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>


#include "Ada_MCP.h" // IO Expander Library
#include "generic_rw_i2c.h"  // generic I2C read/write functions
#include "driver/adc.h"
#include "driver/timer.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "util.h"
#include "esp_adc_cal.h"
#include "ct_module.h"
#include "esp_wifi.h"
#include "soc/sens_reg.h"



#define ESP_INTR_FLAG_DEFAULT 0

#define TAG "gridballast"
//uint8_t p,v;

#define TIMER_DIVIDER   80

#define DEFAULT_VREF 1100

system_state_t mystate;

rwlock_t i2c_lock;

const char * const ct_task_name = "ct_module_task";

static esp_adc_cal_characteristics_t *adc_chars;

static intr_handle_t s_timer_handle;

extern TaskHandle_t wifi_task_handler;
// xQueueHandle adc_queue;


// int flag =0;
// bool ledState = true;


//int timr_group = TIMER_GROUP_1;
//int timr_idx = TIMER_1;
int i=0;
float adc_val[10];
int ct_flag = 1;

extern uint64_t reg_a;
extern uint64_t reg_b;
extern uint64_t reg_c;

/* void IRAM_ATTR timer_group1_isr(void *param) {
	TIMERG1.int_clr_timers.t1 = 1;
	TIMERG1.hw_timer[timr_idx].config.alarm_en = 1;
	ct_flag=1;
}*/
  
int sum = 0;
float curr = 0.0;

/*static void adc_task(void* arg) {
    esp_err_t ret;
	int read_adc2_raw;
     while(1) {
        esp_wifi_stop();
        esp_wifi_deinit();
		 //vTaskSuspend(wifi_task_handler);
         if (ct_flag == 1) {
    	     adc_val[i] = adc1_get_raw(ADC1_CHANNEL_0);
			 ret = adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_9Bit, &read_adc2_raw);
			 if ( ret == ESP_OK ) {
				 printf("ADC2 %d\n",  read_adc2_raw );
			 } else if ( ret == ESP_ERR_INVALID_STATE ) {
				 printf("ADC2 not initialized yet.\n");
			 } else if ( ret == ESP_ERR_TIMEOUT ) {
				 //This can not happen in this example. But if WiFi is in use, such error code could be returned.
				 printf("ADC2 is in use by Wi-Fi.\n");
			 }
		     printf("******* ADC Task current s %f*********\n",	mystate.power);
             //Convert adc_reading to voltage in mV
             uint32_t voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_0), adc_chars);
             printf("Raw: %lf\tVoltage: %dmV\n", adc_val[i], voltage);
           	 adc_val[i] = adc_val[i]*adc_val[i];
             sum += adc_val[i];
             i++;
    	     ct_flag = 0;
    	}
		//wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		//ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
		//vTaskResume(wifi_task_handler);
		//TODO - figure out how to convert to current
    	if(i == 10) {
    	    curr = sqrt(sum/i*1.0);       //RMS current
            rwlock_writer_lock(&system_state_lock);
            get_system_state(&mystate);
            mystate.power = ((curr * curr) / 33000);
            set_system_state(&mystate);
            rwlock_writer_unlock(&system_state_lock);
            sum = 0;
            i=0;
    	}
      vTaskDelay(500/portTICK_PERIOD_MS);
  }
}*/
/*
static void relay_task(void* arg)
{
  while(1)
  {
     rwlock_reader_lock(&system_state_lock);
    get_system_state(&mystate);
    rwlock_reader_unlock(&system_state_lock);

    if (mystate.set_point > 127)
    {
      rwlock_writer_lock(&i2c_lock);  
      begin(0);
      pinMode(4,GPIO_MODE_OUTPUT); 
      digitalWrite(4,1);
      rwlock_writer_unlock(&i2c_lock);  
    }
  }
}*/

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}


 /* Use this function to setup the ADC.
 Input - Channel to be configured
 */
void adc1_config() {
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

	adc1_config_width(ADC_WIDTH_9Bit);
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db);
	
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_BIT_9, DEFAULT_VREF, adc_chars);
	print_char_val_type(val_type);
	//free(adc_chars);

}

void adc2_config() {
    adc2_config_channel_atten( ADC2_CHANNEL_7, ADC_ATTEN_11db );
}

/*void ct_init_task( void ) {
  //adc1_config_width(ADC_WIDTH_9Bit);
  //adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_11db);
  adc1_config();
  adc2_config();
  timer_config_t config;
  config.alarm_en = 1;
  config.auto_reload = 1;
  config.counter_dir = TIMER_COUNT_UP;
  config.divider = TIMER_DIVIDER;
  config.intr_type = TIMER_INTR_LEVEL;
  config.counter_en = false;
  timer_init(timr_group, timr_idx, &config);
  timer_set_alarm_value(timr_group, timr_idx, 5000);
  timer_enable_intr(timr_group, timr_idx);
  timer_isr_register(timr_group, timr_idx, &timer_group1_isr, NULL, 0, &s_timer_handle);
  timer_start(timr_group, timr_idx);
  xTaskCreatePinnedToCore(adc_task, "adc_task", ctUSStackDepth, NULL, ctUXPriority, NULL,0);

  //xTaskCreate(relay_task, "adc_task", 1024, NULL, 10, NULL);


}*/
