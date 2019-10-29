#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "Ada_MCP.h" // IO Expander Library
#include "driver/adc.h"
#include "generic_rw_i2c.h"  // generic I2C read/write functions
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "rwlock.h"
#include "u8g2.h" // LCD driver library
#include "u8g2_esp32_hal.h" // ESP32 HAL library for u8g2
#include "util.h"  
#include "button.h"

#define PIN_MCP_RESET 2
#define PIN_SDA 25
#define PIN_SCL 26
#define LEVEL_HIGH 1
#define _I2C_MASTER_FREQ_HZ     100000     /* I2C master clock frequency */
#define TAG "gridballast"

static system_state_t mystate;
static enum lcd_display_type current_display_state;

/* Displays information about current state of variables 
   Under this state, the variables are read only
 */
static void lcd_display_info (system_state_t *mystate, u8g2_t *u8g2) {
    float freq = mystate->grid_freq;
    float pwr = mystate->power;
    int temp_top = mystate->temp_top;
    int temp_bottom = mystate->temp_bottom;
	int temp_set_point = mystate->set_point;
    int mode = mystate->input_mode;
    int heating_status = mystate->heating_status;
    char str [32];    

	rwlock_writer_lock(&i2c_lock);
	
	sprintf(str, "Freq:%2.4fHz", freq);
	u8g2_DrawStr(u8g2, 10, 10, str);

	sprintf(str,"Pwr:%2.2fW", pwr);
	u8g2_DrawStr(u8g2, 10, 25, str);
	
	sprintf(str, "H:%d", heating_status);
	u8g2_DrawStr(u8g2, 95, 25, str);

	sprintf(str,"Ts:%dF",temp_set_point);
	u8g2_DrawStr(u8g2, 10, 40, str);
	
	sprintf(str,"M:%d ",mode);
	u8g2_DrawStr(u8g2, 80, 40, str);
	
	sprintf(str,"Tt:%dF",temp_top);
	u8g2_DrawStr(u8g2, 10, 55, str);
	
	sprintf(str,"Tb:%dF",temp_bottom);
	u8g2_DrawStr(u8g2, 80, 55, str);
		
	u8g2_SendBuffer(u8g2);	
	rwlock_writer_unlock(&i2c_lock);

}

/*
 Displays the current mode of input reception. The modes can be
 received either by buttons or by OC
 */
static void lcd_change_reception_mode(system_state_t *mystate, u8g2_t *u8g2) {
    char str[32];
	rwlock_writer_lock(&i2c_lock);
	
	sprintf(str, "Input Mode:");	
    u8g2_DrawStr(u8g2, 5, 10, str);
	
	sprintf(str, "0 - Manual");	
    u8g2_DrawStr(u8g2, 5, 25, str);

	sprintf(str, "1 - Auto");	
    u8g2_DrawStr(u8g2, 5, 40, str);

	sprintf(str, "Current Mode:%d", mystate->input_mode);
    u8g2_DrawStr(u8g2, 5, 55, str);

	u8g2_SendBuffer(u8g2);
	rwlock_writer_unlock(&i2c_lock);
}
	

/* 
   LCD mode used to change the temperature set point
*/
static void lcd_change_temp_set_point(system_state_t *mystate, u8g2_t *u8g2) {
    char str[32];
	
	rwlock_writer_lock(&i2c_lock);
		
	sprintf(str, "Temperature Set:");
	u8g2_DrawStr(u8g2, 5, 10, str);	

    sprintf(str, "%dF", mystate->set_point);
	u8g2_DrawStr(u8g2, 55, 25, str);

	u8g2_SendBuffer(u8g2);
	
	rwlock_writer_unlock(&i2c_lock);
}

static void lcd_change_wifi_config(system_state_t *mystate, u8g2_t *u8g2) {
    char str[32];
    rwlock_writer_lock(&i2c_lock);

	sprintf(str, "Provision WiFi");
	u8g2_DrawStr(u8g2, 5, 10, str);
	
	sprintf(str, "SSID:gridballast");
	u8g2_DrawStr(u8g2, 5, 25, str);	

    sprintf(str, "IP:192.168.4.1");
	u8g2_DrawStr(u8g2, 5, 40, str);

	sprintf(str, "Reboot to apply");
	u8g2_DrawStr(u8g2, 5, 55, str);

	u8g2_SendBuffer(u8g2);

    rwlock_writer_unlock(&i2c_lock);
}

/*
 task_lcd - handler function for the LCD task.
            Displays the relevant information on the LCD 
            depending on the value of gb_system_state.display
*/            
static void task_lcd(void *arg) 
{       
     // a structure which will contain all the data for one display
    u8g2_t u8g2;
	 
    // initialize u8g2 structure
    u8g2_Setup_ssd1309_i2c_128x64_noname0_f(&u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
    
    // send init sequence to the display, display is in sleep mode after this,
    u8g2_InitDisplay(&u8g2);

	//wake up display
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_SetContrast(&u8g2, 100);
    u8g2_SetFlipMode(&u8g2, 1);
	
	u8g2_SetFont(&u8g2,u8g2_font_t0_13_mf);

	u8g2_ClearDisplay(&u8g2);
	u8g2_SendBuffer(&u8g2);
	//this is done so that the check in the first switch case fails for the first time
	//on boot up
    current_display_state = NO_OF_LCD_SETTINGS;
    while(1)
    {
        //read system state to access state variables for display
        rwlock_reader_lock(&system_state_lock);
        get_system_state(&mystate);
        rwlock_reader_unlock(&system_state_lock);
        
		//determie the lcd display_mode

		switch (mystate.lcd_display_mode) {
            case DISPLAY_INFO:
				if (current_display_state != DISPLAY_INFO) {
					current_display_state = DISPLAY_INFO;
					u8g2_ClearDisplay(&u8g2);
		            u8g2_SendBuffer(&u8g2);
				}
				lcd_display_info(&mystate, &u8g2);
		        break;
		    case CHANGE_TEMP_SET_POINT:
				if (current_display_state != CHANGE_TEMP_SET_POINT) {
					current_display_state = CHANGE_TEMP_SET_POINT;
				    u8g2_ClearDisplay(&u8g2);
				    u8g2_SendBuffer(&u8g2);
				}
				lcd_change_temp_set_point(&mystate, &u8g2);
				break;
			case CHANGE_RECEPTION_MODE:
				if (current_display_state != CHANGE_RECEPTION_MODE) {
					current_display_state = CHANGE_RECEPTION_MODE;
				    u8g2_ClearDisplay(&u8g2);
				    u8g2_SendBuffer(&u8g2);
				}
				lcd_change_reception_mode(&mystate, &u8g2);
				break;
			case CHANGE_WIFI_CONFIG:
				if (current_display_state != CHANGE_WIFI_CONFIG) {
					current_display_state = CHANGE_WIFI_CONFIG;
				    u8g2_ClearDisplay(&u8g2);
				    u8g2_SendBuffer(&u8g2);
				}
				lcd_change_wifi_config(&mystate, &u8g2);
				break;
			default:
				assert(0);
			    break;
		}    
    }
}


/*Creates a task to handle what is displayed on the LCD screen
  Input - none
  Output - none
*/
void lcd_init_task( void ) 
{
    xTaskCreate(task_lcd, "lcd_task", 4096, NULL, 10, NULL);
}

