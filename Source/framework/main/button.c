/*
 * Handles all button related functionality
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>


#include "Ada_MCP.h" // IO Expander Library
#include "generic_rw_i2c.h"  // generic I2C read/write functions
#include "driver/adc.h"
#include "driver/timer.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "rwlock.h"
#include "util.h"
#include "button.h"
#include "wifi_module.h"

#define ESP_INTR_FLAG_DEFAULT 0

#define PIN_MCP_RESET 2
#define PIN_SDA 25
#define PIN_SCL 26
#define LEVEL_HIGH 1
#define LEVEL_LOW 0

#define I2C_MASTER_FREQ_HZ     100000     /* I2C master clock frequency */

#define TIMER_DIVIDER   80

system_state_t mystate;

rwlock_t i2c_lock;


volatile int button =0;
uint8_t retry_counter = 0;
bool ledState = true;
int first = 0;

/*
 * ISR handler for a button press
 */
void IRAM_ATTR mcp_isr_handler(void* arg) {
  button=1;
}

/*
 * This routine disables all interrupts that are triggered on MCP23017
 * when one of the buttons on the GB board are pushed
 * Input - None 
 * Output - None
 */
void disable_mcp_intr() {
    updateRegisterBit(S1, 0, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(S2, 0, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(S3, 0, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(S4, 0, MCP23017_GPINTENA, MCP23017_GPINTENB);
}

/*
 * This routine enables all interrupts that are triggered on MCP23017
 * when one of the buttons on the GB board are pushed
 * Input - None
 * Output - None
 */
void enable_mcp_intr() {
    updateRegisterBit(S1, 1, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(S2, 1, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(S3, 1, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(S4, 1, MCP23017_GPINTENA, MCP23017_GPINTENB);
}

/*
  Main handler function for the MCP task
*/
void mcp_task(void* arg) 
{
  while(1){
    if (button == 1){ 
        uint8_t pin=getLastInterruptPin();
        uint8_t val=getLastInterruptPinValue();

		//way too often, the INT line from MCP seems to have transitioned from high to low but esp32 is ignorant
		//of it and we are stuck unable to process any further button pushes after that. The exact scenario that
		//produces this is still unclear but it is easily reproducible when button task is made lower priority 
		//than LCD. Therefore all button interrupts are disabled whilst we are trying to process one so that the
		//line stays high. This seems to work well.
		disable_mcp_intr();
		// begin(0);
        //Make sure the button is released
        if ( pin == S1 && val == 0) {
            button = 0;
            rwlock_reader_lock(&system_state_lock);
            get_system_state(&mystate);
            rwlock_reader_unlock(&system_state_lock);  
    		if ((mystate.input_mode == BUTTON_INPUT) &&
               (mystate.lcd_display_mode == CHANGE_TEMP_SET_POINT)) {
                rwlock_writer_lock(&system_state_lock);
                get_system_state(&mystate);
                mystate.set_point++;
                set_system_state(&mystate);
                //Send Data to openchirp, so now openchirp reads from buttons pressed
                send_temp_set_wrapper();
                rwlock_writer_unlock(&system_state_lock);
            }
    		else if (mystate.lcd_display_mode == CHANGE_RECEPTION_MODE) {
    			rwlock_writer_lock(&system_state_lock);
    			get_system_state(&mystate);
    		    mystate.input_mode += 1;
    			mystate.input_mode %= BUTTON_NO_OF_MODES;
    		    set_system_state(&mystate);
    			rwlock_writer_unlock(&system_state_lock);
    	    }
        }
        else if( pin == S2 && val == 0) {
            button = 0;
            rwlock_writer_lock(&system_state_lock);
            get_system_state(&mystate);
            mystate.lcd_display_mode--;
    		mystate.lcd_display_mode += NO_OF_LCD_SETTINGS;
    		mystate.lcd_display_mode %= NO_OF_LCD_SETTINGS;
            set_system_state(&mystate);
            rwlock_writer_unlock(&system_state_lock);
        }
        else if( pin == S3 && val == 0) {
            button = 0;
            rwlock_writer_lock(&system_state_lock);
            get_system_state(&mystate);
            mystate.lcd_display_mode++;
    		mystate.lcd_display_mode %= NO_OF_LCD_SETTINGS;
            set_system_state(&mystate);
            rwlock_writer_unlock(&system_state_lock);
        }    
        else if( pin == S4 && val == 0) {
        	button = 0;
        	rwlock_reader_lock(&system_state_lock);
        	get_system_state(&mystate);
        	rwlock_reader_unlock(&system_state_lock);  
        	if ((mystate.input_mode == BUTTON_INPUT) &&
               (mystate.lcd_display_mode == CHANGE_TEMP_SET_POINT)) {
        		rwlock_writer_lock(&system_state_lock);
        		get_system_state(&mystate);
        		mystate.set_point-- ;
        		set_system_state(&mystate);
                //Send Data to openchirp, so now openchirp reads from buttons pressed 
                send_temp_set_wrapper();
        		rwlock_writer_unlock(&system_state_lock);
            }
            else if (mystate.lcd_display_mode == CHANGE_RECEPTION_MODE) {
        	    rwlock_writer_lock(&system_state_lock);
        		get_system_state(&mystate);
        		mystate.input_mode -= 1;
        		mystate.input_mode += BUTTON_NO_OF_MODES;
    			mystate.input_mode %= BUTTON_NO_OF_MODES;
        		set_system_state(&mystate);
                // printf("my display mode is %d", mystate.lcd_display_mode);
                // printf("input mode is %i", mystate.input_mode);
        		rwlock_writer_unlock(&system_state_lock);
        	}
        }
		else {
			//we got an error
			retry_counter++;
			if (retry_counter > 5) {
				retry_counter = 0;
				//time to reset the chip
				//TODO_Sharan
			}
		}
		enable_mcp_intr();
   } 
   vTaskDelay(900/portTICK_PERIOD_MS);
 }   
}

/*
 * Initialization function for the button task
 */
void button_init_task( void ) {
    
    //  I/O expander reset
    gpio_pad_select_gpio(PIN_MCP_RESET);
    gpio_set_direction(PIN_MCP_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_MCP_RESET, LEVEL_HIGH);
    
    begin(0);
    
    //I/O expander interrupt initialization
    setupInterrupts(true,false, 0);              
    pinMode(3,GPIO_MODE_INPUT);
    pullUp(3,1);
    setupInterruptPin(3,GPIO_INTR_NEGEDGE);
    
    pinMode(2,GPIO_MODE_INPUT);
    pullUp(2,1);
    setupInterruptPin(2,GPIO_INTR_NEGEDGE);
    
    pinMode(1,GPIO_MODE_INPUT);
    pullUp(1,1);
    setupInterruptPin(1,GPIO_INTR_NEGEDGE);
    
    pinMode(0,GPIO_MODE_INPUT);
    pullUp(0,1);
    setupInterruptPin(0,GPIO_INTR_NEGEDGE);
    
    //esp32 interrupt initialization on GPIO4
    gpio_set_intr_type(4, GPIO_INTR_NEGEDGE);       
    gpio_isr_handler_add(4, mcp_isr_handler, NULL);
    
    xTaskCreatePinnedToCore(mcp_task, "mcp_task", 16384, NULL, 11, NULL,0);

}

