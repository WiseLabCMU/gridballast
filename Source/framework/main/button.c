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

/*work under progress*/
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



void IRAM_ATTR mcp_isr_handler(void* arg) {
  button=1;
}

void disable_mcp_intr() {
    updateRegisterBit(0, 0, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(1, 0, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(2, 0, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(3, 0, MCP23017_GPINTENA, MCP23017_GPINTENB);
}

void enable_mcp_intr() {
    updateRegisterBit(0, 1, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(1, 1, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(2, 1, MCP23017_GPINTENA, MCP23017_GPINTENB);
	updateRegisterBit(3, 1, MCP23017_GPINTENA, MCP23017_GPINTENB);
}

void mcp_task(void* arg) 
{
  while(1){
    if (button == 1){ 
        uint8_t pin=getLastInterruptPin();
        uint8_t val=getLastInterruptPinValue();
	    //gpio_intr_disable(4);
		disable_mcp_intr();
		begin(0);
        //printf("%u ",pin);
        //printf("%u \n", val );
		//printf("Raise port high begin\n");
		//vTaskDelay(2000/portTICK_PERIOD_MS);
		//printf("Raise port high end\n");
		//vTaskDelay(300/portTICK_PERIOD_MS);
        //printf("button change\n");
         // Here either the button has been pushed or released.
        if ( pin == S1 && val == 0) { //  Test for release - pin pulled high
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
        		rwlock_writer_unlock(&system_state_lock);
            }
            else if (mystate.lcd_display_mode == CHANGE_RECEPTION_MODE) {
        	    rwlock_writer_lock(&system_state_lock);
        		get_system_state(&mystate);
        		mystate.input_mode -= 1;
        		mystate.input_mode += BUTTON_NO_OF_MODES;
    			mystate.input_mode %= BUTTON_NO_OF_MODES;
        		set_system_state(&mystate);
        		rwlock_writer_unlock(&system_state_lock);
        	}
        }
		else {
			//we got an error
			retry_counter++;
			if (retry_counter > 5) {
				//time to reset the chip y'all
				//TODO_Sharan
			}
		}
		enable_mcp_intr();
   } 
   vTaskDelay(500/portTICK_PERIOD_MS);
 }   
}




void button_init_task( void ) {

  //  I/O expander reset
  gpio_pad_select_gpio(PIN_MCP_RESET);
  gpio_set_direction(PIN_MCP_RESET, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_MCP_RESET, LEVEL_HIGH);

  begin(0);

  /*pinMode(4,GPIO_MODE_OUTPUT); 
   digitalWrite(4,0);

  gpio_pad_select_gpio(13);
  gpio_set_direction(13, GPIO_MODE_OUTPUT);*/


   //I/O expander interrupt initialization
  setupInterrupts(true,false, 0);              
  pinMode(3,GPIO_MODE_INPUT);
  pullUp(3,1);
  setupInterruptPin(3,GPIO_INTR_NEGEDGE);
  //gpio_isr_handler_add(3, mcp_isr_handler, NULL);

  pinMode(2,GPIO_MODE_INPUT);
  pullUp(2,1);
  setupInterruptPin(2,GPIO_INTR_NEGEDGE);
  //gpio_isr_handler_add(2, mcp_isr_handler, NULL);

  pinMode(1,GPIO_MODE_INPUT);
  pullUp(1,1);
  setupInterruptPin(1,GPIO_INTR_NEGEDGE);
  //gpio_isr_handler_add(1, mcp_isr_handler, NULL);

  pinMode(0,GPIO_MODE_INPUT);
  pullUp(0,1);
  setupInterruptPin(0,GPIO_INTR_NEGEDGE);
  //gpio_isr_handler_add(0, mcp_isr_handler, NULL);

  //esp32 interrupt initialization on GPIO4
  gpio_set_intr_type(4, GPIO_INTR_NEGEDGE);       
  gpio_isr_handler_add(4, mcp_isr_handler, NULL);



  xTaskCreatePinnedToCore(mcp_task, "mcp_task", 16384, NULL, 11, NULL,0);




}

