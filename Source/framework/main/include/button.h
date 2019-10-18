/**
 * @file ct_module.h
 *
 * @brief Defines the CT Module API
 *
 * @author Vikram Shanker (vshanker@cmu.edu)
 * @author Rohit Garg (rohitg1@andrew.cmu.edu)
 * @author Sharan Turlapati (sharant@andrew.cmu.edu)
 */

#ifndef __button_h_
#define __button_h_

/** @brief depth of the controller stack */
#define ctUSStackDepth ((unsigned short) 8192) /* bytes */
/** @brief priority of the controller stack */
#define ctUXPriority (2)

/** @brief name of the controller task */
extern const char * const ct_task_name;

enum input_modes {
	BUTTON_INPUT = 0,
	BUTTON_OPENCHIRP_INPUT,
	BUTTON_NO_OF_MODES,
};
	
//GPIO pins to GB switches mapping
//Read carefully, a little non intuitive
#define S1 0
#define S2 3
#define S3 1
#define S4 2

#define SCREEN_DECREMENT S2
#define SCREEN_INCREMENT S3
#define VALUE_INCREMENT S1
#define VALUE_DECREMENT S4

/**
 * @brief function that initializes that controller task
 *
 * @return void
 */
void button_init_task( void );
void handle_button_interrupt(uint8_t pin, uint8_t val);

#endif /* __ct_module_h_ */
