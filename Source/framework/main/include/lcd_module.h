/**
 * @file lcd_module.h
 *
 * @brief Defines the LCD Module API
 *
 * @author Vikram Shanker (vshanker@cmu.edu)
           Sharan Turlapati (sharant@andrew.cmu.edu)
 */

#ifndef __lcd_module_h_
#define __lcd_module_h_

/** @brief depth of the controller stack */
#define lcdUSStackDepth ((unsigned short) 2048) /* bytes */
/** @brief priority of the controller stack */
#define lcdUXPriority (2)

/** @brief name of the controller task */
extern const char * const lcd_task_name;

enum lcd_display_type {
	DISPLAY_INFO = 0,
	CHANGE_TEMP_SET_POINT,
	CHANGE_RECEPTION_MODE,
	CHANGE_WIFI_CONFIG,
	NO_OF_LCD_SETTINGS,
};

/**
 * @brief function that initializes that controller task
 *
 * @return void
 */
void lcd_init_task( void );

#endif /* __lcd_module_h_ */
