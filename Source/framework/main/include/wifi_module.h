/**
 * @file wifi_module.h
 *
 * @brief Defines the Wifi Module API
 *
 * @author Vikram Shanker (vshanker@cmu.edu)
 */

#ifndef __wifi_module_h_
#define __wifi_module_h_

/** @brief depth of the wifi stack */
#define wifiUSStackDepth ((unsigned short) 8192) /* bytes */
/** @brief priority of the wifi stack */
#define wifiUXPriority (2)
#define WIFI_TASK_DELAY 100
#define TIMER_DIVIDER 80

/*enum wifi_module_mode_type {
    MODULE_MODE_NORMAL,
    MODULE_MODE_CONFIG,
    WIFI_NO_OF_MODES,
};*/

/** @brief name of the wifi task */
extern const char * const wifi_task_name;

void send_temp_set_wrapper( void );

/**
 * @brief function that initializes that wifi task
 *
 * @return void
 */
void wifi_init_task( void );

/**
 * @brief exit normal data publish/receive mode and enter configuration mode
 *
 * @note configuration mode can only be exited by rebooting the module
 * @note this function can be called from other tasks or interrupts
 */
void wifi_enter_config_mode();

void init_mode_sta(const char *ssid, const char *password);
void init_mode_ap();
void run_mode_normal();
void run_mode_config();
void init_wifi();
#endif /* __wifi_module_h_ */
