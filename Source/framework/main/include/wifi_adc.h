/*
Header file for wifi_adc2.c
*/

#define wifi_adc_stack_depth 8192
#define wifi_adc_task_priority 8
#define WIFI_TIMER 20
#define TIMER_DIVIDER 80
#define NO_OF_ADC_SAMPLES 6000

typedef enum {
    WIFI_MODULE_MODE_NORMAL,
    WIFI_MODULE_MODE_CONFIG
} wifi_module_mode_t;

void wifi_adc_init();