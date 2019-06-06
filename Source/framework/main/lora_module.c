
// #include <stdio.h>
// #include <string.h>
// #include "esp_event_loop.h"
// #include "esp_log.h"
// #include "esp_system.h"
// #include "esp_wifi.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/event_groups.h"
// #include "freertos/task.h"
// #include "lwip/dns.h"
// #include "lwip/err.h"
// #include "lwip/netdb.h"
// #include "lwip/sockets.h"
// #include "lwip/sys.h"
// #include "cJSON.h"
// #include "esp_request.h"
// #include "util.h"
// #include "config.h"
// #include "nvs.h"
// #include "config_server.h"
// #include "Ada_MCP.h"



// /* OpenChirp transducer ids for system state fields */
// #define TRANSDUCER_ID_TEMP_BOTTOM "temp_bottom"
// #define TRANSDUCER_ID_TEMP_TOP    "temp_top"
// #define TRANSDUCER_ID_GRID_FREQ   "grid_frzequency"
// #define TRANSDUCER_ID_SET_POINT   "temp_set"
// #define TRANSDUCER_ID_RELAY_1     "relay_1"
// #define TRANSDUCER_ID_RELAY_2     "relay_2"

// const char * const lora_task_name = "lora_module_task";
// static const char *TAG = "lora";

// static system_state_t system_state;

// /**
//  * @brief Common wifi initialization that occurs before the wifi task starts
//  * Sets up hardware
//  */
// static void init_wifi(void) {
//     tcpip_adapter_init();

//     wifi_event_group = xEventGroupCreate();
//     ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
//     ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

//     config_init();
// }

// /*****************************************
//  ************ MODULE FUNCTIONS ***********
//  *****************************************/

// /**
//  * @brief pointer to OpenChirp API response body
//  *
//  * This is filled in but the get_transducer_download_callback function
//  */
// static char *transducer_response = NULL;
// /** @brief total length of transducer response array */
// static int transducer_response_len = 0;
// /**
//  * @brief reset and free the transducer response buffer
//  */
// static void reset_transducer_response()
// {
//     if (transducer_response != NULL) {
//         free(transducer_response);
//         transducer_response = NULL;
//     }
//     transducer_response_len = 0;
// }

// static void run_mode_normal_lora() {
//     reset_transducer_response();
//     ESP_LOGI(TAG, "Running normal mode");
//     while (module_mode == MODULE_MODE_NORMAL) {
//         /* Wait for the callback to set the CONNECTED_BIT in the
//            event group.
//         */
//         xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
//                             false, true, portMAX_DELAY);
//         ESP_LOGI(TAG, "Connected to AP");

//         // read system state into local copy
//         rwlock_reader_lock(&system_state_lock);
//         get_system_state(&system_state);
//         rwlock_reader_unlock(&system_state_lock);

//         // send data to openchirp
//         send_data(&system_state);

//         for (int countdown = 9; countdown >= 0; countdown--) {
//             ESP_LOGI(TAG, "%d... ", countdown);
//             vTaskDelay(1000 / portTICK_PERIOD_MS);
//         }
//         ESP_LOGI(TAG, "Starting again!");
//     }
// }

// /**
//  * @brief post a transducer value to OpenChirp using the REST API
//  *
//  * @param transducer_id OpenChirp transducer id
//  * @param value         transducer value as a string (sprintf'd)
//  *
//  * @return 0 on success, -1 on failure
//  */
// static int send_transducer_value_lora(const char *transducer_id, const char *value) {
//     //THIS GUY WILL CHANGE ENTIRELY FOR LORA
//     int url_len = strlen(BASE_URL) + strlen(transducer_id);
//     char *url = malloc(url_len + 1);
//     if (url == NULL) {
//         ESP_LOGE(TAG, "Malloc failed");
//         return -1;
//     }
//     strcpy(url, BASE_URL);
//     strcat(url, transducer_id);

//     ESP_LOGI(TAG, "sending transducer value");
//     request_t *req = req_new(url);
//     req_setopt(req, REQ_SET_METHOD, "POST");
//     req_setopt(req, REQ_SET_HEADER, (void*)AUTH_HEADER);
//     req_setopt(req, REQ_SET_HEADER, (void*)USER_AGENT_HEADER);
//     req_setopt(req, REQ_SET_HEADER, "Connection: close");
//     req_setopt(req, REQ_SET_HEADER, "Content-Type: text/plain");
//     req_setopt(req, REQ_SET_DATAFIELDS, (void*)value);
//     int status = req_perform(req);
//     req_clean(req);
//     free(url);

//     if (status != 200) {
//         ESP_LOGE(TAG, "Error sending transducer value, received non-200 response: %d", status);
//         return -1;
//     }

//     ESP_LOGI(TAG, "posted to transducer %s, value = %s", transducer_id, value);
//     return 0;
// }

// /**
//  * @brief post data from system state to OpenChirp
//  *
//  * @param system_state snapshotted system state struct
//  *
//  * @return 0 on success, -1 on failure
//  */
// static int send_data_lora(system_state_t *system_state) {
//     ESP_LOGI(TAG, "sending data");
//     char data_buf[16]; // make sure this is large enough to hold the sprintf'd value
//     int err = 0;

//     // only update grid frequency if it is nonzero so we don't push bogus value when
//     // the zero crossing circuit is not connected
//     if (err == 0) { //system_state->grid_freq > 5.0) {
//         sprintf(data_buf, "%.4f", system_state->grid_freq);
//         err = send_transducer_value_lora(TRANSDUCER_ID_GRID_FREQ, data_buf);
//     }

//     if (err == 0) {
//         sprintf(data_buf, "%d", system_state->temp_bottom);
//         err = send_transducer_value_lora(TRANSDUCER_ID_TEMP_BOTTOM, data_buf);
//     }

//     if (err == 0) {
//         sprintf(data_buf, "%d", system_state->temp_top);
//         err = send_transducer_value_lora(TRANSDUCER_ID_TEMP_TOP, data_buf);
//     }

//     if (err == 0) {
//         sprintf(data_buf, "%d", system_state->set_point);
//         err = send_transducer_value_lora(TRANSDUCER_ID_SET_POINT, data_buf);
//     }

//     return err;
// }

// void lora_init_task( void ) {

//     printf("Intializing Wifi System...");
//     //Sets up hardware
//     init_lora();//CHANGE
//     //Create Task
//     xTaskCreate(
//                 &run_mode_normal_lora, /* task function */
//                 lora_task_name, /* wifi task name */
//                 loraUSStackDepth, /* stack depth Figure this out */
//                 NULL, /* parameters to fn_name */
//                 loraUXPriority, /* task priority */
//                 NULL /* task handle ( returns an id basically ) */
//                );
//     fflush(stdout);
// }

