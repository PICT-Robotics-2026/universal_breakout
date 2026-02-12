#include <stdio.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"

#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/pulse_cnt.h"

#include "encoders.h"

static const char *TAG = "encoders";

#define ENC_1_A	36
#define ENC_1_B	39

#define ENC_2_A	34
#define ENC_2_B	35

#define ENC_3_A	32
#define ENC_3_B	33

#define ENC_4_A	25
#define ENC_4_B	26

#define PCNT_LOW_LIMIT -30000
#define PCNT_HIGH_LIMIT 30000

int encoder_a[4] = {
    ENC_1_A,
    ENC_2_A,
    ENC_3_A,
    ENC_4_A
};

int encoder_b[4] = {
    ENC_1_B,
    ENC_2_B,
    ENC_3_B,
    ENC_4_B
};

volatile int encoder_ticks[4] = {0, 0, 0, 0};

volatile int encoder_direction[4] = { 1, 1, 1, 1 };

pcnt_unit_handle_t encoder_pcnt_units[4]; 
bool encoder_initialized[4] = { false, false, false, false };

static bool pcnt_encoder_1_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) 
{ 
    BaseType_t high_task_wakeup; 
    QueueHandle_t queue = (QueueHandle_t)user_ctx; 
 
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup); 
 
    if (edata->watch_point_value == PCNT_HIGH_LIMIT) 
    { 
	encoder_ticks[0] += PCNT_HIGH_LIMIT; 
    } 
    else if (edata->watch_point_value == PCNT_LOW_LIMIT) 
    { 
	encoder_ticks[0] += PCNT_LOW_LIMIT; 
    } 
     
    return (high_task_wakeup == pdTRUE); 
} 

static bool pcnt_encoder_2_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) 
{ 
    BaseType_t high_task_wakeup; 
    QueueHandle_t queue = (QueueHandle_t)user_ctx; 
 
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup); 
 
    if (edata->watch_point_value == PCNT_HIGH_LIMIT) 
    { 
	encoder_ticks[1] += PCNT_HIGH_LIMIT; 
    } 
    else if (edata->watch_point_value == PCNT_LOW_LIMIT) 
    { 
	encoder_ticks[1] += PCNT_LOW_LIMIT; 
    } 
     
    return (high_task_wakeup == pdTRUE); 
} 

static bool pcnt_encoder_3_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) 
{ 
    BaseType_t high_task_wakeup; 
    QueueHandle_t queue = (QueueHandle_t)user_ctx; 
 
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup); 
 
    if (edata->watch_point_value == PCNT_HIGH_LIMIT) 
    { 
	encoder_ticks[2] += PCNT_HIGH_LIMIT; 
    } 
    else if (edata->watch_point_value == PCNT_LOW_LIMIT) 
    { 
	encoder_ticks[2] += PCNT_LOW_LIMIT; 
    } 
     
    return (high_task_wakeup == pdTRUE); 
} 

static bool pcnt_encoder_4_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) 
{ 
    BaseType_t high_task_wakeup; 
    QueueHandle_t queue = (QueueHandle_t)user_ctx; 
 
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup); 
 
    if (edata->watch_point_value == PCNT_HIGH_LIMIT) 
    { 
	encoder_ticks[3] += PCNT_HIGH_LIMIT; 
    } 
    else if (edata->watch_point_value == PCNT_LOW_LIMIT) 
    { 
	encoder_ticks[3] += PCNT_LOW_LIMIT; 
    } 
     
    return (high_task_wakeup == pdTRUE); 
} 

pcnt_watch_cb_t watchpoints[4] = {
    pcnt_encoder_1_on_reach,
    pcnt_encoder_2_on_reach,
    pcnt_encoder_3_on_reach,
    pcnt_encoder_4_on_reach,
};

void encoder_set_direction(encoder_t encoder, int direction)
{
    encoder_direction[encoder] = direction;
}

int encoder_get_direction(encoder_t encoder)
{
    return encoder_direction[encoder];
}

void encoder_init(encoder_t encoder) { 

    if (encoder_initialized[encoder])
	return;

    ESP_LOGI("encoder", "init encoder %d", encoder);
    
    ESP_LOGI(TAG, "install pcnt unit"); 
    pcnt_unit_config_t unit_config = { 
	.high_limit = PCNT_HIGH_LIMIT, 
	.low_limit = PCNT_LOW_LIMIT, 
    }; 
    pcnt_unit_handle_t pcnt_unit = NULL; 
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit)); 
 
    ESP_LOGI(TAG, "set glitch filter"); 
    pcnt_glitch_filter_config_t filter_config = { 
	.max_glitch_ns = 1000, 
    }; 
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit,
						&filter_config)); 
 
    ESP_LOGI(TAG, "install pcnt channels"); 
    pcnt_chan_config_t chan_a_config = { 
	.edge_gpio_num = encoder_a[encoder], 
	.level_gpio_num = encoder_b[encoder], 
    }; 
    pcnt_channel_handle_t pcnt_chan_a = NULL; 
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit,
				     &chan_a_config,
				     &pcnt_chan_a)); 
    pcnt_chan_config_t chan_b_config = { 
	.edge_gpio_num = encoder_b[encoder], 
	.level_gpio_num = encoder_a[encoder], 
    }; 
    pcnt_channel_handle_t pcnt_chan_b = NULL; 
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit,
				     &chan_b_config,
				     &pcnt_chan_b)); 
 
    ESP_LOGI(TAG, "set edge and level actions for pcnt channels"); 
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a,
						 PCNT_CHANNEL_EDGE_ACTION_DECREASE,
						 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a,
						  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
						  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b,
						 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
						 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b,
						  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
						  PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); 
 
    ESP_LOGI(TAG, "add watch points and register callbacks"); 
    int watch_points[] = {PCNT_LOW_LIMIT, PCNT_HIGH_LIMIT}; 
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) { 
	ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i])); 
    } 
    pcnt_event_callbacks_t cbs = { 
	.on_reach = watchpoints[encoder], 
    }; 
    QueueHandle_t queue = xQueueCreate(10, sizeof(int)); 
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit,
						       &cbs,
						       queue)); 
 
    ESP_LOGI(TAG, "enable pcnt unit %d", encoder); 
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit)); 
    ESP_LOGI(TAG, "clear pcnt unit %d", encoder); 
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit)); 
    ESP_LOGI(TAG, "start pcnt unit %d", encoder); 
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit)); 
 
    encoder_pcnt_units[encoder] = pcnt_unit; 
    encoder_initialized[encoder] = true;
} 

int encoder_get_position(encoder_t encoder) { 

    // ensures that the encoder is initialized
    encoder_init(encoder);

    int pulse_count = 0; 
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_pcnt_units[encoder], &pulse_count)); 
    int total = encoder_ticks[encoder] + pulse_count; 
    return encoder_direction[encoder] * total; 
}

void encoder_print_directions()
{
    for (int i = 0; i < 4; i++)
    {
	ESP_LOGI("encoders",
		 "encoder_directions: ENCODER %d --> %d",
		 (i + 1),
		 encoder_direction[i]);
    }
}
