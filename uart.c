#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"

#define USB_UART_NUM UART_NUM_0
#define UART_WAIT_TICKS_WRITE 100 // number of freertos ticks to wait for uart to transmit
#define UART_WAIT_TICKS_READ 10000 // number of freertos ticks to wait for uart to read (ideally infinity)

// used for left_right and forth uart communication with host (raspberry pi)
#define USB_TX 1
#define USB_RX 3
#define BUFFER_SIZE 2048

void uart_init()
{
    const uart_port_t uart_num = USB_UART_NUM;

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = BUFFER_SIZE;
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    
    uart_config_t uart_config = {
	.baud_rate = 115200,
	.data_bits = UART_DATA_8_BITS,
	.parity = UART_PARITY_DISABLE,
	.stop_bits = UART_STOP_BITS_1,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    
    // PINS
    ESP_ERROR_CHECK(uart_set_pin(uart_num, USB_TX, USB_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // clear the input
    uart_flush_input(USB_UART_NUM);
}

void write_string(char* string)
{
    uart_write_bytes(USB_UART_NUM, (const char*)string, strlen(string));
    ESP_ERROR_CHECK(uart_wait_tx_done(USB_UART_NUM, UART_WAIT_TICKS_WRITE));
}

//returns length of string read
int read_string(int max_length, char* string)
{
    int length = 0;

    // this loop keeps waiting for uart to recieve data, while also
    // sleeping so the watchdog is triggerred periodically
    while (length == 0)
    {
	ESP_ERROR_CHECK(uart_get_buffered_data_len(USB_UART_NUM,(size_t*) &length));
	vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (length > (max_length - 1))
    {
	ESP_LOGE("UART_ERROR", "Buffer size %d too small to read uart data of size %d", max_length, length);
	return -1;
    }    
    length = uart_read_bytes(USB_UART_NUM, string, length, UART_WAIT_TICKS_READ);
    string[length] = 0; // add null terminator
    
    uart_flush_input(USB_UART_NUM);
    
    return length;
}

