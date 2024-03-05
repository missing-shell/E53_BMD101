#include <stdio.h>  // Include the standard input/output library
#include <stdarg.h> // Include the standard argument library
#include <string.h>
#include "freertos/FreeRTOS.h" // Include the FreeRTOS library
#include "freertos/task.h"     // Include the FreeRTOS task library
#include "driver/uart.h"       // Include the ESP-IDF UART driver library
#include "driver/gpio.h"       // Include the ESP-IDF GPIO driver library
#include "sdkconfig.h"         // Include the SDK configuration file
#include "esp_log.h"           // Include the ESP-IDF log library

#define UART_PORT_NUM (1)
#define BM101_BAND (57600)
#define BM1O1_TX (5)
#define BM101_RX (4)
#define UART_BUF_SIZE (2048)
static const char *TAG = "BM101"; // Initialize the log tag
static void echo_task(void *arg)  // Define the echo task function
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        // Define and initialize the UART configuration
        .baud_rate = BM101_BAND,         // Set the baud rate
        .data_bits = UART_DATA_8_BITS,   // Set the data bits
        .parity = UART_PARITY_DISABLE,   // Disable parity
        .stop_bits = UART_STOP_BITS_1,   // Set the stop bits
        .source_clk = UART_SCLK_DEFAULT, // Set the source clock
    };
    int intr_alloc_flags = 0; // Define the interrupt allocation flags

#if CONFIG_UART_ISR_IN_IRAM                // If the UART ISR is in IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM; // Set the interrupt allocation flags to ESP_INTR_FLAG_IRAM
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags)); // Install the UART driver
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));                                      // Configure the UART parameters
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, BM1O1_TX, BM101_RX, NULL, NULL));                         // Set the UART pins

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE); // Allocate memory for the data buffer
    printf("--------------------BM101_BAND=%d-------------------\n", BM101_BAND);
    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(UART_PORT_NUM, data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS); // Read bytes from the UART
        if (len)
        {                     // If data was read
            data[len] = '\0'; // Null-terminate the data
            for (int i = 0; i < len; i++)
            {
                ESP_LOGI(TAG, "%02X ", data[i]);
            }
        }
    }
}

void uart_task_create(void) // Define the main application function
{
    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL); // Create the echo task
}
