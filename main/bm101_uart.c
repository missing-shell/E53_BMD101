#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT_NUM (1)
#define BM101_BAND (57600)
#define BM1O1_TX (5)
#define BM101_RX (4)
#define UART_BUF_SIZE (4096)

#define SYNC_BYTE 0xAA    // [SYNC]字节
#define PATTERN_CHR_NUM 2 // 模式字符的数量
#define CHR_TOUT (10)
#define PRE_IDLE (10)
static QueueHandle_t uart1_queue;
static const char *TAG = "BM101"; // Initialize the log tag

static void backup(void);

static void uart_event_init(void) // Define the echo task function
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

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart1_queue, 0)); // Install the UART driver
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));                                                // Configure the UART parameters
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, BM1O1_TX, BM101_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));       // Set the UART pins

    // Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(UART_PORT_NUM, SYNC_BYTE, PATTERN_CHR_NUM, CHR_TOUT, 0, PRE_IDLE);
    // Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(UART_PORT_NUM, 100);
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(UART_BUF_SIZE);
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart1_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            bzero(dtmp, UART_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_PORT_NUM);
            switch (event.type)
            {
            // Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(UART_PORT_NUM, dtmp, event.size, portMAX_DELAY);
                backup();

                break;
            // Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            // Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            // Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            // UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(UART_PORT_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(UART_PORT_NUM);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1)
                {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(UART_PORT_NUM);
                }
                else
                {
                    backup();
                }
                break;
            // Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void uart_evnet_task_create(void)
{
    uart_event_init();
    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

static void backup(void)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE); // Allocate memory for the data buffer
    printf("--------------------BM101_BAND=%d-------------------\n", BM101_BAND);
    while (1)
    {
        // Read data from the UART
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, (size_t *)&length));
        int len = uart_read_bytes(UART_PORT_NUM, data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS); // Read bytes from the UART
        if (len)
        {                     // If data was read
            data[len] = '\0'; // Null-terminate the data
            for (int i = 0; i < len; i++)
            {
                ESP_LOGI(TAG, "%02X ", data[i]); // 需要解决看门狗超时问题
            }
        }
    }
}