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
#define UART_PORT_NUM (1) // 使用UART1

#define UART_BUF_SIZE (256)
#define SYNC 0xAA
#define EXCODE 0x55

static const char *TAG = "BM101"; // Initialize the log tag

static void parse_payload();

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

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));     // Install the UART driver
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));                                          // Configure the UART parameters
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, BM1O1_TX, BM101_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // Set the UART pins
}

static void parsePayload(unsigned char *payload, unsigned char pLength)
{
    unsigned char bytesParsed = 0;
    unsigned char code;
    unsigned char length;
    unsigned char extendedCodeLevel;
    int i;
    /* 循环，直到从payload[]数组中解析出所有字节... */
    while (bytesParsed < pLength)
    {
        /* 解析扩展代码级别、代码和长度 */
        extendedCodeLevel = 0;
        while (payload[bytesParsed] == EXCODE)
        {
            extendedCodeLevel++;
            bytesParsed++;
        }
        code = payload[bytesParsed++];
        if (code & 0x80)
        {
            length = payload[bytesParsed++];
        }
        else
        {
            length = 1;
        }
        /* 根据扩展代码级别、代码、长度和[CODE]定义表，适当地处理有效载荷中的下一个“长度”字节的数据。 */
        ESP_LOGI("parsePayload", "EXCODE级别: %d 代码: 0x%02X 长度: %d",
                 extendedCodeLevel, code, length);
        ESP_LOGI("parsePayload", "数据值(s):");
        for (i = 0; i < length; i++)
        {
            ESP_LOGI("parsePayload", " %02X", payload[bytesParsed + i] & 0xFF);
        }
        ESP_LOGI("parsePayload", "\n");
        /* 按数据值的长度增加bytesParsed */
        bytesParsed += length;
    }
}

static void uart_event_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE + 1);
    while (1)
    {
        // 不断读取新的字节，知道读到[SYNC]时才执行下一步
        int len = uart_read_bytes(UART_PORT_NUM, data, 1, 20 / portTICK_PERIOD_MS);
        while (len > 0 && data[0] != SYNC)
        {
            len = uart_read_bytes(UART_PORT_NUM, data, 1, 20 / portTICK_PERIOD_MS);
        }

        // 读取下一个字节的值，确保其为[SYNC]
        len = uart_read_bytes(UART_PORT_NUM, data + 1, 1, 20 / portTICK_PERIOD_MS);
        if (len > 0 && data[1] != SYNC)
        {
            continue; // 如果不是[SYNC]，返回第一步
        }

        // 读取下一个字节，将其视作[PLENGTH]
        len = uart_read_bytes(UART_PORT_NUM, data + 2, 1, 20 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            int pLength = data[2];
            // 如果[PLENGTH]是 170（[SYNC]）,重复第 3 步
            while (pLength == 170)
            {
                len = uart_read_bytes(UART_PORT_NUM, data + 2, 1, 20 / portTICK_PERIOD_MS);
                pLength = data[2];
            }
            // 如果[PLENGTH]比 170 大，返回第一步（PLENGTH 太大）
            if (pLength > 169)
            {
                continue; // 返回第一步
            }

            // 读取剩余的数据载荷
            len = uart_read_bytes(UART_PORT_NUM, data + 3, pLength, 20 / portTICK_PERIOD_MS);
            if (len > 0)
            {
                // 计算校验和
                int checksum = 0;
                for (int i = 0; i < pLength; i++)
                {
                    checksum += data[i + 3];
                    checksum &= 0xFF;
                    checksum = ~checksum & 0xFF;
                }
                // 校验和验证
                len = uart_read_bytes(UART_PORT_NUM, data + pLength + 3, 1, 20 / portTICK_PERIOD_MS);
                if (len > 0 && data[pLength + 3] == checksum)
                {
                    // 解析数据载荷
                    parsePayload(data + 3, pLength);
                }
            }
        }
    }
    free(data);
    vTaskDelete(NULL);
}

void uart_task_create(void)
{
    uart_event_init();
    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 10, NULL);
}
