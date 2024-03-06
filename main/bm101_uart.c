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
#define UART_BUF_SIZE (2048)

#define SYNC_BYTE 0xAA // [SYNC]字节
#define EXCODE 0x80    // 假设EXCODE是一个宏定义，表示扩展代码

static uint8_t Uart2_Buffer[UART_BUF_SIZE]; // Receive buffer
static uint8_t Uart2_Rx = 0;                // Uart2_Buffer index
static uint8_t Uart2_Len;                   // Data length (including CRC after the third byte)
static int checksum = 0;                    // Checksum calculated from the payload
static uint8_t Uart2_Sta = 0;               // Data frame correct flag
static uint8_t Uart2_check;                 // Checksum at the end of the frame
static uint8_t sig_quality = 200;           // Signal quality
static const char *TAG = "BM101";           // Initialize the log tag

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

static void uart_event_task(void *pvParameters)
{
    while (1)
    {
        uint8_t data;
        int len = uart_read_bytes(UART_PORT_NUM, &data, 1, 20 / portTICK_PERIOD_MS);
        if (len)
        {
            Uart2_Buffer[Uart2_Rx] = data;
            Uart2_Rx++;

            if (Uart2_Rx < 3)
            { // Check if the frame header is received
                if (Uart2_Buffer[Uart2_Rx - 1] != SYNC_BYTE)
                {                  // Exception
                    Uart2_Rx = 0;  // Reset index
                    Uart2_Sta = 0; // Reset flag
                }
            }
            else if (Uart2_Rx == 3)
            { // Get payload length
                Uart2_Len = Uart2_Buffer[Uart2_Rx - 1];
            }
            else if (Uart2_Rx < 4 + Uart2_Len)
            {                                           // Receive payload
                checksum += Uart2_Buffer[Uart2_Rx - 1]; // Calculate checksum
            }
            else
            { // Receive checksum
                Uart2_check = Uart2_Buffer[Uart2_Rx - 1];
                checksum &= 0xFF;
                checksum = ~checksum & 0xFF;
                if (checksum != Uart2_check)
                {                  // Checksum error, discard the packet
                    Uart2_Rx = 0;  // Reset index
                    Uart2_Sta = 0; // Reset flag
                    checksum = 0;
                }
                else
                {
                    Uart2_Sta = 1; // Receive complete
                }
            }

            if (Uart2_Sta)
            {                    // Detect flag, indicating successful reception
                parse_payload(); // Call data parsing function
                Uart2_Rx = 0;    // Reset index
                Uart2_Sta = 0;   // Reset flag
                checksum = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelete(NULL);
}

void drawCurve(short int rawValue)
{
    // 在这里实现绘制曲线的逻辑
    ESP_LOGI("BM101", "Drawing curve with raw value: %d", rawValue);
}

void LCD_Clear(int color)
{
    // 在这里实现清屏的逻辑
    ESP_LOGI("BM101", "Clearing LCD with color: %d", color);
}

void LCD_ShowNum(int x, int y, int num, int len, int size)
{
    // 在这里实现显示数字的逻辑
    ESP_LOGI("BM101", "Showing number %d at (%d, %d) with length %d and size %d", num, x, y, len, size);
}

static void parse_payload(void)
{
    uint8_t bytesParsed = 0; // 已处理的字节数
    uint8_t code;
    uint8_t length; // 当前DataRow包含的Value的字节数
    uint8_t extendedCodeLevel;
    short int rawValue = 0; // 心电数据

    while (bytesParsed < Uart2_Len)
    {
        extendedCodeLevel = 0;
        while (Uart2_Buffer[3 + bytesParsed] == EXCODE)
        {
            extendedCodeLevel++;
            bytesParsed++;
        }
        code = Uart2_Buffer[3 + bytesParsed];
        bytesParsed++;
        if (code >= 0x80)
        {
            length = Uart2_Buffer[3 + bytesParsed];
            bytesParsed++;
        }
        else
        {
            length = 1;
        }
        // 现在我们获得了ExCodeLevel, code和DataValue的长度
        // 实际上，扩展代码级别总是0，所以我们可以忽略它
        switch (code)
        {
        case 0x80: // 两字节有符号的原始波形值，大端字节序
            if (sig_quality > 0)
            {
                rawValue = Uart2_Buffer[3 + bytesParsed];
                rawValue <<= 8;
                rawValue |= Uart2_Buffer[4 + bytesParsed];
                drawCurve(rawValue);
            }
            else
            {
                LCD_Clear(1); // 信号质量不佳，清屏
            }
            break;
        case 0x02:
            // 一字节的信号质量数据，0表示质量差，200表示质量好
            sig_quality = Uart2_Buffer[3 + bytesParsed];
            break;
        case 0x03:                                                      // 一字节的心率值数据
            LCD_ShowNum(290, 50, Uart2_Buffer[3 + bytesParsed], 2, 12); // 在屏幕上显示心率值
            break;
        }
        bytesParsed += length;
    }
}

void uart_task_create(void)
{
    uart_event_init();
    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 10, NULL);
}
