#include <stdio.h>  // Include the standard input/output library
#include <stdarg.h> // Include the standard argument library
#include <string.h>
#include "freertos/FreeRTOS.h" // Include the FreeRTOS library
#include "freertos/task.h"     // Include the FreeRTOS task library
#include "driver/uart.h"       // Include the ESP-IDF UART driver library
#include "driver/gpio.h"       // Include the ESP-IDF GPIO driver library
#include "sdkconfig.h"         // Include the SDK configuration file
#include "esp_log.h"           // Include the ESP-IDF log library
#include "uart_tjc.h"
#include "driver/i2c.h"
#include "i2c_master.h"
#include "i2c_senor.h"

void app_main(void)
{
    /*uart*/
    uart_evnet_task_create(); // Start uart task

    i2c_senor_task_create();
    int task_idx = 0;
    while (1)
    {
        printf("----------------------\n");
        char str_send[100];
        snprintf(str_send, sizeof(str_send), "t1.txt=\"%.3f\"\xff\xff\xff", bh1760_lux);
        uart_write_bytes(UART_PORT_NUM, str_send, strlen(str_send));
        printf("bh1760_lux=%.4f\n\n", bh1760_lux);
        printf("sht35_val[0]=%.4lf\n", sht35_val[0]);
        printf("sht35_val[1]=%.4lf\n\n", sht35_val[1]);
        printf("dac_val=%d\n\n", dac_val);
        for (int i = 0; i < 3; i++)
        {
            printf("bus_val[%d]=%.4lf\n", i, bus_val[i]);
            printf("shunt_val[%d]=%.4lf\n", i, shunt_val[i]);
        }
        printf("\n");

        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_PERIOD_MS);
    }
}
