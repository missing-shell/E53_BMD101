#ifndef UART_TJC_H
#define UART_TJC_H

#define UART_TXD (10)                 // Define the TXD pin number
#define UART_RXD (11)                 // Define the RXD pin number
#define UART_RTS (UART_PIN_NO_CHANGE) // Define the RTS pin number
#define UART_CTS (UART_PIN_NO_CHANGE) // Define the CTS pin number

#define UART_PORT_NUM (UART_NUM_1)  // Define the UART port number
#define UART_BAUD_RATE (9600)       // Define the UART baud rate
#define UART_TASK_STACK_SIZE (2048) // Define the task stack size
#define UART_BUF_SIZE (1024)        // Define the buffer size

void uart_evnet_task_create(void);

#endif // UART_TJC_H