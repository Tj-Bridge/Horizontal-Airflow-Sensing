#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "debug.h"
#include "deck.h"
#include "uart2.h"
#include "log.h"
#include "param.h"
#include "FreeRTOS.h"
#include "task.h"

#define BAUDRATE     115200         // UART communication speed
#define MAX_STR_LEN  127            // Max length for incoming UART string

// Buffers and parsed sensor variables
static char uartString[MAX_STR_LEN] = {0};  // Stores last valid received line
static int16_t flowX = 0;                   // Parsed value from UART, X component
static int16_t flowY = 0;                   // Parsed value from UART, Y component
// static int16_t gas = 0;                  // (Optional) gas value, not currently used

/**
 * @brief Task that runs continuously, reading from UART and parsing sensor data.
 */
static void uartReaderTask(void *param) {
    char rxBuffer[MAX_STR_LEN] = {0};  // Temporary buffer to collect UART line
    int idx = 0;                       // Index in rxBuffer

    while (1) {
        uint8_t c;

        // Try to read 1 byte from UART with 10 ms timeout
        if (uart2GetDataWithTimeout(1, &c, 10)) {
            DEBUG_PRINT("Received char: %c\n", c);

            // If newline or carriage return received and buffer isn't empty
            if ((c == '\n' || c == '\r') && idx > 0) {
                // Null-terminate and copy to global string buffer
                rxBuffer[(idx < MAX_STR_LEN) ? idx : MAX_STR_LEN - 1] = '\0';
                strncpy(uartString, rxBuffer, MAX_STR_LEN);

                DEBUG_PRINT("Parsed line: %s\n", uartString);

                // Tokenize line (expected format: "<flowX> <flowY>")
                char *token = strtok(uartString, " ");
                int tokenIdx = 0;

                // Extract and store values
                while (token != NULL) {
                    switch (tokenIdx) {
                        case 0: flowX = atoi(token); break;
                        case 1: flowY = atoi(token); break;
                        // case 2: gas  = atoi(token); break;  // Uncomment if using gas
                    }
                    token = strtok(NULL, " ");
                    tokenIdx++;
                }

                // Reset for next line
                idx = 0;
                memset(rxBuffer, 0, MAX_STR_LEN);
            }
            else {
                // Store character into buffer if there's room
                if (idx < MAX_STR_LEN - 1) {
                    rxBuffer[idx++] = (char)c;
                }

                // Safety: Clear buffer if too long or never ends
                if (idx >= MAX_STR_LEN - 1) {
                    DEBUG_PRINT("Buffer overflow or no newline — clearing buffer\n");
                    idx = 0;
                    memset(rxBuffer, 0, MAX_STR_LEN);
                }
            }
        }

        // Optional delay to yield CPU (uncomment if needed)
        // vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Called when the deck is initialized. Starts UART and the reader task.
 */
static void deck_uart_loggerInit(struct deckInfo_s *info) {
    uart2Init(BAUDRATE);  // Initialize UART with defined baud rate
    xTaskCreate(uartReaderTask, "uartReader", 256, NULL, 3, NULL);  // Start background reader task
}

/**
 * @brief Called by the client to test if the deck is detected.
 */
static bool helloUart(void) {
    DEBUG_PRINT("UART Active!\n");
    return true;
}

// Register the deck driver
const DeckDriver uartLogger = {
    .name = "uartLogger",
    .init = deck_uart_loggerInit,
    .test = helloUart,
};
DECK_DRIVER(uartLogger);

// Register the log variables for the client (to enable logging/plotting)
LOG_GROUP_START(uart_logger)
LOG_ADD(LOG_INT16, flowX, &flowX)
LOG_ADD(LOG_INT16, flowY, &flowY)
// LOG_ADD(LOG_INT16, gas, &gas)  // Uncomment if you're also reading gas data
LOG_GROUP_STOP(uart_logger)
