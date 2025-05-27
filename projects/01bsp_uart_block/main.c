/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is a short example of how to use the UART block mode API.
 *
 * @copyright Inria, 2022
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <nrf.h>
#include "board.h"
#include "board_config.h"
#include "uart_block.h"

#define DB_UART_BAUDRATE (1000000U)  ///< UART baudrate

static void _uart_block_callback(const uint8_t *data, size_t len) {
    db_uart_block_write(0, data, len);
    printf("Echoed %s (%d)\n", (char *)data, len);
}

int main(void) {
    db_board_init();
    db_uart_block_init(0, &db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, &_uart_block_callback);

    while (1) {
        __WFI();
    }
}
