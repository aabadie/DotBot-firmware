/**
 * @file
 * @ingroup samples_bsp
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief This is an example on how to use the BSP QSPI flash driver.
 *
 * @copyright Inria, 2024-present
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include <stdbool.h>

#include "gpio.h"
#include "qspi.h"

static const gpio_t _io0_pin = { .port = 0, .pin = 20 };
static const gpio_t _io1_pin = { .port = 0, .pin = 21 };
static const gpio_t _io2_pin = { .port = 0, .pin = 22 };
static const gpio_t _io3_pin = { .port = 0, .pin = 23 };
static const gpio_t _sck_pin = { .port = 0, .pin = 19 };
static const gpio_t _cs_pin  = { .port = 0, .pin = 17 };

#define READ_BUFFER_SIZE (256U)

static const db_qspi_conf_t _qspi_conf = {
    .io0         = &_io0_pin,
    .io1         = &_io1_pin,
    .io2         = &_io2_pin,
    .io3         = &_io3_pin,
    .cs          = &_cs_pin,
    .sck         = &_sck_pin,
    .sckfreq     = DB_QSPI_SCKFREQ_32MHZ,
    .enable_quad = true,
};

static uint8_t read_buf[READ_BUFFER_SIZE] = { 0 };
// clang-format off
static uint8_t data[READ_BUFFER_SIZE]     = {
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
};
// clang-format on

int main(void) {
    db_qspi_init(&_qspi_conf);
    uint32_t address = 0x00000000;
    db_qspi_block_erase(address);
    db_qspi_read(address, read_buf, READ_BUFFER_SIZE);
    db_qspi_program(address, data, READ_BUFFER_SIZE);
    db_qspi_read(address, read_buf, READ_BUFFER_SIZE);

    while (1) {
        __WFE();
    }
}