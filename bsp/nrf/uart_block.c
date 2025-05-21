/**
 * @file
 * @ingroup bsp_uart_fast
 *
 * @brief  nRF52833-specific definition of the "uart" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2025
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <nrf.h>
#include <nrf_peripherals.h>

#include "board_config.h"
#include "gpio.h"
#include "uart_block.h"
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#include "tz.h"
#endif

//=========================== defines ==========================================

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define NRF_POWER (NRF_POWER_S)
#elif defined(NRF5340_XXAA) && defined(NRF_NETWORK)
#define NRF_POWER (NRF_POWER_NS)
#endif
#define DB_UARTE_CHUNK_SIZE     (64U)
#define DB_UARTE_BUFFER_SIZE    (UINT8_MAX)  ///< Maximum size of the buffer to store received data, 256 bytes

typedef enum {
    IDLE,
    FRAME_RX,
} uart_block_state_t;

typedef struct {
    NRF_UARTE_Type *p;
    IRQn_Type       irq;
} uart_conf_t;

typedef struct {
    uart_block_rx_cb_t  callback;  ///< pointer to the callback function
    uart_block_state_t  state;
    uint8_t             buffer[DB_UARTE_BUFFER_SIZE];  ///< buffer to store the received data
} uart_vars_t;

//=========================== variables ========================================

static const uart_conf_t _devs[UARTE_COUNT] = {
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
    {
#if defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_UARTE0_NS,
#else
        .p = NRF_UARTE0_S,
#endif
        .irq = SERIAL0_IRQn,
    },
    {
#if defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_UARTE1_NS,
#else
        .p = NRF_UARTE1_S,
#endif
        .irq = SERIAL1_IRQn,
    },
    {
#if defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_UARTE2_NS,
#else
        .p = NRF_UARTE2_S,
#endif
        .irq = SERIAL2_IRQn,
    },
    {
#if defined(NRF_TRUSTZONE_NONSECURE)
        .p = NRF_UARTE3_NS,
#else
        .p = NRF_UARTE3_S,
#endif
        .irq = SERIAL3_IRQn,
    },
#elif defined(NRF5340_XXAA) && defined(NRF_NETWORK)
    {
        .p   = NRF_UARTE0_NS,
        .irq = SERIAL0_IRQn,
    },
#else
    {
        .p   = NRF_UARTE0,
        .irq = UARTE0_UART0_IRQn,
    },
    {
        .p   = NRF_UARTE1,
        .irq = UARTE1_IRQn,
    },
#endif
};

static uart_vars_t _uart_vars[UARTE_COUNT] = { 0 };  ///< variable handling the UART context

//=========================== public ===========================================

void db_uart_block_init(uart_t uart, const gpio_t *rx_pin, const gpio_t *tx_pin, uint32_t baudrate, uart_block_rx_cb_t callback) {

#if defined(NRF5340_XXAA)
    if (baudrate > 460800) {
        // On nrf53 configure constant latency mode for better performances with high baudrates
        NRF_POWER->TASKS_CONSTLAT = 1;
    }
#if defined(NRF_APPLICATION)
    // Make sure the peripherals are secure, including DMA
    uint32_t sec_attributes = (SPU_PERIPHID_PERM_SECATTR_Secure << SPU_PERIPHID_PERM_SECATTR_Pos |
                               SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable << SPU_PERIPHID_PERM_SECUREMAPPING_Pos |
                               SPU_PERIPHID_PERM_DMASEC_Secure << SPU_PERIPHID_PERM_DMASEC_Pos);
    // Apply the permission attributes
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM0_SPIS0_TWIM0_TWIS0_UARTE0].PERM = sec_attributes;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM1_SPIS1_TWIM1_TWIS1_UARTE1].PERM = sec_attributes;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM2_SPIS2_TWIM2_TWIS2_UARTE2].PERM = sec_attributes;
    NRF_SPU_S->PERIPHID[NRF_APPLICATION_PERIPH_ID_SPIM3_SPIS3_TWIM3_TWIS3_UARTE3].PERM = sec_attributes;
#endif
#endif

    // configure UART pins (RX as input, TX as output);
    db_gpio_init(rx_pin, DB_GPIO_IN_PU);
    db_gpio_init(tx_pin, DB_GPIO_OUT);

    // configure UART
    _devs[uart].p->CONFIG   = 0;
    _devs[uart].p->PSEL.RXD = (rx_pin->port << UARTE_PSEL_RXD_PORT_Pos) |
                              (rx_pin->pin << UARTE_PSEL_RXD_PIN_Pos) |
                              (UARTE_PSEL_RXD_CONNECT_Connected << UARTE_PSEL_RXD_CONNECT_Pos);
    _devs[uart].p->PSEL.TXD = (tx_pin->port << UARTE_PSEL_TXD_PORT_Pos) |
                              (tx_pin->pin << UARTE_PSEL_TXD_PIN_Pos) |
                              (UARTE_PSEL_TXD_CONNECT_Connected << UARTE_PSEL_TXD_CONNECT_Pos);
    _devs[uart].p->PSEL.RTS = 0xffffffff;  // pin disconnected
    _devs[uart].p->PSEL.CTS = 0xffffffff;  // pin disconnected

    // configure baudrate
    switch (baudrate) {
        case 1200:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud1200 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 9600:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud9600 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 14400:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud14400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 19200:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud19200 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 28800:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud28800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 31250:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud31250 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 38400:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud38400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 56000:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud56000 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 57600:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud57600 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 76800:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud76800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 115200:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud115200 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 230400:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud230400 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 250000:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud250000 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 460800:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud460800 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 921600:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud921600 << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        case 1000000:
            _devs[uart].p->BAUDRATE = (UARTE_BAUDRATE_BAUDRATE_Baud1M << UARTE_BAUDRATE_BAUDRATE_Pos);
            break;
        default:
            // error, return without enabling UART
            return;
    }

    _devs[uart].p->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

    if (callback) {
        _uart_vars[uart].callback    = callback;
        _devs[uart].p->RXD.MAXCNT    = 1;
        _devs[uart].p->RXD.PTR       = (uint32_t)_uart_vars[uart].buffer;
        _devs[uart].p->INTENSET      = (UARTE_INTENSET_ENDRX_Enabled << UARTE_INTENSET_ENDRX_Pos);
        _devs[uart].p->TASKS_STARTRX = 1;
        NVIC_EnableIRQ(_devs[uart].irq);
        NVIC_SetPriority(_devs[uart].irq, 0);
        NVIC_ClearPendingIRQ(_devs[uart].irq);
    }

    // Configure the timer
    NRF_TIMER4->TASKS_CLEAR = 1;
    NRF_TIMER4->PRESCALER   = 4;  // Run TIMER at 1MHz
    NRF_TIMER4->BITMODE     = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
    NRF_TIMER4->INTENSET    = (1 << (TIMER_INTENSET_COMPARE0_Pos + TIMER4_CC_NUM - 1));
    NVIC_SetPriority(TIMER4_IRQn, 2);
    NVIC_EnableIRQ(TIMER4_IRQn);
}

void db_uart_block_write(uart_t uart, const uint8_t *buffer, size_t length) {
    _devs[uart].p->EVENTS_ENDTX = 0;
    _devs[uart].p->TXD.PTR      = (uint32_t)&length;
    _devs[uart].p->TXD.MAXCNT   = 1;
    _devs[uart].p->TASKS_STARTTX = 1;
    while (!_devs[uart].p->EVENTS_ENDTX) {
        asm volatile("" :::);
    }

    uint16_t pos = 0;
    // Send DB_UARTE_CHUNK_SIZE (64 Bytes) maximum at a time
    while (pos < length) {
        _devs[uart].p->EVENTS_ENDTX = 0;
        _devs[uart].p->TXD.PTR      = (uint32_t)&buffer[pos];
        if ((pos + DB_UARTE_CHUNK_SIZE) > length) {
            _devs[uart].p->TXD.MAXCNT = length - pos;
        } else {
            _devs[uart].p->TXD.MAXCNT = DB_UARTE_CHUNK_SIZE;
        }
        _devs[uart].p->TASKS_STARTTX = 1;
        while (!_devs[uart].p->EVENTS_ENDTX) {
            asm volatile("" :::);
        }
        pos += DB_UARTE_CHUNK_SIZE;
    }
}

//=========================== interrupts =======================================

static void _uart_isr(uart_t uart) {
    // check if the interrupt was caused by a fully received package
    if (_devs[uart].p->EVENTS_ENDRX) {
        _devs[uart].p->EVENTS_ENDRX = 0;
        // make sure we actually received new data
        if (_uart_vars[uart].state == IDLE && _uart_vars[uart].buffer[0]) {
            // first byte received, start a new frame
            _devs[uart].p->RXD.MAXCNT = _uart_vars[uart].buffer[0];
            _uart_vars[uart].state = FRAME_RX;
            NRF_TIMER4->TASKS_CAPTURE[TIMER4_CC_NUM - 1] = 1;
            NRF_TIMER4->CC[TIMER4_CC_NUM - 1] += 20000;
            NRF_TIMER4->TASKS_START = 1;
        } else if (_uart_vars[uart].state == FRAME_RX && _uart_vars[uart].callback) {
            NRF_TIMER4->TASKS_STOP = 1;
            // we received a frame that is smaller than half the buffer size, so we can process it now
            _devs[uart].p->RXD.PTR = (uint32_t)_uart_vars[uart].buffer;
            _devs[uart].p->RXD.MAXCNT = 1;
            _uart_vars[uart].callback(_uart_vars[uart].buffer, _devs[uart].p->RXD.AMOUNT);
            _uart_vars[uart].state = IDLE;
        }
        _devs[uart].p->TASKS_STARTRX = 1;
    }
};

#if defined(NRF5340_XXAA)
void SERIAL0_IRQHandler(void) {
    _uart_isr(0);
}

#if defined(NRF5340_XXAA_APPLICATION)
void SERIAL1_IRQHandler(void) {
    _uart_isr(1);
}

void SERIAL2_IRQHandler(void) {
    _uart_isr(2);
}

void SERIAL3_IRQHandler(void) {
    _uart_isr(3);
}
#endif  // NRF5340_XXAA_APPLICATION

#else  // NRF5340_XXAA
void UARTE0_UART0_IRQHandler(void) {
    _uart_isr(0);
}

void UARTE1_IRQHandler(void) {
    _uart_isr(1);
}
#endif

void TIMER4_IRQHandler(void) {
   if (NRF_TIMER4->EVENTS_COMPARE[TIMER4_CC_NUM - 1]) {
        NRF_TIMER4->EVENTS_COMPARE[TIMER4_CC_NUM - 1] = 0;
        NRF_TIMER4->TASKS_STOP = 1;
        _devs[0].p->RXD.MAXCNT = 1;
        _uart_vars[0].state = IDLE;
        _devs[0].p->TASKS_STARTRX = 1;
   }
}
