/* *************************************************

Basic MCP2515 CAN library. Rx and Tx of CAN frames
 
Just to test the PICO to MCP2515 interface. Connections:
 
MCP2515         PICO            (pin)  
---------------------------------------------------------
Vcc    <-->     Vbus            (#40)
GND    <-->     GND             (#38 or other ground pin)
CS     <-->     SPI0_CSn        (#22)
SO     <-->     SPIO_RX         (#21)
SI     <-->     SPIO_TX         (#25)
SCK    <-->     SPIO_SCK        (#24)
INT    <-->     GPIO_21         (#27)         

(c) dkroeske@gmail.com

v1.0    2023-10-10: Initial code
v1.1    2023-10-17: Added rx, tx en err callback functions

***************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "mcp2515_regs.h"
#include "mcp2515.h"

#define MCP2515_INT_PIN 21   // Pico GPIO 21 = pin 27

static inline void spi_cs_select();
static inline void spi_cs_deselect();

// CAN function pointers
void (*can_rx_fp)(CAN_DATA_FRAME_STRUCT *) = NULL;
void (*can_tx_fp)(CAN_DATA_FRAME_STRUCT *) = NULL;
void (*can_err_fp)(CAN_ERR_FRAME_STRUCT *) = NULL;

// App GPIO irq forwarder (voor knop/andere IO in main.c)
static void (*app_gpio_irq_fp)(uint gpio, uint32_t events) = NULL;

void can_set_gpio_irq_handler(void (*f)(uint gpio, uint32_t events))
{
    app_gpio_irq_fp = f;
}

// ISR -> main-context flag
static volatile bool mcp2515_irq_pending = false;

/* ************************************************************** */
void can_init(uint8_t mode)
{
    // Init SPI
    spi_init(spi_default, 5*1000 * 1000);
    spi_set_format(spi_default, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "spi CS"));

    gpio_init(MCP2515_INT_PIN);
    gpio_set_dir(MCP2515_INT_PIN, GPIO_IN);
    gpio_pull_up(MCP2515_INT_PIN);

    gpio_set_irq_enabled_with_callback(
        MCP2515_INT_PIN,
        GPIO_IRQ_EDGE_FALL,
        true,
        &mcp2515_callback
    );

    // Init mcp2515
    mcp2515_init(mode);
}


/**************************************************************** */
uint8_t can_tx_extended_data_frame(CAN_DATA_FRAME_STRUCT *frame)
{
    uint8_t err = 0, tx_buf_id;

    // Find free transmitbuffer (out of 3)
    uint8_t status = mcp2515_read_status();
    if (!(status & 0x04)) {
        tx_buf_id = 0x00;
    } else if (!(status & 0x10)) {
        tx_buf_id = 0x01;
    } else if (!(status & 0x40)) {
        tx_buf_id = 0x02;
    } else {
        err = 1;
    }

    if (0 == err) {
        uint8_t buf[14];
        uint8_t datalen = frame->datalen <= 8 ? frame->datalen : 8;

        buf[0] = (uint8_t)((frame->id << 3) >> 24);
        buf[1] = (uint8_t)((((frame->id << 3) | (frame->id & 0x00030000)) >> 16)
                           | EXIDE);
        buf[2] = frame->id >> 8;
        buf[3] = frame->id;
        buf[4] = datalen;
    
        for (uint8_t idx = 0; idx < datalen; idx++) {
            buf[5 + idx] = frame->data[idx];
        }
    
        mcp2515_load_tx_buffer(tx_buf_id, buf, 13);
        mcp2515_RTS(tx_buf_id);
    } else {
        printf("Error finding free transmitbuffer\n");
    }

    return err;
}

/* ************************************************************** */
void can_set_rx_handler( void (*f)(CAN_DATA_FRAME_STRUCT *) )
{
    if (f != NULL) {
        can_rx_fp = f;
    }
}

/* ************************************************************** */
void can_set_tx_handler( void (*f)(CAN_DATA_FRAME_STRUCT *) )
{
    if (f != NULL) {
        can_tx_fp = f;
    }
}

/* ************************************************************** */
void can_set_err_handler( void (*f)(CAN_ERR_FRAME_STRUCT *) )
{
    if (f != NULL) {
        can_err_fp = f;
    }
}


/* ************************************************************** */
void mcp2515_init(uint8_t mode) 
{
    mcp2515_reset();
    sleep_ms(10);

    mcp2515_write_register(RXM0SIDH, 0x00);
    mcp2515_write_register(RXM0SIDL, 0x00);
    mcp2515_write_register(RXM0EID8, 0x00);
    mcp2515_write_register(RXM0EID0, 0x00);

    mcp2515_write_register(CNF1, 0x00);
    mcp2515_write_register(CNF2, 0xB1);
    mcp2515_write_register(CNF3, 0x85);

    // 500 kbps @ 8 MHz: Tq=0.25us (BRP=0), 8 Tq/bit, sample ~75%
    //mcp2515_write_register(CNF1, 0x00); // SJW=1, BRP=0
    //mcp2515_write_register(CNF2, 0x91); // BTLMODE=1, PHSEG1=3, PRSEG=2
    //mcp2515_write_register(CNF3, 0x01); // PHSEG2=2


    mcp2515_write_register(CANINTE, MERRE | ERRIE | RX1IE | RX0IE); 

    mcp2515_write_register(CANCTRL, mode);

    uint8_t dummy = mcp2515_read_register(CANSTAT);
    if (mode != (dummy & 0xE0)) {
        printf("Error setting mode!\n");
        mcp2515_write_register(CANCTRL, mode);
    }
}

/* ************************************************************** */
static void mcp2515_handle_interrupt_status(uint8_t status)
{
    uint8_t buf[14];

    // RX0 uitlezen (ook als RX1 ook set is)
    if (status & RX0IF) {
        mcp2515_read_rx_buffer(0, buf, 14);

        CAN_DATA_FRAME_STRUCT frame = {0};
        
        
        if (buf[1] & 0x08) { // extended
    frame.id =
        ((uint32_t)buf[0] << 21) |
        ((uint32_t)(buf[1] & 0xE0) << 13) |
        ((uint32_t)(buf[1] & 0x03) << 16) |
        ((uint32_t)buf[2] << 8)  |
         (uint32_t)buf[3];
} else { // standard
    frame.id = ((uint16_t)buf[0] << 3) | (buf[1] >> 5);
}

        
        frame.datalen = (buf[4] & 0x0F) <= 8 ? (buf[4] & 0x0F) : 8;
        
        for (uint8_t idx = 0; idx < frame.datalen; idx++) {
           frame.data[idx] = buf[5 + idx];
        }
        
        if (can_rx_fp != NULL) {
            can_rx_fp(&frame);
        }
    }

    // RX1 uitlezen (ook als RX0 ook set is)
    if (status & RX1IF) {
        mcp2515_read_rx_buffer(1, buf, 14);

        CAN_DATA_FRAME_STRUCT frame = {0};
        
        
        if (buf[1] & 0x08) { // extended
    frame.id =
        ((uint32_t)buf[0] << 21) |
        ((uint32_t)(buf[1] & 0xE0) << 13) |
        ((uint32_t)(buf[1] & 0x03) << 16) |
        ((uint32_t)buf[2] << 8)  |
         (uint32_t)buf[3];
} else { // standard
    frame.id = ((uint16_t)buf[0] << 3) | (buf[1] >> 5);
}

        
        frame.datalen = (buf[4] & 0x0F) <= 8 ? (buf[4] & 0x0F) : 8;
        
        for (uint8_t idx = 0; idx < frame.datalen; idx++) {
           frame.data[idx] = buf[5 + idx];
        }
        
        if (can_rx_fp != NULL) {
            can_rx_fp(&frame);
        }
    }

    uint8_t eflg = mcp2515_read_register(EFLG);
    if (eflg != 0 && can_err_fp != NULL) {
        CAN_ERR_FRAME_STRUCT err;
        err.rREC     = mcp2515_read_register(REC);
        err.rTEC     = mcp2515_read_register(TEC);
        err.rEFLG    = eflg;
        err.rCANINTF = status;
        can_err_fp(&err);
    }

    mcp2515_bit_modify(CANINTF, 0xFF, 0x00);
    mcp2515_bit_modify(EFLG,   0xFF, 0x00);
}

/* ************************************************************** */
void mcp2515_callback(uint gpio, uint32_t events)
{
    if (gpio == MCP2515_INT_PIN) {
        mcp2515_irq_pending = true;
    } else {
        // Alles wat niet MCP2515 is: doorsturen naar main.c
        if (app_gpio_irq_fp != NULL) {
            app_gpio_irq_fp(gpio, events);
        }
    }
}

/* ************************************************************** */
void can_poll(void)
{
    if (!mcp2515_irq_pending) {
        return;
    }

    mcp2515_irq_pending = false;

    for (int i = 0; i < 8; i++) {
        uint8_t status = mcp2515_read_register(CANINTF);
        if (status == 0) {
            break;
        }
        mcp2515_handle_interrupt_status(status);
    }
}

/* ************************************************************** */
void mcp2515_reset()
{
    uint8_t buf[] = { CAN_RESET };
    
    uint32_t irq_status = save_and_disable_interrupts();
    spi_cs_select();
    spi_write_blocking(spi_default, buf, 1);
    spi_cs_deselect();
    restore_interrupts(irq_status);
}


/* ************************************************************** */
uint8_t mcp2515_read_register(uint8_t addr)
{
    uint8_t buf[2];
    buf[0] = CAN_READ;
    buf[1] = addr;
    uint8_t data;

    uint32_t irq_status = save_and_disable_interrupts();
    spi_cs_select();
    spi_write_blocking(spi_default, buf, 2);
    spi_read_blocking(spi_default, 0, &data, 1);
    spi_cs_deselect();
    restore_interrupts(irq_status);
        
    return data;
}

/* *************************************************************** */
void mcp2515_read_rx_buffer(uint8_t buffer_id, uint8_t *data, uint8_t len)
{
    uint8_t cmd;
    switch(buffer_id) {
        case 0x00: cmd = 0x90; break;
        case 0x01: cmd = 0x94; break;
        default:   cmd = 0x90; break;
    }
    
    uint32_t irq_status = save_and_disable_interrupts();
    spi_cs_select();
    spi_write_blocking(spi_default, &cmd, 1);
    spi_read_blocking(spi_default, 0, data, len);
    spi_cs_deselect();
    restore_interrupts(irq_status);
}

/* ************************************************************** */
void mcp2515_write_register(uint8_t addr, uint8_t data)
{
    uint8_t buf[3];
    buf[0] = CAN_WRITE;
    buf[1] = addr;
    buf[2] = data;
        
    uint32_t irq_status = save_and_disable_interrupts();
    spi_cs_select();
    spi_write_blocking(spi_default, buf, 3);
    spi_cs_deselect();
    restore_interrupts(irq_status);
} 


/* ************************************************************** */
void mcp2515_load_tx_buffer(uint8_t buffer_id, uint8_t *data, uint8_t len) 
{
    uint8_t cmd;
    switch(buffer_id) {
        case 0x00: cmd = 0x40; break;
        case 0x01: cmd = 0x42; break;
        case 0x02: cmd = 0x44; break;
        default:   cmd = 0x40; break;
    }

    uint32_t irq_status = save_and_disable_interrupts();
    spi_cs_select();
    spi_write_blocking(spi_default, &cmd, 1);
    spi_write_blocking(spi_default, data, len);
    spi_cs_deselect();
    restore_interrupts(irq_status);
}


/* ************************************************************** */
void mcp2515_RTS(uint8_t tx_buf_id)
{
    uint8_t err = 0;
    uint8_t data = 0x00;

    switch(tx_buf_id) {
        case 0x00: data = CAN_RTS_TXB0; break;
        case 0x01: data = CAN_RTS_TXB1; break;
        case 0x02: data = CAN_RTS_TXB2; break;
        default:
            err = 1;
            printf("\tInvalid tx_buf_id: 0x%.2X\n", tx_buf_id);
    }

    if (!err) {
        uint32_t irq_status = save_and_disable_interrupts();
        spi_cs_select();
        spi_write_blocking(spi_default, &data, 1);
        spi_cs_deselect();
        restore_interrupts(irq_status);
    }
}


/* ************************************************************** */
uint8_t mcp2515_read_status(void)
{
    uint8_t txbuf[3];
    txbuf[0] = CAN_RD_STATUS;
    txbuf[1] = 0x00;
    txbuf[2] = 0x00;
    uint8_t rxbuf[2];
        
    uint32_t irq_status = save_and_disable_interrupts();
    spi_cs_select();
    spi_write_blocking(spi_default, txbuf, 3);
    spi_read_blocking(spi_default, 0, rxbuf, 2);
    spi_cs_deselect();
    restore_interrupts(irq_status);
    
    return rxbuf[0];
}


/* ************************************************************** */
uint8_t mcp2515_rx_status(void)
{
    uint8_t txbuf[3];
    txbuf[0] = CAN_RX_STATUS;
    txbuf[1] = 0x00;
    txbuf[2] = 0x00;
    uint8_t rxbuf[2];
        
    uint32_t irq_status = save_and_disable_interrupts();
    spi_cs_select();
    spi_write_blocking(spi_default, txbuf, 3);
    spi_read_blocking(spi_default, 0, rxbuf, 2);
    spi_cs_deselect();
    restore_interrupts(irq_status);

    return rxbuf[0];
}


/* ************************************************************** */
void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    uint8_t buf[4];
    buf[0] = CAN_BIT_MODIFY;
    buf[1] = addr;
    buf[2] = mask;
    buf[3] = data;
    
    uint32_t irq_status = save_and_disable_interrupts();
    spi_cs_select();
    spi_write_blocking(spi_default, buf, 4);
    spi_cs_deselect();
    restore_interrupts(irq_status);
}

/* ************************************************************** */
static inline void spi_cs_select() 
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0); 
    asm volatile("nop \n nop \n nop");
}

/* ************************************************************** */
static inline void spi_cs_deselect()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}
