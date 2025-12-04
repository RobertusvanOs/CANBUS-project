/* *************************************************

Basic MCP2515 CAN application. Rx and Tx of CAN frames
 
(c) dkroeske@gmail.com

v1.0    2023-10-10: Initial code
v1.1    2023-12-04: Added filtering example

***************************************************/

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "mcp2515.h"
#include "mcp2515_regs.h"

// Function prototypes
//
// CAN callbacks
void on_can_rx(CAN_DATA_FRAME_STRUCT *frame);
void on_can_tx(CAN_DATA_FRAME_STRUCT *frame);
void on_can_err(CAN_ERR_FRAME_STRUCT *err);
//
// CAN frame/err debug printf's
void debug_config();
void debug_dataframe(CAN_DATA_FRAME_STRUCT *frame);
void debug_errframe(CAN_ERR_FRAME_STRUCT *frame); 
//
//SPI check
int mcp2515_check_spi();

#define NODE_ROLE 1

#define STARTUP_DELAY_MS 10000

int main() {

    stdio_usb_init();
    sleep_ms(STARTUP_DELAY_MS);
        
    can_init(REQOP_NORMAL);

    if (!mcp2515_check_spi()) {
        printf("MCP2515 SPI fout\n");
        while (true) {
            sleep_ms(1000);
        }
    }

    debug_config();

    can_set_rx_handler(&on_can_rx);
    can_set_tx_handler(&on_can_tx);
    can_set_err_handler(&on_can_err);

    CAN_DATA_FRAME_STRUCT tx_frame;    
    uint16_t x = 0;
    
    while (true) {

        if (NODE_ROLE == 1) {
            tx_frame.id = 0x101;
            tx_frame.datalen = 2;
            tx_frame.data[0] = (uint8_t) x;
            tx_frame.data[1] = (uint8_t) (x >> 8);
            x++;
            can_tx_extended_data_frame(&tx_frame);
        }

        for (int i = 0; i < 10; i++) {
            can_poll();
            sleep_ms(5);
        }
    }
    
}


/* CAN CALLBACK FUNCTIONS */

/* ************************************************************* */
void on_can_rx(CAN_DATA_FRAME_STRUCT *frame) 
/* 
short   :         
inputs  :        
outputs : 
notes   :         
Version : DMK, Initial code
***************************************************************** */
{
    if (frame->id != 0x101 && frame->id != 0x50) {
        return;
    }
    debug_dataframe(frame);

    if (NODE_ROLE == 2 && frame->id == 0x101) {
        CAN_DATA_FRAME_STRUCT tx_frame;
        tx_frame.id = 0x50;
        tx_frame.datalen = frame->datalen;
        for (uint8_t i = 0; i < frame->datalen; i++) {
            tx_frame.data[i] = frame->data[i];
        }
        can_tx_extended_data_frame(&tx_frame);
    }
}

/* ***************************************************************************************** */
void on_can_tx(CAN_DATA_FRAME_STRUCT *frame) 
/* 
short   :         
inputs  :        
outputs : 
notes   :         
Version : DMK, Initial code
***************************************************************** */
{
    puts(">> can_tx()");
    puts("<< can_tx()");
}

/* ***************************************************************************************** */
void on_can_err(CAN_ERR_FRAME_STRUCT *err) 
/* 
short   :         
inputs  :        
outputs : 
notes   :         
Version : DMK, Initial code
***************************************************************** */
{
    puts(">> on_can_err()");
    debug_errframe(err);
    puts("<< on_can_err()");
}


/* UTIL FUNCTIONS */

/* ************************************************************* */
void debug_config() 
/* 
short   :         
inputs  :        
outputs : 
notes   :         
Version : DMK, Initial code
***************************************************************** */
{
    uint8_t data =  mcp2515_read_register(CANSTAT);
    switch (data >> 5) {
        case 0x00: 
            printf("Normal Operating mode\n");
            break;
        case 0x01: 
            printf("Sleep mode\n");
            break;
        case 0x02: 
            printf("Loopback mode\n");
            break;
        case 0x03: 
            printf("Listen-Only mode\n");
            break;
        case 0x04: 
            printf("Configuration mode\n");
            break;
        default: 
            printf("Invalid mode!\n");
    }
}

/* ***************************************************************************************** */
void debug_dataframe(CAN_DATA_FRAME_STRUCT *frame) 
/* 
short   : Print CAN dataframe to terminal
inputs  :        
outputs : 
notes   :         
Version : DMK, Initial code
***************************************************************** */
{
    char buf[1048] = "";
    char line[80];
    sprintf(line, "[DATAFRAME] ");
    strcat(buf, line);

    sprintf(line, "ID: 0x%.8X ", (unsigned int)frame->id);
    strcat(buf, line);
    
    sprintf(line, "LEN: %.2d ", (unsigned int)frame->datalen);
    strcat(buf, line);

    sprintf(line, "DATA: ");
    strcat(buf,line);
    for(uint8_t idx = 0; idx < frame->datalen; idx++) {
        sprintf(line, "0x%.2X ", frame->data[idx]);
        strcat(buf, line);
    }
    
    puts(buf);
}

/* ***************************************************************************************** */
void debug_errframe(CAN_ERR_FRAME_STRUCT *frame) 
/* 
short   : Print CAN errors to terminal
inputs  :        
outputs : 
notes   :         
Version : DMK, Initial code
***************************************************************** */
{
    printf("*********** MCP 2515 ERROR *************\n");
    printf("REC    : 0x%.2X\n", frame->rREC);
    printf("TEC    : 0x%.2X\n", frame->rTEC);
    printf("EFLG   : 0x%.2X\n", frame->rEFLG);
    printf("CANINTF: 0x%.2X\n", frame->rCANINTF);
}
int mcp2515_check_spi()
{
    uint8_t data = mcp2515_read_register(CANSTAT);
    if ((data & 0xE0) == 0x00 || (data & 0xE0) == 0x40) {
        return 1;
    } else {
        return 0;
    }
}
