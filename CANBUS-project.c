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
//Node_Role 1 is de zender, Node_Role 2 de ontvanger.
#define NODE_ROLE 1
//
// Definieer de digitale ingang voor de knop.
#define CLAXON_KNOP 2
//
//Claxon sensor en claxon ID.
#define CLAXON_SENSOR_ID 0x599
#define CLAXON_STATUS_ID 0x733
//SPI check. 
int mcp2515_check_spi();
#define MCP2515_TEST_REG BFPCTRL
//
//We hebben tijd nodig om serial monitor te openen, zodat wij de eerste frames kunnen ontvangen.
#define STARTUP_DELAY_MS 8000

bool getCanFlag();
bool getButtonVal();
void setCanFlag(bool val);

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

    while (true) {

        if (NODE_ROLE == 1) {
            static uint8_t prev_knop_status = 0xFF;
            uint8_t knop_status = getButtonVal() ? 1 : 0;

            if (getCanFlag() && knop_status != prev_knop_status) {

                printf("Knop status=%d\n", knop_status);

                CAN_DATA_FRAME_STRUCT tx_frame;
                tx_frame.id = CLAXON_SENSOR_ID;
                tx_frame.datalen = 1;
                tx_frame.data[0] = knop_status;

                int rc = can_tx_extended_data_frame(&tx_frame);
                printf("can_tx rc=%d, ID=0x%08X, LEN=%d, DATA0=0x%02X\n",
                       rc,
                       (unsigned int)tx_frame.id,
                       tx_frame.datalen,
                       tx_frame.data[0]);

                prev_knop_status = knop_status;
                setCanFlag(false);
            }
        }

        sleep_ms(5);
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
   if (NODE_ROLE == 2) {
        if (frame->id != CLAXON_SENSOR_ID || frame->datalen == 0) {
            return;
        }

        debug_dataframe(frame);  // laat 0x599 met 0x00/0x01 zien

        CAN_DATA_FRAME_STRUCT tx_frame;
        tx_frame.id = CLAXON_STATUS_ID;
        tx_frame.datalen = 1;
        tx_frame.data[0] = frame->data[0];
        can_tx_extended_data_frame(&tx_frame);

    } else if (NODE_ROLE == 1) {
        if (frame->id != CLAXON_STATUS_ID || frame->datalen == 0) {
            return;
        }

        debug_dataframe(frame);
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
    // Lees de oorspronkelijke waarde van het testregister
    uint8_t orig = mcp2515_read_register(MCP2515_TEST_REG);

    // Schrijf een testpatroon
    mcp2515_write_register(MCP2515_TEST_REG, 0x55);
    uint8_t a = mcp2515_read_register(MCP2515_TEST_REG);

    // Zet de oude waarde netjes terug
    mcp2515_write_register(MCP2515_TEST_REG, orig);

    // Debug, één keer bij opstart
    printf("SPI-test: orig=0x%02X a=0x%02X\n", orig, a);

    // Als SPI werkt, verandert het register naar 0x05 (meestal ≠ orig)
    // Als SPI kapot is, blijft a gelijk aan orig (0x00 of 0xFF of iets anders)
    if (a == orig) {
        return 0;   // fout
    } else {
        return 1;   // oké
    }
}
