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
#include "pico/util/queue.h"
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
//Claxon sensor en claxon ID.
#define CLAXON_SENSOR_ID 0x599
#define CLAXON_STATUS_ID 0x733

// Led die aangeeft of de claxon aan is.
#define CLAXON_LED 15

// Knop pin (in main.c)
#define CLAXON_KNOP 2

// Debounce tijd (microseconden)
#define KNOP_DEBOUNCE_US 20000u   // 20 ms

//SPI check. 
int mcp2515_check_spi();
#define MCP2515_TEST_REG BFPCTRL
//
//We hebben tijd nodig om serial monitor te openen, zodat wij de eerste frames kunnen ontvangen.
#define STARTUP_DELAY_MS 8000

// Heartbeat-instellingen.
#define HEARTBEAT_BASE_ID      0x700
#define HEARTBEAT_INTERVAL_MS  1000
#define HEARTBEAT_TIMEOUT_MS   1500

// Heartbeat-status.
absolute_time_t next_heartbeat;
absolute_time_t last_rx_heartbeat;
bool peer_online = false;

// Foutstatus.
bool node_offbus = false;
bool error_reported = false;

// ---- Knop events via queue (voorkomt verloren edges) ----
typedef struct {
    uint8_t level;      // 0/1
    uint32_t t_us;      // timestamp (microseconds)
} button_event_t;

static queue_t button_queue;
static volatile uint32_t button_queue_overflow = 0;

// Huidige knopstand (optioneel, handig voor debug)
static volatile bool buttonVal = false;

bool getButtonVal()
{
    return buttonVal;
}

// ---- GPIO IRQ handler “haakje” vanuit mcp2515.c ----
void can_set_gpio_irq_handler(void (*f)(uint gpio, uint32_t events));

// ---- Knop IRQ handler in main.c ----
void app_gpio_irq_handler(uint gpio, uint32_t events)
{
    if (gpio == CLAXON_KNOP) {

        // GPIO_IRQ_EDGE_FALL = knop ingedrukt (pull-up) -> level=1
        // GPIO_IRQ_EDGE_RISE = knop los -> level=0
        uint8_t level;
        if (events & GPIO_IRQ_EDGE_FALL) {
            level = 1;
        } else if (events & GPIO_IRQ_EDGE_RISE) {
            level = 0;
        } else {
            return;
        }

        buttonVal = (level != 0);

        button_event_t ev;
        ev.level = level;
        ev.t_us = time_us_32();

        if (!queue_try_add(&button_queue, &ev)) {
            button_queue_overflow++;
        }
    }
}

int main() {

    stdio_usb_init();
    sleep_ms(STARTUP_DELAY_MS);

    // Led init.
    gpio_init(CLAXON_LED);
    gpio_set_dir(CLAXON_LED, GPIO_OUT);
    gpio_put(CLAXON_LED, 0);

    // Queue init (voor IRQ aan)
    queue_init(&button_queue, sizeof(button_event_t), 64);
        
    // Knop init (in main.c)
    gpio_init(CLAXON_KNOP);
    gpio_set_dir(CLAXON_KNOP, GPIO_IN);
    gpio_pull_up(CLAXON_KNOP);

    can_init(REQOP_NORMAL);

    // Register onze app gpio handler (knop zit nu in main.c)
    can_set_gpio_irq_handler(&app_gpio_irq_handler);

    // Knop IRQ aanzetten (callback is al geregistreerd door can_init)
    gpio_set_irq_enabled(CLAXON_KNOP, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Nieuwe variabelen voor periodieke SPI-check en herstel.
    absolute_time_t next_spi_check = make_timeout_time_ms(1000);
    bool spi_error = false;

    // Initiale SPI-test; bij fout in 'spi_error' blijven we proberen te herstellen.
    int spi_ok = mcp2515_check_spi();
    if (!spi_ok) {
        printf("MCP2515 SPI fout bij opstart\n");
        spi_error = true;
    }

    debug_config();

    // Heartbeat timers initialiseren.
    next_heartbeat = make_timeout_time_ms(HEARTBEAT_INTERVAL_MS);
    last_rx_heartbeat = get_absolute_time();
    peer_online = false;
    node_offbus = false;
    error_reported = false;

    can_set_rx_handler(&on_can_rx);
    can_set_tx_handler(&on_can_tx);
    can_set_err_handler(&on_can_err);

    uint8_t prev_knop_status = 0xFF;

    // Debounce state
    static uint32_t last_edge_us = 0;

    while (true) {

        can_poll();

        // Verwerk ALLE knop-events (edge sensitive + geen verloren transitions)
        if (NODE_ROLE == 1 && !node_offbus) {
            button_event_t ev;
            while (queue_try_remove(&button_queue, &ev)) {

                // Debounce: negeer edges die te snel na elkaar komen
                if ((uint32_t)(ev.t_us - last_edge_us) < KNOP_DEBOUNCE_US) {
                    continue;
                }
                last_edge_us = ev.t_us;

                uint8_t knop_status = ev.level ? 1 : 0;

                if (knop_status != prev_knop_status) {

                    printf("Knop status=%d t_us=%lu\n", knop_status, (unsigned long)ev.t_us);

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
                }
            }

            // Optioneel: melding als queue overflowt (1x per loop is zat)
            static uint32_t prev_overflow = 0;
            uint32_t now_overflow = button_queue_overflow;
            if (now_overflow != prev_overflow) {
                printf("WAARSCHUWING: button_queue overflow=%lu\n", (unsigned long)now_overflow);
                prev_overflow = now_overflow;
            }
        } else {
            // Als zender offbus is: queue leegtrekken zodat je geen oude events verstuurt bij herstel
            button_event_t drop;
            while (queue_try_remove(&button_queue, &drop)) { }
        }

        // Heartbeat verzenden.
        // Bij NODE_ROLE 1 stoppen we bij node_offbus; ontvanger blijft altijd sturen.
        bool hb_blocked = (NODE_ROLE == 1 && node_offbus);   // aanpassing
        if (!hb_blocked &&
            absolute_time_diff_us(get_absolute_time(), next_heartbeat) <= 0) {

            CAN_DATA_FRAME_STRUCT hb;
            hb.id = HEARTBEAT_BASE_ID + NODE_ROLE;
            hb.datalen = 1;
            hb.data[0] = 0xAA;
            can_tx_extended_data_frame(&hb);

            next_heartbeat = make_timeout_time_ms(HEARTBEAT_INTERVAL_MS);
        }

        // Heartbeat-timeout: andere node niet meer zichtbaar.
        if (peer_online &&
            absolute_time_diff_us(last_rx_heartbeat, get_absolute_time()) >
                (HEARTBEAT_TIMEOUT_MS * 1000)) {

            printf("Node niet meer zichtbaar op de bus\n");
            peer_online = false;

            if (NODE_ROLE == 1) {          // alleen zender in foutstatus zetten
                node_offbus = true;
                error_reported = true;
            }
        }

        // Periodieke SPI-test en eventueel herinitialiseren MCP2515.
        if (absolute_time_diff_us(get_absolute_time(), next_spi_check) <= 0) {
            int ok = mcp2515_check_spi();
            if (!ok) {
                if (!spi_error) {
                    printf("MCP2515 SPI fout\n");
                    spi_error = true;
                }
            } else {
                if (spi_error) {
                    printf("MCP2515 SPI hersteld, CAN init\n");
                    can_init(REQOP_NORMAL);
                    can_set_rx_handler(&on_can_rx);
                    can_set_tx_handler(&on_can_tx);
                    can_set_err_handler(&on_can_err);
                    debug_config();
                    spi_error = false;

                    // Na herinit: handler opnieuw zetten (knop blijft in main)
                    can_set_gpio_irq_handler(&app_gpio_irq_handler);
                }
            }
            next_spi_check = make_timeout_time_ms(1000);
        }

        sleep_ms(0);
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
    // Heartbeat van andere node herkennen en eventueel herstel uitvoeren.
    uint32_t expected_hb = HEARTBEAT_BASE_ID + (NODE_ROLE == 1 ? 2 : 1);
    if (frame->id == expected_hb) {
        last_rx_heartbeat = get_absolute_time();

        if (node_offbus && NODE_ROLE == 1) {       // herstel alleen relevant voor zender
            printf("Node zichtbaar op de bus\n");
            can_init(REQOP_NORMAL);
            can_set_rx_handler(&on_can_rx);
            can_set_tx_handler(&on_can_tx);
            can_set_err_handler(&on_can_err);
            debug_config();

            // Na herstel: handler opnieuw zetten
            can_set_gpio_irq_handler(&app_gpio_irq_handler);

            node_offbus = false;
            error_reported = false;
            peer_online = true;
        } else {
            if (!peer_online) {
                printf("Node zichtbaar op de bus\n");
            }
            peer_online = true;
        }
        return;
    }

   if (NODE_ROLE == 2) {
        if (frame->id != CLAXON_SENSOR_ID || frame->datalen == 0) {
            return;
        }

        // Led volgt de daadwerkelijke claxon-aanvraag die binnenkomt.
        gpio_put(CLAXON_LED, frame->data[0] ? 1 : 0);

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

        // Optioneel: op zender kun je ook de led laten volgen op status.
        gpio_put(CLAXON_LED, frame->data[0] ? 1 : 0);

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
    // Eénmalige foutmelding en éénmalige err-debug.
    if (!error_reported) {
        printf("FOUT: CAN-bus of andere node\n");
        debug_errframe(err);

        if (NODE_ROLE == 1) {        // ontvanger blijft ‘online’; zender gaat off-bus
            node_offbus = true;
        }
        error_reported = true;
    }
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
    //printf("SPI-test: orig=0x%02X a=0x%02X\n", orig, a);

    // Als SPI werkt, verandert het register naar 0x05 (meestal ≠ orig)
    // Als SPI kapot is, blijft a gelijk aan orig (0x00 of 0xFF of iets anders)
    if (a == orig) {
        return 0;   // fout
    } else {
        return 1;   // oké
    }
}
