/* *************************************************

Basic MCP2515 CAN application. Rx and Tx of CAN frames
 
(c) dkroeske@gmail.com

v1.0    2023-10-10: Initial code
v1.1    2023-12-04: Added filtering example
v2.0    2026-1-21: Testopstelling door R. van Os en G. Lassche

***************************************************/

//Aan: cycling zenden, Uit: Een aantal keer bevestigen, DONE
//ACK: uit de code halen. 
//Ontvanger: 250ms geen signaal = claxon uit. 
//Heartbeat optioneel toevoegen. DONE

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
void can_poll(void);
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
#define ENABLE_HEARTBEAT   1   // set to 0 to disable
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

// Claxon cyclisch herhalen (ms) zodat een “domme” ontvanger blijft volgen
#define CLAXON_CYCLIC_MS 50
uint8_t cyclish_uit_teller = 0;

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

// Herstel zonder heartbeat (ms)
#define CAN_RECOVER_INTERVAL_MS 1000

// Failsafe ontvanger (role 2): 250ms geen claxonframe = claxon uit
#define FAILSAFE_TIMEOUT_MS 250

#if ENABLE_HEARTBEAT
// Heartbeat-status.
absolute_time_t next_heartbeat;
absolute_time_t last_rx_heartbeat;
bool peer_online = false;
#endif

// Foutstatus.
bool node_offbus = false;
bool error_reported = false;

// ---- Receiver failsafe state (role 2) ----
static absolute_time_t last_rx_claxon;
static bool claxon_output = false;
static bool failsafe_tripped = false;

// ---- Knop events via queue (voorkomt verloren edges) ----
typedef struct {
    bool state;      // 0/1
    uint32_t t_us;   // timestamp (microseconds)
} button_event_t;

static queue_t button_queue;
static volatile uint32_t button_queue_overflow = 0;

static inline bool knop_status_from_pin(void)
{
    return (gpio_get(CLAXON_KNOP) == 0) ? 1 : 0;
}

// ---- GPIO IRQ handler “haakje” vanuit mcp2515.c ----
void can_set_gpio_irq_handler(void (*f)(uint gpio, uint32_t events));

// ---- Knop IRQ handler in main.c ----
void app_gpio_irq_handler(uint gpio, uint32_t events)
{
    if (gpio == CLAXON_KNOP) {
        button_event_t evt = {
            .t_us   = time_us_32(),
            .state  = knop_status_from_pin()
        };
        queue_try_add(&button_queue, &evt);
    }
}

// ---- 1x melden helper ----
static inline void report_once(const char *msg)
{
    if (!error_reported) {
        printf("%s\n", msg);
        error_reported = true;
    }
}

int main() {

    stdio_usb_init();
    sleep_ms(STARTUP_DELAY_MS);

    // Led init.
    gpio_init(CLAXON_LED);
    gpio_set_dir(CLAXON_LED, GPIO_OUT);

    //Led knipper om start-up aan te tonen
    for(int i = 0; i < 2; i++){
        gpio_put(CLAXON_LED, 1);
        sleep_ms(500);
        gpio_put(CLAXON_LED, 0);
        sleep_ms(500);
    }

    // Queue init (voor IRQ aan)
    queue_init(&button_queue, sizeof(button_event_t), 3);
        
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

    // Herstel-timer voor CAN (zonder heartbeat)
    absolute_time_t next_can_recover = make_timeout_time_ms(CAN_RECOVER_INTERVAL_MS);

    #define UART_TIMEOUT 100
    absolute_time_t next_uart_print = make_timeout_time_ms(UART_TIMEOUT);
    CAN_DATA_FRAME_STRUCT last_tx_frame = {0};
    int last_rc = 0;
    CAN_DATA_FRAME_STRUCT last_tx_frame_printed = {0};

    // Receiver failsafe init (role 2)
    last_rx_claxon = get_absolute_time();
    claxon_output = false;
    failsafe_tripped = false;
    gpio_put(CLAXON_LED, 0);

    // Initiale SPI-test; bij fout in 'spi_error' blijven we proberen te herstellen.
    int spi_ok = mcp2515_check_spi();
    if (!spi_ok) {
        printf("MCP2515 SPI fout bij opstart\n");
        spi_error = true;
    }

    debug_config();

    #if ENABLE_HEARTBEAT
    // Heartbeat timers initialiseren.
    // (SPI-check is hierboven al als eerste gedaan)
    next_heartbeat = make_timeout_time_ms(HEARTBEAT_INTERVAL_MS);
    last_rx_heartbeat = get_absolute_time();
    peer_online = false;
    node_offbus = false;
    error_reported = false;
    #endif

    can_set_rx_handler(&on_can_rx);
    can_set_tx_handler(&on_can_tx);
    can_set_err_handler(&on_can_err);

    // Claxon cyclisch zenden
    uint8_t claxon_state = 0;
    absolute_time_t next_claxon_tx = make_timeout_time_ms(CLAXON_CYCLIC_MS);

    //tijd variabele voor debouncing button
    static uint32_t last_accepted_us = 0;

    while (true) {
        can_poll();

        // FAILSAFE role 2: geen claxonframes binnen FAILSAFE_TIMEOUT_MS => claxon uit
        if (NODE_ROLE == 2 && claxon_output) {
            if (absolute_time_diff_us(last_rx_claxon, get_absolute_time()) >
                (FAILSAFE_TIMEOUT_MS * 1000)) {

                gpio_put(CLAXON_LED, 0);
                claxon_output = false;

                if (!failsafe_tripped) {
                    CAN_DATA_FRAME_STRUCT tx_frame;
                    tx_frame.id = CLAXON_STATUS_ID;
                    tx_frame.datalen = 1;
                    tx_frame.data[0] = 0;
                    can_tx_extended_data_frame(&tx_frame);
                    failsafe_tripped = true;
                }
            }
        }

        // Herstel zonder heartbeat: zender terug laten komen, maar NIET opnieuw foutmelding armeren.
        #if !ENABLE_HEARTBEAT
        if (NODE_ROLE == 1 && node_offbus && !spi_error) {
            if (absolute_time_diff_us(get_absolute_time(), next_can_recover) <= 0) {
                can_init(REQOP_NORMAL);
                can_set_rx_handler(&on_can_rx);
                can_set_tx_handler(&on_can_tx);
                can_set_err_handler(&on_can_err);
                can_set_gpio_irq_handler(&app_gpio_irq_handler);

                // Belangrijk: zender weer vrijgeven zodat hij kan herstellen als de ontvanger terug is
                node_offbus = false;

                // Belangrijk: NIET resetten -> foutmelding blijft 1x per offline-periode
                // error_reported = false;

                next_can_recover = make_timeout_time_ms(CAN_RECOVER_INTERVAL_MS);
            }
        }
        #endif

        // Cyclisch de knop-pin uitlezen.
        claxon_state = knop_status_from_pin();

        // ZENDER: pin-waarheid + stabiele debounce + edge + cyclisch herhalen
        if (NODE_ROLE == 1 && !node_offbus) {

            //Knop uitlezen via IRQ-trigger en debounce toepassen
            button_event_t ev;
            if (queue_try_remove(&button_queue, &ev)) {
                if ((uint32_t)(ev.t_us - last_accepted_us) > KNOP_DEBOUNCE_US) {
                    last_accepted_us = ev.t_us;
                    claxon_state = ev.state;

                    // BRACE-FIX: alleen zenden als debounce is geaccepteerd
                    CAN_DATA_FRAME_STRUCT tx_frame;
                    tx_frame.id = CLAXON_SENSOR_ID;
                    tx_frame.datalen = 1;
                    tx_frame.data[0] = claxon_state;

                    last_tx_frame = tx_frame;
                    int rc = can_tx_extended_data_frame(&tx_frame);
                    last_rc = rc;

                    //Na een edge: cyclische timer resetten
                    next_claxon_tx = make_timeout_time_ms(CLAXON_CYCLIC_MS);
                    cyclish_uit_teller = 0; //teller resetten voor cyclish-uit signaal
                }
            }

            // Cyclisch herhalen (zonder printf)
            if (absolute_time_diff_us(get_absolute_time(), next_claxon_tx) <= 0) {
                if(claxon_state)
                { //Alleen cyclish aan signaal sturen.
                    CAN_DATA_FRAME_STRUCT tx_frame;
                    tx_frame.id = CLAXON_SENSOR_ID;
                    tx_frame.datalen = 1;
                    tx_frame.data[0] = claxon_state;
                    (void)can_tx_extended_data_frame(&tx_frame);
                }
                else if (cyclish_uit_teller < 5) {
                    CAN_DATA_FRAME_STRUCT tx_frame;
                    tx_frame.id = CLAXON_SENSOR_ID;
                    tx_frame.datalen = 1;
                    tx_frame.data[0] = claxon_state; // 0
                    (void)can_tx_extended_data_frame(&tx_frame);
                    cyclish_uit_teller++;           
                }

                next_claxon_tx = make_timeout_time_ms(CLAXON_CYCLIC_MS);
            }

        } else {
            // Als zender offbus is: queue leegtrekken zodat je geen oude events verstuurt bij herstel
            button_event_t drop;
            while (queue_try_remove(&button_queue, &drop)) { }

            // Als zender offbus is: cyclisch zenden pauzeren (geen extra busload)
            next_claxon_tx = make_timeout_time_ms(CLAXON_CYCLIC_MS);
        }

        #if ENABLE_HEARTBEAT
        bool hb_blocked = (NODE_ROLE == 1 && node_offbus);
        if (!hb_blocked &&
            absolute_time_diff_us(get_absolute_time(), next_heartbeat) <= 0) {

            CAN_DATA_FRAME_STRUCT hb;
            hb.id = HEARTBEAT_BASE_ID + NODE_ROLE;
            hb.datalen = 1;
            hb.data[0] = 0xAA;
            can_tx_extended_data_frame(&hb);

            next_heartbeat = make_timeout_time_ms(HEARTBEAT_INTERVAL_MS);
        }

        // Heartbeat timeout -> eerst SPI-check opnieuw; op basis daarvan foutmelding kiezen.
        if (peer_online &&
            absolute_time_diff_us(last_rx_heartbeat, get_absolute_time()) >
                (HEARTBEAT_TIMEOUT_MS * 1000)) {

            int ok = mcp2515_check_spi();
            if (!ok) {
                spi_error = true;
                report_once("MCP2515 SPI fout gedetecteerd");
            } else {
                report_once("andere node niet meer zichtbaar");
            }

            peer_online = false;

            if (NODE_ROLE == 1) {
                node_offbus = true;
            }
        }
        #endif

        // Periodieke SPI-test en eventueel herinitialiseren MCP2515.
        // - Zonder heartbeat: exact zoals het was (cyclisch).
        // - Met heartbeat: alleen cyclisch proberen te herstellen ALS we al in spi_error zitten.
        #if !ENABLE_HEARTBEAT
        if (absolute_time_diff_us(get_absolute_time(), next_spi_check) <= 0) {
            int ok = mcp2515_check_spi();
            if (!ok) {
                if (!spi_error) {
                    printf("MCP2515 SPI fout gedetecteerd.\n");
                    spi_error = true;
                }
            } else {
                if (spi_error) {
                    printf("SPI aan het reinitialiseren...\n");
                    can_init(REQOP_NORMAL);
                    can_set_rx_handler(&on_can_rx);
                    can_set_tx_handler(&on_can_tx);
                    can_set_err_handler(&on_can_err);
                    can_set_gpio_irq_handler(&app_gpio_irq_handler);

                    ok = mcp2515_check_spi();
                    if(ok){
                        spi_error = false;
                        printf("MCP2515 SPI hersteld.\n");
                    }
                }
            }
            next_spi_check = make_timeout_time_ms(3000);
        }
        #else
        if (spi_error && absolute_time_diff_us(get_absolute_time(), next_spi_check) <= 0) {
            // Alleen herstelpogingen wanneer SPI al fout staat
            int ok = mcp2515_check_spi();
            if (ok) {
                printf("SPI aan het reinitialiseren...\n");
                can_init(REQOP_NORMAL);
                can_set_rx_handler(&on_can_rx);
                can_set_tx_handler(&on_can_tx);
                can_set_err_handler(&on_can_err);
                can_set_gpio_irq_handler(&app_gpio_irq_handler);

                ok = mcp2515_check_spi();
                if (ok) {
                    spi_error = false;
                    printf("MCP2515 SPI hersteld.\n");
                    // foutmelding opnieuw armeren voor volgende echte uitval
                    error_reported = false;
                }
            }
            next_spi_check = make_timeout_time_ms(3000);
        }
        #endif

        if (absolute_time_diff_us(get_absolute_time(), next_uart_print) <= 0) {
            if(last_tx_frame.data[0] != last_tx_frame_printed.data[0])
            {
                printf("Knop status=%u", (unsigned)claxon_state);
                printf("can_tx rc=%d, ID=0x%08X, LEN=%d, DATA0=0x%02X\n",
                last_rc,
                (unsigned int)last_tx_frame.id,
                last_tx_frame.datalen,
                last_tx_frame.data[0]);
                last_tx_frame_printed.data[0] = last_tx_frame.data[0];
                next_uart_print = make_timeout_time_ms(UART_TIMEOUT);
            }
        }
        sleep_ms(0);
    }
    
}


/* CAN CALLBACK FUNCTIONS */

void on_can_rx(CAN_DATA_FRAME_STRUCT *frame) 
{
    #if ENABLE_HEARTBEAT
    uint32_t expected_hb = HEARTBEAT_BASE_ID + (NODE_ROLE == 1 ? 2 : 1);
    if (frame->id == expected_hb) {
        last_rx_heartbeat = get_absolute_time();

        if (node_offbus && NODE_ROLE == 1) {
            printf("Node zichtbaar op de bus\n");
            can_init(REQOP_NORMAL);
            can_set_rx_handler(&on_can_rx);
            can_set_tx_handler(&on_can_tx);
            can_set_err_handler(&on_can_err);
            debug_config();
            can_set_gpio_irq_handler(&app_gpio_irq_handler);

            node_offbus = false;
            peer_online = true;
            error_reported = false; // foutmelding opnieuw armeren na herstel
        } else {
            if (!peer_online) {
                printf("Node zichtbaar op de bus\n");
            }
            peer_online = true;
            error_reported = false; // foutmelding opnieuw armeren na herstel
        }
        return;
    }
    #endif

   if (NODE_ROLE == 2) {
        if (frame->id != CLAXON_SENSOR_ID || frame->datalen == 0) {
            return;
        }

        // Receiver failsafe: markeer laatste geldige claxonframe
        last_rx_claxon = get_absolute_time();
        failsafe_tripped = false;

        claxon_output = frame->data[0] ? true : false;
        gpio_put(CLAXON_LED, claxon_output ? 1 : 0);

        CAN_DATA_FRAME_STRUCT tx_frame;
        tx_frame.id = CLAXON_STATUS_ID;
        tx_frame.datalen = 1;
        tx_frame.data[0] = frame->data[0];
        can_tx_extended_data_frame(&tx_frame);

    } else if (NODE_ROLE == 1) {
        if (frame->id != CLAXON_STATUS_ID || frame->datalen == 0) {
            return;
        }

        // Zodra status weer binnenkomt: re-arm foutmelding voor een volgende echte uitval
        if (error_reported) {
            error_reported = false;
        }

        gpio_put(CLAXON_LED, frame->data[0] ? 1 : 0);
    }
}

void on_can_tx(CAN_DATA_FRAME_STRUCT *frame) 
{
    (void)frame;
}

void on_can_err(CAN_ERR_FRAME_STRUCT *err) 
{
    (void)err;

    // Zet zender in offbus; melding maar één keer
    if (NODE_ROLE == 1) {
        node_offbus = true;
        report_once("Node niet meer zichtbaar op de bus");
    }
}


/* UTIL FUNCTIONS */

void debug_config() 
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

void debug_dataframe(CAN_DATA_FRAME_STRUCT *frame) 
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

void debug_errframe(CAN_ERR_FRAME_STRUCT *frame) 
{
    printf("*********** MCP 2515 ERROR *************\n");
    printf("REC    : 0x%.2X\n", frame->rREC);
    printf("TEC    : 0x%.2X\n", frame->rTEC);
    printf("EFLG   : 0x%.2X\n", frame->rEFLG);
    printf("CANINTF: 0x%.2X\n", frame->rCANINTF);
}

int mcp2515_check_spi(void)
{
    const uint8_t test = 0x05;

    uint8_t orig = mcp2515_read_register(MCP2515_TEST_REG);

    mcp2515_write_register(MCP2515_TEST_REG, test);
    uint8_t rd = mcp2515_read_register(MCP2515_TEST_REG);

    mcp2515_write_register(MCP2515_TEST_REG, orig);

    // 1 = OK (readback komt overeen met 0x05), 0 = fout
    return (rd == test);
}
