#ifndef MCP2515_INC
#define MCP2515_INC

typedef struct {
	uint32_t id;
	uint8_t datalen;
	uint8_t data[8];
} CAN_DATA_FRAME_STRUCT;

typedef struct {
	uint16_t id;
} CAN_REMOTE_FRAME_STRUCT;

typedef struct {
	uint8_t rTEC;
        uint8_t rREC;
        uint8_t rEFLG;
        uint8_t rCANINTF;
} CAN_ERR_FRAME_STRUCT;

// Init CAN bus 
void can_init(uint8_t mode);

// Receive dataframe

// Transmit dataframe
uint8_t can_tx_extended_data_frame(CAN_DATA_FRAME_STRUCT *frame);

// CAN callbacks
void can_set_rx_handler( void (*f)(CAN_DATA_FRAME_STRUCT *) );
void can_set_tx_handler( void (*f)(CAN_DATA_FRAME_STRUCT *) );
void can_set_err_handler( void (*f)(CAN_ERR_FRAME_STRUCT *) );

// MCP2515 helper functions
void can_poll(void);

void mcp2515_reset(void);
void mcp2515_init(uint8_t mode);
uint8_t mcp2515_read_register(uint8_t addr);
void mcp2515_read_rx_buffer(uint8_t buffer_id, uint8_t *data, uint8_t len);
void mcp2515_write_register(uint8_t addr, uint8_t data);
void mcp2515_load_tx_buffer(uint8_t buffer_id, uint8_t *data, uint8_t len); 
void mcp2515_RTS(uint8_t tx_buf_id);
uint8_t mcp2515_read_status(void);
uint8_t mcp2515_rx_status(void);
void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t data);
void mcp2515_callback(uint gpio, uint32_t events);
	
#endif