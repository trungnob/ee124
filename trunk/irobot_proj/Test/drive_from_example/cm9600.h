//----------------------------------------------------------------
// PROTOTYPES, PORT, AND BAUD DEFINITIONS FOR CM9600 "SOFT" UART
//----------------------------------------------------------------

//------------------------------
//     FUNCTION PROTOTYPES
//------------------------------
//COMMON
void init_aux_UART(uint8_t port, uint8_t brate);  // Initialize "soft" UART

//TRANSMIT
void ByteXmtAux(uint8_t);       // Send a single byte
void aux_send_line(char *ibuf); // Send an ASCIZ string
uint8_t aux_UART_busy(void);    // Check if transmitter is busy

//RECEIVE FUNCTIONS
uint8_t aux_char_rdy(void); // Return number of chars waiting to be read
char aux_read_char(void);   // Read 1 char from FIFO
void aux_rcv_enable(void);  // Enable receiver
void aux_rcv_disable(void); // Disable receiver


//------------------------------
//   DEFINE ePORT SETTINGS
//------------------------------
#define AUX_EPORT_CENTER 1
#define AUX_EPORT_RIGHT  2
#define AUX_EPORT_LEFT   3

//------------------------------
//  DEFINE BAUD RATE SETTINGS
//  W/RCVR INTS RUN 3 x BAUD
//------------------------------
#define AUX_BAUD_2400    79
#define AUX_BAUD_4800    39
#define AUX_BAUD_9600    19
#define AUX_BAUD_19200    9

