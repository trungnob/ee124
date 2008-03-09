/*-------------------------------------------------------------------
 * cm9600.c / cm9600.h
 * Designed to run on Create Command Module wih ATMega168 MCU
 *
 * John A. Qualtrough
 *-------------------------------------------------------------------
 * Transmit data via simulated UART port using Port B discrete
 * Data sent in 10-bit format 1 Start,8 Data,No Parity,1 Stop
 * Process uses TIMER 2 and Port B bit 1, 2, or 3
 *-------------------------------------------------------------------
 *
 *
 * ====================== REVISION HISTORY ==========================
 *
 * 1.00	06-08-2007	John Qualtrough Created
 *
 * Added send_aux_line to transmit char array in background.
 * Added predefined PORT and BAUD settings to cm9600.h
 *-------------------------------------------------------------------
 *
 * 2.00 06-10-2007  John Qualtrough Update
 *
 * Interrupt turned on/off as needed for transmission
 * Ring values checked/reset in done-bit state.
 *-------------------------------------------------------------------
 *
 * 2.10 06-11-2007  John Qualtrough Update
 *
 * Clear Timer 2 count register after enabling interrupts.
 *-------------------------------------------------------------------
 *
 * 2.11 06-11-2007  John Qualtrough Update
 *
 * Added local start_ctc2() and stop_ctc2() functions.
 *-------------------------------------------------------------------
 *
 * 2.12 06-12-2007  John Qualtrough Update
 *
 * Expanded line buffer from 32 to 48 bytes
 * Removed redundant state from transmit state machine
 * Simplified aux_UART_busy() logic
 *-------------------------------------------------------------------
 *
 * 3.00 06-16-2007  John Qualtrough Update
 *
 * Added full-duplex (receive) capability on ePort pin 2 (PortC)
 *-------------------------------------------------------------------
 *
 * 3.10 06-16-2007  John Qualtrough Update
 *
 * Added receive ring buffer.
 *-------------------------------------------------------------------
 *
 * 3.11 06-18-2007  John Qualtrough Update
 *
 * Transmitter bug fix
 *-------------------------------------------------------------------
 *
 * 3.20 06-20-2007  John Qualtrough Update
 *
 * Changed receiver sample time to startbit interrupt+1
 * Added aux_rcv_enable() and aux_rcv_disable() functions.
 * Eliminated call to strlen()
 *-------------------------------------------------------------------
 */

//-----------------
//    Includes
//-----------------
#include <avr/interrupt.h>
#include <string.h>
#include "cm9600.h"

//-----------------
//    Constants
//-----------------
// Simulated UART States
enum{ SEND_DONE, SEND_START, SEND_DATA, SEND_STOP, STATE_INIT };
enum{ RCV_IDLE,  RCV_RDY };

#define MAX_AUX_BUF 48

//-----------------
//     Common
//-----------------
static volatile uint8_t gAuxPort;
//-----------------
//   Transmitter
//-----------------
static volatile uint8_t gUartXmt;
static volatile uint8_t gXmtState  = SEND_DONE;
static volatile uint8_t gBitnum = 0x01;
static volatile uint8_t gXringNdx=0;
static volatile char   *gXring;
//-----------------
//    Receiver
//-----------------
static volatile char    gRring[MAX_AUX_BUF];
static volatile uint8_t gRhead=0;
static volatile uint8_t gRtail=0;
static volatile uint8_t gRcvrEnabled=1;

//-------------------------------------------------
//       Init timer 2 and port B output pin
//     for user selected baud aux UART output
//  NOTE: See port and baud settings in cm9600.h
//-------------------------------------------------
void init_aux_UART(uint8_t port, uint8_t brate)
{
  cli();

  gAuxPort = port;

//-----------------------
// Init State Variables
//-----------------------
  gXringNdx = 0;
  gXring[0] = '\0';
  gXmtState = SEND_DONE;  // ISR state maching init
  gBitnum = 0x01;         // Prepare for MSB first order
  gRhead = 0;
  gRtail = 0;
  aux_rcv_enable();       // Start with receiver enabled.
  
//----------------------------------
// Port transmit line configuration
//----------------------------------   
  DDRB  |= _BV(gAuxPort);  // setup desired PORTB bit as output
  PORTB |= _BV(gAuxPort);  // Set logic level to high "Mark"

//----------------------------------
// Port receive line configuration
//----------------------------------   
  DIDR0 &= (0x3F^_BV(gAuxPort)); // Enable PORTC input buffer
  DDRC  &= ~_BV(gAuxPort);       // Set PORTC bit for input.
  PORTC |= _BV(gAuxPort);        // Enable pull-up

//----------------------------------------------------
//   Set up timer 2 for 3X baud rate interrupts
//      CTC mode with divide by 32 prescaler
//   Atmel Doc Rev. 2545K–AVR–04/07 pages 153-158
//----------------------------------------------------
  TCCR2A = _BV(WGM21);    // Configure CTC mode
  OCR2A  = brate;         // 18432000/(32*BAUD*3) = brate+1
  
//-----------------
// Enable Timer 2
//-----------------  
  TCNT2  =  brate-1;                // Start shortly after enabled.
  TCCR2B = _BV(CS21) | _BV(CS20);   // Start clock divide by 32
  TIMSK2 = _BV(OCIE2A);             // Enable terminal count irq.

  sei();
}

//-------------------------------------------------
//        Send single byte via emulated UART
//-------------------------------------------------
void ByteXmtAux(uint8_t value)
{
  sei(); // Be sure interrupts are enabled
  while(aux_UART_busy()) // Wait while xmit in progress.
  {
    asm("nop"); // I don't like empty wait loops.
  }
  
  cli();
  gUartXmt  = value;        // Set data byte to be sent.
  gXmtState = SEND_START;   // Activate transmitter state machine.
  sei();
}

//---------------------------------------------------------
//  Send ASCII NUL terminated (ASCIZ) string out aux UART
//  String can be no longer than MAX_AUX_BUF characters
//---------------------------------------------------------
void aux_send_line(char *ibuf)
{
  uint8_t len=0;
  
  sei();
  while(aux_UART_busy())  // Wait if string is in queue.
  {
    asm("nop");
  }
  
  gXring = &ibuf[1]; // 1st char is sent below.
  while(gXring[len])                       // Equivalent to strlen()
  {
    len++;
  }
  cli();
  gXringNdx = len;                          // Pass length to ISR
  gUartXmt = ibuf[0];                       // Queue up first character.
  gXmtState = SEND_START;                   // Activate transmit state machine.
  sei();
}

//----------------------------------------------------
//   Return busy status of aux UART transmitter
//----------------------------------------------------
uint8_t aux_UART_busy(void)
{
  return(gXmtState != SEND_DONE);
}

//----------------------------------------------------
//    Returns number of chars waiting to be read
//----------------------------------------------------
uint8_t aux_char_rdy(void)
{
  if (gRhead >= gRtail)
     return (gRhead-gRtail);
  else
     return (MAX_AUX_BUF - (gRtail-gRhead));  
}

//----------------------------------------------------
//          Return next char in rcv FIFO
//----------------------------------------------------
char aux_read_char(void)
{
  char temp;
  
  temp = gRring[gRtail++];
  if (gRtail >= MAX_AUX_BUF) gRtail=0;
  return temp;
}
    
//----------------------------------------------------
//          Enable Receiver
//----------------------------------------------------
void aux_rcv_enable(void)
{
  gRcvrEnabled = 1;
}
    
//----------------------------------------------------
//          Disable Receiver
//----------------------------------------------------
void aux_rcv_disable(void)
{
  gRcvrEnabled = 0;
}
    
//-------------------------------------------------
//    Timer 2 ISR runs at 3X the Baud Rate
// Transmits character in 10-bit UART protocol
//      1 Start, 8 data, No parity, 1 stop.
//-------------------------------------------------
ISR(TIMER2_COMPA_vect)
{  
  static volatile uint8_t ringNdx = 0;
  static volatile uint8_t irqcnt  = 0;
  static volatile int8_t  rcving  = 0;
  static volatile uint8_t bittime = 0;
  static volatile uint8_t rcvbyte = 0;


  if (++irqcnt >= 3) irqcnt=0; // Faster than modulo 3
  
  if (gRcvrEnabled) // Has user disabled receiver?
  {
    if (rcving)
    {
      if (irqcnt == bittime)
      {
        rcvbyte >>= 1;
        if (PINC & _BV(gAuxPort)) rcvbyte |= 0x80;
        rcving++;
      
        if (rcving >= 10) // Discard the start-bit
        {
          rcving = 0;
          gRring[gRhead++] = rcvbyte;
          if (gRhead >= MAX_AUX_BUF) gRhead = 0;
        }
      }
    }
    else if ((PINC & _BV(gAuxPort)) == 0) // Start bit?
    {
      bittime = irqcnt+1;  // When to sample subsequent bits
      if (++bittime >= 3) bittime=0; // Faster than modulo 3
      rcving  = 1;         // Boolean and bit count
      rcvbyte = 0;         // Reset received byte
    }
  }
  else // Keep receiver state var reset while disabled.
  {
    rcving  = 0;
  }
  
  if (irqcnt==0)  // Only process transmit at Baud rate.
  {
    switch (gXmtState)
    {
      case SEND_START:    // Send single low start bit (Space)
         PORTB &= ~_BV(gAuxPort);
         gXmtState = SEND_DATA;
         gBitnum = 0x01;  // Data will be LSB first.
         break;

      case SEND_DATA:
         if (gUartXmt & gBitnum) // Send a 1 or a 0?
           PORTB |= _BV(gAuxPort);
         else
           PORTB &= ~_BV(gAuxPort);

         if (gBitnum & 0x80) // If the MSB was sent, next is Stop bit.
           gXmtState = SEND_STOP;
         else
           gBitnum <<= 1; // Otherwise, send next bit at next bit time.
         break;

      case SEND_STOP: // Stop bit : "Marking" state for 1 bit time.
         PORTB |= _BV(gAuxPort);
         gBitnum = 0x01;
         if (gXringNdx)  // See if a char array has been queued.
         {
           gUartXmt  = gXring[ringNdx++]; // Set data byte to be sent.
           gXmtState = SEND_START;        // Activate transmitter state machine.
           gXringNdx--;                   // Decrement char count.
         }
         else // All done. Clean up and disable timer2 interrupts.
         {
           ringNdx = 0;
           gXmtState = SEND_DONE;
         }
         break;

      default: // Unknown state and STATE_INIT. Set vars to init values.
        break;
    }//end switch
  }// end if irqcnt==0

}

//========================================= 
//              LOCAL FUNCTIONS 
//=========================================
// NONE.

/*__________________________________________________________________________________________________________________

                              >>><<<  GENERAL UART PROTOCOL TECHNICAL NOTES  >>><<<<
____________________________________________________________________________________________________________________

In Async UART protocol a value of 1 is called a Mark and a value of 0 is called a Space.
When a serial line is idle, the line is said to be “Marking”, or transmitting continuous 1 values.
The Start bit always has a value of 0 (a Space). A start bit is sent before each character.
After the Start Bit, the individual bits of the byte of data are sent, LSB first.
After 8 data bits are transmitted, the Stop Bit is sent. Stop bit always has a value of 1 (a Mark).
This means that there will always be a Mark (1) to Space (0) transition on the line at the start of every byte.
This guarantees that sender and receiver can resynchronize their clocks regardless of the data being transmitted.
____________________________________________________________________________________________________________________*/
#ifdef TEST_AUX_RCV

#include <stdio.h>
#include "..\creator\oi.h"
int main(void)
{
   volatile uint32_t counter;
   volatile uint8_t outc = 64;
   volatile uint8_t inch = '0';
   volatile uint8_t buf[32];
   
   init_aux_UART(AUX_EPORT_CENTER,AUX_BAUD_19200);
   while(1)
   {
       aux_rcv_enable();
       counter = 100000;
       outc++;
       ByteXmtAux(outc);
       
       while(counter != 0) // Delay
       {
          counter--;
       }
       while(aux_char_rdy())
       {
         inch = aux_read_char();
       }
       aux_rcv_disable();
       sprintf(buf,"?fSENT: %c  REC: %c",outc,inch);
       aux_send_line(buf);       
       counter = 100000;
       while(counter != 0) // Delay
       {
          counter--;
       }
       if (outc>'Z') outc=64;
   }
}
#endif
