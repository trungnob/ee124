/* drive.c
 * Designed to run on Create Command Module
 *
 * The basic architecture of this program can be re-used to easily 
 * write a wide variety of Create control programs.  All sensor values
 * are polled in the background (using the serial rx interrupt) and 
 * stored in the sensors array as long as the function 
 * delayAndUpdateSensors() is called periodically.  Users can send commands
 * directly a byte at a time using byteTx() or they can use the 
 * provided functions, such as baud() and drive().
 */


// Includes
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include "oi.h"
#include "adc.h"
#include "cm9600.h"	//for debug


// Constants
#define RESET_SONG 0
#define START_SONG 1
#define BUMP_SONG  2
#define END_SONG   3

#define RIGHT_ANGLE 80
#define FULL_ANGLE 170
#define GRID_RES 200		//virtual grid resolution

enum{F,B,L,R};				//indicating the orientation of the iRobot in comparation to its "predefined" direction
							//F: iRobot facing the correct forward direction; B: iRobot facing the opposite to the correct direction
							//L: facing to the left of the correct directon; R: facing to the right of the correct direction
uint8_t e = 0 ;				//tolerance error of the ADC readings

// Global variables
volatile uint16_t timer_cnt = 0;
volatile uint8_t timer_on = 0;
volatile uint8_t sensors_flag = 0;
volatile uint8_t sensors_index = 0;
volatile uint8_t sensors_in[Sen6Size];
volatile uint8_t sensors[Sen6Size];
volatile int16_t distance = 0;
volatile int16_t angle = 0;

volatile uint8_t X = 0;		//Accelerometer X value
volatile uint8_t Y = 0;		//Accelerometer Y value
volatile uint8_t Y_min = 0;	//Y_min == Y value when iRobot facing uphill



// Functions
void byteTx(uint8_t value);
void delayMs(uint16_t time_ms);
void delayAndUpdateSensors(unsigned int time_ms);
void initialize(void);
void powerOnRobot(void);
void baud(uint8_t baud_code);
void drive(int16_t velocity, int16_t radius);
uint16_t randomAngle(void);
void defineSongs(void);

void  ReadADC_XY (void);		//Read the Accelerometer values
void Rotate_and_Find();		//Rotate until the iRobot is facing uphill when it is on a ramp


int main (void) 
{
  uint32_t counter;	//for debug
  uint8_t buf[32];	//for debug
  
  uint8_t leds_cnt = 99;
  uint8_t leds_state = 0;
  uint8_t leds_on = 1;

  int16_t turn_angle = 0;
  uint8_t turn_dir = 1;
  uint8_t turning = 0;
  uint8_t backing_up = 0;

  uint8_t on_track = 1;			//on_track==1 (on the right track); on_track==0 (still avoiding obstacle or cliffs)
  uint8_t obsF = 0;				//indicate if there're obstacles in front, back, left, and right of iRobot
  uint8_t obsR = 0;
  uint8_t obsL = 0;
  uint8_t obsB = 0;   
  uint8_t orient = F;
  uint8_t hill_climbing = 0;	//indicate if it is on a hill or flat surface
 	
  // Set up Create and module
  initialize();
  init_aux_UART(AUX_EPORT_CENTER, AUX_BAUD_9600);	//added to debug
  LEDBothOff;
  powerOnRobot();
  byteTx(CmdStart);
  baud(Baud28800);
  defineSongs();
  byteTx(CmdControl);
  byteTx(CmdFull);

  // Stop just as a precaution
  drive(0, RadStraight);

  // Play the reset song and wait while it plays
  byteTx(CmdPlay);
  byteTx(RESET_SONG);
  delayAndUpdateSensors(750);
  
  ReadADC_XY();	
  
  //Debugging tools : sending values of X, Y through bluetooth device 
  aux_rcv_disable();
  sprintf(buf,"X = %d Y = %d\n",X, Y  );
  aux_send_line(buf);       
  counter = 100000;
  while(counter != 0) // Delay
    counter--;
  	
	
	
  for(;;)
  {

    if(++leds_cnt >= 100)
    {
		leds_cnt = 0;
		leds_on = !leds_on;

		if(leds_on)
		{
			byteTx(CmdLeds);
			byteTx(LEDsBoth);
			byteTx(128);
			byteTx(255);
			LEDBothOff;
		}
		else
		{
			byteTx(CmdLeds);
			byteTx(0x00);
			byteTx(0);
			byteTx(0);
			LEDBothOn;
		}
    }

    delayAndUpdateSensors(10);

    if(UserButtonPressed)
    {
		// Play start song and wait
		byteTx(CmdPlay);
		byteTx(START_SONG);
		delayAndUpdateSensors(2813);
		
		ReadADC_XY();		//getting X, Y value

		// for debugging purpose: sending out the X, Y values to a bluetooth device
		aux_rcv_disable();
        sprintf(buf,"X = %d Y = %d\n",X, Y  );
        aux_send_line(buf);       
        counter = 100000;
        while(counter != 0) // Delay
        {
          counter--;
        }	

	
		//checking if the iRobot is on a hill 
		if ((abs(X-127) > 4) || (abs(Y-127) > 4)) {
			Rotate_and_Find();
			hill_climbing = 1;
			drive(0,RadStraight);
			delayMs(100);
		}	
		
        
		// Drive around until a button or unsafe condition is detected
		while(!(UserButtonPressed)
            && (!sensors[SenChAvailable])
			&& (!(sensors[SenBumpDrop] & WheelDropAll))
        )
		{
        // Keep turning until the specified angle is reached
        if(turning)
        {
          if(backing_up)
          {
            if ((-distance) > 10)
			{
				backing_up = 0;
				distance = 0;  	
			}  
            drive(-200, RadStraight);
          }
          else
          {
            if(turn_dir)		//left turn
            {
              if(angle > turn_angle) 
				turning = 0;
              drive(200, RadCCW);
            }
            else
            {
              if((-angle) > turn_angle)
                turning = 0;
              drive(200, RadCW);
            }
          }
        }
        else if((sensors[SenBumpDrop] & BumpEither)||(sensors[SenCliffL])
            || (sensors[SenCliffFL])
            || (sensors[SenCliffFR])
            || (sensors[SenCliffR]))  // Check for a bump or cliff
        {
			drive (0,RadStraight);
			turn_dir=1;	//left turn
			turn_angle=RIGHT_ANGLE;
			
			//Determine the iRobot orientation, and update its virtual environment specified by (obsF, obsR, obsL, obsF)
			if (orient == F)
			{
				if ((sensors[SenBumpDrop] & BumpEither)||sensors[SenCliffFL]||sensors[SenCliffFR])
					obsF=1;
				else if (sensors[SenCliffL])
					obsL = 1;
				else if (sensors[SenCliffR])
					obsR = 1;
				if (!obsL && !sensors[SenCliffFL])
					orient=L; 
				else if (!obsR && !sensors[SenCliffFR])
				{
					turn_dir=0;
					orient=R;
				}
				else if (!obsB)
				{
					turn_angle=FULL_ANGLE;
					orient=B;
				}	
				else 
					break;
			}
			else if (orient == L)
			{
				if ((sensors[SenBumpDrop] & BumpEither)||sensors[SenCliffFL]||sensors[SenCliffFR])
					obsL=1;
				else if (sensors[SenCliffL])
					obsB = 1;
				else if (sensors[SenCliffR])
					obsF = 1;
				if (!obsF && !sensors[SenCliffFR])
				{
					turn_dir=0;
					orient=F;
				}
				else if (!obsR)
				{
					turn_angle=FULL_ANGLE;
					orient=R;
				}
				else if (!obsB && !sensors[SenCliffFL])
					orient=B;
				else 
					break;
			}
			else if (orient == R)
			{
				if ((sensors[SenBumpDrop] & BumpEither)||sensors[SenCliffFL]||sensors[SenCliffFR])
					obsR=1;
				else if (sensors[SenCliffL])
					obsF = 1;
				else if (sensors[SenCliffR])
					obsB = 1;
				if (!obsF && !sensors[SenCliffFL])
					orient=F;
				else if (!obsL)
				{
					turn_angle=FULL_ANGLE;
					orient=L;
				}
				else if (!obsB && !sensors[SenCliffFR])
				{
					turn_dir=0;
					orient=B;
				}
				else
					break;
			}
			else 
			{
				if ((sensors[SenBumpDrop] & BumpEither)||sensors[SenCliffFL]||sensors[SenCliffFR])
					obsB=1;
				else if (sensors[SenCliffL])
					obsR = 1;
				else if (sensors[SenCliffR])
					obsL = 1;
				if (!obsF) 
				{
					turn_angle=FULL_ANGLE;
					orient=F;
				}
				else if (!obsR && !sensors[SenCliffL])
					orient=R;
				else if (!obsL && !sensors[SenCliffR])
				{
					turn_dir=0;
					orient=L;
				}
				else 
					break;
			}
			backing_up = 1;
			turning = 1;
			distance = 0;
			angle = 0;
			on_track = 0;		

          // Play the bump song
          byteTx(CmdPlay);
          byteTx(BUMP_SONG);
        }
		
		// find the original track of the iRobot
		else if (!on_track & !turning)  
		{
			//after moving 1 gid forward in the direction specified in sensbump else statement	
			if (distance > GRID_RES)
			{
				angle = 0;
				distance = 0;
				turn_angle = RIGHT_ANGLE;
				
				//from the current orientation of the iRobot, it finds a way to get back to 
				//to the right track
			    if (orient==L)
				{
					obsF=0;
					obsB=0;
					turn_dir=0;
					orient=F;
					on_track=1;
					turning = 1;
				}
				else if (orient==R)
				{
					obsF=0;
					obsB=0;
					turn_dir=1;
					orient=F;
					on_track=1;
					turning=1;
				}
				else if (orient==B)
				{
					if (obsL)	//if there is an obstacle on the left previously (likely to be a wall)
					{
						turn_dir=1;
						orient=R;
					}
					else 
					{
						turn_dir=0;
						orient=L;
					}
					obsL=0;
					obsR=0;
					turning=1;
				}
			}
			else 
				drive(200, RadStraight);
		}
		//Uyen: End
        else 
        {
		
         
		  if (hill_climbing ) 
		  {
			delayMs(100);
			ReadADC_XY();
			
			// Check if iRobot reaches the top of the hill
			if ((abs(X-127) < 2) && (abs(Y-127) < 2)) 
			{
				drive(100, RadStraight);
				delayMs(100);
				break;
			}	
		  }
		  
		  // Otherwise, drive straight
          drive(200, RadStraight);
		  obsR=0;
		  obsL=0;
		}


        // Flash the leds in sequence
        if(++leds_cnt >= 10)
        {
          leds_cnt = 0;
          if(turning)
          {
            // Flash backward while turning
            if(leds_state == 0)
              leds_state = 4;
            else
              leds_state--;
          }
          else
          {
            if(leds_state == 4)
              leds_state = 0;
            else
              leds_state++;
          }

          if(leds_state == 0)
          {
            // robot Power LED Amber
            byteTx(CmdLeds);
            byteTx(0x00);
            byteTx(128);
            byteTx(255);
            LEDBothOff;
          }
          else if(leds_state == 1)
          {
            // Play LED on
            byteTx(CmdLeds);
            byteTx(LEDPlay);
            byteTx(0);
            byteTx(0);
            LEDBothOff;
          }
          else if(leds_state == 2)
          {
            // Advance LED on
            byteTx(CmdLeds);
            byteTx(LEDAdvance);
            byteTx(0);
            byteTx(0);
            LEDBothOff;
          }
          else if(leds_state == 3)
          {
            // Robot LEDs off, CM left LED on
            byteTx(CmdLeds);
            byteTx(0x00);
            byteTx(0);
            byteTx(0);
            LED2On;
            LED1Off;
          }
          else if(leds_state == 4)
          {
            // Robot LEDs off, CM right LED on
            byteTx(CmdLeds);
            byteTx(0x00);
            byteTx(0);
            byteTx(0);
            LED1On;
            LED2Off;
          }
        }

       
		// wait a little more than one robot tick for sensors to update
        delayAndUpdateSensors(20);
      }

      // Stop driving
      drive(0, RadStraight);

      // Play end song and wait
      delayAndUpdateSensors(500);
      byteTx(CmdPlay);
      byteTx(END_SONG);
      delayAndUpdateSensors(2438);

    }
  }
}


void Rotate_and_Find(void)
{

	Y_min = 118;
    e = 0;
	int8_t direction = RadCCW;
	
	// Using X value to determine the direction to rotate
	ReadADC_XY();
	if (X >= 128 ) direction = RadCCW;
	else  direction  = RadCW;
	
	// Rotating to face the hill-top within error tolerance e+2
	while (abs(Y - Y_min) > e+2)
	{
		drive(150,direction);	
		ReadADC_XY();	
	}
	
	drive(0,RadStraight);
	delayMs(1000);
	ReadADC_XY();
	
	if (X >= 128 ) direction = RadCCW;
	else  direction  = RadCW;
	
	// Rotating to face hill-top within error tolerance of e
	// Rotate slower with stopping in between to stablize the iRobot to get rid of error of X, Y due to vibration
	while ((abs(Y - Y_min) > e) || ((abs(X-126)) > 2))
	{
	    // Rotate 100ms @ velocity 70
		drive(70,direction);	
		delayMs(100);	
		// Stop to stablize the iRobot before updating values X,Y
		drive (0, RadStraight);
		delayMs(300);
		ReadADC_XY();
	}
	
}


void  ReadADC_XY (void)
{
  ADMUX &= ~(0x07) ; 	// deselect channel
  ADMUX |= INCH_6; 		// select channel C6
  ADCSRA |= my_ADSC;	// start converting
  while (ADCSRA  & my_ADSC);  //busy converting ...
  X = ADCH;

  ADMUX |= INCH_7; 		// select channel C7
  ADCSRA |= my_ADSC;	// start converting
  while (ADCSRA & my_ADSC);
  Y = ADCH;
}


// Initialize the Mind Control's ATmega168 microcontroller
void initialize(void)
{
  cli();

  // Set I/O pins
  DDRB = 0x10;
  PORTB = 0xCF;
  DDRC = 0x00;
  PORTC = 0xFF;
  DDRD = 0xE6;
  PORTD = 0x7D;

  // Set up timer 1 to generate an interrupt every 1 ms
  TCCR1A = 0x00;
  TCCR1B = (_BV(WGM12) | _BV(CS12));
  OCR1A = 71;
  TIMSK1 = _BV(OCIE1A);

  // Set up the serial port with rx interrupt
  UBRR0 = 19;
  UCSR0B = (_BV(RXCIE0) | _BV(TXEN0) | _BV(RXEN0));
  UCSR0C = (_BV(UCSZ00) | _BV(UCSZ01));

  //Setting for ADC readings
  PRR &= ~_BV(PRADC); 				// Turn off  power save
  ADCSRA |= my_ADEN | ADPS_7;   	// Enabled, prescaler = 128
  ADMUX |= REFS_1 | my_ADLAR;   	// set voltage reference to AVcc and Left Justified
  
  // Turn on interrupts
  sei();
}




// Serial receive interrupt to store sensor values
SIGNAL(SIG_USART_RECV)
{
  cli();
  uint8_t temp;
  temp = UDR0;

  if(sensors_flag)
  {
    sensors_in[sensors_index++] = temp;
    if(sensors_index >= Sen6Size)
      sensors_flag = 0;
  }
  sei();
}




// Timer 1 interrupt to time delays in ms
SIGNAL(SIG_OUTPUT_COMPARE1A)
{
 cli();
  if(timer_cnt)
    timer_cnt--;
  else
    timer_on = 0;
 sei();	
}




// Transmit a byte over the serial port
void byteTx(uint8_t value)
{
  while(!(UCSR0A & _BV(UDRE0))) ;
  UDR0 = value;
}




// Delay for the specified time in ms without updating sensor values
void delayMs(uint16_t time_ms)
{
  timer_on = 1;
  timer_cnt = time_ms;
  while(timer_on) ;
}




// Delay for the specified time in ms and update sensor values
void delayAndUpdateSensors(uint16_t time_ms)
{
  uint8_t temp;

  timer_on = 1;
  timer_cnt = time_ms;
  while(timer_on)
  {
    if(!sensors_flag)
    {
      for(temp = 0; temp < Sen6Size; temp++)
        sensors[temp] = sensors_in[temp];

      // Update running totals of distance and angle
      distance += (int)((sensors[SenDist1] << 8) | sensors[SenDist0]);
      angle += (int)((sensors[SenAng1] << 8) | sensors[SenAng0]);

      byteTx(CmdSensors);
      byteTx(6);
      sensors_index = 0;
      sensors_flag = 1;
    }
  }
}





void powerOnRobot(void)
{
  // If Create's power is off, turn it on
  if(!RobotIsOn)
  {
      while(!RobotIsOn)
      {
          RobotPwrToggleLow;
          delayMs(500);  // Delay in this state
          RobotPwrToggleHigh;  // Low to high transition to toggle power
          delayMs(100);  // Delay in this state
          RobotPwrToggleLow;
      }
      delayMs(3500);  // Delay for startup
  }
}




// Switch the baud rate on both Create and module
void baud(uint8_t baud_code)
{
  if(baud_code <= 11)
  {
    byteTx(CmdBaud);
    UCSR0A |= _BV(TXC0);
    byteTx(baud_code);
    // Wait until transmit is complete
    while(!(UCSR0A & _BV(TXC0))) ;

    cli();

    // Switch the baud rate register
    if(baud_code == Baud115200)
      UBRR0 = Ubrr115200;
    else if(baud_code == Baud57600)
      UBRR0 = Ubrr57600;
    else if(baud_code == Baud38400)
      UBRR0 = Ubrr38400;
    else if(baud_code == Baud28800)
      UBRR0 = Ubrr28800;
    else if(baud_code == Baud19200)
      UBRR0 = Ubrr19200;
    else if(baud_code == Baud14400)
      UBRR0 = Ubrr14400;
    else if(baud_code == Baud9600)
      UBRR0 = Ubrr9600;
    else if(baud_code == Baud4800)
      UBRR0 = Ubrr4800;
    else if(baud_code == Baud2400)
      UBRR0 = Ubrr2400;
    else if(baud_code == Baud1200)
      UBRR0 = Ubrr1200;
    else if(baud_code == Baud600)
      UBRR0 = Ubrr600;
    else if(baud_code == Baud300)
      UBRR0 = Ubrr300;

    sei();

    delayMs(100);
  }
}




// Send Create drive commands in terms of velocity and radius
void drive(int16_t velocity, int16_t radius)
{
  byteTx(CmdDrive);
  byteTx((uint8_t)((velocity >> 8) & 0x00FF));
  byteTx((uint8_t)(velocity & 0x00FF));
  byteTx((uint8_t)((radius >> 8) & 0x00FF));
  byteTx((uint8_t)(radius & 0x00FF));
}




// Return an angle value in the range 53 to 180 (degrees)
uint16_t randomAngle(void)
{
    return (53 + ((uint16_t)(random() & 0xFF) >> 1));
}



// Define songs to be played later
void defineSongs(void)
{
  // Reset song
  byteTx(CmdSong);
  byteTx(RESET_SONG);
  byteTx(4);
  byteTx(60);
  byteTx(6);
  byteTx(72);
  byteTx(6);
  byteTx(84);
  byteTx(6);
  byteTx(96);
  byteTx(6);

  // Start song
  byteTx(CmdSong);
  byteTx(START_SONG);
  byteTx(6);
  byteTx(69);
  byteTx(18);
  byteTx(72);
  byteTx(12);
  byteTx(74);
  byteTx(12);
  byteTx(72);
  byteTx(12);
  byteTx(69);
  byteTx(12);
  byteTx(77);
  byteTx(24);

  // Bump song
  byteTx(CmdSong);
  byteTx(BUMP_SONG);
  byteTx(2);
  byteTx(74);
  byteTx(12);
  byteTx(59);
  byteTx(24);

  // End song
  byteTx(CmdSong);
  byteTx(END_SONG);
  byteTx(5);
  byteTx(77);
  byteTx(18);
  byteTx(74);
  byteTx(12);
  byteTx(72);
  byteTx(12);
  byteTx(69);
  byteTx(12);
  byteTx(65);
  byteTx(24);
}





