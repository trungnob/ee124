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


// Constants
#define RESET_SONG 0
#define START_SONG 1
#define BUMP_SONG  2
#define END_SONG   3
#define Grid_Res 100

<<<<<<< .mine
=======
#define RIGHT_ANGLE 81
#define FULL_ANGLE 180
#define GRID_RES 200		//virtual grid resolution
enum{F,B,L,R};

>>>>>>> .r14
// Global variables
volatile uint16_t timer_cnt = 0;
volatile uint8_t timer_on = 0;
volatile uint8_t sensors_flag = 0;
volatile uint8_t sensors_index = 0;
volatile uint8_t sensors_in[Sen6Size];
volatile uint8_t sensors[Sen6Size];
volatile int16_t distance = 0;
volatile int16_t abs_distance=0;
volatile int16_t angle = 0;
enum{F,B,L,R};


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

int main (void) 
{
  uint8_t leds_cnt = 99;
  uint8_t leds_state = 0;
  uint8_t leds_on = 1;

  int16_t turn_angle = 0;
  uint8_t turn_dir = 1;
  uint8_t turning = 0;
  uint8_t backing_up = 0;

<<<<<<< .mine
  uint8_t on_track = 1;		
  int16_t distance_from_X = 1; 
  int16_t distance_from_Y=0;
  uint8_t obF=0;
  uint8_t obB=0;
  uint8_t obL=0;
  uint8_t obR=0;
  uint8_t orient=F; 
  
	
=======
  uint8_t on_track = 1;		//Uyen
  int8_t distance_from_y = 0; //Uyen: in term of GRID_RES
  uint8_t obsF = 0;
  uint8_t obsR = 0;
  uint8_t obsL = 0;
  uint8_t obsB = 0;   
  uint8_t orient = F;
 	
>>>>>>> .r14
  // Set up Create and module
  initialize();
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

		// Drive around until a button or unsafe condition is detected
		while(!(UserButtonPressed)
            && (!sensors[SenCliffL])
            && (!sensors[SenCliffFL])
            && (!sensors[SenCliffFR])
            && (!sensors[SenCliffR])
            && (!sensors[SenChAvailable])
        )
		{
/*if (obF==1 && orient ==F)
			{
				turning=1;
				turn_dir=1;
				turn_angle=90;
				backing_up=0;
				orient=L;
			}
else if (obL==1  && orient ==L)
			{
				turning =1;
				turn_dir =0;
				turn_angle=180;
				backing_up=0;
				orient=R;
			}
else if (obR==1 && orient == R)
			{
				turning =1;
				turn_dir=0;
				turn_angle=90;
				backing_up=0;
				orient = B;
			}
else if (obB==1 && orient == B)
			{
			 drive(0, RadStraight);
			}

     // Keep turning until the specified angle is reached
else   */ if(turning)
        {
<<<<<<< .mine
			if(backing_up)
=======
          if(backing_up)
          {
            if ((-distance) > 5)
>>>>>>> .r14
			{
<<<<<<< .mine
            if((-distance) > Grid_Res)
			{
              backing_up = 0;
			  distance = 0;  	//Uyen
=======
				backing_up = 0;
				distance = 0;  	//Uyen
>>>>>>> .r14
			}  
            drive(-200, RadStraight);
			if (orient==F) 
			distance_from_X--;
			else if (orient==B)
			distance_from_X++;
			else if (orient==L)
			distance_from_Y++;
			else if (orient==R)
			distance_from_Y--;
			}
			
          }
			else if ((obL==0)||(obR==0)||(obF==0)||(obB==0))
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
<<<<<<< .mine
=======
        }
        else if(sensors[SenBumpDrop] & BumpEither)  // Check for a bump
        {
          // Set the turn parameters and reset the angle
			/*if(sensors[SenBumpDrop] & BumpLeft)
			{
				turn_dir = 0;
				if ()
				
			}	
			else
				turn_dir = 1;
			*/
			turn_dir=1;	//left turn
			turn_angle=RIGHT_ANGLE;
			if (orient == F)
			{
				obsF=1;
				if (!obsL)
					orient=L; 
				else if (!obsR)
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
				obsL=1;
				if (!obsF)
				{
					turn_dir=0;
					orient=F;
				}
				else if (!obsR)
				{
					turn_angle=FULL_ANGLE;
					orient=R;
				}
				else if (!obsB)
					orient=B;
				else 
					break;
			}
			else if (orient == R)
			{
				obsR=1;
				if (!obsF)
					orient=F;
				else if (!obsL)
				{
					turn_angle=FULL_ANGLE;
					orient=L;
				}
				else if (!obsB)
				{
					turn_dir=0;
					orient=B;
				}
				else
					break;
			}
			else 
			{
				obsB=1;
				if (!obsF) 
				{
					turn_angle=FULL_ANGLE;
					orient=F;
				}
				else if (!obsR)
					orient=R;
				else if (!obsL)
				{
					turn_dir=0;
					orient=L;
				}
				else 
					break;
			}
			backing_up = 1;
			turning = 1;
			//if (distance > GRID_RES)
			//	distance = distance - GRID_RES;
			//distance = 0;
			angle = 0;
			//turn_angle = 90;//randomAngle();
			on_track = 0;		//Uyen

          // Play the bump song
          byteTx(CmdPlay);
          byteTx(BUMP_SONG);
        }
		//Uyen: drive forward 20mm and start turning back toward the "straight" direction
		else if (!on_track & !turning)  
		{
			if (distance > GRID_RES)//after moving 1 gid forward in the direction specified in sensbump else statement
			{
				angle = 0;
				distance = 0;
				turn_angle = RIGHT_ANGLE;
				/*if (orient==F)
				{
					obsR=0;
					obsL=0;
					if (distance_from_y>0) //on the right of y-axis
					{	
						turn_dir=1;
						orient=L;
						turning=1;
					}
					else if (distance_from_y<0)
					{
						turn_dir=0;
						orient=R;
						turning=1;
					}
				}*/
			    if (orient==L)
				{
					obsF=0;
					obsB=0;
					//distance_from_y--;
					//if (distance_from_y<=0)   //at or moving away from center
					//{	
						turn_dir=0;
						orient=F;
						//if (distance_from_y==0)
							on_track=1;
						turning = 1;
					//}
				}
				else if (orient==R)
				{
					obsF=0;
					obsB=0;
					//distance_from_y++;
					//if (distance_from_y>=0)
					//{
						turn_dir=1;
						orient=F;
						//if (distance_from_y==0)
							on_track=1;
						turning=1;
					//}
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
				drive(300, RadStraight);
			/*if (distance > 20)
		    {
				turn_dir = !turn_dir;	
				on_track = 1;
			    distance = 0;
				angle = 0;
				turning = 1;
				turn_angle = 90;
			}  
>>>>>>> .r14
			else 
			{
				drive(0,RadStraight);
			}
			angle=0;
			*/
		}
<<<<<<< .mine
		
		//Uyen: drive forward 20mm and start turning back toward the "straight" direction
		
        else if((sensors[SenBumpDrop] & BumpEither))  // Check for a bump or obstacle
        {
			on_track=0;
          distance=0;
          backing_up = 1;
		  angle=0;
          turning = 1;
		    if (orient==F ) 
		  {
			obF=1;
			turn_angle=90;
			turn_dir=1;
			orient=L;
		  }
		  else if (orient==L )
		  {
			obL=1;
		   turn_angle=180;
		   turn_dir=1;
		   orient=R;
		  }
		  else if (orient ==R )
		  {
			obR=1;
			turn_angle=90;
			turn_dir=0;
			orient=B;
		  }
		  else 
		  {
			
			obB=1;
		  }

        		
          // Play the bump song
          byteTx(CmdPlay);
          byteTx(BUMP_SONG);
        }
=======
>>>>>>> .r14
       /* else if ((obF==0 && orient==F)|| (obL==0 &&  orient ==L) || (obR==0 && orient==R) ||(obB==0 && orient==B))
        {
          // Otherwise, drive straight
          drive(300, RadStraight);
<<<<<<< .mine
		 if (distance>=Grid_Res) //move to next square 
		 {
		  distance=0;
		  angle =0;
		if (orient==F)
		{
			obB=1;
			obL=0;
			obR=0;
			obF=0;
			if (distance_from_Y > 0)
			{	
				turning=1;
				turn_dir=1;
				turn_angle=90;
				orient=L;
			}
			else if (distance_from_Y<0) 
			{ 
				turning =1;
				turn_dir=0;
				turn_angle=90;
				orient=R;
			}
			else 
			{
			turning =0;
			distance_from_X++;
			}
			
		}	
		else if  (orient == L)
			{
				obR=1;
				//reset orientation to FRONT
				turning=1;
				turn_dir=0;
				turn_angle=90;
				orient=F;
			}
		else if (orient == R)
			{
				obL=1;
				obR=0;
				obB=0;
				obF=0;
				turning=1;
				turn_dir=1;
				turn_angle=90;
				orient=F;
			}
		else if (orient == B)
			{
				obF=1;
				obL=0;
				obR=0;
				turning=1;
				turn_dir=1;
				turn_angle=180;
				orient=F;
			}
		 
			
		 }*/
		 
		  
		  
		  if (distance>=Grid_Res) //Move to next  Square
		    {
			    angle=0;
				
				distance=0; //Reset distance (set origin to new square)
				backing_up=0;
				if (obF==0)
				{
				 on_track=1;
				 
				 distance_from_X++;
				}
				else if ((obF==1) && (obL==1) && (obR==1)) //Reset the "wall" around  
					{
						obF=1;
						obL=0;
						obR=0;
						
					}
				else if ((obF==1)&& (obL==1) &&(obR==0))
						{
							obF=0;
							obL=1;
							obR=0;
							
						}
				else if ((obF==1) &&(obL==0))
						{
							obF=0;
							obL=0;
							distance_from_Y--;
						}
				
				if (distance_from_X >=0)
				{
					if(distance_from_Y==0)
					{
						{
							if (orient==F)
							{
								orient=F;
								turning=0;
								
							}
							else if (orient == R)
							{
							   orient=F;
							   turning=1;
							   turn_dir=1;
							   turn_angle=90;
							}
							else if (orient == L)
							{
								orient=F;
								turning=1;
								turn_dir=0;
								turn_angle=90;
							}
						}
					}
					else if(distance_from_Y>0)
					{
						{
							if (orient==F)
							{
								orient=L;
								turning=1;
								turn_dir=1;
								turn_angle=90;
								distance_from_Y--;
							}
							else if (orient == R)
							{
							   orient=F;
							   turning=1;
							   turn_dir=1;
							   turn_angle=90;
							}
							else if (orient == L)
							{
								orient=L;
								turning=0;
								distance_from_Y--;
							}
						}
					}
					else if (distance_from_Y <0)
							{
								if (orient==F)
								{
								orient = R;
								turning=1;
								turn_dir=0;
								turn_angle=90;
								distance_from_Y++;
								}
								
								else if (orient==R)
								{
								orient =R;
								turning=0;
								distance_from_Y++;
								}
								else if (orient == L)
								{
								orient = F;
								turning =1;
								turn_dir=0;
								turn_angle=90;
								}
							}
                
				}
				
				
				
				else 
					{
					if(distance_from_Y==0)
					{
						{
							if (orient==B)
							{
								orient=R;
								turning=1;
								turn_dir=1;
								turn_angle=90;
							}
							else if (orient == F)
							{
							   orient=F;
							   turning=0;
							}
							else if (orient == R)
							{
								orient=F;
								turning=1;
								turn_dir=1;
								turn_angle=90;
							}
							else if (orient == L)
							{
								orient =F;
								turning=1;
								turn_dir=0;
								turn_angle=90;
							}
						}
					}
					else if (distance_from_Y <0)
							{
								if (orient==F)
								{
								orient =R;
								turning=1;
								turn_dir=0;
								turn_angle=90;
								}
								
								else if (orient==R)
								{
								orient = R;
								turning=0;
								}
								else if (orient == L)
								{
								orient = F;
								turning =1;
								turn_dir=0;
								turn_angle=90;
								}
							}
						
					}
			}
		
        
		
		}
=======
		  obsR=0;
		  obsL=0;
		  if (distance > GRID_RES)	//Uyen: reset distance everytime it goes through a new grid
			distance = 0;
        }
>>>>>>> .r14


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




// Serial receive interrupt to store sensor values
SIGNAL(SIG_USART_RECV)
{
  uint8_t temp;


  temp = UDR0;

  if(sensors_flag)
  {
    sensors_in[sensors_index++] = temp;
    if(sensors_index >= Sen6Size)
      sensors_flag = 0;
  }
}




// Timer 1 interrupt to time delays in ms
SIGNAL(SIG_OUTPUT_COMPARE1A)
{
  if(timer_cnt)
    timer_cnt--;
  else
    timer_on = 0;
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
	 abs_distance+= (int)((sensors[SenDist1] << 8) | sensors[SenDist0]);
      angle += (int)((sensors[SenAng1] << 8) | sensors[SenAng0]);

      byteTx(CmdSensors);
      byteTx(6);
      sensors_index = 0;
      sensors_flag = 1;
    }
  }
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

  // Turn on interrupts
  sei();
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





