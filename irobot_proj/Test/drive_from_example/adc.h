
//Define ADC setups for accelerometer
//(Not defined here are reserved bits)

//ADMUX - ADC Multiplexer Seletion Register (8 bits)
	//select voltage ref
#define REFS_0	(0<<6)		//AREF 
#define REFS_1	(1<<6)		//AVcc 
#define REFS_3	(3<<6)		//Internal 1.1 Voltage Reference 
	
#define my_ADLAR	0x20		//left justified conversion result in ADC Data Register
	//ADC input channel selection
#define	INCH_0	0			//ADC0
#define INCH_1	1			//ADC1
#define INCH_2	2 			//ADC2
#define INCH_3	3			//ADC3
#define	INCH_4	4			//ADC4
#define INCH_5	5			//ADC5
#define	INCH_6	6			//ADC6
#define	INCH_7	7			//ADC7
#define INCH_14 14			//1.1V (V_bandgap)
#define INCH_15 15			//GND

//ADCSRA - ADC Control & Status Register A (8 bits)
#define my_ADEN	0x80		//ADC Enable
#define my_ADSC	0x40		//ADC Start Conversion
#define my_ADATE	0x20		//ADC Auto Trigger Enable
#define my_ADIF	0x10		//ADC Interrupt Flag 
#define my_ADIE	0x08		//ADC Interrupt Enable
	//division factor b/t system clock and ADC clock
#define ADPS_0	0			//:2
#define ADPS_1	1			//:2
#define ADPS_2	2			//:4
#define ADPS_3	3			//:8
#define ADPS_4	4			//:16
#define ADPS_5	5			//:32
#define ADPS_6	6			//:64
#define ADPS_7	7			//:128

//ADCSRB - ADC Control and Status Register B (8 bits)
#define my_ACME	0x40		//?
	//ADC Auto Trigger Source
#define ADTS_0	0			//Free Running Mode	
#define ADTS_1	1			//Analog Comparator
#define ADTS_2	2			//External Interrupt Request 0
#define ADTS_3	3			//Timer/Counter0 Compare Match A
#define ADTS_4	4			//Timer/Counter0 Overflow
#define ADTS_5	5			//Timer/Counter1 Compare Match B
#define ADTS_6	6			//Timer/Counter1 Overflow
#define ADTS_7	7			//Timer/Counter1 Capture Event

//DIDR0 - Digital Input Disable Register 0
#define ADC_0D	(1<<0)		//Set PC.0 as ADC0
#define ADC_1D	(1<<1)		//set PC.1 as ADC1
#define ADC_2D	(1<<2)   	//set PC.2 as ADC2	
#define ADC_3D	(1<<3)		//set PC.3 as ADC3
#define ADC_4D	(1<<4)		//set PC.4 as ADC4
#define ADC_5D	(1<<5)		//set PC.5 as ADC5s
