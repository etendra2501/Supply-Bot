/*
 * Team Id: SB#2183
 * Author List: Natasha Choudhary,Pavit Kaur,Ritika Gupta,Etendra Verma
 * Filename: main.c
 * Theme: Supply bot
 * Functions:adc_pin_config(),adc_init(),ADC_Conversion(unsigned char Ch),velocity (unsigned char left_motor, unsigned char right_motor),motion_pin_config(),buzzer_pin_config(),port_init(),motion_set (unsigned char Direction),forward(),backward(),left(),right(),stop(),timer1_init(), buzzer_on(), buzzer_off(),servo_1_free(),init_devices()
 * Global Variables: THRESHOLD,VELOCITY_MAX,VELOCITY_MIN,VELOCITY_LOW,ADC_Value,Left_white_line,Center_white_line,Right_white_line
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "send.c"

#define		THRESHOLD		30        // set the threshold for all three sensor
#define		VELOCITY_MAX	50		//set the maximum,minimum velocity
#define		VELOCITY_MIN	35
#define 	VELOCITY_LOW	0

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

void adc_pin_config (void)
{
	DDRC = 0x00;   //set PORTc direction as input
	PORTC = 0x00;  
}
void adc_init()
{
	ADCSRA = 0x00;
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	return a;
}
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR1AH = 0x00;
	OCR1AL = left_motor;
	OCR1BH = 0x00;
	OCR1BL = right_motor;
}

void motion_pin_config (void)
{
	DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
	PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
	DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
	PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to initialize ports
void port_init()
{
	adc_pin_config();
	motion_pin_config();
	buzzer_pin_config();
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortBRestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortBRestore = PORTB; 			// reading the PORTB's original status
	PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
	PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void)         //both wheels forward
{
	motion_set(0x06);
}

void backward (void)        //both wheels backward
{
	motion_set(0x09);
}

void left (void)            //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void)           //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void stop (void)            //hard stop
{
	motion_set(0x00);
}
void timer1_init(void)
{
	TCCR1B = 0x00; //stop
	TCNT1H = 0xFF; //setup
	TCNT1L = 0x01;
	OCR1AH = 0x00;
	OCR1AL = 0xFF;
	OCR1BH = 0x00;
	OCR1BL = 0xFF;
	ICR1H  = 0x00;
	ICR1L  = 0xFF;
	TCCR1A = 0xA1;
	TCCR1B = 0x0D; //start Timer
}


void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}
void init_devices (void)
{
	cli();         //Clears the global interrupts
	port_init();
	timer1_init();
	adc_init();
	sei();         //Enables the global interrupts
}

//Main Function

int main(void) {
	init_devices();
	char receive_data;

	uart0_init(UART_BAUD_SELECT(9600, F_CPU));
	uart0_flush();
	uart0_puts("*** Arduino Uno UART0 ECHO ***\n");

	while(1)
	{

		receive_data = uart0_readByte();
		if(receive_data == 0x38)        //ASCII value of 8
		{
			stop(); 
			servo_1_free(); 
			forward();
			                 
		}
		else
		{
			unsigned char flag ;

			velocity(VELOCITY_MAX,VELOCITY_MAX);    // Set the speed to max velocity
			forward();                              // start to move froward
	
			Left_white_line = ADC_Conversion(2);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(4);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(5);	//Getting data of Right WL Sensor
			

			flag=0;

			if(Center_white_line<THRESHOLD)               // Is middle Whiteline is within threshold limit
			{
				flag=1;
				velocity(VELOCITY_MAX,VELOCITY_MAX);      // Run robot at max velocity
			}

			if(((Left_white_line-50)>THRESHOLD) && (flag==0))  // Is left Whiteline is not within threshold limit
			{
				flag=1;
				velocity(VELOCITY_MAX,VELOCITY_MIN);      // Run robot left wheel at max velocity and right wheel
			}

			if((Right_white_line>THRESHOLD) && (flag==0)) // Is right Whiteline is not within threshold limit
			{
				flag=1;
				velocity(VELOCITY_MIN,VELOCITY_MAX);      // Run robot right wheel at max velocity and left wheel
			}

			if(Center_white_line>THRESHOLD && (Left_white_line-50)>THRESHOLD && Right_white_line>THRESHOLD && (flag == 0))
			// if all Whiteline sensor are not within threshold limit
			{
				flag=1;
				velocity(VELOCITY_LOW,VELOCITY_LOW);      // stop the robot
			}

		
			}
		}
	
	return 0;
}
