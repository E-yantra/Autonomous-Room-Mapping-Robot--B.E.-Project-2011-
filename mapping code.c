/****************************************************************************************************************************
Written by:Ankit Trivedi, Ankita Shrotriya, Rahul Patil, Prathamesh Akolkar K.J.Somaiya Engineering college (sion).
 
AVR Studio Version 4.17, Build 666

This experiment demonstrates the working of Autonomous Room Mapping Robot.

Concepts covered: External Interrupts
                  Position control,
                  Servo Motor Control using PWM
                  Wireless Serial Communication via zigbee
                  LCD interfacing
                  ADC Sensor
                  Direction algorithm
                  x-y coordinate algorithm
                  180 degree scanning 
                  Matrix generation 
                  navigation algorithm


Note: 
 
Make sure that in the configuration options following settings are done for proper operation of the code

Microcontroller: atmega2560
Frequency: 14745600
Optimization: -O0 

*****************************************************************************************************************************/

 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.c"  //included to support lcd interface

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
unsigned char ADC_Conversion(unsigned char);  //to select the required adc channel 
unsigned char ADC_Value;  //to accept the value from the required adc 
unsigned char sharp, distance, adc_reading;  //use for distance mesurement formula
unsigned int value;   //use to accept value of battery voltage
float BATT_Voltage, BATT_V;  //use to accept value of battery voltage



//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//----------------------------------------------------------------------------------------------------------


//function for position encoder


//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to initialize ports
void port_init()
{
 servo1_pin_config ();
 lcd_port_config();
 motion_pin_config(); //robot motion pins config
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config	
 adc_pin_config();
 buzzer_pin_config();
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

//ISR for right position encoder
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}



void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}


void stop (void)
{
  motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}


void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}
//-----------------------------------------------------------------------------------------------------------------------------

//function for adc convertor


//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00; //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}



// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}


// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}



//---------------------------------------------------------------------------------------------------------------


//function for buzzer

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
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



//-------------------------------------------------------------------------------------------------------------------------


//function for serial communication via zigbee


//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

//-----------------------------------------------------------------------------------------------------------------------


//function to generate pwn for servo motor

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86)+25 ;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}






//---------------------------------------------------------------------------------------------------------------


//function to display the direction of robo
 
char direct[4]={'N', 'E', 'S', 'W'};
int num=0;

void direction(void)
{
	if(num>=0 && num<4)
	{
		lcd_cursor(1,11);
		lcd_wr_char(direct[num]);	
	}
	else if(num<0)
	{
		num=3;
		lcd_cursor(1,11);
		lcd_wr_char(direct[num]);	
	}
	else if(num>3)
	{
		num=0;
		lcd_cursor(1,11);
		lcd_wr_char(direct[num]);	
	}
	return 0;
}



//---------------------------------------------------------------------------------------------------------------

//function to display the co_ordinates


 int x,y=0;

void coordinate_lcd(void)
{
    
	char xyupdate;
	char direct[4]={'N', 'E', 'S', 'W'};
	xyupdate=direct[num];
	if(xyupdate=='N')
	{
		y=y+1;
  		lcd_print(1,6,y,2);
	}
	else if(xyupdate=='E')
	{
		x=x+1;
  		lcd_print(1,2,x,2);
	}
	else if(xyupdate=='S')
	{
		y=y-1;
  		lcd_print(1,6,y,2);
	}
	else if(xyupdate=='W')
	{
		x=x-1;
		lcd_print(1,2,x,2);
	}
}

//--------------------------------------------------------------------------------------------------------------------------

//function for display


void print_screen(void)
{	
	lcd_cursor(1,1);
  	lcd_string("x");
	lcd_cursor(1,5);
  	lcd_string("y");
	lcd_cursor(1,9);
  	lcd_string("D:");
  	lcd_cursor(2,1);
  	lcd_string("T");
	lcd_cursor(2,6);
  	lcd_string("R");
}

//----------------------------------------------------------------------------------------------------


//function for 180 degree scanning and matrix generation


//function for 180 degree scanning

void servo(void)
{

unsigned int T,R;
//servo_1(0);
_delay_ms(1000); 


unsigned char i = 0;
for (i = 0; i <=21; i++)
{
servo_1(i*10);
_delay_ms(100);
UDR0=x;
_delay_ms(100);
UDR0=y;
_delay_ms(100);
T=(i*8.6);
UDR0=T;
lcd_print(2,2,T,3);
_delay_ms(100);
sharp = ADC_Conversion(10);
_delay_ms(100);						//Stores the Analog value of front sharp connected to ADC channel 10 into variable "sharp"
value = ((Sharp_GP2D12_estimation(sharp))/10);	
UDR0= value;			//Stores Distance calsulated in a variable "value".
lcd_print(2,7,value,3);
}
_delay_ms(500);


int p,q,r;

servo_1(0);
_delay_ms(1000);
sharp = ADC_Conversion(10);
_delay_ms(100);						//Stores the Analog value of front sharp connected to ADC channel 10 into variable "sharp"
p = (((Sharp_GP2D12_estimation(sharp))/10)/20);	
lcd_print(2,11,p,2);
_delay_ms(100); 

servo_1(90);
_delay_ms(500);
sharp = ADC_Conversion(10);
_delay_ms(100);						//Stores the Analog value of front sharp connected to ADC channel 10 into variable "sharp"
q = (((Sharp_GP2D12_estimation(sharp))/10)/20);	
lcd_print(1,13,q,2);
_delay_ms(100);

servo_1(210);
_delay_ms(500);
sharp = ADC_Conversion(10);
_delay_ms(100);						//Stores the Analog value of front sharp connected to ADC channel 10 into variable "sharp"
r = (((Sharp_GP2D12_estimation(sharp))/10)/20);	
lcd_print(2,14,r,2);
_delay_ms(100);


_delay_ms(1000);
 servo_1_free();

//--------------------------------------------------------------------------------------------------------------------------------


//function for matrix generation


char NESW;
char direct[4]={'N', 'E', 'S', 'W'};
NESW=direct[num];
int a,b,k,j;

int arr[7][7];
int arr_N[7][7];
int arr_E[7][7];
int arr_S[7][7];
int arr_W[7][7];




//for north
if(NESW=='N')
{
//for left matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=x-p;a<=p;a++)
{

   arr_N[y][a]=1;
}
}
}


//for center matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=y;a<=y+q;a++)
{

   arr_N[a][x]=1;
}
}
}


//for center matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=x;a<=x+r;a++)
{

   arr_N[y][a]=1;
}
}
}


for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
arr[k][j]= arr[k][j] | arr_N[k][j];
//UDR0= arr[k][j];
//_delay_ms(100);
}
}
}

//for east

else if(NESW=='E')
{

//for left matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=y;a<=p+y;a++)
{

   arr_E[a][x]=1;
}
}
}


//for center matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=x;a<=x+q;a++)
{

   arr_E[y][a]=1;
}
}
}


//for right matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=y-r;a<=y;a++)
{

   arr_E[a][x]=1;
}
}
}


for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
arr[k][j]= arr[k][j] | arr_E[k][j];
//UDR0= arr[k][j];
//_delay_ms(100);
}
}
}


//for south

else if(NESW=='S')
{

//for left matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=x;a<=x+p;a++)
{

   arr_S[y][a]=1;
}
}
}


//for center matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=y-q;a<=y;a++)
{

   arr_S[a][x]=1;
}
}
}


//for right matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=x-r;a<=x;a++)
{

   arr_S[y][a]=1;
}
}
}


for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
arr[k][j]= arr[k][j] | arr_S[k][j];
//UDR0= arr[k][j];
//_delay_ms(100);
}
}
}


//for west

else if(NESW=='W')
{
//for left matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=y-p;a<=y;a++)
{

   arr_W[a][x]=1;
}
}
}


//for center matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=x-q;a<=x;a++)
{

   arr_W[y][a]=1;
}
}
}


//for right matrix
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
  
for(a=y;a<=y+r;a++)
{

   arr_W[a][x]=1;
}
}
}


for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
arr[k][j]= arr[k][j] | arr_W[k][j];
//UDR0= arr[k][j];
//_delay_ms(100);
}
}
}

//-------------------------------------------------------------------------------------------------------------------------

//function for taking appropriate action for navigation


if(p!=0)
		{
		left_degrees(80);        //Rotate robot right by 90 degrees
		stop();
		num=num-1;
 		direction();
		_delay_ms(1000);

		forward_mm(150);          //Moves robot forward 100mm
		stop();
		direction();
		coordinate_lcd();
		_delay_ms(1000);
                }
         
else if(p==0 && q==0 && r!=0)
		 {
		 right_degrees(80);        //Rotate robot right by 90 degrees
		stop();
		num=num+1;
 		direction();
		_delay_ms(1000);

		forward_mm(150);          //Moves robot forward 100mm
		stop();
		direction();
		coordinate_lcd();
		_delay_ms(1000);
		}

else if(p==0 && q!=0)
		{
		forward_mm(180);          //Moves robot forward 100mm
		stop();
		direction();
		coordinate_lcd();
		_delay_ms(1000);
		}

else if(p==0 && q==0 && r==0)
		 {
		 right_degrees(80);        //Rotate robot right by 90 degrees
		stop();
		num=num+1;
 		direction();
		_delay_ms(1000);

		 right_degrees(80);        //Rotate robot right by 90 degrees
		stop();
		num=num+1;
 		direction();
		_delay_ms(1000);

		forward_mm(150);          //Moves robot forward 100mm
		stop();
		direction();
		coordinate_lcd();
		_delay_ms(1000);
		}	

///////////////////////////////////////////////////////////////////


 
}

//----------------------------------------------------------------------------------------------------------

//Function to initialize all the devices
void init_devices()
{
 cli(); //Clears the global interrupt
 port_init();  //Initializes all the ports
 timer1_init();
 uart0_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 adc_init();
 sei();   // Enables the global interrupt 
}

//----------------------------------------------------------------------------------------------------


//Main Function

int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
        print_screen();	
	while(1)
{  
servo();    //function for mapping technique
int k,j;
int arr[7][7];

if(x==0 && y==0)  //stop operation when return to original location (0,0) block
{
for(k=0;k<7;k++)
{
for(j=0;j<7;j++)
{
UDR0= arr[k][j];
_delay_ms(500);
}
}

break;

}
}
}

