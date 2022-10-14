/*
 =========================================================================================
                                    STOP WATCH SYSTEM
 =========================================================================================
 */

#include<avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#define CMP_VALUE 1000
enum states{CLR,SET};

unsigned char g_flag=0;
unsigned char ticks[6]={0};


/*=======================================================================================
                                      INITIALIZATIONS
  =======================================================================================*/
void Timer1_CTC_Init(void)
{
	//SREG &= ~(1<<7);
	TCNT1 = 0;                 /*Set the initial count = zero.*/
	/*F_clock= 1MHz,
	 *Set pre-scaler 1024,
	 *So that F_timer = 1M/1024= 1KHz,
	 *T_timer= 1ms,
	 *Counts to get 1 second = 1000.
	 */
	OCR1A = CMP_VALUE;
	TIMSK |= (1<< OCIE1A);     /*Enable timer1 output compare A match interrupt.*/

	/* TCCR1A register:
	 * -Disconnect (OC1A and OC1B) COM1A1=0 COM1A0=0 COM1B0=0 COM1B1=0 (Normal port operation)
	 * -FOC1A=1 FOC1B=0
	 * -CTC Mode WGM10=0 WGM11=0 (Mode Number 4)
	 */
	TCCR1A = (1<<FOC1A);

	/* TCCR1B register
	 * -CTC Mode WGM12=1 WGM13=0 (Mode Number 4)
	 * -Pre-scaler = F_CPU/1024 CS10=1 CS11=0 CS12=1
	 */
	TCCR1B=(1<<CS10)|(1<<CS12); //pre-scaler=1024
	TCCR1B|=(1<<WGM12); //CTC mode
	//SREG |= (1<<7);
}
void Ext_Interrupt0_Init (void)
{
	DDRD &= ~(1<<PD2);   /*Pin2 at portD is an input for the push button*/
	PORTD |= (1<<PD2);   /*Enable internal pull up resistor*/
	GICR |= (1<<INT0);   /*Enable external interrupt INT0*/
	MCUCR |= (1<<ISC01);   /*Configure INT0 with falling edge*/
}
void Ext_Interrupt1_Init (void)
{
	DDRD &= ~(1<<PD3);   /*Pin3 at portD is an input for the push button*/
	PORTD &= ~(1<<PD3);   /*disable internal pull up resistor*/
	GICR |= (1<<INT1);   /*Enable external interrupt INT1*/
	MCUCR |= (1<<ISC10) | (1<<ISC11);   /*Configure INT1 with rising edge*/

}
void Ext_Interrupt2_Init (void)
{
	DDRB &= ~(1<<PB2);   /*Pin2 at portB is an input for the push button*/
	PORTB = (1<<PB2);   /*Enable internal pull up resistor*/
	GICR |= (1<<INT2);   /*Enable external interrupt INT2*/
	MCUCSR &= ~(1<<ISC2);   /*Configure INT2 with falling edge*/
}

void Decoder_Init (void)
{
	DDRC=0x0F;      //first 4bits of port C as output pins
	PORTC=0;
}
void Seven_Segment_Control (void)
{
	DDRA |= 0x3F;    /*Set six four pins as outputs for the control*/
	//PORTA = 1;     /*Update the first 7-segment unit (second1)*/
}

/*======================================================================================
                                   DISPLAYING FUNCTIONS
  ======================================================================================*/
void Display_ON (void)
{
	for (int i=0; i<6; i++)
	{
		//PORTA = (1<<i);
		//PORTC = ticks[i];
		PORTA=(PORTA&~(0x3F)) | (1<<i);
		PORTC=((PORTC&(0xF0)) | (ticks[i] & 0x0F) );
		_delay_ms(5);

	}
}
void Stop_Watch_Increment(void)
{
	ticks[0]++;
	if(ticks[0]==10)
	{
		ticks[0]=0;
		ticks[1]++;
		if( (ticks[0]==0) && (ticks[1]==6) )
		{
			ticks[1]=0;
			ticks[2]++;
		}
	}
	if(ticks[2]==10)
	{
		ticks[2]=0;
		ticks[3]++;
		if( (ticks[2]==0) && (ticks[3]==6) )
		{
			ticks[3]=0;
			ticks[4]++;
		}
	}
	if(ticks[4]==10)
	{
		ticks[4]=0;
		ticks[5]++;

	}
}


/*======================================================================================
                                          ISRs
  ======================================================================================*/
ISR(TIMER1_COMPA_vect)
{
	SREG |= (1<<7);   /*Enable nested interrupts*/
	g_flag=SET;
}
ISR(INT0_vect){      /*RESET*/
	SREG|=(1<<7);    /*Enable interrupts nesting*/

	for(int i=0;i<6;i++){
		ticks [i] = 0;
	}
	TCNT1=0;
}

ISR(INT1_vect){       /*PAUSE*/
	SREG|=(1<<7);      /*Enable interrupts nesting*/

	TCCR1B &= ~(1<<CS12);
	TCCR1B &= ~(1<<CS11);
	TCCR1B &= ~(1<<CS10);
}

ISR(INT2_vect){
	SREG|=(1<<7);    /*Enable nested interrupts*/
	//resume stop watch ,we can do this by reactive timer1 (connect to clock source again)
	//enable _clock with pre-scaler=1024
	TCCR1B|=(1<<CS10);
	TCCR1B|=(1<<CS12);
}


int main (void)
{
	SREG |= (1<<7);       /*Enable the I-bit (Global interrupt enable)*/

	Ext_Interrupt0_Init ();
	Ext_Interrupt1_Init ();
	Ext_Interrupt2_Init ();

	Timer1_CTC_Init();
	Decoder_Init ();
	Seven_Segment_Control ();


	while (1)
	{
		Display_ON();
		if(g_flag == SET)
		{
			Stop_Watch_Increment();
			g_flag = CLR;
		}

	}
}

