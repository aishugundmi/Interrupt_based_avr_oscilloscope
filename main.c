#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/pgmspace.h>

#define BAUDRATE 115200UL
//#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define BAUD_PRESCALLER 16


uint16_t adc_value0;
uint8_t ser[3];

void adc_init(void);
uint16_t read_adc(uint8_t channel);

void USART_init(void);
void USART_send(unsigned char data);
void USART_putstring(char* StringPtr);

void timer0pwm_init();

void adc_init(void)
{
    ADCSRA |= ((1<<ADPS2)|(1<<ADPS1));    //16Mhz/64
    ADMUX |= (1<<REFS0);                //Voltage reference from Avcc (5v)
    ADCSRA |= (1<<ADEN);                //Turn on ADC
    ADCSRA |= (1<<ADIE);               //Enables ADC interupt
    ADCSRA |= (1<<ADSC);               //Starts  conversion
}

uint16_t read_adc(uint8_t channel)
{
    ADMUX &= 0xF0;                   //Clear the older channel that was read
    ADMUX |= channel;                //ADC channel to be read
    ADCSRA |= (1<<ADSC);             //Starts  conversion
    while(ADCSRA & (1<<ADSC));       //Wait until the conversion is done
    return ADCW;                     //Returns the ADC value
}

ISR(ADC_vect)
{
    adc_value0 = read_adc(0);        //Read one ADC channel

    ser[1]=((uint16_t)adc_value0) >> 8;     // high byte
    ser[2]=((uint16_t)adc_value0) & 0x00FF; // low byte

    USART_send(ser[0]);
	USART_send(ser[1]);
	USART_send(ser[2]);
}


void timer0pwm_init()
{
    TCCR0A |= (1<<COM0A1) | (1<<WGM01) | (1<<WGM00);
    TCCR0B |= (1<<CS02);  //pre scaling 256
  ///  OCR0A = 64;  //25% duty cycle  255*(25/100)
    OCR0A = 127;  //50% duty cycle
    sei();
}




int main(void)
{
    DDRD |= (1<<PD6);   //PWM output OC0A

    adc_init();
    USART_init();
    timer0pwm_init();

   /// USART_putstring("Oscilloscope\n");
    ser[0]=0xF7;

    while(1)
    {

    }
    return 0;
}


void USART_init(void)
{
    UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALLER);

    UCSR0A |= (1<<U2X0);      //U2X0=1

    UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_send( unsigned char data)
{
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void USART_putstring(char* StringPtr)
{
    while(*StringPtr != 0x00){
    USART_send(*StringPtr);
    StringPtr++;}
}


