#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "fifo.h"

#define BAUDRATE 115200UL
//#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define BAUD_PRESCALLER 16


uint16_t adc_value0;
uint8_t ser[3];

void adc_init(uint8_t channel);

void USART_init(void);
void USART_send(unsigned char data);
void USART_putstring(char* StringPtr);

void timer0pwm_init();

void adc_init(uint8_t channel)
{
    ADCSRA |= ((1 << ADPS2) | (1 << ADPS1));    //16Mhz/64
    ADMUX |= (1 << REFS0);                  //Voltage reference from Avcc (5v)

    ADMUX |= channel;                //ADC channel to be read

    ADCSRA |= (1 << ADATE);          //enabble auto trigger
    ADCSRA |= (1 << ADEN);               //Turn on ADC
    ADCSRA |= (1 << ADIE);               //Enables ADC interupt
    ADCSRA |= (1 << ADSC);               //Starts  conversion

    sei();
}

ISR(ADC_vect)
{
    adc_value0 = ADCW;        //Read one ADC channel

    push_to_tx_fifo(0xF7);
    push_to_tx_fifo(((uint16_t)adc_value0) >> 8);       // high byte
    push_to_tx_fifo(((uint16_t)adc_value0) & 0x00FF);   // low byte

    UCSR0B |= (1<<UDRIE0);    // Enable UDRE interrupt
}

ISR(USART_UDRE_vect)
{
    if(fifo_data_available())
    {
        UDR0 = pop_tx_fifo();
    }
    else
    {
		UCSR0B &= ~(1<<UDRIE0);		// disable Data-Register-Empty-Interrupt
	}
}

void timer0pwm_init()
{
    TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B |= (1 << CS02);  //pre scaling 256
  ///  OCR0A = 64;  //25% duty cycle  255*(25/100)
    OCR0A = 127;  //50% duty cycle
    sei();
}


int main(void)
{
    DDRD |= (1 << PD6);   //PWM output OC0A

    uint8_t channel = 0b00000000;
    timer0pwm_init();
    adc_init(channel);
    USART_init();


   /// USART_putstring("Oscilloscope\n");


    while(1)
    {

    }
    return 0;
}


void USART_init(void)
{
    UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALLER);

    UCSR0A |= (1 << U2X0);      //U2X0=1

    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); //8 bit transmission

   // UCSR0B |= (1 << TXCIE0); //enable transmit interrupt
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



