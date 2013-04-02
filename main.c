#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define Y_AXIS_CHANNEL          0     /* PORTC0*/
#define X_AXIS_CHANNEL          1     /* PORTC1*/

#define PWM0_PORT               PORTB
#define PWM0_DDR                DDRB
#define PWM0_PIN                3 

#define PWM0_SET_DUTY(D)        OCR0A = ((100-D) * 255 / 100)

#define PWM1_PORT               PORTD
#define PWM1_DDR                DDRD
#define PWM1_PIN                6 

#define PWM1_SET_DUTY(D)        OCR2A = ((100-D) * 255 / 100)

#define SET_PIN_OUTPUT(REG,PIN) (REG |= (1 << PIN))

void init_ADC(void){

    // ADC voltage reference
    // REFS1 REFS0 Description
    //  0     0    AREF
    //  0     1    AVcc
    //  1     0    Reserved
    //  1     1    Internal 1.1V
 
    // When ADLAR = 1 (Left Adjusted)
    //---------------------------------------------------------
    //| ADC9 | ADC8 | ADC7 | ADC6 | ADC5 | ADC4 | ADC3 | ADC2 | ADCH
    //---------------------------------------------------------
    //| ADC1 | ADC0 |      |      |      |      |      |      | ADCL
    //---------------------------------------------------------

    // ADCH = Vin * 1024 / VREF
    
    // Left adjust 10 bit ADC value just need to read ADCH 8-bit precision
    ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<ADLAR);

    // Select channel by default
    ADMUX |= 0;

    // ADC Prescaler Selections
    // ADPS2 ADPS1 ADPS0 Division Factor
    //   0     0     0          2
    //   0     0     1          2
    //   0     1     0          4
    //   0     1     1          8
    //   1     0     0         16
    //   1     0     1         32
    //   1     1     0         64
    //   1     1     1        128
    
    // Enable ADC, Enable Interrupt and 128 clk division factor
    ADCSRA = (1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|(0<<ADPS2)|(0<<ADPS1)|(1<<ADPS0);

    // ADC Auto Trigger Sources
    // ADTS2 ADTS1 ADTS0 Trigger Source
    //  0    0     0    Free Running mode
    //  0    0     1    Analog Comparator
    //  0    1     0    External Interrupt Request 0
    //  0    1     1    Timer/Counter0 Compare Match A
    //  1    0     0    Timer/Counter0 Overflow
    //  1    0     1    Timer/Counter1 Compare Match B
    //  1    1     0    Timer/Counter1 Overflow
    //  1    1     1    Timer/Counter1 Capture Event

    ADCSRB = (0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0);

    DIDR0 = (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(0<<ADC1D)|(0<<ADC0D);
}

void init_timer0(void)
{
    // Freq = F_CPU / prescaler / 255 
    // Freq = 20000000 / 256 / 255 = 306Hz 
    
    // CS02 CS01 CS00  Description
    //  0    0    0    No Clock Source (Timer/Counter stopped)
    //  0    0    1    No Prescaling
    //  0    1    0    clk/8
    //  0    1    1    clk/64
    //  1    0    0    clk/256
    //  1    0    1    clk/1024
    //  1    1    0    External T0 pin failing edge
    //  1    1    1    External T0 pin rising edge

    OCR0A = 128;

    // Fast PWM Port operation
    TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00);

    TCCR0B = (1<<FOC0A)|(1<<CS02)|(0<<CS01)|(0<<CS00)|(0<<WGM02);

    // Use overflow interrupt
    TIMSK0 = (0<<OCIE0B)|(0<<OCIE0A)|(0<<TOIE0);
    
}

void init_timer2(void){

    // Freq = F_CPU / prescaler / 2 * OCR2A
    // OCR2A = F_CPU / prescaler / Freq / 2 

    // CS22 CS21 CS20  Description
    //  0    0    0    No Clock Source (Timer/Counter stopped)      
    //  0    0    1    No Prescaling
    //  0    1    0    clk/8
    //  0    1    1    clk/32
    //  1    0    0    clk/64 
    //  1    0    1    clk/128
    //  1    1    0    clk/256
    //  1    1    1    clk/1024
   
    // Freq = 125Hz
    OCR2A = 127;

    TCCR2A = (1<<COM2A1)|(1<<COM2A0)|(1<<WGM21)|(1<<WGM20);

    TCCR2B = (1<<CS22)|(1<<CS21)|(0<<CS20)|(1<<FOC2A)|(0<<WGM22);

    TIMSK2 = (0<<OCIE2B)|(0<<OCIE2A)|(0<<TOIE2);
}

uint8_t ADC_read(uint8_t channel){

    // Clear MUX
    ADMUX  &= 0xF0; 

    // Enable Channel
    ADMUX  |= ( channel & 0x0F );

    // Start Conversion
    ADCSRA |= (1<<ADSC);

    // Wait for Conversion to finish
    while(ADCSRA & (1 << ADSC));

    // Return ADC Value
    return ADCH;
}

int main(void) {

    SET_PIN_OUTPUT(PWM0_DDR,PWM0_PIN);
    SET_PIN_OUTPUT(PWM1_DDR,PWM1_PIN);

    init_timer0();
    init_timer2();

    init_ADC();

    sei(); /* Enable interrupts */

    PWM0_SET_DUTY(50);
    PWM1_SET_DUTY(50);

    uint8_t x_axis = 0;
    uint8_t y_axis = 0;

    while(1){


      x_axis = ADC_read(X_AXIS_CHANNEL);
      y_axis = ADC_read(Y_AXIS_CHANNEL);

      PWM0_SET_DUTY(x_axis*100/255);
      PWM1_SET_DUTY(y_axis*100/255);

    }

    return 0;
}
