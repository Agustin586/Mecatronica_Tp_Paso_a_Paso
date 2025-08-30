/* TP: Stepper Motor
 *
 * Asignatura:  Dinámica y Control de Sistemas Mecatrónicos
 *
 * Catedra: DSF
 *
 */
#include <msp430.h>
#include <inttypes.h>
#include <math.h>

#define FREQ_RELOJ  1000000
#define DELAY_1S    __delay_cycles(10000)

#define BUTTON1_READ (P2IN & BIT6)
#define BUTTON2_READ (P8IN & BIT1)

#define STEP_FULL       1
#define STEP_HALF       2
#define STEP_QUARTER    4
#define STEP_EIGHTH     8
#define STEP_SIXTEENTH  16

#define TOTAL_ANGLE     360
#define PASO            1.8
#define VUELTA_COMPLETA(x) (TOTAL_ANGLE*x)/PASO

/* Function that sets the clocks registers*/
void configureClocks()
{
}

int precision = 1;
int *const pPrecision = &precision;
int count = 0;

/* configure precision of step.
 * 1: full step
 * 2: half step
 * 4: quarter step
 * 8: eighth step
 * 16: sixteenth step
 */
void setPrecision(int p)
{
    switch (p)
    {
    case 1:
        P4OUT &= ~BIT2; // set output for MS1
        P2OUT &= ~BIT7; // set output for MS2
        P3OUT &= ~BIT2; // set output for MS3
        break;
    case 2:
        P4OUT |= BIT2; // set output for MS1
        P2OUT &= ~BIT7; // set output for MS2
        P3OUT &= ~BIT2; // set output for MS3
        break;
    case 4:
        P4OUT &= ~BIT2; // set output for MS1
        P2OUT |= BIT7; // set output for MS2
        P3OUT &= ~BIT2; // set output for MS3
        break;
    case 8:
        P4OUT |= BIT2; // set output for MS1
        P2OUT |= BIT7; // set output for MS2
        P3OUT &= ~BIT2; // set output for MS3
        break;
    case 16:
        P4OUT |= BIT2; // set output for MS1
        P2OUT |= BIT7; // set output for MS2
        P3OUT |= BIT2; // set output for MS3
        break;
    }
}

void setStep(int step)
{
}

/* Function that produces one step*/
void step()
{
    P3OUT |= BIT3;
    __delay_cycles(10);
    P3OUT &= ~BIT3;
}

/* Function that changes the direction of rotation of the motor*/
void changeDir()
{
    P3OUT ^= BIT4;
}

/* Function that stops current actions, puts the driver in reset and no enable mode*/
void restart()
{
    P6OUT &= ~BIT6;     // put reset
    P4OUT |= BIT1;      // enable
    *pPrecision = 1;
    setPrecision(*pPrecision);
    __delay_cycles(100);
}
/*
 ///////////////////////////////////////////////////////////////////////////////////////////////
 funciones de los pulsadores para modificar.

 Insertar codigo despues del delay impuesto para que se realicen los cambios iniciales.
 ///////////////////////////////////////////////////////////////////////////////////////////////
 */
void button1()
{
    P1OUT |= BIT6;          // driver out of sleep mode
    P6OUT |= BIT6;          // set output for ~RESET
    P4OUT &= ~BIT1;         // set output for ~ENABLE
    __delay_cycles(100);    // time for the changes to be done

    //Agregue las funcionalidades del pulsador aqui debajo
    int i = 0;
    for (i = 0; i < VUELTA_COMPLETA(2); i++)
    {
        step();
        DELAY_1S;
    }
}

void button2()
{
#define MAX_COUNT 4

    P1OUT |= BIT6; // driver out of sleep mode
    P6OUT |= BIT6; // set output for ~RESET
    P4OUT &= ~BIT1; // set output for ~ENABLE
    __delay_cycles(100); // time for the changes to be done

    //Agregue las funcionalidades del pulsador aqui debajo
    count++;

    if (count > MAX_COUNT)
        count = 0;

    setPrecision(pow(2, count));

    while(BUTTON2_READ);
}

void button3()
{
    P1OUT |= BIT6; // driver out of sleep mode
    P6OUT |= BIT6; // set output for ~RESET
    P4OUT &= ~BIT1; // set output for ~ENABLE
    __delay_cycles(100); // time for the changes to be done
    //Agregue las funcionalidades del pulsador aqui debajo

}

/* pressing this button the state is reseted (interruption enabled for this one) */
void button4()
{
    P1OUT |= BIT6; // driver out of sleep mode
    restart();
}

void configure_timerA()
{
    TA0CCTL0 |= CCIE;
    TA0CTL |= TASSEL_2 + ID_2 + MC_1; //using SMCLK with prescalr of 8 in upmode.
    TA0CCR0 = 60000;   //Timer Count
}

/*
 ///////////////////////////////////////////////////////////////////////////////////////////////
 main del programa
 ///////////////////////////////////////////////////////////////////////////////////////////////
 */

void main()
{
    WDTCTL = WDTPW + WDTHOLD; //Stop watchdog timer (sends pwrd and hold)
    configureClocks(); // configure the clocks
//    configureTimerA(); //set and configure Timer A
    P1SEL = 0x00;
    P2SEL = 0x00;
    P3SEL = 0x00;
    P4SEL = 0x00;

    /* input pins */
    P2DIR &= ~BIT6; // set input for button 1
    P8DIR &= ~BIT1; // set input for button 2
    P2DIR &= ~BIT3; // set input for button 3
    P2DIR &= ~BIT2; // set input for button 4

    /* output pins */
    P6DIR |= BIT6; // set output for RESET
    P4DIR |= BIT1; // set output for ENABLE
    P1DIR |= BIT6; // set output for SLEEP
    P3DIR |= BIT3; // set output for STEP
    P3DIR |= BIT4; // set output for DIR
    P4DIR |= BIT2; // set output for MS1
    P2DIR |= BIT7; // set output for MS2
    P3DIR |= BIT2; // set output for MS3
    P6DIR |= BIT5; // set output for LED

    /* interruptable pins */
    P2IE |= BIT2;       // set interruptions for button 4
    P2IES &= ~BIT2;     // set interruption edge on low to high
    P2IFG &= ~BIT2;     // clean flag

    /* initial state to 0 */
    P3OUT &= ~BIT3;
    P3OUT &= ~BIT4;
    P4OUT &= ~BIT1;
    P4OUT &= ~BIT2;
    P2OUT &= ~BIT7;
    P4OUT &= ~BIT7;

    _BIS_SR(GIE);     //enable general interuptions

    changeDir();
    setPrecision(STEP_FULL);

    while (1)
    {
        P1OUT &= ~BIT6; // driver in sleep mode

        if (BUTTON1_READ)
        {
            button1();
        }
        if (BUTTON2_READ)
        {
            button2();
        }
//        if ((P2IN & BIT3) != 0)
//        {
//            button3();
//        }
        __delay_cycles(10000); // to avoid entering multiple times when pushing only once
    }
}

// Interruption routine for the port 2
#pragma vector= PORT2_VECTOR
__interrupt void SW_1(void)
{
    P2IFG &= ~BIT2;                 // clean flag
    button4();                      // call routine of the button
}

// Timer A0 interrupt service routine
//#pragma vector=TIMER0_A0_VECTOR
//__interrupt void Timer_A0 (void){
//}
