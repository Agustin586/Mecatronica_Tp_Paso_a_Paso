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
#include <stdint.h>

#define FREQ_RELOJ  1000000
#define FREQ_1      __delay_cycles(10000)
#define FREQ_2      __delay_cycles(7000)
#define FREQ_3      __delay_cycles(5000)
#define FREQ_4      __delay_cycles(4000)
#define FREQ_5      __delay_cycles(3000)

#define BUTTON1_READ (P2IN & BIT6)
#define BUTTON2_READ (P8IN & BIT1)
#define BUTTON3_READ (P2IN & BIT3)

#define STEP_FULL       1
#define STEP_HALF       2
#define STEP_QUARTER    4
#define STEP_EIGHTH     8
#define STEP_SIXTEENTH  16

#define PASO            1.8
#define MAX_INDEX       4

int precision = 1;
int *const pPrecision = &precision;
int count = 0;
uint16_t microStep = 1;
uint16_t index = 0;
uint8_t dir_ant = 0;
int sec[] = { 180, -270, 135, -90, 45 };

void step();
void stepToMotor();
uint16_t convAngleToStep(uint16_t angle);

void stepToMotor()
{
    uint16_t i = 0;
    uint16_t cant_vueltas = convAngleToStep(abs(sec[index]));

    for (i = 0; i < cant_vueltas; i++)
    {
        // --- ENVIAMOS EL PASO AL MOTOR ---
        step();

        // --- DELAY ---
        switch (index)
        {
        case 0:
            FREQ_1;
            break;
        case 1:
            FREQ_2;
            break;
        case 2:
            FREQ_3;
            break;
        case 3:
            FREQ_4;
            break;
        case 4:
            FREQ_5;
            break;
        default:
            __delay_cycles(10000);
            break;
        }
    }
}

uint16_t convAngleToStep(uint16_t angle)
{
    return (angle * microStep) / PASO;
}

/* Function that sets the clocks registers*/
void configureClocks()
{
}

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
    uint16_t i = 0;
    uint16_t cant_vueltas = VUELTA_COMPLETA(2, pow(2, count));

    for (i = 0; i < cant_vueltas; i++)
    {
        step();

        switch (count)
        {
        case 0:
            __delay_cycles(10000);
            break;
        case 1:
            __delay_cycles(10000 / 2);
            break;
        case 2:
            __delay_cycles(10000 / 4);
            break;
        case 3:
            __delay_cycles(10000 / 8);
            break;
        case 4:
            __delay_cycles(10000 / 16);
            break;
        default:
            break;
        }
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

    microStep = pow(2, count);
    setPrecision(microStep);

    while (BUTTON2_READ)
        ;
}

void button3()
{
    P1OUT |= BIT6; // driver out of sleep mode
    P6OUT |= BIT6; // set output for ~RESET
    P4OUT &= ~BIT1; // set output for ~ENABLE
    __delay_cycles(100); // time for the changes to be done

    // --- CONFIGURA LA DIRECCION ---
    if (sec[index] >= 0)
        P3OUT |= BIT4;
    else
        P3OUT &= ~BIT4;

    // --- ENVIA PASOS AL MOTOR ---
    stepToMotor();

    // --- CAMBIA DE INDICE ---
    index++;

    if (index > MAX_INDEX)
        index = 0;

    while (BUTTON3_READ)
        ;
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

        if (BUTTON2_READ)
            button2();

        if (BUTTON3_READ)
            button3();

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
