/* TP: Stepper Motor
 *
 * Asignatura:  Dinámica y Control de Sistemas Mecatrónicos
 *
 * Catedra: DSF
 *
 * Enunciado:   Implemente una función que utilice un timer A para que el motor se mueva a 1 Hz al presionar
 el pulsador 1, mantenga este comportamiento, pero además permita utilizar otros pulsadores que modifiquen
 Enunciado_TP-MPP – Rev. 2025 DyCSM Código: TP-MPP Página 8 de 8
 su frecuencia, por ejemplo aumentándola a 2 Hz. Luego auméntela a 10Hz y analice lo ocurrido comparando
 los resultados con la curva torque-velocidad de la Figura 6.
 */
#include <msp430.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <stdbool.h>

// FRECUENCIA DE VELOCIDAD
#define FREQ_STEP_MAX   1000 // Hz
#define FREQ_STEP_MIN   100  // Hz
#define FREQ_ACTUAL(Nmax,N) (((int32_t)(FREQ_STEP_MIN - FREQ_STEP_MAX) * (int32_t)(N)) / (int32_t)(Nmax) + FREQ_STEP_MAX)
// LECTURA DE BOTONES
#define BUTTON1_READ    (P2IN & BIT6)
#define BUTTON2_READ    (P8IN & BIT1)
#define BUTTON3_READ    (P2IN & BIT3)
// CANTIDAD DE PASOS
#define STEP_FULL       1
#define STEP_HALF       2
#define STEP_QUARTER    4
#define STEP_EIGHTH     8
#define STEP_SIXTEENTH  16
// PASOS MINIMOS
#define PASO            1.8
#define MAX_INDEX       4
// CONFIG TIMER A
#define FREQ_RELOJ          1000000.0
#define TIMER_A_PRESCALER   8.0
#define TIMER_A_FREQ        FREQ_RELOJ/TIMER_A_PRESCALER
// PERIODO DE TIMER A
#define TIMER_A_FREQ_OUTPUT(pasosPorSegundo)  (TIMER_A_FREQ)/(pasosPorSegundo)
// FRECUENCIA DEL MOTOR PASO A PASO
#define MOTOR_PASOS_POR_SEGUNDO(x)  200.0*x
#define MOTOR_VUELTA_1HZ            1.0
#define MOTOR_VUELTA_2HZ            MOTOR_VUELTA_1HZ*2
#define MOTOR_VUELTA_10HZ           MOTOR_VUELTA_1HZ*10

// === VARIABLES ===
int precision = 1;
int count     = 0;
int maxPasos  = 0;

int *const pPrecision = &precision;

volatile bool    stepEnabled = false;
volatile bool    finVuelta   = true;

typedef enum {
    MOTOR_FREQ_MODO_1HZ = 0,
    MOTOR_FREQ_MODO_2HZ,
    MOTOR_FREQ_MODO_10HZ
}motor_modo_enum;

motor_modo_enum motorModo = MOTOR_FREQ_MODO_1HZ;

// === FUNCIONES ===
void stepWithTimerA();
void resetVuelta();
void configPeriodTimerA();
void step();
void configure_timerA();
void delay_cycles_var(uint32_t ciclos);
uint16_t convAngleToStep(uint16_t angle);

// --- CUERPO DE FUNCIONES ---
void delay_cycles_var(uint32_t ciclos)
{
    while (ciclos--)
    {
        __no_operation();   // cada NOP = 1 ciclo
    }
}

uint16_t convAngleToStep(uint16_t angle)
{
//    return (angle * microStep) / PASO;
}

/* Function that sets the clocks registers*/
void configureClocks()
{
    // Ajusta DCO a 1 MHz
    UCSCTL3 = SELREF_2;                      // FLL reference = REFO
    UCSCTL4 |= SELA_2;                       // ACLK = REFO

    __bis_SR_register(SCG0);                 // Disable FLL control loop
    UCSCTL0 = 0x0000;                        // Set lowest DCOx and MODx
    UCSCTL1 = DCORSEL_5;                     // Select DCO range ~16 MHz
    UCSCTL2 = FLLD_1 + 31;                   // (N+1)*FLLRef = DCO, N+1=32
                                             // DCO = 32 * 32768 Hz = 1.048 MHz
    __bic_SR_register(SCG0);                 // Enable FLL control loop
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
    // --- ANTIRREBOTE ---
    while (BUTTON1_READ)
        ;

    // --- MENSAJE POR PANTALLA ---
    printf("VELOCIDAD DE 1HZ\r\n");

    // --- CONFIGURAICIONES DEL TIMER A ---
    motorModo = MOTOR_FREQ_MODO_1HZ;
    resetVuelta();
}

void button2()
{
    P1OUT |= BIT6; // driver out of sleep mode
    P6OUT |= BIT6; // set output for ~RESET
    P4OUT &= ~BIT1; // set output for ~ENABLE
    __delay_cycles(100); // time for the changes to be done

    //Agregue las funcionalidades del pulsador aqui debajo
    // --- ANTIRREBOTE ---
    while (BUTTON2_READ)
        ;

    // --- MENSAJE POR PANTALLA ---
    printf("VELOCIDAD DE 2HZ\r\n");

    // --- CONFIGURACIONES DEL TIMER A ---
    motorModo = MOTOR_FREQ_MODO_2HZ;
    resetVuelta();
}

void button3()
{
    P1OUT |= BIT6; // driver out of sleep mode
    P6OUT |= BIT6; // set output for ~RESET
    P4OUT &= ~BIT1; // set output for ~ENABLE
    __delay_cycles(100); // time for the changes to be done

    // Nueva de aca para abajo
    // --- ANTIRREBOTE ---
    while (BUTTON3_READ)
        ;

    // --- MENSAJE POR PANTALLA ---
    printf("VELOCIDAD DE 10HZ\r\n");

    // --- CONFIGURACIONES DEL TIMER A ---
    motorModo = MOTOR_FREQ_MODO_10HZ;
    resetVuelta();
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
    TA0CTL   |= TASSEL_2 + ID_3 + MC_1; //using SMCLK with prescalr of 8 in upmode.
    TA0CCR0   = 625;
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

    _BIS_SR(GIE);               //enable general interuptions

    changeDir();
    setPrecision(STEP_FULL);

    configure_timerA();         //set and configure Timer A

    while (1)
    {
        if (BUTTON1_READ)
            button1();

        if (BUTTON2_READ)
            button2();

        if (BUTTON3_READ)
            button3();

        stepWithTimerA();
    }
}

void stepWithTimerA()
{
    if (stepEnabled)
    {
        P3OUT |=  BIT3;
        __delay_cycles(3);
        P3OUT &= ~BIT3;
        __delay_cycles(3);

        stepEnabled = !stepEnabled;
        count++;
    }

    if (count == maxPasos)
    {
        finVuelta = true;
        P1OUT &= ~BIT6;             // driver in sleep mode
    }
}

void resetVuelta()
{
    configPeriodTimerA();

    count = 0;
    finVuelta = false;
}

void configPeriodTimerA()
{
    switch (motorModo)
    {
    case MOTOR_FREQ_MODO_1HZ:
        TA0CCR0 = TIMER_A_FREQ_OUTPUT(
                    MOTOR_PASOS_POR_SEGUNDO(MOTOR_VUELTA_1HZ));
        setPrecision(STEP_FULL);
        maxPasos = 200;
        break;
    case MOTOR_FREQ_MODO_2HZ:
        TA0CCR0 = TIMER_A_FREQ_OUTPUT(
                    MOTOR_PASOS_POR_SEGUNDO(MOTOR_VUELTA_2HZ));
        TA0CCR0 = 500;
        setPrecision(STEP_FULL);
        maxPasos = 200;
        /*
         * Limitacion del motor: No puede hacer 400 pasos por segundo, osea 2 vueltas en un segundo.
         * */
        break;
    case MOTOR_FREQ_MODO_10HZ:
        TA0CCR0 = TIMER_A_FREQ_OUTPUT(
                    MOTOR_PASOS_POR_SEGUNDO(MOTOR_VUELTA_10HZ));
        /*
         * Limitacion del motor: Lo mismo que arriba
         * */
        break;
    default:
        printf("Error");
        assert(0);
        break;
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
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
{
    if (!finVuelta)
        stepEnabled = !stepEnabled;
}
