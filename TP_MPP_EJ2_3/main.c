/* TP: Stepper Motor
 *
 * Asignatura:  Din�mica y Control de Sistemas Mecatr�nicos
 *
 * Catedra: DSF
 *
 * Enunciado:   Implemente una funci�n que utilice un timer A para que el motor se mueva a 1 Hz al presionar
 el pulsador 1, mantenga este comportamiento, pero adem�s permita utilizar otros pulsadores que modifiquen
 Enunciado_TP-MPP � Rev. 2025 DyCSM C�digo: TP-MPP P�gina 8 de 8
 su frecuencia, por ejemplo aument�ndola a 2 Hz. Luego aum�ntela a 10Hz y analice lo ocurrido comparando
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
#define FREQ_STEP_MAX   10 // Hz
#define FREQ_STEP_MIN   1  // Hz
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
// FRECUENCIA DEL MOTOR PASO A PASO
#define MOTOR_PASOS_POR_SEGUNDO(x)  200.0*x

// === VARIABLES ===
int precision = 1;
int count     = 0;

int *const pPrecision = &precision;

volatile bool   stepEnabled  = false;
volatile int    tmrSotfPasos = 0;

int countPasos = 0;

typedef enum {
    MOTOR_FREQ_MODO_1HZ = 0,
    MOTOR_FREQ_MODO_2HZ,
    MOTOR_FREQ_MODO_10HZ
}motor_modo_enum;

motor_modo_enum motorModo = MOTOR_FREQ_MODO_1HZ;

// === FUNCIONES ===
void stepWithTimerA();
void configPeriodTimerA(float freq);
void step();
void configure_timerA();
void delay_cycles_var(uint32_t ciclos);
void initParameters();

// --- CUERPO DE FUNCIONES ---
void delay_cycles_var(uint32_t ciclos)
{
    while (ciclos--)
    {
        __no_operation();   // cada NOP = 1 ciclo
    }
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
    printf("RAMPA DE VELOCIDAD\r\n");

    // --- CONFIGURAICIONES DEL TIMER A ---
    tmrSotfPasos = 200;
    stepEnabled = true;
    configPeriodTimerA(1);
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
    TA0CCR0   = 625;                    // 125 Khz --> 5 ms
}

void initParameters()
{
    stepEnabled = false;
    motorModo = MOTOR_FREQ_MODO_1HZ;
    tmrSotfPasos = 0;
    countPasos = 0;

    setPrecision(STEP_FULL);
    configure_timerA();
}

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

    initParameters();

    while (1)
    {
        if (BUTTON1_READ)
            button1();

        stepWithTimerA();
    }
}

void stepWithTimerA()
{
    if (!tmrSotfPasos && stepEnabled)
    {
        step();
        countPasos++;
        if (countPasos >= 10)
            countPasos = 1;
        configPeriodTimerA(countPasos);   // Reinicia el TimerA0
    }
}

/**
 * @brief Configura el timer por software para generar los
 * pasos del motor cada cierto periodo especificado.
 * 
 * @param freq frecuencia de pasos
 */
void configPeriodTimerA(float freq)
{
    float T = 0;

    T = 1.0/freq;
    tmrSotfPasos =  (int)(T/0.005); // Configurado en 5ms
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
    if (tmrSotfPasos)
        tmrSotfPasos--;
}
