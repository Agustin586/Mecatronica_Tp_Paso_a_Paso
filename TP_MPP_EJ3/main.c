/**
 * @file main.c
 * @brief Ejercicio 3 del Tp de motor paso a paso
 * @author Zuliani, Agustin 
 */

#include <msp430.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <stdbool.h>

// =======================================================
// DEFINES
// =======================================================
// LECTURA DE BOTONES
#define BUTTON1_READ    (P2IN & BIT6)
#define BUTTON2_READ    (P8IN & BIT1)
#define BUTTON3_READ    (P2IN & BIT3)

#define FREQ_CONSTANTE			10	// Hz
#define FREQ_MITAD_CONSTANTE 	5

#define PASO 1.8

#define STEP_FULL       1
#define STEP_HALF       2
#define STEP_QUARTER    4
#define STEP_EIGHTH     8
#define STEP_SIXTEENTH  16

// =======================================================
// VARIABLES
// =======================================================
int precision = 1;
int *const pPrecision = &precision;
volatile int tmrSotfPasos = 0;

typedef enum {
	EST_IDLE = 0,
	EST_PULSADOR1,
	EST_PULSADOR2,
	EST_PULSADOR3
}estMotor_t;

estMotor_t stateMefMotor = EST_IDLE;

// =======================================================
// DECLARACION DE FUNCIONES
// =======================================================
void configureClocks();
void setPrecision(int p);
void step();
void changeDir();
void configure_timerA();
void button1();
void button2();
void button3();
void button4();
void restart();
void mefMotorInit();
void mefMotor();
void stepWithTimerA();
void configPeriodTimerA(float freq);
uint16_t convAngleToStep(uint16_t angle);

// =======================================================
// CUERPO DE FUNCIONES
// =======================================================
void configureClocks()
{
    // Ajusta DCO a 1 MHz
    UCSCTL3 = SELREF_2;                      
    UCSCTL4 |= SELA_2;                       

    __bis_SR_register(SCG0);                 
    UCSCTL0 = 0x0000;                        
    UCSCTL1 = DCORSEL_5;                     
    UCSCTL2 = FLLD_1 + 31;                   
                                             
    __bic_SR_register(SCG0);                 
}

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

void step()
{
    P3OUT |= BIT3;
    __delay_cycles(10);
    P3OUT &= ~BIT3;
}

void changeDir()
{
    P3OUT ^= BIT4;
}

void configure_timerA()
{
    TA0CCTL0 |= CCIE;
    TA0CTL   |= TASSEL_2 + ID_3 + MC_1; //using SMCLK with prescalr of 8 in upmode.
    TA0CCR0   = 625;                    // 125 Khz --> 5 ms
}

void button1()
{
    P1OUT |= BIT6;          
    P6OUT |= BIT6;          
    P4OUT &= ~BIT1;         
    __delay_cycles(100);    

    // --- ANTIRREBOTE ---
    while (BUTTON1_READ)
        ;

    // --- MENSAJE POR PANTALLA ---
    printf("[DEBUG] Pulsador 1\r\n");
}

void button2()
{
    P1OUT |= BIT6; // driver out of sleep mode
    P6OUT |= BIT6; // set output for ~RESET
    P4OUT &= ~BIT1; // set output for ~ENABLE
    __delay_cycles(100); // time for the changes to be done

	// --- ANTIRREBOTE ---
    while (BUTTON2_READ)
        ;

	// --- MENSAJE POR PANTALLA ---
	printf("[DEBUG] Pulsador 2\r\n");
}

void button3()
{
    P1OUT |= BIT6; // driver out of sleep mode
    P6OUT |= BIT6; // set output for ~RESET
    P4OUT &= ~BIT1; // set output for ~ENABLE
    __delay_cycles(100); // time for the changes to be done

	// --- ANTIRREBOTE ---
    while (BUTTON3_READ)
        ;

	// --- MENSAJE POR PANTALLA ---
	printf("[DEBUG] Pulsador 3\r\n");
}

void button4()
{
    P1OUT |= BIT6; // driver out of sleep mode
    restart();
}

void restart()
{
    P6OUT &= ~BIT6;     // put reset
    P4OUT |= BIT1;      // enable
    *pPrecision = 1;
    setPrecision(*pPrecision);
    __delay_cycles(100);
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
	configure_timerA();
	setPrecision(STEP_FULL);

	mefMotorInit();

	while (1)
	{
		mefMotor();
	}
	
}

void mefMotorInit()
{
	stateMefMotor = EST_IDLE;
	printf("[DEBUG] Modo de Motor IDLE\r\n");
}

void mefMotor()
{
	static int numStep = 0;
	static bool dosVueltas = false;
	static int stepCount = 0;

	switch (stateMefMotor)
	{
		case EST_IDLE:
			// Acciones...
			P1OUT &= ~BIT6; // driver in sleep mode
			
			// Transicion
			if (BUTTON1_READ)
			{
				// Debug...
				printf("[DEBUG] Modo de Motor 1\r\n");

				// Acciones...
				button1();
				stepCount = 0;
				stateMefMotor = EST_PULSADOR1;
				configPeriodTimerA(FREQ_CONSTANTE);
			}
			break;
		case EST_PULSADOR1:
			// Acciones...
			if (!tmrSotfPasos)
			{
				stepCount++;
				step();
				configPeriodTimerA(FREQ_CONSTANTE);
			}

			// Transicion
			if (BUTTON2_READ)
			{
				// Debug...
				printf("[DEBUG] Modo de Motor 2\r\n");

				// Acciones...
				button2();
				changeDir();
				configPeriodTimerA(FREQ_CONSTANTE);
				numStep = convAngleToStep(180);
				stateMefMotor = EST_PULSADOR2;
			}
			else if (BUTTON3_READ)
			{
				// Debug...
				printf("[DEBUG] Modo de Motor 3\r\n");

				// Acciones...
				button3();
				if (stepCount > 400)
					dosVueltas = true;
				if (dosVueltas)
				{
					changeDir();
					numStep = convAngleToStep(360);
				}
				else
				{
					configPeriodTimerA(FREQ_MITAD_CONSTANTE);
				}
				stateMefMotor = EST_PULSADOR3;
			}
			break;
		case EST_PULSADOR2:
			// Acciones...
			if (!tmrSotfPasos && numStep)
			{
				numStep--;
				step();
				configPeriodTimerA(FREQ_CONSTANTE);
			}

			// Transicion
			if (!numStep)
			{
				// Debug...
				printf("[DEBUG] Modo de Motor 1\r\n");

				// Acciones...
				dosVueltas = false;
				stepCount = 0;
				changeDir();
				configPeriodTimerA(FREQ_CONSTANTE);
				stateMefMotor = EST_PULSADOR1;
			}

			break;
		case EST_PULSADOR3:
			// Acciones...
			if (!tmrSotfPasos && dosVueltas && numStep)
			{
				step();
				numStep--;
				configPeriodTimerA(FREQ_CONSTANTE);
			}
			if (!tmrSotfPasos && !dosVueltas)
			{
				step();
				configPeriodTimerA(FREQ_MITAD_CONSTANTE);
			}
			
			// Transiciones...
			if (BUTTON1_READ)
			{
				// Debug...
				printf("[DEBUG] Modo de Motor 1\r\n");

				// Acciones...
				button1();
				configPeriodTimerA(FREQ_CONSTANTE);
				if (dosVueltas)
					changeDir();
				stepCount = 0;
				dosVueltas = false;
				stateMefMotor = EST_PULSADOR1;
			}

			break;
		default:
			break;
	}
}

uint16_t convAngleToStep(uint16_t angle)
{
	return (angle / PASO);
}

void configPeriodTimerA(float freq)
{
    float T = 0;
    T = 1.0/freq;
    tmrSotfPasos =  (int)(T/0.005);
}

#pragma vector= PORT2_VECTOR
__interrupt void SW_1(void)
{
    P2IFG &= ~BIT2;                 // clean flag
    button4();                      // call routine of the button
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
{
    if (tmrSotfPasos)
        tmrSotfPasos--;
}
