# Control-PID-para-Motor-de-CD
control pid para motor de cd 
/*
 * File:   principal.c
 * Author: josue
 *
 * Created on 6 de mayo de 2019, 12:45 AM
 */

// PIC16F886 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#define _XTAL_FREQ 4000000 


int inicia_ADC();
int convercion_ADC(int canal);

//**************** DECLARACION DE VARIABLES PARA PID *********************//
    float Ref, Y, e, e_1, e_2, u, u_1=0;
    float T, kp, ti, td, q0, q1, q2;
    float k, tao, theta;

void main(void) {
    
    TRISAbits.TRISA4=1;     //entrada timer 0
    OPTION_REG =0b11101000;
    inicia_ADC();
    TRISCbits.TRISC1=0;     //salida PWM
    T2CON=0b00000110;
    CCP2CON=0b00001100;
    PR2=250;

   //*************************************************************************//
   //*****************   SINTONIA POR ZIEGLER y NICHOLS    *******************//
   //*************************************************************************//
   k=0.9;
   tao=0.85;
   theta=0.15;
   //*************************************************************************//
   kp=(1.2*tao)/(k*theta);
   ti=2*theta;
   td=0.5*theta;
   //*************************************************************************//
   
   // Calculo do controle PID digital
   
   T=0.1;
   
   q0=kp*(1+T/(2*ti)+td/T);
   q1=-kp*(1-T/(2*ti)+(2*td)/T);
   q2=(kp*td)/T;
   
   while (1){
   
       TMR0=0x00;
       __delay_ms(100);
       Y=(TMR0*10)/8;
       Y=2*3.1416*Y;
       
       Ref=convercion_ADC(0);
       
       PID();
       
       
       
   }
    
}

int PID(){

e=1*(Ref-Y);
    // Controle PID
      u = u_1 + q0*e + q1*e_1 + q2*e_2; //Ley del controlador PID discreto
    
    if (u >= 5000.0)        //Saturo la accion de control 'uT' en un tope maximo y minimo
     u = 5000.0;
    
    if (u <= 0.0)
     u = 0.0;
     
     
     //Retorno a los valores reales
     e_2=e_1;
     e_1=e;
     u_1=u;
     
     CCPR2L=u;
}
