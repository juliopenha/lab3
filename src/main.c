#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/tm4c1294ncpdt.h"
#include "system_TM4C1294.h" 
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"

#define MAX (10)
#define PRECISAO (4095)
#define TICKS_IN_PERIOD (20000)
#define SW1       (GPIO_PIN_0)    //PJ0
#define SW2       (GPIO_PIN_1)    //PJ1



uint32_t PotValue;
uint32_t dc;
uint8_t horario = 0;


void GPIOInit(){
    
  ///Buttons GPIO

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Habilita GPIO J (push-button SW1 = PJ0, push-button SW2 = PJ1)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)); // Aguarda final da habilitação
  
  // Configura os dois pinos para leitura do estado das chaves SW1 e SW2
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, SW1 | SW2);
  
  // Configura a força para 2 mAe resistor fraco de pull-up
  GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0| GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  
  // Configura o PF4 com interrupção na borda de descida 
  GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0| GPIO_PIN_1, GPIO_FALLING_EDGE);
  // Habilita a interrupção do pino PF4
  GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0| GPIO_PIN_1);
  // Habilita a interrupção do GPIOJ
  IntEnable(INT_GPIOJ_TM4C129);
  
  
  ///MotorDir GPIO (PE0 e PE1) e Pot Setpoint GPIO (PE4)  
  // Enable and wait for the GPIOE peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){};
  
  // Set pins 0 and 1 as output, SW controlled
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  
  // Set pins 4 as analog input and adc
  //GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);    
  //GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA ,GPIO_PIN_TYPE_ANALOG);
  
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);
  
  ///PWM GPIO (PF2)
  // Enable and wait for the GPIOF peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
  
  // Configure GPIO Port F pin 2 to be used as PWM.//
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
  
  // Enable PWM2 functionality on GPIO Port F pin 2.
  GPIOPinConfigure(0x00050806);   //GPIO_PF2_M0PWM2: 0x00050806
 
}

//initialize  the  PWM2  with  a  1Khz  KHzfrequency, and with a 0% duty cycle
void PWMInit (void){

  // The TM4C1294NCPDT microcontroller contains *one* PWM module, 
  // with four PWM generator blocks and a control block, for a total of 8 PWM outputs.
  // Each PWM generator block produces two PWM signals that share the same timer and frequency
  // PF2_M0PWM2 : Motion Control Module 0 PWM 2. This signal is controlled by Module 0 PWM Generator 1  
  
  // Enable the PWM0 peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  
  // Wait for the PWM0 module to be ready.
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)){}
  
  // Configure the PWM generator for count down mode with immediate updates
  // to the parameters. PWM_GEN_MODE_DBG_RUN permite ao contador continuar mesmo no debug
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN | PWM_GEN_MODE_NO_SYNC);

 
  // Set the period.  
  // 20MHz => 20.000.000 ticks every second or 1 tick every 50 nanosecond
  // 1Khz => x microseconds
  // How many clock ticks in 20 mcsec? = 20e-6 / 50e-9 = 400
  // Or: 20Mhz / 1Khz = 20000 ticks = TICKS_IN_PERIOD
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, TICKS_IN_PERIOD);
  
  // Set the pulse width of PWM2 for a 0% duty cycle.
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
  
  // Start the timers in generator 1.
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  
  // Enable the outputs.//
  PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT), true);

}

void ADCInit(){  
  
  //Enable and Wait the ADC0 module.  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){}
  
  //// Enable the first sample sequencer to capture the value of channel 0 when
  // the processor trigger occurs.//
  ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH9);
  ADCSequenceEnable(ADC0_BASE, 0);
 
  // Clear the interrupt status flag.
  //ADCIntClear(ADC0_BASE, 0);
  
}

//Grava valor discreto do Pot em PotValue;
//12 bits de precisao => (0 a 4095)
void PotRead (void){  
  //// Trigger the sample sequence.//
  ADCProcessorTrigger(ADC0_BASE, 0);
  
  //// Wait until the sample sequence has completed.//
  while(!ADCIntStatus(ADC0_BASE, 0, false)){}
  
  // Clear the ADC interrupt flag.
  //ADCIntClear(ADC0_BASE, 3);
  
  //// Read the value from the ADC.//
  ADCSequenceDataGet(ADC0_BASE, 0, &PotValue);
  
  // Delay 250ms
  //SysCtlDelay(SysCtlClockGet() / 12);
  
}

void GPIOJ_Handler(void){

     // 1. Check the PIN, make sure that the IRQ occurred from the PIN you defined
    if ((GPIO_PORTJ_AHB_RIS_R & 0x01) == 0x00) {
        // The code for IRQ
								
        horario = 1;  
  
			
			
    }else {
      
      horario = 0;
    
    }
  
  
  GPIOIntClear(GPIO_PORTJ_BASE, SW1|SW2);
  
}

void RunMotor(){
  
  
  if(horario){
    // Girar motor no Sentido Horario
  GPIOPinWrite(GPIO_PORTE_BASE,(GPIO_PIN_1 | GPIO_PIN_0),(GPIO_PIN_0));
  } else {
    // Girar motor no Sentido Anti-Horario
    GPIOPinWrite(GPIO_PORTE_BASE,(GPIO_PIN_1 | GPIO_PIN_0),(GPIO_PIN_1));    
  }
  
  
  PotRead();
  
  //IMPLEMENTAR DIRECAO DO MOTOR
  
  dc = ((float) PotValue / (float) PRECISAO) * (float) TICKS_IN_PERIOD;
  
  // Set the pulse width of PWM2 with dc
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, dc);
  
}



void main(void){
    
  FPUEnable();
  FPULazyStackingEnable();
  
  GPIOInit();
  PWMInit();
  ADCInit();
  IntMasterEnable();
  
  while(1){      
      
    RunMotor();
    
  }
  
}