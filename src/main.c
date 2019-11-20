#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "system_tm4c1294.h" // CMSIS-Core
#include "driverleds.h" // device drivers
#include "cmsis_os2.h" // CMSIS-RTOS

// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/tm4c1294ncpdt.h" // dá um monte de warning
#include "system_TM4C1294.h" 
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/qei.h"

#define MAX (10)
#define MAX_POT (4095)
#define MAX_RPM (12000)
#define TICKS_IN_PERIOD (20000)
#define SW1       (GPIO_PIN_0)    //PJ0
#define SW2       (GPIO_PIN_1)    //PJ1
#define Kp (2)
#define Ki (0)
#define Kd (0)
#define LOAD (120000)
#define PPR (18)
#define EDGES (4)

extern void UARTStdioIntHandler(void);

uint32_t PotValue;
uint32_t dc;
uint8_t direction = 0;
uint32_t velocity = 0;
uint32_t setpoint = 0;

uint32_t qei_vel; 
uint32_t qei_dir; 
uint32_t error = 0;
uint32_t prev_error = 0;
uint32_t P = 0;
uint32_t I = 0;
uint32_t D = 0;
int32_t PIDValue;

long double div;
long double quo;
double ratio;

osThreadId_t PIDControl_t, PulseCount_t, Communication_t, ControlInit_t;
uint32_t i = 0;

void UARTInit(void){
  // Enable the GPIO Peripheral used by the UART.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

  // Enable UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

  // Configure GPIO Pins for UART mode.
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  // Initialize the UART for console I/O.
  UARTStdioConfig(0, 9600, SystemCoreClock);
  
  UARTprintf("----------------------------------\n");
  
} // UARTInit

void UART0_Handler(void){
  UARTStdioIntHandler();
} // UART0_Handler

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
  
  ///QEI GPIO

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){};
  
  
  GPIOPinTypeQEI(GPIO_PORTL_BASE, (GPIO_PIN_2 | GPIO_PIN_1));
    
  GPIOPinConfigure(0x000A0406); // GPIO_PL1_PHA0: x000A0406   
  GPIOPinConfigure(0x000A0806);  // GPIO_PL2_PHB0: 0x000A0806
    
    
  ///PWM GPIO (PF2)
  // Enable and wait for the GPIOF peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
  
  // Configure GPIO Port F pin 2 to be used as PWM.//
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
  
  // Enable PWM2 functionality on GPIO Port F pin 2.
  GPIOPinConfigure(GPIO_PF2_M0PWM2);//(0x00050806);   //GPIO_PF2_M0PWM2: 0x00050806
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

void QEIInit(){
  
  //// Enable the QEI0 peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
  
  //// Wait for the QEI0 module to be ready.//
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)){}
  
  
  //// Configure the quadrature encoder to capture edges on both signals .
  //Using a 1000 line encoder at four edges per line, there are 4000 pulses per
  // revolution; therefore set the maximum position to 3999 as the count
  // is zero based.
  
  QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET |QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), EDGES * PPR - 1);  
  QEIPositionSet(QEI0_BASE, 0);
  
  QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, LOAD);
  
  // Enable the quadrature encoder.//
  QEIEnable(QEI0_BASE);
  
  QEIVelocityEnable(QEI0_BASE);

}

void PotRead (void){
  //Grava valor discreto do Pot em PotValue;
  //12 bits de precisao => (0 a 4095)
  
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
								
        direction = 1;  

    }else {
      
      direction = 0;
    
    }
  
  
  GPIOIntClear(GPIO_PORTJ_BASE, SW1|SW2);
  
}

/** Função RunMotor
void RunMotor(){
  if(direction){
    // Girar motor no Sentido Horario
  GPIOPinWrite(GPIO_PORTE_BASE,(GPIO_PIN_1 | GPIO_PIN_0),(GPIO_PIN_0));
  } else {
    // Girar motor no Sentido Anti-Horario
    GPIOPinWrite(GPIO_PORTE_BASE,(GPIO_PIN_1 | GPIO_PIN_0),(GPIO_PIN_1));    
  }
  
  
  PotRead();
  
  //IMPLEMENTAR DIRECAO DO MOTOR
  
  dc = ((float) PotValue / (float) MAX_POT) * (float) TICKS_IN_PERIOD;
  
  // Set the pulse width of PWM2 with dc
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, dc);
  
}
**/

void PIDControl(void *arg){
  uint8_t led = (uint32_t)arg;
  uint8_t state = 0;
  while(1){
    osThreadFlagsWait(0x001, osFlagsWaitAny, osWaitForever);
    state ^= led;
    LEDWrite(led, state);
    
    if(direction){
    // Girar motor no Sentido Horario
    GPIOPinWrite(GPIO_PORTE_BASE,(GPIO_PIN_1 | GPIO_PIN_0),(GPIO_PIN_0));
    } else {
      // Girar motor no Sentido Anti-Horario
      GPIOPinWrite(GPIO_PORTE_BASE,(GPIO_PIN_1 | GPIO_PIN_0),(GPIO_PIN_1));    
    }
    error = velocity - setpoint;
    P = error;
    I = I + error;
    D = error - prev_error;
    PIDValue = (Kp * P) + (Ki * I) + (Kd * D);
    
    prev_error = error;
    
    if(PIDValue > TICKS_IN_PERIOD) {
      PIDValue = TICKS_IN_PERIOD;
    }
    if(PIDValue < 0) {
      PIDValue = 0;
    }
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 10000);
    osThreadFlagsSet(PulseCount_t, 0x010);
  } // while
} // PIDControl

void PulseCount(void *arg){
  uint32_t delay = (uint32_t)arg;
  uint32_t tick;
  
  while(1){
    osThreadFlagsWait(0x0010, osFlagsWaitAny, osWaitForever);
    tick = osKernelGetTickCount();
    QEIPositionGet(QEI0_BASE);
    qei_vel = QEIVelocityGet(QEI0_BASE);
    qei_dir = QEIDirectionGet(QEI0_BASE);
    
    velocity = qei_vel * ratio;
    
    osThreadFlagsSet(Communication_t, 0x100);
    UARTprintf("<Temporiza> enviado\n");
    osDelayUntil(tick + delay);
  } // while
} // PulseCount

void Communication(void *arg){
  UARTprintf("Use o Potenciometro para ajustar o setpoint\n");
//  UARTprintf(" e os botões para a direção");
//  UARTprintf(" - SW1 -> ANTI-HORÁRIO / SW2 -> HORÁRIO\n");
  
  while(1){
    osThreadFlagsWait(0x100, osFlagsWaitAny, osWaitForever);
    PotRead();
    setpoint = PotValue*MAX_RPM/MAX_POT;
    for(i = 0; i< 100000; i++);
    UARTprintf("Setpoint: %i Velocity: %i Direction: %i\n", setpoint, velocity, direction);
    
    osThreadFlagsSet(PIDControl_t, 0x001);
  } // while
} // Communication

void ControlInit(void *arg){
//    UARTprintf("INICIALIZANDO SISTEMA DE CONTROLE...\n");
    osThreadFlagsSet(Communication_t, 0x100);
//    UARTprintf("SISTEMA DE CONTROLE INICIADO\n");
} // ControlInit

void main(void){
  SystemInit();
  LEDInit(LED1);
  UARTInit();
  FPUEnable();
  FPULazyStackingEnable();
  
  GPIOInit();
  PWMInit();
  ADCInit();
  QEIInit();
  IntMasterEnable();
  
  osKernelInitialize();
 
  PIDControl_t = osThreadNew(PIDControl, (void *)LED1, NULL);
  PulseCount_t = osThreadNew(PulseCount, (void *)1000, NULL);
  Communication_t = osThreadNew(Communication, NULL, NULL);
  ControlInit_t = osThreadNew(ControlInit, NULL, NULL);
  
  quo = (SystemCoreClock * 60);
  div = (LOAD * PPR * EDGES);
  ratio = quo / div;
  
  if(osKernelGetState() == osKernelReady) {
    osKernelStart();
  }
  while(1);
  
}