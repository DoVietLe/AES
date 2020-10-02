#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.c"

#define BAUD_RATE 115200
#define GPIO_PA0_U0RX 0x00000001
#define GPIO_PA1_U0TX 0x00000401
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
#define BUTTON GPIO_PIN_4

// Reserves memory to move the temperature FIFO into.
uint32_t temperatureFIFO[4];
// Variables to calculate the temperature with.
volatile uint32_t tAverage, tCelsius, tFahrenheit;
// Keeps track of the LED status.
volatile bool ledOn = false;

int main(void) {
    uint32_t loadVal;

    // Calculates the cycle values for a 0.5s delay.
    loadVal = (SysCtlClockGet() / 2);

    // Sets up the system clock.
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Enables peripherals.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Sets up peripherals.
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(WTIMER0_BASE, TIMER_A, loadVal);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, BUTTON);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Sets up interrupts.
    IntEnable(INT_WTIMER0A);
    TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_GPIOF);
    GPIOIntTypeSet(GPIO_PORTF_BASE, BUTTON, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
    IntMasterEnable();

    // Enables stuff.
    TimerEnable(WTIMER0_BASE, TIMER_A);
    ADCSequenceEnable(ADC0_BASE, 2);

    UARTprintf("Starting...");
    while (1)
    {
    }
}

void timerhandler(void) {
    // Clears interrupt flag.
    TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Starts ADC conversion.
    ADCIntClear(ADC0_BASE, 2);
    ADCProcessorTrigger(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, temperatureFIFO);

    tAverage = (temperatureFIFO[0] + temperatureFIFO[1] + temperatureFIFO[2] + temperatureFIFO[3] + 2)/4;
    tCelsius = (1475 - ((2250*tAverage))/4096)/10;
    tFahrenheit = ((tCelsius*9) + 160)/5;

    UARTprintf("ADC: %3d\t", tAverage);
    UARTprintf("Celsius: %3dC\t", tCelsius);
    UARTprintf("Fahrenheit: %3dF", tFahrenheit);
    UARTprintf("\n\r");
}

void buttonpresshandler() {
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
    ledOn = !ledOn;
    if (ledOn) {
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED|BLUE_LED|GREEN_LED);
    } else {
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, 0);
    }
}

