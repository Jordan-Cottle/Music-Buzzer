#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"

#define ADC0_PERIPH SYSCTL_PERIPH_ADC0
#define ADC0 ADC0_BASE

#define ADC1_PERIPH SYSCTL_PERIPH_ADC1
#define ADC1 ADC1_BASE

#define ADC_SEQUENCER 3
#define ADC_CHANNEL ADC_CTL_CH4

#define MAX_ADC_READ 4095
#define MIN_ADC_READ 0

#define TIMER_PERIPH SYSCTL_PERIPH_TIMER0
#define TIMER_BASE TIMER0_BASE

#define BUZZER_PERIPH SYSCTL_PERIPH_GPIOF
#define BUZZER_PORT GPIO_PORTF_BASE
#define BUZZER_PIN GPIO_PIN_2

#define BUTTON_PERIPH SYSCTL_PERIPH_GPIOF
#define BUTTON_PORT GPIO_PORTF_BASE
#define BUTTON_PIN GPIO_PIN_4

#define STICK_CLICK_PERIPH SYSCTL_PERIPH_GPIOE
#define STICK_CLICK_PORT GPIO_PORTE_BASE
#define STICK_CLICK_PIN GPIO_PIN_4

#define VERTICAL_AXIS_PERIPH SYSCTL_PERIPH_GPIOD
#define VERTICAL_AXIS_PORT GPIO_PORTD_BASE
#define VERTICAL_AXIS_PIN GPIO_PIN_3

#define HORIZONTAL_AXIS_PERIPH SYSCTL_PERIPH_GPIOB
#define HORIZONTAL_AXIS_PORT GPIO_PORTB_BASE
#define HORIZONTAL_AXIS_PIN GPIO_PIN_5

#define DEBOUNCE_DELAY 5000
#define INPUT_DELAY 64

// prototypes
void toggleSound();
void updateFrequency();
void setPWMPeriod(uint32_t);
uint32_t readADC(uint32_t, uint32_t);
void updateInputDelay();

// Initialize a port for use
void initPeriph(uint32_t port){
    SysCtlPeripheralEnable(port);
    while(!SysCtlPeripheralReady(port));
}

void initInput(uint32_t periph, uint32_t port, uint8_t pin){
    // Enable the GPIO port for use
    initPeriph(periph);

    // Configure pin as input
    GPIOPinTypeGPIOInput(port, pin);
}

void initOutput(uint32_t periph, uint32_t port, uint8_t pin){
    // Enable the GPIO port for use
    initPeriph(periph);

    // Configure pin as output
    GPIOPinTypeGPIOOutput(port, pin);
}

// configure a periodic timer to trigger an interrupt on a regular basis
void initPeriodicTimer(uint32_t timerPeriph, uint32_t timerBase, uint32_t count, void (*interuptHandler)(void)){
    // enable Timer
    initPeriph(timerPeriph);

    // set clock source
    TimerClockSourceSet(timerBase, TIMER_CLOCK_SYSTEM);

    // configure timer type
    TimerConfigure(timerBase, TIMER_CFG_A_PERIODIC);

    // set initial value
    TimerLoadSet(timerBase, TIMER_A, count);

    TimerIntRegister(timerBase, TIMER_A, interuptHandler);
    TimerIntEnable(timerBase, TIMER_TIMA_TIMEOUT);

    TimerEnable(timerBase, TIMER_A);
}

void initButtonInterupt(uint32_t buttonPeriph, uint32_t buttonPort, uint8_t buttonPin, void (*interuptHandler)(void)){
    initPeriph(buttonPeriph);

    initInput(buttonPeriph, buttonPort, buttonPin);
    // configure pull up resistor
    GPIOPadConfigSet(buttonPort, buttonPin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

   // set interrupt
   GPIOIntTypeSet(buttonPort, buttonPin, GPIO_FALLING_EDGE);

   GPIOIntRegister(buttonPort, interuptHandler);
   GPIOIntEnable(buttonPort, buttonPin);
}

// configure PWM to output to a specific pin
void initPWMModule(uint32_t outputPort, uint8_t outputPin){
   // Configure PWM Clock to match system
   SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // enable PWM module connected to port f pins
   SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

   //Configure PF2 Pin as PWM
   GPIOPinConfigure(GPIO_PF2_M1PWM6);
   GPIOPinTypePWM(outputPort, outputPin);

   //Configure PWM Options
   //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
   PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

   //Set the Period (expressed in clock ticks)
   PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 0);

   //Set PWM duty-50% (Period /2)
   PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 0);

   // Enable the PWM generator
   PWMGenEnable(PWM1_BASE, PWM_GEN_3);

   // Turn on the Output pins
   PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
}

// Set up an adc to read from a specific pin
void initADC(uint32_t adcPeriph, uint32_t adcBase, uint32_t sequencer, uint32_t channel, uint32_t readPeriph, uint32_t readPort, uint8_t readPin){
    // Initialize ports for use
    initPeriph(adcPeriph);
    initPeriph(readPeriph);

    // configure pin for adc use
    GPIOPinTypeADC(readPort, readPin);

    // Configure Sequencer
    ADCSequenceDisable(adcBase, sequencer);
    ADCSequenceConfigure(adcBase, sequencer, ADC_TRIGGER_ALWAYS, 0);

    ADCSequenceStepConfigure(adcBase, sequencer, 0, channel | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(adcBase, sequencer);
}


uint32_t inputDelay = 5;
void timerInterruptHandler(){
    static const uint32_t delays [6] = {0, 64, 128, 256, 512, 1024};
    static uint32_t ticks = 0;

    uint32_t delay = delays[inputDelay];
    TimerIntClear(TIMER_BASE, TIMER_TIMA_TIMEOUT);
    if (ticks >= delay){
        updateFrequency();
        ticks = 0;
    }else{
        ticks++;
    }

}

void stickInterruptHandler(){
    SysCtlDelay(DEBOUNCE_DELAY);
    if(!GPIOPinRead(STICK_CLICK_PORT, STICK_CLICK_PIN)){
        toggleSound();
    }
    GPIOIntClear(STICK_CLICK_PORT, 0xFF); // clear interrupt flag
}

void buttonInterruptHandler(){
    SysCtlDelay(DEBOUNCE_DELAY);
    if(!GPIOPinRead(BUTTON_PORT, BUTTON_PIN)){
        updateInputDelay();
    }
    GPIOIntClear(BUTTON_PORT, 0xFF); // clear interrupt flag
}

void init(){
    // configure clock
    SysCtlClockSet(SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initOutput(BUZZER_PERIPH, BUZZER_PORT, BUZZER_PIN);
    initButtonInterupt(STICK_CLICK_PERIPH, STICK_CLICK_PORT, STICK_CLICK_PIN, stickInterruptHandler);
    initButtonInterupt(BUTTON_PERIPH, BUTTON_PORT, BUTTON_PIN, buttonInterruptHandler);
    initPWMModule(BUZZER_PORT, BUZZER_PIN);
    initPeriodicTimer(TIMER_PERIPH, TIMER_BASE, 0xFFFF, timerInterruptHandler);

    initADC(ADC0_PERIPH, ADC0, ADC_SEQUENCER, ADC_CHANNEL,
            HORIZONTAL_AXIS_PERIPH, HORIZONTAL_AXIS_PORT, HORIZONTAL_AXIS_PIN);

    initADC(ADC1_PERIPH, ADC1, ADC_SEQUENCER, ADC_CHANNEL,
            VERTICAL_AXIS_PERIPH, VERTICAL_AXIS_PORT, VERTICAL_AXIS_PIN);
}

bool play = true;
void toggleSound(){
    play = !play;
}

void updateInputDelay(){
    if(inputDelay == 0){
        inputDelay = 5;
    }else{
        inputDelay--;
    }
}

uint32_t frequency = 0;
void updateFrequency(){
    uint32_t horizontalValue = readADC(ADC0, ADC_SEQUENCER);
    uint32_t verticalValue = readADC(ADC1, ADC_SEQUENCER);
    frequency = ((horizontalValue/100) * (verticalValue/100))*50 + 40000;
}

void setPWMPeriod(uint32_t period){
    //Set the Period (expressed in clock ticks)
       PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, period);

       //Set PWM duty-50% (Period /2)
       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, period/2);
}

uint32_t readADC(uint32_t adcBase, uint32_t sequencer){
    if(ADCIntStatus(adcBase, sequencer, false)){
        uint32_t value;
        ADCSequenceDataGet(adcBase, sequencer, &value);
        return value;
    }
    return 0;
}


int main(void)
{
    init();

    while(1)
    {
        if(play){
            setPWMPeriod(frequency);
        }else{
            setPWMPeriod(0);
        }
    }
}
