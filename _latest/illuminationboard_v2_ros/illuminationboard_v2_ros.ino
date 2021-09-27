/*************************************MAIN HEADER*************************************
 * 
 * @project:  Illuminationboard V2
 * @author:   Laurin Heitzer, Michael Riner
 * 
 * @brief:    software for Illuminationboard V2 Rev3, reads potentiometer and outputs
 *            pwm according to it. Also controllable via ROSSerial
 *            --> see documentation for more info
 *            
 * @version:  2.0
 * 
 *************************************************************************************/
#define USE_USBCON

#include "Arduino.h"
#include "wiring_private.h"

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include <SPI.h>

#define PWR_EN      29
#define INT_TRIG   	2
#define PWM_MCU     4
#define EXT_SIG_EN  3
#define LED_SWITCH  20
#define BRIGHTNESS 	A3
#define RES_ADC    	8
#define FILTER     	100
#define FAN_CTRL	  16
#define ADC_CONV	  6
#define BTN_TST		  11

#define PWM_PER     255

bool ros_used = false;
bool ros_trigger_used = false;
enum pwm_config {potentiometer, ros_sub} pwm_source;

void init_adc();
void init_clock_gclk5();
void init_timer_tcc0();
void init_pwm();
void pwm_set(uint8_t dutycycle);
void pwm_set_rosserial(const std_msgs::UInt8& msg);
void trigger_set_rosserial(const std_msgs::Bool& msg);
void config_ib_rosserial(const std_msgs::UInt8& msg);

// ros::NodeHandle_<ArduinoHardware, 25, 25, 1024, 1024> nh;
ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt8> pwm_sub("Illuminationboard/pwm", pwm_set_rosserial);
ros::Subscriber<std_msgs::Bool> trigger_sub("Illuminationboard/trigger", trigger_set_rosserial);
ros::Subscriber<std_msgs::UInt8> config_sub("Illuminationboard/config", config_ib_rosserial);

void setup() {

  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(pwm_sub);
  nh.subscribe(config_sub);
  nh.subscribe(trigger_sub);

  pinMode(PWR_EN, OUTPUT);
  pinMode(INT_TRIG, OUTPUT);
  pinMode(PWM_MCU, OUTPUT);
  pinMode(EXT_SIG_EN, OUTPUT);
  pinMode(LED_SWITCH, INPUT);
  pinMode(BTN_TST, INPUT);
  pinMode(FAN_CTRL, OUTPUT);
  pinMode(ADC_CONV, OUTPUT);

  digitalWrite(INT_TRIG, digitalRead(LED_SWITCH));
  
  init_clock_gclk5();
  init_timer_tcc0();
  init_pwm();
  init_adc();

  SPI.begin();             // initialize SPI for temperature sensor (ADC)

//  Serial1.begin(9600);      // hardware serial
//  Serial.begin(9600);       // USB serial

  digitalWrite(PWR_EN, 1);  // enables power on the LEDPCB

  analogReadResolution(RES_ADC);

  ros_used = 0;
  pwm_source = potentiometer;
}


uint16_t adc_result = 0;
static int avg[FILTER];
static int i = 0;
int mv_avg = 0;
bool switch_new = digitalRead(LED_SWITCH);

void loop() {

	nh.spinOnce();
  
  if(pwm_source == potentiometer)   // ADC averaging; could also be done in hardware (averaging feature of ADC)
  {  
    avg[i] = adc_result;

    if (i < FILTER)
        i++;
    else
        i = 0;
    
    for (int j = 0; j < FILTER; j++)
        mv_avg += avg[j];
    mv_avg = mv_avg / FILTER;
    
    nh.spinOnce();

    if (mv_avg > 255)
        mv_avg = 255;

    // analogWrite(FAN_CTRL, mv_avg); //fan no longer used
    // write ADC average into ccb0 register of tcc0 to set dutycycle of PWM
    pwm_set(mv_avg);
  }

  nh.spinOnce();

  if(!ros_used)
  {
  
    switch_new = digitalRead(LED_SWITCH);

    digitalWrite(INT_TRIG, switch_new);
    digitalWrite(EXT_SIG_EN, switch_new);
    if(!switch_new)
    {
      if(digitalRead(BTN_TST))
      {
        digitalWrite(INT_TRIG, HIGH);
        digitalWrite(EXT_SIG_EN, HIGH);
      }
      else
      {
        digitalWrite(INT_TRIG, LOW);
        digitalWrite(EXT_SIG_EN, LOW);
      }
    }
  }

  ros_used = nh.connected();
}

/**
 * @function  init_adc
 * @author    Laurin Heitzer
 * 
 * @brief     initializes ADC in freerun mode (continuous conversion) with interrupt
 * 
*/
void init_adc()
{
	pinPeripheral(A3, PIO_ANALOG);

  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[A3].ulADCChannelNumber;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch
  
  ADC->CTRLB.bit.FREERUN = 0x1;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->CTRLA.bit.ENABLE = 0x1;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->INTFLAG.bit.RESRDY = 0x1;     // Clear ready flag
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->SWTRIG.bit.START = 1;         // Start ADC
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  while (!(ADC->INTFLAG.bit.RESRDY));
  // startTime = micros();
  ADC->INTFLAG.bit.RESRDY = 0x1;     // Discard first reading

  ADC->INTENSET.bit.RESRDY = 1;
  NVIC_EnableIRQ(ADC_IRQn);
}

/**
 * @function  init_clock_gclk5
 * @author    Laurin Heitzer
 * 
 * @brief     initializes GCLK5 with 8MHz internal oscillator as source
 * 
*/
void init_clock_gclk5()
{
    // Set the divisor for GCLK5.
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |  // Set divisor to 1
                    GCLK_GENDIV_ID(5);    // For GCLK5  
  while(GCLK->STATUS.bit.SYNCBUSY); 
  
    // Set the clock source, duty cycle, and enable GCLK5  
  REG_GCLK_GENCTRL = GCLK_GENCTRL_SRC_OSC8M |    // Set 8MHz source
                     GCLK_GENCTRL_IDC |          // Improve Duty Cycle
                     GCLK_GENCTRL_GENEN |        // Enable GCLK
                     GCLK_GENCTRL_ID(5);         // For GLCK5    
  while(GCLK->STATUS.bit.SYNCBUSY);    
}

/**
 * @function  init_timer_tcc0
 * @author    Laurin Heitzer
 * 
 * @brief     initializes timer TCC0 with GCLK5 as clock source
 * 
*/
void init_timer_tcc0()
{
    // Route GLCK5 to TCC0 & TCC1, and enable the clock.
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID_TCC0_TCC1 | // Route GCLK5 to TCC0 & 1
                     GCLK_CLKCTRL_CLKEN |        // Enable the clock
                     GCLK_CLKCTRL_GEN_GCLK5;     // Select GCLK5
  while(GCLK->STATUS.bit.SYNCBUSY);  
}

/**
 * @function  init_pwm
 * @author    Laurin Heitzer
 * 
 * @brief     initializes PWM and routes it to PB10. PWM frequency is ~31.3kHz
 * 
*/
void init_pwm()
{
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  REG_TCC0_PER = PWM_PER;
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1
             | TCC_CTRLA_ENABLE;     // Requires SYNC on CTRLA
  while( TCC0->SYNCBUSY.bit.ENABLE
             || TCC0->SYNCBUSY.bit.WAVE
             || TCC0->SYNCBUSY.bit.PER );  
             
  PORT->Group[g_APinDescription[4].ulPort]
    .PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;
  
  PORT->Group[g_APinDescription[4].ulPort]
    .PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;
  
  REG_TCC0_CCB0 = 0;    //set dutycycle to 0 at start (LED off)
  while(TCC0->SYNCBUSY.bit.CCB0);
}

/**
 * @function  pwm_set
 * @author    Laurin Heitzer
 * 
 * @param			uint8_t dutycycle
 * @brief     sets the pwm dutycycle (range: 0-100)
 * 
*/
void pwm_set(uint8_t dutycycle)
{
  REG_TCC0_CCB0 = dutycycle;
    while(TCC0->SYNCBUSY.bit.CCB0);
}

/**
 * @function  pwm_set_rosserial
 * @author    Laurin Heitzer
 * 
 * @param			std_msgs::UInt8 msg
 * @brief     sets the pwm dutycycle via rosserial (range: 0-100) and disables adc pot pwm
 * 
*/
void pwm_set_rosserial(const std_msgs::UInt8& msg)
{
  ros_used = true;
  pwm_source = ros_sub;

  REG_TCC0_CCB0 = msg.data; //sets pwm dutycycle (range: 0-100)
    while(TCC0->SYNCBUSY.bit.CCB0);
}

/**
 * @function  trigger_set_rosserial
 * @author    Laurin Heitzer
 * 
 * @param			std_msgs::Bool msg
 * @brief     disables the trigger switch and enables ros trigger
 * 
*/
void trigger_set_rosserial(const std_msgs::Bool& msg)
{
  if(!ros_trigger_used)
  {
    ros_trigger_used = true;
  }

  if(msg.data == true) 
    digitalWrite(INT_TRIG, HIGH);
  else if(msg.data == false)
    digitalWrite(INT_TRIG, LOW);
}

/**
 * @function  config_ib_rosserial
 * @author    Laurin Heitzer
 * 
 * @param			std_msgs::UInt8 msg
 * @brief     change configuration of IlluminationBoard; BIT0: External [0] / Internal [1]; BIT1: ros_pwm [0] / potentiometer [1]
 * 
*/
void config_ib_rosserial(const std_msgs::UInt8& msg)
{
  if(msg.data & 0x01)
    digitalWrite(EXT_SIG_EN, HIGH);
  else if(!(msg.data & 0x01))
    digitalWrite(EXT_SIG_EN, LOW);
  
  if(msg.data & 0x02)
    pwm_source = potentiometer;
  else if(!(msg.data & 0x02))
    pwm_source = ros_sub;
}

/**
 * @function  ADC_Handler
 * @author    Laurin Heitzer
 * 
 * @brief     ADC interrupt handler, clears interrupt flag and reads adc result
 * 
*/
void ADC_Handler() 
{
	ADC->INTFLAG.bit.RESRDY = 0x01; // Clear ready flag

	adc_result = ADC->RESULT.reg;
}