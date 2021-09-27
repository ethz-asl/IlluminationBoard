/*************************************MAIN HEADER*************************************

   @project:  Illuminationboard V2
   @author:   Laurin Heitzer, Michael Riner

   @brief:    software for Illuminationboard V2 Rev1, reads potentiometer and outputs
              pwm according to it.

   @BRD_VERSION:  2.1

 *************************************************************************************/

#include <SPI.h>

#define BRD_VERSION     21 // on V2 rev. 2 & 2.1 the LED_SWITCH is on a non-interrupt pin of the MKRZERO package

#define PWR_EN      29
#define INT_TRIG    2
#define PWM_OUT     4
#define EXT_SIG_EN  3
#define LED_SWITCH  20
#define BRIGHTNESS  A3
#define RES_ADC     8
#define FILTER      100
#define FAN_CTRL    16
#define ADC_CONV    6
#define BTN_TST     11

#define PWM_PER 255

void init_clock_gclk5();
void init_timer_tcc0();
void init_pwm();

void setup() {  // setup begin

  uint16_t ctr = 0;

  pinMode(PWR_EN, OUTPUT);
  pinMode(INT_TRIG, OUTPUT);
  pinMode(PWM_OUT, OUTPUT);
  pinMode(EXT_SIG_EN, OUTPUT);
  pinMode(LED_SWITCH, INPUT);
  pinMode(BTN_TST, INPUT);
  pinMode(FAN_CTRL, OUTPUT);
  pinMode(ADC_CONV, OUTPUT);

  digitalWrite(INT_TRIG, LOW);
  digitalWrite(EXT_SIG_EN, HIGH); // enable internal trigger
  digitalWrite(ADC_CONV, HIGH);   // set ADC Conversion pin to high (SPI transmission starts when CONV changes to low)

#if(BRD_VERSION != 21)
  attachInterrupt(digitalPinToInterrupt(LED_SWITCH), ISR_Switch, CHANGE);
#endif
  digitalWrite(INT_TRIG, digitalRead(LED_SWITCH));

  init_clock_gclk5();
  init_timer_tcc0();
  init_pwm();

  SPI.begin();             // initialize SPI for temperature sensor (ADC)

  Serial1.begin(9600);      // hardware serial
  Serial.begin(9600);       // USB serial

  digitalWrite(PWR_EN, 1);  // enables power on the LEDPCB

} // setup end

#if(BRD_VERSION != 21)
void ISR_Switch() {
  delay(100);
  digitalWrite(INT_TRIG, digitalRead(LED_SWITCH));
}
#endif

static int avg[FILTER];
static int i = 0;
int mv_avg = 0;

#if(BRD_VERSION == 21)
bool switch_new = digitalRead(LED_SWITCH);
#endif

void loop() {

  analogReadResolution(RES_ADC);
  analogWriteResolution(RES_ADC);
  avg[i] = analogRead(BRIGHTNESS);

  if (i < FILTER)
    i++;
  else
    i = 0;

  for (int j = 0; j < FILTER; j++)
    mv_avg += avg[j];
  mv_avg = mv_avg / FILTER;

  if (mv_avg > 255)
    mv_avg = 255;


  Serial.println(mv_avg);

  analogWrite(FAN_CTRL, mv_avg);

  // write ADC average into ccb0 register of tcc0 to set dutycycle of PWM
  REG_TCC0_CCB0 = mv_avg;
  while (TCC0->SYNCBUSY.bit.CCB0);

  SPI.transfer(0b10101010); // spi dummy write for testing purposes

#if(BRD_VERSION == 21)
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
#endif
}

/**
   @function  init_clock_gclk5
   @author    Laurin Heitzer

   @brief     initializes GCLK5 with 8MHz internal oscillator as source

*/
void init_clock_gclk5()
{
  // Set the divisor for GCLK5.
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |  // Set divisor to 1
                    GCLK_GENDIV_ID(5);    // For GCLK5
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Set the clock source, duty cycle, and enable GCLK5
  REG_GCLK_GENCTRL = GCLK_GENCTRL_SRC_OSC8M |    // Set 8MHz source
                     GCLK_GENCTRL_IDC |          // Improve Duty Cycle
                     GCLK_GENCTRL_GENEN |        // Enable GCLK
                     GCLK_GENCTRL_ID(5);         // For GLCK5
  while (GCLK->STATUS.bit.SYNCBUSY);
}

/**
   @function  init_timer_tcc0
   @author    Laurin Heitzer

   @brief     initializes timer TCC0 with GCLK5 as clock source

*/
void init_timer_tcc0()
{
  // Route GLCK5 to TCC0 & TCC1, and enable the clock.
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID_TCC0_TCC1 | // Route GCLK5 to TCC0 & 1
                     GCLK_CLKCTRL_CLKEN |        // Enable the clock
                     GCLK_CLKCTRL_GEN_GCLK5;     // Select GCLK5
  while (GCLK->STATUS.bit.SYNCBUSY);
}

/**
   @function  init_pwm
   @author    Laurin Heitzer

   @brief     initializes PWM and routes it to PB10. PWM frequency is ~31.3kHz

*/
void init_pwm()
{
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  REG_TCC0_PER = PWM_PER;
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1
                    | TCC_CTRLA_ENABLE;     // Requires SYNC on CTRLA
  while ( TCC0->SYNCBUSY.bit.ENABLE
          || TCC0->SYNCBUSY.bit.WAVE
          || TCC0->SYNCBUSY.bit.PER );

  PORT->Group[g_APinDescription[4].ulPort]
  .PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;

  PORT->Group[g_APinDescription[4].ulPort]
  .PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;

  REG_TCC0_CCB0 = 0;    //set dutycycle to 0 at start (LED off)
  while (TCC0->SYNCBUSY.bit.CCB0);
}
