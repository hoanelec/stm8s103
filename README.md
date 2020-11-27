//
//  This program shows how you can Generate a single pulse which
//  lasts for 30 uS by using Timer 2 and Port D, Pin 4. STM8S
//  microcontroller.
//
//  This software is provided under the CC BY-SA 3.0 licence.  A
//  copy of this licence can be found at:
//
//  http://creativecommons.org/licenses/by-sa/3.0/legalcode
//
#include <stdio.h>
#include <stdlib.h>
#if defined DISCOVERY
#include <iostm8S105c6.h>
#elif defined PROTOMODULE
#include <iostm8s103k3.h>
#else
#include <iostm8s103f3.h>
#endif
#include <intrinsics.h>
#define GPIOA_BaseAddress       0x5000
#define GPIOB_BaseAddress       0x5005
#define GPIOC_BaseAddress       0x500A
#define GPIOD_BaseAddress       0x500F
#define GPIOE_BaseAddress       0x5014
#define GPIOF_BaseAddress       0x5019
#define GPIOG_BaseAddress       0x501E
#define GPIOH_BaseAddress       0x5023
#define GPIOI_BaseAddress       0x5028
#define F_CPU 16000000UL
#define dly_const (F_CPU / 16000000.0F)
#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21
#define _NULL_VAL 0
#define TIMEOUT 1000
#define __IO  volatile
#define uint8_t unsigned char
char _mType = DHT11;
volatile int i = 0;
volatile unsigned long millis = 0;
uint8_t temp = 0;
uint8_t dem = 0;
uint8_t *counter = &dem;
uint8_t cookedDHTData[5];
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus, BitAction;
void UARTPrintString(char *message);
typedef enum
{
  GPIO_MODE_IN_FL_NO_IT      = (uint8_t)0x00,  /*!< Input floating, no external interrupt */
  GPIO_MODE_IN_PU_NO_IT      = (uint8_t)0x40,  /*!< Input pull-up, no external interrupt */
  GPIO_MODE_IN_FL_IT         = (uint8_t)0x20,  /*!< Input floating, external interrupt */
  GPIO_MODE_IN_PU_IT         = (uint8_t)0x60,  /*!< Input pull-up, external interrupt */
  GPIO_MODE_OUT_OD_LOW_FAST  = (uint8_t)0xA0,  /*!< Output open-drain, low level, 10MHz */
  GPIO_MODE_OUT_PP_LOW_FAST  = (uint8_t)0xE0,  /*!< Output push-pull, low level, 10MHz */
  GPIO_MODE_OUT_OD_LOW_SLOW  = (uint8_t)0x80,  /*!< Output open-drain, low level, 2MHz */
  GPIO_MODE_OUT_PP_LOW_SLOW  = (uint8_t)0xC0,  /*!< Output push-pull, low level, 2MHz */
  GPIO_MODE_OUT_OD_HIZ_FAST  = (uint8_t)0xB0,  /*!< Output open-drain, high-impedance level,10MHz */
  GPIO_MODE_OUT_PP_HIGH_FAST = (uint8_t)0xF0,  /*!< Output push-pull, high level, 10MHz */
  GPIO_MODE_OUT_OD_HIZ_SLOW  = (uint8_t)0x90,  /*!< Output open-drain, high-impedance level, 2MHz */
  GPIO_MODE_OUT_PP_HIGH_SLOW = (uint8_t)0xD0   /*!< Output push-pull, high level, 2MHz */
}GPIO_Mode_TypeDef;

/**
* @brief  Definition of the GPIO pins. Used by the @ref GPIOx_Init function in
* order to select the pins to be initialized.
*/

typedef enum
{
  GPIO_PIN_0    = ((uint8_t)0x01),  /*!< Pin 0 selected */
  GPIO_PIN_1    = ((uint8_t)0x02),  /*!< Pin 1 selected */
  GPIO_PIN_2    = ((uint8_t)0x04),  /*!< Pin 2 selected */
  GPIO_PIN_3    = ((uint8_t)0x08),   /*!< Pin 3 selected */
  GPIO_PIN_4    = ((uint8_t)0x10),  /*!< Pin 4 selected */
  GPIO_PIN_5    = ((uint8_t)0x20),  /*!< Pin 5 selected */
  GPIO_PIN_6    = ((uint8_t)0x40),  /*!< Pin 6 selected */
  GPIO_PIN_7    = ((uint8_t)0x80),  /*!< Pin 7 selected */
  GPIO_PIN_LNIB = ((uint8_t)0x0F),  /*!< Low nibble pins selected */
  GPIO_PIN_HNIB = ((uint8_t)0xF0),  /*!< High nibble pins selected */
  GPIO_PIN_ALL  = ((uint8_t)0xFF)   /*!< All pins selected */
}GPIO_Pin_TypeDef;
typedef struct GPIO_struct
{
  __IO uint8_t ODR; /*!< Output Data Register */
  __IO uint8_t IDR; /*!< Input Data Register */
  __IO uint8_t DDR; /*!< Data Direction Register */
  __IO uint8_t CR1; /*!< Configuration Register 1 */
  __IO uint8_t CR2; /*!< Configuration Register 2 */
}
GPIO_TypeDef;
uint8_t GPIO_ReadInput(GPIO_TypeDef* GPIOx)
{
  return ((uint8_t)GPIOx->IDR);
}
BitStatus GPIOx_ReadInputPin(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin)
{
  return (BitStatus)(((GPIOx->IDR & (uint8_t)GPIO_Pin))==(uint8_t)GPIO_Pin?1:0);
}
void GPIO_WriteOutput(GPIO_TypeDef* GPIOx, uint8_t level)
{
  GPIOx->ODR =level;
}
void GPIOx_Init(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, GPIO_Mode_TypeDef GPIO_Mode)
{
  /*----------------------*/
  /* Check the parameters */
  /*----------------------*/
  
  // assert_param(IS_GPIO_MODE_OK(GPIO_Mode));
  //assert_param(IS_GPIO_PIN_OK(GPIO_Pin));
  
  /* Reset corresponding bit to GPIO_Pin in CR2 register */
  GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
  
  /*-----------------------------*/
  /* Input/Output mode selection */
  /*-----------------------------*/
  
  if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x80) != (uint8_t)0x00) /* Output mode */
  {
    if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x10) != (uint8_t)0x00) /* High level */
    {
      GPIOx->ODR |= (uint8_t)GPIO_Pin;
    } 
    else /* Low level */
    {
      GPIOx->ODR &= (uint8_t)(~(GPIO_Pin));
    }
    /* Set Output mode */
    GPIOx->DDR |= (uint8_t)GPIO_Pin;
  } 
  else /* Input mode */
  {
    /* Set Input mode */
    GPIOx->DDR &= (uint8_t)(~(GPIO_Pin));
  }
  
  /*------------------------------------------------------------------------*/
  /* Pull-Up/Float (Input) or Push-Pull/Open-Drain (Output) modes selection */
  /*------------------------------------------------------------------------*/
  
  if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x40) != (uint8_t)0x00) /* Pull-Up or Push-Pull */
  {
    GPIOx->CR1 |= (uint8_t)GPIO_Pin;
  } 
  else /* Float or Open-Drain */
  {
    GPIOx->CR1 &= (uint8_t)(~(GPIO_Pin));
  }
  
  /*-----------------------------------------------------*/
  /* Interrupt (Input) or Slope (Output) modes selection */
  /*-----------------------------------------------------*/
  
  if ((((uint8_t)(GPIO_Mode)) & (uint8_t)0x20) != (uint8_t)0x00) /* Interrupt or Slow slope */
  {
    GPIOx->CR2 |= (uint8_t)GPIO_Pin;
  } 
  else /* No external interrupt or No slope control */
  {
    GPIOx->CR2 &= (uint8_t)(~(GPIO_Pin));
  }
}
//
//  Timer 2 Overflow handler.
//
volatile char flag = 0;
volatile char flagTim4 = 0;
#pragma vector = TIM2_OVR_UIF_vector
__interrupt void TIM2_UPD_OVF_IRQHandler(void)
{
  TIM2_CR1_CEN = 0;
  flag = 1;
  TIM2_SR1_UIF = 0;               //  Reset the interrupt otherwise it will fire again straight away.
}
//--------------------------------------------------------------------------------
//
//  Timer 1 Overflow handler.
//
volatile uint8_t flagtimer1 = 0;
#pragma vector = TIM1_OVR_UIF_vector
__interrupt void TIM1_UPD_OVF_IRQHandler(void)
{
  //          PB_ODR_ODR4 = !PB_ODR_ODR4; //0;                //  Signal to the user that Timer 1 has stopped.
  TIM1_SR1 &= ~(1<<0);//Clear interruption mark
  millis++;
  if(millis == 0xFFFFFFFF)//1ms*1000=1s
  {
    millis = 0;
    //  PB_ODR ^= 1<<5;//LED lamp flipped once in 1s
  }
  //TIM1_CR1_CEN = 0;               //  Stop Timer 1.
  //  flagtimer1 = 1;
  //  TIM1_SR1_UIF = 0;               //  Reset the interrupt otherwise it will fire again straight away.
  //  millis++;
}
//#pragma vector = TIM4_OVR_UIF_vector
//__interrupt void TIM4_UPD_OVF_IRQHandler(void) {
//  //PB_ODR_ODR4 = !PB_ODR_ODR4;
//  //  TIM4_CR1_CEN = 0;
//  //flagTim4 = 1;
//  TIM4_SR_UIF = 0;
//}
//--------------------------------------------------------------------------------
//
//  Set up Timer 1, channel 3 to output a single pulse lasting 240 uS.
//

//void reLoadTimer1()
//{
//  i = 0;
//  TIM1_CR1_CEN = 0;               //  Stop Timer 1.
//  TIM1_SR1_UIF = 0;               //  Reset the interrupt otherwise it will fire again straight away.
//  TIM1_CR1_CEN = 1;               //  Stop Timer 1.
//}
//
//  Setup the system clock to run at 16MHz using the internal oscillator.
//
void InitialiseSystemClock()
{
  CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
  CLK_ICKR_HSIEN = 1;                 //  Enable the HSI.
  CLK_ECKR = 0;                       //  Disable the external clock.
  while (CLK_ICKR_HSIRDY == 0);       //  Wait for the HSI to be ready for use.
  CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
  CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
  CLK_PCKENR2 = 0xff;                 //  Ditto.
  CLK_CCOR = 0;                       //  Turn off CCO.
  CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
  CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
  CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
  CLK_SWCR = 0;                       //  Reset the clock switch control register.
  CLK_SWCR_SWEN = 1;                  //  Enable switching.
  while (CLK_SWCR_SWBSY != 0);        //  Pause while the clock switch is busy.
}

//
//  Setup the port used to signal to the outside world that a timer even has
//  been generated.
//
void SetupOutputPorts()
{
  // PB_ODR = 0;             //  All pins are turned off.
  PB_DDR_DDR5 = 1;        //  Port D, pin 4 is used as a signal.
  PB_CR1_C15 = 1;         //  Port D, pin 4 is Push-Pull
  PB_CR2_C25 = 1;         //  Port D, Pin 4 is generating a pulse under 2 MHz.
  PB_ODR_ODR5 = 1;
  
  PD_DDR_DDR3 = 1;        //  Port D, pin 4 is used as a signal.
  PD_CR1_C13 = 0;        //  Port D, pin 4 is Open-Drain
  PD_CR2_C23 = 1;         //  Port D, Pin 4 is generating a pulse under 2 MHz.
  PD_ODR_ODR3 = 0;
  //  PD4 is used to indicate the firing of the update/overflow event for Timer 1
  PB_DDR_DDR4 = 1;
  PB_CR1_C14 = 1;
  PB_CR2_C24 = 1;
  //  PB_ODR_ODR4 = 0;
  
}
//void delay_us(unsigned long loops)
//{
//  //register unsigned int loops = (dly_const * value) ;
//
//  // while(loops)
//  // {
//  //asm ("nop");
//  //loops--;
//  //  }
//  while (loops)
//  {
//    TIM4_CR1_CEN = 1;
//    while (flagTim4);
//    flagTim4 = 0;
//    loops--;
//  }
//}
//
//  Setup Timer 2 to generate an interrupt every 480 clock ticks (30 uS).
//
void delay(unsigned long ms)
{
  while (ms)
  {
    TIM2_CR1_CEN = 1;
    while (!flag);
    flag = 0;
    ms--;
  }
}
//
//  Reset Timer 2 to a known state.
//
void InitialiseTimer2()
{
  TIM2_CR1 = 0;               // Turn everything TIM2 related off.
  TIM2_IER = 0;
  TIM2_SR2 = 0;
  TIM2_CCER1 = 0;
  TIM2_CCER2 = 0;
  TIM2_CCER1 = 0;
  TIM2_CCER2 = 0;
  TIM2_CCMR1 = 0;
  TIM2_CCMR2 = 0;
  TIM2_CCMR3 = 0;
  TIM2_CNTRH = 0;
  TIM2_CNTRL = 0;
  TIM2_PSCR = 0;
  TIM2_ARRH  = 0;
  TIM2_ARRL  = 0;
  TIM2_CCR1H = 0;
  TIM2_CCR1L = 0;
  TIM2_CCR2H = 0;
  TIM2_CCR2L = 0;
  TIM2_CCR3H = 0;
  TIM2_CCR3L = 0;
  TIM2_SR1 = 0;
}
//void SetupTimer4_1us() {
//  TIM4_CR1 = 0x81; // thi?t l?p timer4 ch? d? n?p l?i
//  TIM4_PSCR = 4; // h? s? chia là 4
//  TIM4_ARR = 100; // TOP counter là 100
//  // = 0;       //  Up counter.
//  TIM4_IER_UIE = 1;
//  
//  //  TIM4_IER = 0x01; // cho phép ng?t
//}
// TIM4_PSCR = 0x01;
//  TIM4_ARR  = 0xFF;
// TIM4_CR1  = 0;
// TIM4_CR1  |= (1<<2);
// TIM4_EGR  = 1;
//  TIM4_CNTR = 0x00;
//  TIM4_IER &= ~(1<<0);  //Disable interrupt
void SetupTimer1()
{
  //  TIM1_ARRH = 0x3e;       //  Reload counter = 960
  //  TIM1_ARRL = 0x80;
  //  TIM1_PSCRH = 0x00;         //  Prescalar = 0 (i.e. 1)
  //  TIM1_PSCRL = 0x00;
  //  //
  //  //  Select 0 for up counting or 1 for down counting.
  //  //
  //  TIM1_CR1_DIR = 0;       //  Up counter.
  //  TIM1_IER_UIE = 1;       //  Turn interrupts on.
  //  TIM1_CR1_CEN = 1;
  TIM1_PSCRH=0;//Be sure to write high eight first
  TIM1_PSCRL=0;//1 frequency division, timer clock equals system clock = 16m
  
  TIM1_ARRH=0X3e;//We must first install eight high positions and then eight low positions.
  TIM1_ARRL=0X80;//1ms reload value 16000,
  
  TIM1_CNTRH=0;
  TIM1_CNTRL=0;//It is necessary to clear down the counter
  
  TIM1_IER |= 1<<0;//Enable tim1 update interrupt
  TIM1_SR1 |= 1<<0;//Clear tim1 update interrupt flag
  
  TIM1_CR1 |= 1<<7;//Allow reassembly to enable timer
  TIM1_CR1 |= 1<<4;//Select Downward Counting Mode
  TIM1_CR1 |= 1<<0;//Enabling counter
}
void SetupTimer2_1ms()
{
  TIM2_PSCR = 0x00;       //  Prescaler = 1.
  TIM2_ARRH = 0x3e;       //  High byte of 16000.
  TIM2_ARRL = 0x80;       //  Low byte of 16000.
  TIM2_IER_UIE = 1;
}
//void SetupTimer2_20us()
//{
//  TIM2_PSCR = 0x00;
//  TIM2_ARRH = 0x00;
//  TIM2_ARRL = 0xf0;
//  TIM2_IER_UIE = 1;
//}
//1.3us
void delay_us(unsigned long microseconds) {
  TIM4_PSCR = 0x00; // Set prescaler 
  // Set count to approximately 1uS (clock/microseconds in 1 second)
  // The -1 adjusts for other runtime costs of this function
  TIM4_ARR = ((16000000L)/2000000) ;
  TIM4_CR1 = 1; // Enable counter
  for (; microseconds > 1; --microseconds) {
    while (TIM4_SR_UIF == 0);
    // Clear overflow flag
    TIM4_SR_UIF=0;
  }
}
int detectPulse(uint8_t _mincycles , uint8_t _maxcycles, GPIO_TypeDef* GPIOx ,GPIO_Pin_TypeDef GPIO_Pin_x, BitStatus _level)
{
  //  uint8_t _cnt = 0;//reset
  temp = dem;
  dem = 0;
  if(GPIOx_ReadInputPin(GPIOx,GPIO_Pin_x) == _level)
    while(GPIOx_ReadInputPin(GPIOx,GPIO_Pin_x) == _level )
    {
      dem++;
      if(dem == _maxcycles+1) 
        return 0 ;
      delay_us(1);
    }
  else
    while(GPIOx_ReadInputPin(GPIOx,GPIO_Pin_x) == !_level )
    {
      dem++;
      if(dem == _maxcycles+1) 
        return 0 ;
      delay_us(1);
    }
  //  if(dem != _maxcycles)
  //    return 1;
  return 1;
  //   delay_us(1);
  //delay_us(1);
  //  while (_cnt <= _maxcycles) //_maxcycles >=1
  //  {
  //    _cnt++;
  //    if (GPIOx_ReadInputPin(GPIOx,GPIO_Pin_x) == _level)
  //    {
  //      break;
  //    }
  //    delay_us(1);
  //  }
  //  while (GPIOx_ReadInputPin(GPIOx,GPIO_Pin_x) == _level && dem <= _maxcycles)
  //  {
  //    dem++;
  //    delay_us(1);
  //  }
  //  if (dem == _maxcycles)
  //    return 0 ;
  //  if (_cnt < _mincycles || _cnt == _maxcycles)
  //    return 0;
}
float convertCtoF(float c) {
  return c * 9 / 5 + 32;
}
unsigned long _lastreadtime = 0;
unsigned long _firstreading = 0;
unsigned long currenttime = 0;
uint8_t mRead(void) {    
  PB_ODR_ODR4 = 1;
  TIM1_CR1_CEN = 0;
  currenttime = millis;
  if (currenttime < _lastreadtime) {
    _lastreadtime = 0;
  }
  if (!_firstreading && ((currenttime - _lastreadtime) < 2000)) {
    //  UARTPrintString("return last correct measurement");
    return 1; // return last correct measurement
    //delay(2000 - (currenttime - _lastreadtime));
  }
  _firstreading = 0;
  
  _lastreadtime = millis;
  cookedDHTData[0] = cookedDHTData[1] = cookedDHTData[2] = cookedDHTData[3] = cookedDHTData[4] = 0;
  GPIOx_Init((GPIO_TypeDef*)GPIOD_BaseAddress, GPIO_PIN_3,GPIO_MODE_IN_PU_NO_IT);//pull up 1
  delay(100);
  PB_ODR_ODR4 = 0;
  GPIOx_Init((GPIO_TypeDef*)GPIOD_BaseAddress, GPIO_PIN_3,GPIO_MODE_OUT_OD_LOW_FAST);//out 0
  delay(20);
  PB_ODR_ODR4 = 1;
  GPIOx_Init((GPIO_TypeDef*)GPIOD_BaseAddress, GPIO_PIN_3,GPIO_MODE_IN_PU_NO_IT);
  delay_us(11);
  PB_ODR_ODR4 = 0;
  if (detectPulse(0, 80,(GPIO_TypeDef*)GPIOD_BaseAddress, GPIO_PIN_3,RESET)) //uint8_t detectPulse(uint8_t _mincycles ,uint8_t _maxcycles,uint8_t _pin,uint8_t _level)
  {
    PB_ODR_ODR4 = !  PB_ODR_ODR4 ;
  }
  else
  {
    return 0;
  }
  if (detectPulse(0, 80, (GPIO_TypeDef*)GPIOD_BaseAddress, GPIO_PIN_3,SET)) //uint8_t detectPulse(uint8_t _mincycles ,uint8_t _maxcycles,uint8_t _pin,uint8_t _level)
  {
    PB_ODR_ODR4 = !  PB_ODR_ODR4 ;   
  }
  else
  {
    return 0;
  }
  
  for (int i = 0; i < 40; i++)
  {
    if (detectPulse(0, 55, (GPIO_TypeDef*)GPIOD_BaseAddress, GPIO_PIN_3,RESET)) //uint8_t detectPulse(uint8_t _mincycles ,uint8_t _maxcycles,uint8_t _pin,uint8_t _level)
    {
PB_ODR_ODR4 = !  PB_ODR_ODR4 ;
    }
    else
    {
      return 0;
    }
    if (detectPulse(0, 70, (GPIO_TypeDef*)GPIOD_BaseAddress, GPIO_PIN_3,SET)) //uint8_t detectPulse(uint8_t _mincycles ,uint8_t _maxcycles,uint8_t _pin,uint8_t _level)
    {
      PB_ODR_ODR4 = !  PB_ODR_ODR4 ;
      cookedDHTData[i / 8] <<= 1;
      if (*counter < temp)
      {
        cookedDHTData[i / 8] |= 0;
      }
      else
        cookedDHTData[i / 8] |= 1;
    }
    else
    {
      return 0;
    }
  }
  
  if (cookedDHTData[4] == ((cookedDHTData[0] + cookedDHTData[1] + cookedDHTData[2] + cookedDHTData[3]) & 0xFF))//0x69 0x45 0x00 0x20 0x04 
  {
    return 1;
  }
  return 0 ;
}
float mReadTemperature(uint8_t S)
{
  float f;
  if (mRead()) {
    switch (_mType) {
      case DHT11:
        f = cookedDHTData[2]+cookedDHTData[3]*0.1;
        if (cookedDHTData[3] & 0x80) {
          f = -1 - f;
        }
//        f += (cookedDHTData[3] & 0x0f)*0.1 ;
        if (S) f = convertCtoF(f);
        return f;
      case DHT22:
      case DHT21:
        f = cookedDHTData[2] & 0x7F;
        f *= 256;
        f += cookedDHTData[3];
        f /= 10;
        if (cookedDHTData[2] & 0x80) f *= -1;
        if (S) f = convertCtoF(f);
        return f;
    }
  }
  return _NULL_VAL;
}

float mReadHumidity(void) {
  float f;
  if (mRead() ) {
    //   UARTPrintString(_mType);
    switch (_mType) {
    case DHT11:
      f = cookedDHTData[0]+cookedDHTData[1]*0.1;
      return f;
    case DHT22:
    case DHT21:
      f = cookedDHTData[0];
      f *= 256;
      f += cookedDHTData[1];
      f /= 10;
      return f;
    }
  }
  // UARTPrintString("Read Hum fail\n");
  return _NULL_VAL;
}
void InitialiseUART()
{
  //
  //  Clear the Idle Line Detected bit in the status rerister by a read
  //  to the UART1_SR register followed by a Read to the UART1_DR register.
  //
  unsigned char tmp = UART1_SR;
  tmp = UART1_DR;
  //
  //  Reset the UART registers to the reset values.
  //
  UART1_CR1 = 0;
  UART1_CR2 = 0;
  UART1_CR4 = 0;
  UART1_CR3 = 0;
  UART1_CR5 = 0;
  UART1_GTR = 0;
  UART1_PSCR = 0;
  //
  //  Now setup the port to 115200,n,8,1.
  //
  UART1_CR1_M = 0;        //  8 Data bits.
  UART1_CR1_PCEN = 0;     //  Disable parity.
  UART1_CR3_STOP = 0;     //  1 stop bit.
  UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud
  UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
  //
  //  Disable the transmitter and receiver.
  //
  UART1_CR2_TEN = 0;      //  Disable transmit.
  UART1_CR2_REN = 0;      //  Disable receive.
  //
  //  Set the clock polarity, lock phase and last bit clock pulse.
  //
  UART1_CR3_CPOL = 1;
  UART1_CR3_CPHA = 1;
  UART1_CR3_LBCL = 1;
  //
  //  Turn on the UART transmit, receive and the UART clock.
  //
  
}

//
//  Send the message in the string to UART1.
//
void UARTPrintString(char *message)
{
  char *ch = message;
  while (*ch)
  {
    UART1_DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
    while (UART1_SR_TXE == 0);          //  Wait for transmission to complete.
    ch++;                               //  Grab the next character.
  }
}
void UARTPrintNumber(unsigned char _value)
{
  UART1_DR = (unsigned char)(1)&0x80;
  while(UART1_SR_TXE == 0);
}
void UART1_SendData8(uint8_t Data)
{
  /* Transmit Data */
  UART1_DR = Data;
}
void enableUART1(uint8_t _key)
{
  if(_key == 0||_key == 1)
  {
    TIM1_CR1_CEN = _key;
    UART1_CR2_TEN = _key;
    UART1_CR2_REN = _key;
    UART1_CR3_CKEN = _key;
  }
  
}
char buffer[6];
void commitData(char *_buff)
{
  UARTPrintString(_buff);
  UARTPrintString("\n");
}
char * toArray(char* numberArray,float number)
{
  int deciPart = (int)number;
  for (int  i = 0; i < 2; i++)
  {
    *(numberArray+i) = ((int)number % 10)+'0';
  }
//  *(numberArray+2) = '.';
//  for(int j = 3; j<5; j++)
//    *(numberArray+j) = ((int)deciPart % 10)+'0';
  return numberArray;
}

void main()
{
  __disable_interrupt();
//  InitialiseSystemClock();
//  SetupOutputPorts();
//  InitialiseTimer2();
//  SetupTimer2_1ms();
//  SetupTimer1();
//  InitialiseUART();
  __enable_interrupt();
  while (1)
  {
    float h = mReadHumidity();    
    float t = mReadTemperature(0); 
    TIM1_CR1_CEN = 1;
    UART1_CR2_TEN = 1;
    UART1_CR2_REN = 1;
    UART1_CR3_CKEN = 1;
    if (t >30)
    {  
      UARTPrintString("Nhiet do: ");
      commitData(buffer);
    }
    else
    {
      UARTPrintString("Indentify temperature\n");
    }
    if (h != _NULL_VAL)
    {    
      UARTPrintString("Do am: ");
      commitData(toArray(buffer,h));
    }
    else
      UARTPrintString("Indentify humidity\n");
    //    sprintf(buffer, "%d", (int)GPIOx_ReadInputPin((GPIO_TypeDef*)GPIOB_BaseAddress,GPIO_PIN_5));
    delay(100);
    UART1_CR2_TEN  = 0;
    UART1_CR2_REN  = 0;
    UART1_CR3_CKEN = 0;
    delay(3000);    
  }
}

