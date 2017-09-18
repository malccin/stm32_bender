
#include "stm32f1xx_hal.h"
#include "stm32f105xc.h"
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include "sine_wave.c"
#include "I2C.h"

#define ADS1110_Adres		0x92
#define AMS5310_Adres		0x80
#define AMS5310_2_Adres		0x82
typedef enum {Error = 0, Success = !Error } Status;
#define Timed(x) Timeout = 0x3E8; while (x) { if (Timeout-- == 0) goto errReturn;}
uint32_t Timeout = 0;

static uint16_t sine3[]={

		2048,2176,2304,2431,2557,2680,2801,2919,
		3034,3145,3251,3353,3449,3540,3625,3704,
		3776,3842,3900,3951,3995,4031,4059,4079,
		4091,4095,4091,4079,4059,4031,3995,3951,
		3900,3842,3776,3704,3625,3540,3449,3353,
		3251,3145,3034,2919,2801,2680,2557,2431,
		2304,2176,2048,1919,1791,1664,1538,1415,
		1294,1176,1061,950,844,742,646,555,
		470,391,319,253,195,144,100,64,
		36,16,4,0,4,16,36,64,
		100,144,195,253,319,391,470,555,
		646,742,844,950,1061,1176,1294,1415,
		1538,1664,1791,1919,2048,
};

static uint16_t sine200_10kHz[]={
		2048,2112,2176,2240,2304,2368,2431,2494,
		2557,2619,2680,2741,2801,2861,2919,2977,
		3034,3090,3145,3198,3251,3302,3353,3402,
		3449,3495,3540,3583,3625,3665,3704,3741,
		3776,3810,3842,3872,3900,3927,3951,3974,
		3995,4014,4031,4046,4059,4070,4079,4086,
		4091,4094,4095,4094,4091,4086,4079,4070,
		4059,4046,4031,4014,3995,3974,3951,3927,
		3900,3872,3842,3810,3776,3741,3704,3665,
		3625,3583,3540,3495,3449,3402,3353,3302,
		3251,3198,3145,3090,3034,2977,2919,2861,
		2801,2741,2680,2619,2557,2494,2431,2368,
		2304,2240,2176,2112,2048,1983,1919,1855,
		1791,1727,1664,1601,1538,1476,1415,1354,
		1294,1234,1176,1118,1061,1005,950,897,
		844,793,742,693,646,600,555,512,
		470,430,391,354,319,285,253,223,
		195,168,144,121,100,81,64,49,
		36,25,16,9,4,1,0,1,
		4,9,16,25,36,49,64,81,
		100,121,144,168,195,223,253,285,
		319,354,391,430,470,512,555,600,
		646,693,742,793,844,897,950,1005,
		1061,1118,1176,1234,1294,1354,1415,1476,
		1538,1601,1664,1727,1791,1855,1919,1983,

};



enum { SRAM_BB_REGION_START = 0x20000000 };
enum { SRAM_BB_REGION_END = 0x200fffff };
enum { SRAM_BB_ALIAS = 0x22000000 };
enum { PERIPH_BB_REGION_START = 0x40000000 };
enum { PERIPH_BB_REGION_END = 0x400fffff };
enum { PERIPH_BB_ALIAS = 0x42000000 };
#define SRAM_ADR_COND(adres) ( (uint32_t)&adres >= SRAM_BB_REGION_START && (uint32_t)&adres <= SRAM_BB_REGION_END )
#define PERIPH_ADR_COND(adres) ( (uint32_t)&adres >= PERIPH_BB_REGION_START && (uint32_t)&adres <= PERIPH_BB_REGION_END )
#define BB_SRAM2(adres, bit)( SRAM_BB_ALIAS + ((uint32_t)&adres - SRAM_BB_REGION_START)*32u + (uint32_t)(bit*4u) )
#define BB_PERIPH(adres, bit)( PERIPH_BB_ALIAS + ((uint32_t)&adres - PERIPH_BB_REGION_START)*32u + (uint32_t)(__builtin_ctz(bit))*4u)
/* bit - bit mask, not bit position! */
#define BB(adres, bit) *(__IO uint32_t *)( SRAM_ADR_COND(adres) ? BB_SRAM2(adres, bit) : \
( PERIPH_ADR_COND(adres) ? BB_PERIPH(adres, bit) : 0 ))
#define BB_SRAM(adres, bit) *(__IO uint32_t *)BB_SRAM2(adres, bit)

typedef enum
{
/* Push-Pull */
gpio_mode_output_PP_2MHz = 2,
gpio_mode_output_PP_10MHz = 1,
gpio_mode_output_PP_50MHz = 3,
/* Open-Drain */
gpio_mode_output_OD_2MHz = 6,
gpio_mode_output_OD_10MHz = 5,
gpio_mode_output_OD_50MHz = 7,
/* Push-Pull */
gpio_mode_alternate_PP_2MHz = 10,
gpio_mode_alternate_PP_10MHz = 9,
gpio_mode_alternate_PP_50MHz = 11,
/* Open-Drain */
gpio_mode_alternate_OD_2MHz = 14,
gpio_mode_alternate_OD_10MHz = 13,
gpio_mode_alternate_OD_50MHz = 15,
/* Analog input (ADC) */
gpio_mode_input_analog = 0,
/* Floating digital input. */
gpio_mode_input_floating = 4,
/* Digital input with pull-up/down (depending on the ODR reg.). */
gpio_mode_input_pull = 8
}GpioMode_t;
typedef enum
{
PA0 = 0x00000001,
PA1 = 0x00000002,
PA2 = 0x00000004,
PA3 = 0x00000008,
PA4 = 0x00000010,
PA5 = 0x00000020,
PA6 = 0x00000040,
PA7 = 0x00000080,
PA8 = 0x00000100,
PA9 = 0x00000200,
PA10 = 0x00000400,
PA11 = 0x00000800,
PA12 = 0x00001000,
PA13 = 0x00002000,
PA14 = 0x00004000,
PA15 = 0x00008000,
PB0 = 0x00000001,
PB6 = 0x00000040,
PB7 = 0x00000080,
PB10 = 0x00000400,
PB11 = 0x00000800,
PB13 = 0x00002000,
PB14 = 0x00004000,
PB15 = 0x00008000,
PC8 = 0x00000100,
PC9 = 0x00000200,
PC4 = 0x00000010,
PC5 = 0x00000020,
}GpioPin_t;

uint8_t stringComplete = 0;
char inData[100];
char *inParse[100];
char inString[20] = "";
uint8_t indeks = 0;
uint32_t temp1;
uint8_t flaga_start =0;

uint32_t licznik;
uint16_t licznik_1;
uint16_t dane1[1500];
uint16_t dane2[1500];
uint16_t dane_adc[2];


 uint16_t voltage;
 uint16_t configregister;
 uint16_t dane_z_ads;
 uint16_t pomiar_1, pomiar_ostatni_1,pomiar_2, pomiar_ostatni_2;
 int16_t wynik_1, wynik_2;
 volatile uint16_t stat_1, stat_2;
 uint16_t licznik_1, licznik_2;
 uint8_t flaga_stop,error_1, flaga_2;


void SystemClock_Config(void);
void gpio_pin_cfg(GPIO_TypeDef * const port,GpioPin_t pin, GpioMode_t mode);
void send_USART(uint32_t x);
void USART_PutString(uint8_t * str);
void put_char(uint8_t ch);
void AMS_5310_read();
void ADS1110_Read();




int main(void)
{



  //HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  RCC->APB2ENR = RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_TIM1EN;
  RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_I2C2EN | RCC_APB1ENR_I2C1EN;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN | RCC_AHBENR_DMA2EN;

  SysTick_Config(8000000 * 0.1);

  //usart

    gpio_pin_cfg(GPIOA, PA9,  gpio_mode_alternate_PP_2MHz);
    gpio_pin_cfg(GPIOA, PA10, gpio_mode_input_floating);

  //LED

  	gpio_pin_cfg(GPIOC, PC4, gpio_mode_output_PP_2MHz);
  	gpio_pin_cfg(GPIOC, PC5, gpio_mode_output_PP_2MHz);


  	//usart config

  	USART1->BRR = 24000000/115200;
  	USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE|USART_CR1_RXNEIE;
  	NVIC_EnableIRQ(USART1_IRQn);


  	//adc

  	gpio_pin_cfg(GPIOA, PA0, gpio_mode_input_analog);
  	//gpio_pin_cfg(GPIOC, PC4, gpio_mode_input_analog);

  	//dac

  	gpio_pin_cfg(GPIOA, PA4, gpio_mode_input_analog);
  	//gpio_pin_cfg(GPIOA, PA5, gpio_mode_input_analog);


  	//I2C



  	gpio_pin_cfg(GPIOB, PB6,gpio_mode_alternate_OD_2MHz ); //SCL1
  	gpio_pin_cfg(GPIOB, PB7,gpio_mode_alternate_OD_2MHz ); //SDA1

  	gpio_pin_cfg(GPIOB, PB10,gpio_mode_alternate_OD_2MHz ); //SCL2
  	gpio_pin_cfg(GPIOB, PB11,gpio_mode_alternate_OD_2MHz ); //SDA2


  	//dac config

  			//master
  			TIM1->CR2 = TIM_CR2_MMS_2;
  			//TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
  			TIM1->PSC = 999;
  			TIM1->ARR = 3999;
  			TIM1->CCR1 = 200;
  			TIM1->CR1 = TIM_CR1_CEN;

  			//slave
  			//TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
  		    TIM2->PSC = 4-1;
  			TIM2->ARR = 2-1;
  			//TIM2->CR1 = TIM_CR1_OPM;
  			TIM2->CR2 = TIM_CR2_MMS_1;
  			//TIM2->SMCR = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0;
  			//TIM2->CR1 = TIM_CR1_CEN;

  			DMA2_Channel3->CMAR = (uint32_t)sine3;
  			DMA2_Channel3->CPAR = (uint32_t)&DAC->DHR12R1;
  			DMA2_Channel3->CNDTR = 100;
  			DMA2_Channel3->CCR = DMA_CCR_DIR | DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
  			DAC->CR = DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_TSEL1_2 | DAC_CR_DMAEN1 | DAC_CR_BOFF1;
  	        //DMA_CCR_CIRC
  			//TIM2->CR1 = TIM_CR1_CEN;

  			//DAC->CR = DAC_CR_EN1 | DAC_CR_BOFF1;
  			//DAC->DHR12R1 = 4095/2;


  			//ADC_config

  						DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
  						DMA1_Channel1->CMAR = (uint32_t)dane_adc;
  						DMA1_Channel1->CNDTR = 2;
  						DMA1_Channel1->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC| DMA_CCR_CIRC | DMA_CCR_EN ;

  					    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_DMA;

  					  	for(volatile uint32_t delay = 100000; delay; delay--);

  					  	BB(ADC1->CR2, ADC_CR2_RSTCAL) = 1;
  					  	while (BB(ADC1->CR2, ADC_CR2_RSTCAL));
  					  	BB(ADC1->CR2, ADC_CR2_CAL) = 1;
  					  	while (BB(ADC1->CR2, ADC_CR2_CAL));


  					  	ADC1->CR2 |= ADC_CR2_CONT;
  					  	ADC1->CR1 |= ADC_CR1_SCAN;
  					  	ADC1->SQR3 = 0 | 14 << 5;
  					  	ADC1->SQR1 = ADC_SQR1_L_0;
  					  	//ADC1->SMPR2 = ADC_SMPR2_SMP0;
  					  	ADC1->SMPR2 = ADC_SMPR2_SMP2;
  					  	//ADC1->CR2 |= ADC_CR2_ADON;



  	//I2C Config

  	if(I2Cx_Init(I2C1,100000) != I2C_SUCCESS){


  	  	// BB(GPIOC->ODR, PC5) = 1;



  	}else{

  	  		//if(I2Cx_IsDeviceReady(I2C2,ADS1110_Adres,50) == I2C_SUCCESS){

  	  	   // BB(GPIOC->ODR, PC5) = 1;



  	  		//}

  	  		//if(I2Cx_IsDeviceReady(I2C1,AMS5310_Adres,50) == I2C_SUCCESS){
  	  		//BB(GPIOC->ODR, PC5) = 1;
  	  		//}
  	}




  	if(I2Cx_Init(I2C2,100000) != I2C_SUCCESS){


  	 //BB(GPIOC->ODR, PC5) = 1;



  	}else{

  		if(I2Cx_IsDeviceReady(I2C2,AMS5310_2_Adres,50) == I2C_SUCCESS){

  	    //BB(GPIOC->ODR, PC5) = 1;



  		}

  		//if(I2Cx_IsDeviceReady(I2C1,AMS5310_Adres,50) == I2C_SUCCESS){
  		//BB(GPIOC->ODR, PC5) = 1;
  		//}
	}




  while (1)  {

	  /*uint8_t timeout = 500;




	  while ((USART1->SR & USART_SR_RXNE) && (--timeout > 0))
	  	{

		  if(timeout <= 0){


		  }else{

		  if(stringComplete == 0){

	  		USART1->SR &= ~USART_SR_RXNE;
	  	    char inChar = USART1->DR;

	  	    inData[indeks] = inChar;

	  	    indeks++;

	  	    inString[indeks - 1] += inChar;

	  	    if (inChar == '\n')
	  	    {

	  	      indeks = 0;

	  	      stringComplete = 1;
	  	    }



		  }
		  }
		 // BB(GPIOC->ODR, PC4) = 0;

	  	  }*/









	  if(flaga_start == 0){

	  //ADS1110_Read();
	  //odczyt();

	  licznik_1 += 1;


	  if(licznik_1 == 50){

		  AMS_5310_read();


		  //licznik_1 = 0;
	  }

	  if(licznik_1 == 100){




		  AMS_5310_2_read();

		 licznik_1 = 0;

	  }


	  }



	  if (stringComplete == 1)  {









	  								BB(GPIOC->ODR, PC4) = 1;

	  							    ParseSerialData();

	  							    inString[20] = "";

	  							    stringComplete = 0;




	  							    BB(GPIOC->ODR, PC4) = 0;
	  		}




  }





}

__attribute__((interrupt)) void USART1_IRQHandler(void){

	BB(GPIOC->ODR, PC4) = 1;

	while ((USART1->SR & USART_SR_RXNE) && (stringComplete == 0))
	{



		USART1->SR &= ~USART_SR_RXNE;
	    char inChar = USART1->DR;

	    inData[indeks] = inChar;

	    indeks++;

	    inString[indeks - 1] += inChar;

	    if (inChar == '\n')
	    {

	      indeks = 0;

	      stringComplete = 1;
	    }
	  }

	BB(GPIOC->ODR, PC4) = 0;

}

void ParseSerialData(){



  char *p = inData;

  char *str;

  int count = 0;

  char string_temp[20];

  uint8_t temp;

  while ((str = strtok_r(p, ";", &p)) != NULL)
  {

    inParse[count] = str;

    count++;
  }


  if (count == 1)  {

    char *func = inParse[0];

    //char *prop = inParse[1];


    switch (*func)
    {
      case 'V':


        USART_PutString("MW-WIRELESS LOGGER");
        USART_PutString("\n");

        break;



      case 'B':

    	flaga_start = 1;
    	//send_USART(temp1);
    	//GPIOC->ODR ^= GPIO_ODR_ODR13;



        break;

      case 'S':

    	flaga_start = 0;


        break;

      case 'C':

    	  ADC1->CR2 |= ADC_CR2_ADON;

    	  for(int i = 0; i < 199; i++){

    	      	  	    	  	    	  	dane1[i]= dane_adc[0];

    	      	  	    	  	    	  	dane2[i]= dane_adc[1];



    	      	  	    	  	    	  }

    	  DMA2_Channel3->CCR &= ~DMA_CCR_EN;
    	  DMA2_Channel3->CNDTR = 100;
    	  DMA2_Channel3->CCR |=  DMA_CCR_EN;
    	  //DAC->CR = DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_TSEL1_2 | DAC_CR_DMAEN1 | DAC_CR_BOFF1;


    	  TIM2->CR1 = TIM_CR1_CEN;
    	 // for(volatile uint32_t delay = 100000; delay; delay--);
    	  for(int i = 200; i < 1499; i++){

    	  	    	  	    	  	if(i < 1500){
    	  	    	  	    	  	dane1[i]= dane_adc[0];
    	  	    	  	    	  	dane2[i]= dane_adc[1];
    	  	    	  	    	  	}else{

    	  	    	  	    	  	//dane1[i]= dane1[10];
    	  	    	  	    	  	}





    	  	    	  	    	  }

    	 TIM2->CR1 &= ~TIM_CR1_CEN;
    	 ADC1->CR2 |= ADC_CR2_ADON;



    	  //wyslij Usartem

    	  	  	USART_PutString("START");
    	  		USART_PutString("#");

    	  		for(int i = 0; i<1499; i++){


    	  						for(volatile uint32_t delay = 13000; delay; delay--);
    	  						USART_PutString("CH1"); //channel 1
    	  						USART_PutString("$");
    	  					    send_USART(dane1[i]);
    	  					    USART_PutString("#");


    	  						}





    	  		// GPIOC->ODR ^= GPIO_ODR_ODR13;


    	  		//USART_PutString("STOP");
    	  		//USART_PutString("#");




    	  		USART_PutString("START");
    	  		USART_PutString("#");

    	  				for(int i = 0; i<1499; i++){


    	  								for(volatile uint32_t delay = 13000; delay; delay--);
    	  							    USART_PutString("CH2");
    	  							    USART_PutString("$");
    	  							    send_USART(dane2[i]);
    	  							    USART_PutString("#");

    	  								}



    	  							   // GPIOC->ODR ^= GPIO_ODR_ODR13;


    	  				USART_PutString("STOP");
    	  				USART_PutString("#");


    	  				USART_PutString("\n");


        break;


      case 'M':

    	  TIM2->CR1 &= ~TIM_CR1_CEN;

        break;

      case 'L':



    	  temp = 0x8E;

    	// I2Cx_Write(I2C2,&temp,1,ADS1110_Adres);

    	  break;

      case 'Z':

    	  wynik_1 = 0;




    	  break;

      case 'N':

    	  USART_PutString("START");
    	  USART_PutString("#");

    	  USART_PutString("CH3");
    	  USART_PutString("$");

    	  sprintf(string_temp, "%d", dane_z_ads);

    	  USART_PutString(string_temp);


    	  USART_PutString("#");

    	  //USART_PutString("STOP");
    	 // USART_PutString("#");

    	  if (stat_1 > 32 || stat_1 < 95) {


    	  //USART_PutString("START");
    	 // USART_PutString("#");

    	  USART_PutString("CH4");
    	  USART_PutString("$");

    	  sprintf(string_temp, "%d", stat_1);

    	  USART_PutString(string_temp);


    	  USART_PutString("#");

    	 // USART_PutString("STOP");
    	 // USART_PutString("#");


    	 // USART_PutString("START");
    	 // USART_PutString("#");

    	  USART_PutString("CH5");
    	  USART_PutString("$");

    	  sprintf(string_temp, "%d", wynik_1);

    	  USART_PutString(string_temp);


    	  USART_PutString("#");

    	 // USART_PutString("STOP");
    	  //USART_PutString("#");


    	 // USART_PutString("\n");

    	  }

    	  if (stat_2 > 32 || stat_2 < 95) {


    	           	  //USART_PutString("START");
    	           	 // USART_PutString("#");

    	           	  USART_PutString("CH6");
    	           	  USART_PutString("$");

    	           	  sprintf(string_temp, "%d", stat_2);

    	           	  USART_PutString(string_temp);


    	           	  USART_PutString("#");

    	           	 // USART_PutString("STOP");
    	           	 // USART_PutString("#");


    	           	 // USART_PutString("START");
    	           	 // USART_PutString("#");

    	           	  USART_PutString("CH7");
    	           	  USART_PutString("$");

    	           	  sprintf(string_temp, "%d", wynik_2);

    	           	  USART_PutString(string_temp);


    	              USART_PutString("#");

    	           	 // USART_PutString("STOP");
    	           	 // USART_PutString("#");
//

    	           	 // USART_PutString("\n");








    	  }

    	  	  	  	  //USART_PutString("#");
    	  	  	  	  USART_PutString("STOP");
    		      	  USART_PutString("#");


    		      	  USART_PutString("\n");



              break;

      case 'H':

    	  USART_PutString("START");
    	      	  USART_PutString("#");

         	 if (stat_2 > 32 || stat_2 < 95) {


         	  //USART_PutString("START");
         	 // USART_PutString("#");

         	  USART_PutString("CH6");
         	  USART_PutString("$");

         	  sprintf(string_temp, "%d", stat_2);

         	  USART_PutString(string_temp);


         	  USART_PutString("#");

         	 // USART_PutString("STOP");
         	 // USART_PutString("#");


         	 // USART_PutString("START");
         	 // USART_PutString("#");

         	  USART_PutString("CH7");
         	  USART_PutString("$");

         	  sprintf(string_temp, "%d", wynik_2);

         	  USART_PutString(string_temp);


         	  USART_PutString("#");

         	  USART_PutString("STOP");
         	  USART_PutString("#");


         	  USART_PutString("\n");





         	  }else{

         		  USART_PutString("STOP");
         		      	  USART_PutString("#");


         		      	  USART_PutString("\n");

         	  }

                   break;
    }
  }
  if (count == 2)
  {/*

    char *func = inParse[0];
    // Define value 2 as a property value
    char *prop = inParse[1];

    // Call the relevant identified function
    switch (*func)
    {

      case 'S':


        {int val_temp = atoi(prop);

        if(val_temp == 1){



       }
       if(val_temp == 2){

        if (stat_2 > 32 || stat_2 < 95) {
        temp = ("#");
        temp += stat_2;
        blink_led();
        Serial.println(temp);
        blink_led();

       }


       }
       }
        break;

      case 'Z':

      /*{int val_temp = atoi(prop);

      if(val_temp == 1){

       payload_1 = 0;

       }
       if(val_temp == 2){

       //wynik_2 = 0;

       }

      }
       break;

     case 'T':

      {int val_temp = atoi(prop);

      if(val_temp == 1){
        if (stat_1 > 32 || stat_1 < 95) {
        temp = ("#");
        //temp += temperatura_1;
        blink_led();
        Serial.println(temp);
        blink_led();
       }
       }

       if(val_temp == 2){
        if (stat_2 > 32 || stat_2 < 95) {
        temp = ("#");
        //temp += temperatura_2;
        blink_led();
        Serial.println(temp);
        blink_led();
       }
       }
      }
       break;
    }







  }*/
}

  flaga_stop = 0;

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    //Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
void gpio_pin_cfg(GPIO_TypeDef * const port,GpioPin_t pin, GpioMode_t mode){

pin = __builtin_ctz(pin)*4;
uint32_t volatile * cr_reg;
uint32_t cr_val;
cr_reg = &port->CRL;
if (pin > 28){
pin -= 32;
cr_reg = &port->CRH;
}
cr_val = *cr_reg;
cr_val &= ~((uint32_t)(0x0f << pin));
cr_val |= (uint32_t)(mode << pin);
*cr_reg = cr_val;
}
void send_USART(uint32_t x){

	char value[10];
	int i = 0;

	do {

		value[i++] = (char)(x % 10) + '0';
		x /= 10;


	}while(x);


	while(i){

		put_char(value[--i]);


	}

		//USART_PutString("\n");
}
void USART_PutString(uint8_t * str){
while(*str != 0)
  {
	put_char(*str);
    str++;
}
}
void put_char(uint8_t ch){
	while(!(USART1->SR & USART_SR_TXE));
		USART1->DR = ch;
}




void AMS_5310_read(){



		volatile uint64_t temp;
		volatile uint8_t byte1;
		volatile uint8_t byte2;
		volatile uint8_t byte3;
		volatile uint8_t byte4;
		volatile uint8_t byte5;

		BB(GPIOC->ODR, PC5) = 1;


		//I2C_READ_REG_I2C2(AMS5310_Adres,&temp,5);

		I2Cx_Read(I2C2, &temp, 5, AMS5310_Adres);

		temp = temp & 0xFFFFFFFFFF;
		byte1 = temp & 0xFF;
		byte2 = temp >> 12;
		byte3 = byte2 & 0xF;
		pomiar_1 = (byte1 << 4);
		pomiar_1 |= byte3;
		stat_1 = temp >> 24;
		stat_1 = stat_1 & 0xFF;



		if(stat_1 < 0x20 || stat_1 > 0x5F){



		}else{

			if (pomiar_ostatni_1 != pomiar_1) {



			        if (pomiar_ostatni_1 > 50 && pomiar_ostatni_1 < 4050) {

			          wynik_1 += (pomiar_ostatni_1 - pomiar_1);
			        }



			        if (pomiar_ostatni_1 >= 4050 && pomiar_1 <= 50) {

			          wynik_1 += pomiar_1 + (4095 - pomiar_ostatni_1);

			        }

			        if (pomiar_ostatni_1  <= 50 && pomiar_1 >= 4050) {

			          wynik_1 -= (4095 - pomiar_1) + pomiar_ostatni_1;
			        }





			        pomiar_ostatni_1 = pomiar_1;



			      }




		}


		BB(GPIOC->ODR, PC5) = 0;


}


void AMS_5310_2_read(){



		volatile uint64_t temp2;
		volatile uint8_t byte1;
		volatile uint8_t byte2;
		volatile uint8_t byte3;
		volatile uint8_t byte4;
		volatile uint8_t byte5;






		//BB(GPIOC->ODR, PC4) = 1;



		//I2C_READ_REG_I2C2(AMS5310_2_Adres,&temp2,5);

		I2Cx_Read(I2C2, &temp2, 5, AMS5310_2_Adres);

		temp2 = temp2 & 0xFFFFFFFFFF;
		byte1 = temp2 & 0xFF;
		byte2 = temp2 >> 12;
		byte3 = byte2 & 0xF;
		pomiar_2 = (byte1 << 4);
		pomiar_2 |= byte3;
		stat_2 = temp2 >> 24;
		stat_2 = stat_2 & 0xFF;



		if(stat_2 < 0x20 || stat_2 > 0x5F){



		}else{

			if (pomiar_ostatni_2 != pomiar_2) {



			        if (pomiar_ostatni_2 > 50 && pomiar_ostatni_2 < 4050) {

			          wynik_2 += (pomiar_ostatni_2 - pomiar_2);
			        }



			        if (pomiar_ostatni_2 >= 4050 && pomiar_2 <= 50) {

			          wynik_2 += pomiar_2 + (4095 - pomiar_ostatni_2);

			        }

			        if (pomiar_ostatni_2  <= 50 && pomiar_2 >= 4050) {

			          wynik_2 -= (4095 - pomiar_2) + pomiar_ostatni_2;
			        }





			        pomiar_ostatni_2 = pomiar_2;

			      }




		}

		//BB(GPIOC->ODR, PC4) = 0;
}


void ADS1110_Read(){

	//BB(GPIOC->ODR, PC8) = 1;

	uint32_t data_reg;
    uint8_t config_reg;

    //BB(GPIOC->ODR, PC5) = 0;


    I2C_READ_REG_I2C2(ADS1110_Adres,&data_reg,2);

    //I2C_READ_REG(ADS1110_Adres,&data_reg);

	//BB(GPIOC->ODR, PC5) = 1;

	config_reg = data_reg >> 16;
	dane_z_ads = data_reg & 0xFFFF;
	//dane_z_ads = data_reg;
    dane_z_ads = ((dane_z_ads & 0xFF) << 8) | (dane_z_ads >> 8);
    //dane_z_ads = 65535 - dane_z_ads;
	//configregister =  config_reg;
	//I2Cx_Read(I2C2, &buf[0], 2,ADS1110_Adres);
	//voltage = buf[0] << 8 | buf[1];
	//configregister = buf[2];

   // BB(GPIOC->ODR, PC8) = 0;
}



void I2C_READ_REG( uint8_t adres, uint8_t * dane)
{
	uint32_t dummy;

	I2C2->CR1 |= I2C_CR1_START;
	while( !( I2C2->SR1 & I2C_SR1_SB ));
	I2C2->DR = adres;
	while( !( I2C2->SR1 & I2C_SR1_ADDR ));
	dummy = I2C2->SR2;
	while( !( I2C2->SR1 & I2C_SR1_TXE ));
	//I2C1->DR = reg_adres;
	//while( !( I2C1->SR1 & I2C_SR1_BTF ));
	I2C2->CR1 |= I2C_CR1_START;
	while( !( I2C2->SR1 & I2C_SR1_SB ));
	I2C2->DR = adres | 0x01;
	while( !( I2C2->SR1 & I2C_SR1_ADDR ));
	dummy = I2C2->SR2;


	       I2C2->CR1 &= ~I2C_CR1_ACK;

	   while( !( I2C2->SR1 & I2C_SR1_RXNE ));
	    dane = I2C2->DR;


	I2C2->CR1 |= I2C_CR1_STOP;



}

void I2C_READ_REG_I2C1(uint8_t adres, uint8_t * dane, uint8_t len )
{

	uint32_t dummy;
	int16_t timeout = 1000;
	int16_t timeout2 = 1000;
	int16_t timeout3 = 1000;





	/*I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB ));
	I2C1->DR = adres;
	while(!( I2C1->SR1 & I2C_SR1_ADDR));
	dummy = I2C1->SR2;
	while(!(I2C1->SR1 & I2C_SR1_TXE ));*/
	//I2Cx->DR = reg_adres;
	//while( !( I2Cx->SR1 & I2C_SR1_BTF ));
	I2C1->CR1 |= I2C_CR1_START;
	while((!( I2C1->SR1 & I2C_SR1_SB ))  && (--timeout > 0));

	if(timeout != 0){
	BB(GPIOC->ODR, PC4) = 1;
	//I2C1->CR1 |= I2C_CR1_START;
	//while(!(I2C1->SR1 & I2C_SR1_SB))
	I2C1->DR = adres | 0x01;
	while((!( I2C1->SR1 & I2C_SR1_ADDR ))  && (--timeout2 > 0));

	dummy = I2C1->SR2;
	//I2C1->CR1 = I2C_CR1_ENPEC;
	I2C1->CR1 |= I2C_CR1_ACK;

	__disable_irq();
	while( len )
	{
	    if( len == 1 ){
		I2C1->CR1 &= ~I2C_CR1_ACK;
		I2C1->CR1 |= I2C_CR1_STOP;
	    }

	    while((!( I2C1->SR1 & I2C_SR1_RXNE ))  && (--timeout3 > 0));
	    *( dane++ ) = I2C1->DR;

	   len--;



	}

	I2C1->CR1 |= I2C_CR1_STOP;
	__enable_irq();
	}

	/* errReturn:

	 BB(GPIOC->ODR, PC5) = 1;*/
	  // Any cleanup here
	//return Error;
	BB(GPIOC->ODR, PC4) = 0;

}

void I2C_READ_REG_I2C2(uint8_t adres, uint8_t * dane, uint8_t len ){


	uint32_t dummy;
	int16_t timeout11 = 1000;
	int16_t timeout22 = 1000;
	int16_t timeout33 = 1000;

		//BB(GPIOC->ODR, PC5) = 1;



		/*I2C2->CR1 |= I2C_CR1_START;
		while(!(I2C2->SR1 & I2C_SR1_SB ));
		I2C2->DR = adres;
		while(!( I2C2->SR1 & I2C_SR1_ADDR));
		dummy = I2C2->SR2;
		while(!(I2C2->SR1 & I2C_SR1_TXE ));
		//I2Cx->DR = reg_adres;
		//while( !( I2Cx->SR1 & I2C_SR1_BTF ));*/

		I2C2->CR1 |= I2C_CR1_START;

		//while((!( I2C2->SR1 & I2C_SR1_SB )) && (--timeout11 > 0));

		while(!( I2C2->SR1 & I2C_SR1_SB ));

		I2C2->DR = adres | 0x01;

		while((!( I2C2->SR1 & I2C_SR1_ADDR )));

		dummy = I2C2->SR2;

		I2C2->CR1 |= I2C_CR1_ACK;

		//__disable_irq();

		while( len )
		{
			//BB(GPIOC->ODR, PC5) = 1;
			 if( len == 1 ){
			I2C2->CR1 &= ~I2C_CR1_ACK;
			I2C2->CR1 |= I2C_CR1_STOP;
			 }


		    while((!( I2C2->SR1 & I2C_SR1_RXNE )));
		    *( dane++ ) = I2C2->DR;

		   len--;
		}

		I2C2->CR1 |= I2C_CR1_STOP;

		//__enable_irq();



		//BB(GPIOC->ODR, PC5) = 1;





		//BB(GPIOC->ODR, PC4) = 1;








		/* errReturn:

		 BB(GPIOC->ODR, PC5) = 1;*/
		  // Any cleanup here
		//return Error;
		//BB(GPIOC->ODR, PC5) = 0;
		//BB(GPIOC->ODR, PC4) = 0;
}




