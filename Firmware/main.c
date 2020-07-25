/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Parameters ----------------------------------------------------------------*/

#define SWIPE_SPEED   10    // inches per second

#define bool uint8_t
#define HIGH 1
#define LOW 0

const uint16_t leading_zeros[3] = { 10, 20, 10 };
const uint16_t trailing_zeros[3] = { 10, 10, 10 };
const uint8_t track_density[3] = { 210, 75, 210 }; // bits per inch for each track
const uint8_t track_sublen[] = { 32, 48, 48 };
const uint8_t track_bitlen[] = { 7, 5, 5 };

uint32_t track_period_us[3];  // bit period of each track in microseconds
                              // initialized in calc_track_periods()

static bool f2f_pole; // used to track signal state during playback (F2F encoding)

/* Implementations -----------------------------------------------------------*/

typedef struct {
  char *tracks[3];  // track data
} card;

const card test_card = {    //Default test card
  {
    "%B4444444444444444^ABE/LINCOLN^291110100000931?",
    ";4444444444444444=29111010000093100000?",
    0
  },
};

// service code to make requirements more lax and disable chip-and-PIN (third digit)
const char* sc_rep = { "101" };

//The card that user type in
card *c;

/* Private function prototypes -----------------------------------------------*/
void USART_Configuration(void);
void GPIO_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void calc_track_periods(void);
void static delay_us(uint32_t time);
void static delay_ms(uint32_t time);
void blink(int times);
static void invert_coil_pole(void);
void f2f_play_bit(bool bit, uint32_t period_us);
void play_zeros(uint8_t n, uint32_t period_us);
void play_byte(uint8_t byte, uint8_t n_bits, uint32_t period_us, uint8_t *lrc);
void play_track(card *c, uint8_t track_num);
void play_card(card *c);

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
int main(void)
{
	int i, j;
	char *track;
	
	//Defaultly load the test card
	*c=test_card;
	
	//Initialize GPIO, USART, and interruptions
	GPIO_Configuration();
	USART_Configuration();
	NVIC_Configuration();
	EXTI_Configuration();
	
	calc_track_periods();
	
	//Setup complete, blink to show
	blink(3);
	
	//Once the Serial is up, print
  printf("Startup complete. Please enter the data to spoof:\r\n");
	
	while (1)
	{
		for(i=0;i<3;i++)
		{
			printf("Please enter the %d track, ending with '#'\r\n",i);
			track = c->tracks[i];
			j=0;
			while(1)
			{
				while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
				if(USART_ReceiveData(USART1)!='#')
				{
					track[j]=USART_ReceiveData(USART1);
					j++;
				}
				else
				{
					track[j]='\0';
					continue;
				}
			}
		}
		printf("All three tracks recorded, simulation may start now.\r\n");
  }
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE); 						 
  /**
  *  The coil is on GPIOA Pin 0 and GPIO Pin 5
  *  The activate button is on GPIOA Pin 7, and on-board LED on GPIOC Pin 13
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : EXTI_Configuration
* Description    : Configures the different EXTI lines.
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource7);
  EXTI_ClearITPendingBit(EXTI_Line7);

  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;	  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : EXTI9_5_IRQHandler
* Description    : This function handles External lines 5-9 interrupt request (button press).
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	if ( EXTI_GetITStatus(EXTI_Line7) != RESET ) 
	{
		delay_ms(5);
	  play_card(c);
		blink(3);
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
}

/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configure USART1 
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/
void USART_Configuration(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);
  /*
  *  USART1_TX -> PA9 , USART1_RX ->	PA10
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure); 
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  USART_ClearFlag(USART1,USART_FLAG_TC);
  USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*******************************************************************************
* Function Name  : calc_track_periods
* Description    : Calculate track period from the set swipe speed
* Input          : None
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/

void calc_track_periods(void)
{
	int i = 0;
  for (i = 0; i < 3; i++)
    track_period_us[i] = (uint32_t) (1e6 / (SWIPE_SPEED * track_density[i]));
}

/*******************************************************************************
* Function Name  : delay_us
* Description    : Delay certain us of time
* Input          : Time to delay
* Output         : None
* Return         : None
* Attention		   : Static function, in-file use only
*******************************************************************************/

void static delay_us(uint32_t time)
{
  u32 i=8*time;
  while(i--);
}

/*******************************************************************************
* Function Name  : delay_ms
* Description    : Delay certain ms of time
* Input          : Time to delay
* Output         : None
* Return         : None
* Attention		   : Static function, in-file use only
*******************************************************************************/

void static delay_ms(uint32_t time)
{
  u32 i=8000*time;
  while(i--);
}

/*******************************************************************************
* Function Name  : blink
* Description    : Blink the on-board LED for certain times
* Input          : Times of the blinking
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/

void blink(int times)
{
  int i;
  for (i = 0; i < times; i++) {
    GPIO_SetBits(GPIOA , GPIO_Pin_13);
    delay_ms(200);
    GPIO_ResetBits(GPIOA , GPIO_Pin_13);
    delay_ms(200);
   } 
}

/*******************************************************************************
* Function Name  : invert_coil_pole
* Description    : Invert the polarity of the coil
* Input          : None
* Output         : None
* Return         : None
* Attention		   : Static function, in-file use only
*******************************************************************************/

static void invert_coil_pole(void)
{
	if (f2f_pole == 1)
	{
		GPIO_SetBits(GPIOA , GPIO_Pin_0);
		GPIO_ResetBits(GPIOA , GPIO_Pin_5);
		f2f_pole = 0;
	}
  else
	{
		GPIO_SetBits(GPIOA , GPIO_Pin_5);
		GPIO_ResetBits(GPIOA , GPIO_Pin_0);
		f2f_pole = 1;
	}
}

/*******************************************************************************
* Function Name  : f2f_play_bit
* Description    : Mimic a certain bit by manipulating the magnetic of the coil
* Input          : Bit to play and the bit period
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/

void f2f_play_bit(bool bit, uint32_t period_us)
{
  const uint32_t half_period = period_us / 2;
  // invert magnetic pole at start of period
  invert_coil_pole();

  delay_us(half_period);

  // at half period, determine whether to invert pole
  if (bit)
    invert_coil_pole();

  delay_us(half_period);
}

/*******************************************************************************
* Function Name  : play_zeros
* Description    : Quickly send a series of zeros
* Input          : Number of the zeros and the period time
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/

void play_zeros(uint8_t n, uint32_t period_us)
{
	int i;
  for (i = 0; i < n; i++)
    f2f_play_bit(0, period_us); 
}

/*******************************************************************************
* Function Name  : play_byte
* Description    : Send a byte of data
* Input          : Byte to play, bits count, period, and LRC pointer
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/

void play_byte(uint8_t byte, uint8_t n_bits, uint32_t period_us, uint8_t *lrc)
{
    bool parity = 1;
		int i;
    for (i = 0; i < n_bits; i++) {
      bool bit = byte & 1;
      parity ^= bit;
                                                // TIMING SENSITIVE
      f2f_play_bit(bit, period_us - 30);        // subtract 30us to compensate for delay 
                                                // caused by extra processing within this loop
      byte >>= 1;
      if (lrc)
        *lrc ^= bit << i;
    }
    f2f_play_bit(parity, period_us);  // last bit is parity
}

/*******************************************************************************
* Function Name  : play_track
* Description    : Play an entire track
* Input          : Card to play, and the number of the pending track
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/

void play_track(card *c, uint8_t track_num)
{
  char *track = c->tracks[track_num];
	uint32_t period_us;
	char *track_byte;
	uint8_t lrc = 0;
	int cnt=0;
  
	if (!track) return; // check that the track exists

  period_us = track_period_us[track_num];
  

  // lead playback with zeros
  play_zeros(leading_zeros[track_num], period_us);

  // play back track data
  for (track_byte = track; *track_byte != 0; track_byte++)
	{
    uint8_t tb;
		if(cnt<0)
			cnt--;
		else 
		{ // look for FS
			if(*track_byte=='^')
				cnt++;
			if(*track_byte=='=')
				cnt+=2;
				
			if(cnt==2)
				cnt=-1;
		}
		if (cnt<-5 && cnt>-9) // SS is located 4 chars after the last FS
			tb = sc_rep[8+cnt] - track_sublen[track_num];
		else
			tb = *track_byte - track_sublen[track_num];

    play_byte(tb, track_bitlen[track_num] - 1, period_us, &lrc);
  }

  // play LRC
  play_byte(lrc, track_bitlen[track_num] - 1, period_us, 0);

  // end playback
  play_zeros(trailing_zeros[track_num], period_us);
}

/*******************************************************************************
* Function Name  : play_card
* Description    : Play an entire card
* Input          : Card to play
* Output         : None
* Return         : None
* Attention		   : None
*******************************************************************************/

void play_card(card *c)
{
	int i;
	
  // let user know playback has begun by turning on LED
	GPIO_SetBits(GPIOA , GPIO_Pin_13);
	GPIO_ResetBits(GPIOA , GPIO_Pin_5);
	GPIO_ResetBits(GPIOA , GPIO_Pin_0);

  f2f_pole = 0;

  for (i = 0; i < 3; i++)
    play_track(c, i);

  // turn off LED and make sure coils are off
  GPIO_ResetBits(GPIOA , GPIO_Pin_13);
	GPIO_ResetBits(GPIOA , GPIO_Pin_5);
	GPIO_ResetBits(GPIOA , GPIO_Pin_0);
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

