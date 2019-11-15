
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */


#define START_ADR_WRITE			0x08002800																	// Адрес с которого разрешается писать данные
#define END_ADR_WRITE				0x08007F00																	// Адрес до которого разрешается писать данные
#define START_ADR_CONF			0x08007F00																	// Адрес начала конфигурации устр-ва

uint16_t   MBS_WritePage = 0x0800;	

uint32_t MBS_LastAdresClear = 0;	

#define FlAdr (uint32_t)(0x08000000)





const unsigned char auchCRCHi[] =
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

const unsigned char auchCRCLo[] =
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};



const uint32_t USART_const [3] = {2400,4800,9600};
const uint8_t  TIMER_const [3] = {15,8,4};
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

volatile uint8_t snd_cnt=0;

uint32_t MBS_JumpAdres = 0x08002800;	

volatile uint8_t MBS_Fl_Stay_Btld=0;
volatile uint8_t MBS_Fl_Jump_Region=0;


volatile uint8_t mb_err=0;
volatile uint8_t mb_err_prev=0;

uint16_t CRCCod;




uint16_t max=0;
uint16_t min=0xFFFF;

uint16_t MBPause=0;
uint16_t MBPauseCnt=0;

uint32_t zeros=0;

unsigned char res_buffer[160];					// приемный буфер
unsigned char write_buffer[20];					// буфер для передачи
volatile unsigned char res_wr_index;


uint16_t reg_MB[30];



uint32_t FBI[10];
uint16_t abcd[8];
uint32_t FBI0;
uint32_t FBI1;
uint32_t FBI2;
uint32_t FBI3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
unsigned int CRC16 (unsigned char *pucFrame, unsigned int usLen)
													// pucFrame - указатель на начало буфера (адрес)
													// usLen - длина пакета
{
	volatile unsigned char   ucCRCHi = 0xFF;					// старший байт црц
	volatile unsigned char   ucCRCLo = 0xFF;					// младший байт црц
	volatile int             iIndex;
	int i=0;

	while (usLen--)									// цикл, usLen уменьшается на 1 при каждой итерации
	{
		iIndex = ucCRCLo ^ pucFrame[i];				// ксорим ucCRCLo  и байт, на который указывает pucFrame.
		i++;										// полученное значение будет индексом iIndex в таблицах. pucFrame инкрементируется.
		ucCRCLo = ucCRCHi ^ (auchCRCHi[iIndex] );	// ксорим ucCRCHi и значение из таблицы aucCRCHi с индексом iIndex.
		ucCRCHi = ( auchCRCLo[iIndex] );			// ucCRCHi равно значению из таблицы aucCRCLo с индексом iIndex
	}
	return ( ucCRCHi << 8 | ucCRCLo );				// Возвращаем старший и младший байт CRC в виде 16-разрядного слова
}


static void MX_TIM2_Reinit(uint8_t bd)
{
	__HAL_RCC_TIM21_CLK_ENABLE();
  htim2.Instance = TIM21;
  htim2.Init.Prescaler = 24000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIMER_const[bd];
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	
	TIM21->PSC = 24000 - 1; 
	TIM21->ARR = TIMER_const[bd]; 
	TIM21->DIER |= TIM_DIER_UIE; 
	TIM21->CR1 |= TIM_CR1_OPM;
	//TIM14->CR1 |= TIM_CR1_CEN; 
	NVIC_SetPriority(TIM21_IRQn, 0); 
	NVIC_EnableIRQ(TIM21_IRQn);
}



static void USART2_ReInit(uint8_t bd)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = USART_const[bd];
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

uint8_t Flash_Read_8(uint32_t adress)
{
	return *(uint8_t*) adress;
}

void Flash_Write_16(uint32_t adress, uint32_t data)
{
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adress, data);
	HAL_FLASH_Lock();
}


void Flash_Clear_Sector(uint32_t adress)
{
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef Flash_eraseInitStruct;
	Flash_eraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
	Flash_eraseInitStruct.PageAddress  = adress;
	Flash_eraseInitStruct.NbPages        = 2;
	uint32_t PgError = 0;
	HAL_FLASHEx_Erase(&Flash_eraseInitStruct, &PgError);
	//FLASH_PageErase(adress);
	//FLASH->CR &= ~FLASH_CR_PER;																		// Какой-то глюк не сбрасывается бит PER - стирание флеш
	HAL_FLASH_Lock();
}

uint32_t FLASH_Read(uint32_t address)
{
	return (*(__IO uint32_t*)address);
}



uint32_t* MBS_GetLink_JumpAdres(void) 
{
	return &MBS_JumpAdres; 
}		

/**
	* @brief           Метод перехода в область флеш памяти, для выполнения другого алгоритма (прыжок в другую программу)
	* @param	 adress: Адрес ячейки флеш памяти, которую требуется перейти	
  */
void Flash_Jump_Adress(uint32_t adress)															// Переход в область другой программы во флеш памяти
{
	__set_PRIMASK(1);																							// Отключаем глобальные прерывания(обязательно перед переходом)																						 					
	
	typedef 	void (*pFunction)(void);														// Объявляем тип функции-ссылки
	pFunction Jump_To_Application;																// Объявляем функцию-ссылку

  uint32_t JumpAddress = *(__IO uint32_t*) (adress + 4); 						// Адрес перехода на вектор (Reset_Handler) 		
  Jump_To_Application = (pFunction) JumpAddress;  							// Указатель на функцию перехода
	__set_MSP(*(volatile uint32_t*) adress);														// Указываем адрес вектора стека(Stack Pointer)	
  Jump_To_Application();                          							// Переходим на основную программу
}	

/**
  * @brief  Деинициализация всей включенной периферии и тактирования, переход в загрузчик
  */
void DeInitAll_Peripheral_And_Jump_BTLDR(void)
{
	if(MBS_Fl_Jump_Region)											// флаг - перейти в область памяти другой программы
	{
		// Останавливаем все задействованные таймеры
		HAL_TIM_Base_Stop_IT(&htim2);					 									// остановить таймер 3					 			
		
		
		// Отключаем все прерывания
		HAL_NVIC_DisableIRQ(TIM2_IRQn);				  	
		HAL_NVIC_DisableIRQ(TIM21_IRQn);				  	
		HAL_NVIC_DisableIRQ(TIM22_IRQn);			
		HAL_NVIC_DisableIRQ(USART2_IRQn);	 		  		  	
		HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);									
		HAL_NVIC_DisableIRQ(RCC_IRQn);				  								// отключить прерывания Осцилятора
		
		// Деинициализация портов
		HAL_GPIO_DeInit(GPIOA, 0xff);
		HAL_GPIO_DeInit(GPIOB, 0xff);

		
		// Деинициализация таймеров
		HAL_TIM_Base_DeInit(&htim2);

		
		// Деинициализация приемо-передатчиков
		HAL_UART_DeInit(&huart2);
		
		HAL_RCC_DeInit();
		HAL_DeInit();
		
		// Выключаем тактирование портов GPIO
		__HAL_RCC_GPIOA_CLK_DISABLE();
		__HAL_RCC_GPIOB_CLK_DISABLE();
		__HAL_RCC_GPIOC_CLK_DISABLE();
		
		HAL_IWDG_Refresh(&hiwdg);																// Сброс счетчика сторожевого таймера

		Flash_Jump_Adress(MBS_GetLink_JumpAdres()[0]);					// Переходим в область памяти основной программы
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//*****************************************************************************
	// 																	ВНИМАНИЕ!!!
	//*****************************************************************************	
	// ПОСЛЕ ПЕРЕСБОРКИ ПРОЕКТА ЧЕРЕЗ CUBE_MX, НЕОБХОДИМО В ФАЙЛЕ (system_stm32f1xx.c)
	// ФУНКЦИИ (void SystemInit (void)) ЗАКОММЕНТИРОАТЬ СЛЕДУЮЩИЕ СТРОКИ:
	//#ifdef VECT_TAB_SRAM
	// SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET;  /* Vector Table Relocation in Internal SRAM.  */
	//#else
  // SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH. */
	//#endif 
	// ОНИ ОТВЕЧАЮТ ЗА МЕСТОНАХОЖДЕНИЕ ВО ФЛЕШЕ КОНТРОЛЛЕРА, ТАБЛИЦЫ ВЕКТОРОВ ПРЕРЫВАНИЙ.
	
		if(SCB->VTOR == 0x08002800)						
		{
			//MBS_GetLink_Flag_StayBtld()[0] = 1;											// Выставляем флаг - остаться в загрузчике
			MBS_Fl_Stay_Btld = 1;
		}
	
	__set_PRIMASK(1);									  											// Отключаем глобальные прерывания	
	SCB->VTOR = (uint32_t)0x08000000;  												// Переопределяем начало таблицы векторов прерываний 
	__set_PRIMASK(0);	
	//MBS_Fl_Stay_Btld = 1;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

	

	
	//MB_HZ_I=FBI[5]>>16;
	//MB_HZ_F=FBI[5]&0x0000FFFF;
	
	
	NVIC_SetPriority(USART2_IRQn, 0); 
  NVIC_EnableIRQ(USART2_IRQn);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); 
	
	

	USART2_ReInit(2);
  MX_TIM2_Reinit(2);
  /* USER CODE END 2 */
	
	 __HAL_RCC_GPIOC_CLK_ENABLE();


  /*GPIO_InitTypeDef GPIO_InitStruct;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);


  //Configure GPIO pin : PC14 
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);*/

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//HAL_IWDG_Refresh(&hiwdg);
		//HAL_Delay(1000);
			HAL_IWDG_Refresh(&hiwdg);
		
			HAL_Delay(1000);
			HAL_IWDG_Refresh(&hiwdg);
		if(MBS_Fl_Stay_Btld==0)
		{
			
			MBS_Fl_Jump_Region=1;
		}
		DeInitAll_Peripheral_And_Jump_BTLDR();
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void USART2_IRQHandler(void)
{
	if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{	
		
			TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
			TIM21->CNT=0;
			res_buffer[res_wr_index]=(uint8_t)(USART2->RDR);
			//HAL_UART_Receive(&huart2, &x, 1, 100);
			
			res_wr_index++;						
			TIM21->CR1 |= TIM_CR1_CEN; 
	}
	HAL_UART_IRQHandler(&huart2);
}





void TIM21_IRQHandler(void)
{
	TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
	HAL_TIM_IRQHandler(&htim2);


	
	
	int i;
	if (res_buffer[0]==247)

  {
		
	  CRCCod=CRC16(res_buffer, (res_wr_index));	// Расчет СRC
	  if (CRCCod==0)								// Проверка CRC в посылке
	  {											// Если верно - выполняем действие

		  switch (res_buffer[1]) {
		  /*case 0x03:							// Чтение регистров
		  {
			  if (res_buffer[0]==247&&(res_buffer[2]<1) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-1)<30))
			  {
					mb_err=0;
				  write_buffer[0]=res_buffer[0];					// Адрес устройства
				  write_buffer[1]=0x03;						// Та-же функция
				  write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

				  for (i=0; i<res_buffer[5]; i++)				// Значения регистров
				  {
					  write_buffer[4+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])& 0x00FF;//%256;		// Младший байт (2-ой)
					  write_buffer[3+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])>> 8;///256;	// Старший байт (1-ый)
				  }		
					
				  CRCCod=CRC16(write_buffer, (write_buffer[2]+3));			// Расчет CRC

				  write_buffer[write_buffer[2]+3] = CRCCod & 0x00FF;			// мл. байт CRC
				  write_buffer[write_buffer[2]+4] = CRCCod >> 8;				// ст. байт CRC

				  HAL_UART_Transmit(&huart2,write_buffer,write_buffer[2]+5,100);
			  }

			  else
			  {
					mb_err=1;
				  write_buffer[0]=res_buffer[0];					// адрес блока
				  write_buffer[1]=0x83;						// та-же функция + взведенный бит ошибки
				  write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
				  CRCCod=CRC16(write_buffer, 3);				// расчет CRC
				  write_buffer[3] = CRCCod & 0x00FF;			// мл. байт CRC
				  write_buffer[4] = CRCCod >> 8;				// ст. байт CRC
				  HAL_UART_Transmit(&huart2,write_buffer,5,100);
			  }
			  break;
		  }*/
			case 0x4D:
			{
				if(res_wr_index == (5 + res_buffer[4] + 2) || res_buffer[4]%2 == 0)	// Проверка размера пакета и четность кол-ва записываемых байт данных
				{
					uint32_t fleshAdrStart = MBS_WritePage << 16 | res_buffer[2] << 8 | res_buffer[3];
					uint16_t val = res_buffer[4];
					uint32_t data = 0;
					
					if(fleshAdrStart >= START_ADR_WRITE &&																	// Адрес с которого разрешается писать данные
						 fleshAdrStart  < END_ADR_WRITE)																			// Адрес до которого разрешается писать данные
					{
						//if(fleshAdrStart == 0x08000000)																				// Если обновляем основную программу
						//	for(uint16_t i = 0; i < 0x1000; i+= 0x400)
						//		Flash_Clear_Sector(START_ADR_CONF+i);															// Стираем 4 страницы флеша, где храняться настройки (на случай если изменилась карта памяти)
						
						if(MBS_LastAdresClear <= fleshAdrStart) 															// Если перед записью страница флеш памяти не была очищена
						{
							if(!MBS_LastAdresClear)
								MBS_LastAdresClear = fleshAdrStart;
							
							Flash_Clear_Sector(MBS_LastAdresClear);															// Адрес страницы флеша, которую необходимо стереть
							MBS_LastAdresClear += 0x100;																				
						}
						for(uint8_t i = 0; i < val; i += 4)																				// перезапись прошивки из принятого пакета во флеш																			
						{
							data = res_buffer[5 + i] | res_buffer[6 + i] << 8 | res_buffer[7 + i] << 16 | res_buffer[8 + i] << 24;
							Flash_Write_16(fleshAdrStart + i,  data);
						}
							write_buffer[0] = res_buffer[0]; 																										// формируем ответный пакет 
							write_buffer[1] = 0x4D; 
							write_buffer[2] = res_buffer[2];
							write_buffer[3] = res_buffer[3];
							write_buffer[4] = res_buffer[4];
							snd_cnt=5;
					}
					else
					{
						write_buffer[0]=res_buffer[0];					// адрес блока
						write_buffer[1]=res_buffer[1]|0x80;						// та-же функция + взведенный бит ошибки
						write_buffer[2]=0x03;				// код ошибки - недопустимая функция -> адрес выходит за пределы заршенного
						snd_cnt=3;
					}
				}
				else
				{
					write_buffer[0]=res_buffer[0];					// адрес блока
					write_buffer[1]=res_buffer[1]|0x80;						// та-же функция + взведенный бит ошибки
					write_buffer[2]=0x03;				// код ошибки - недопустимая функция -> адрес выходит за пределы заршенного
					snd_cnt=3;
				}
				break;

			}
			
			/*case 0x4E:
			{
				if(res_wr_index == 8)
				{
					
					write_buffer[0] = res_buffer[0]; 
					write_buffer[1] = 0x4E;
					write_buffer[2] = res_buffer[5];    																								// кол-во байт, которое надо прочитать
					
					uint32_t adress = MBS_WritePage << 16 | res_buffer[2] << 8 | res_buffer[3];
					uint8_t size = 3 + res_buffer[5] + 2;
					uint8_t num = res_buffer[5];
					
					for(uint8_t i=0;i<num;i++)
					{
						write_buffer[i+3] = Flash_Read_8(adress+i);
					}
					snd_cnt=size-2;
				}
				else
				{
					write_buffer[0]=res_buffer[0];					// адрес блока
					write_buffer[1]=res_buffer[1]|0x80;						// та-же функция + взведенный бит ошибки
					write_buffer[2]=0x03;				// код ошибки - недопустимая функция -> адрес выходит за пределы заршенного
					snd_cnt=3;
				}

				break;
			}*/
			case 0x4F:
			{
				//if(res_wr_index == 8)
				//{	
					write_buffer[0] = res_buffer[0]; 
					write_buffer[1] = 0x4F; 
					write_buffer[2] = res_buffer[2];
					write_buffer[3] = res_buffer[3];
					
					uint8_t fl_preJump = 0;
					if(!MBS_Fl_Stay_Btld)																									// Если не выставлен флаг остаться в загрузчике
					{
						if((uint16_t)(res_buffer[2]<<8 | res_buffer[3]) == 0x0003)													// если подфункция в запросе - остаться в загрузчике
						{
							MBS_Fl_Stay_Btld = 1;
							write_buffer[4] = ~res_buffer[4]; 
							write_buffer[5] = ~res_buffer[5];
							snd_cnt=6;
						}
						else
						{
						}
					}
					else
					{
						switch((uint16_t)(res_buffer[2]<<8 | res_buffer[3])) 															// номер подфункции диагностической функции загрузчика
						{
							case 0x0000: 	
							{
								write_buffer[4] = res_buffer[4]; 
								write_buffer[5] = res_buffer[5]; 		
								snd_cnt=6;
								break; 																							// эхо
							}
							case 0x0001: 	
							{	
								write_buffer[4] = 0x00; 
								write_buffer[5] = 0x00;
								snd_cnt=6;									
								break; 																							// получить версию бутлоадера
							}
							case 0x0002: 
							{
								write_buffer[4] = res_buffer[4]; 
								write_buffer[5] = res_buffer[5];
								fl_preJump = 1;	// взводим флаг - пред-я подготовка перед переходом 
								//MBS_Fl_Jump_Region = 1;
								snd_cnt=6;											
								break; 	// прыжок по адресу (перейти в область программы этого адреса)
							}
							case 0x0003:  
							{	
								MBS_Fl_Stay_Btld = 1;
								write_buffer[4] = ~res_buffer[4]; 
								write_buffer[5] = ~res_buffer[5];
								snd_cnt=6;
								break; 																							// остаться в загрузчике
							}
							case 0x0004:  
							{	
								MBS_WritePage += res_buffer[5];
								write_buffer[4] = res_buffer[4]; 
								write_buffer[5] = res_buffer[5]; 
								snd_cnt=6;
								break; 																							// выбор страницы памяти (по 64кбайта)
							}						
							case 0x0005:  
							{	
								write_buffer[4] = 0x00; 
								write_buffer[5] = 0x00; 													// код устройства
								snd_cnt=6;
								break; 																							// получить тип устройства
							}						
							default: 	 	 	
							{	
								write_buffer[0]=res_buffer[0];					// адрес блока
								write_buffer[1]=res_buffer[1]|0x80;						// та-же функция + взведенный бит ошибки
								write_buffer[2]=0x01;				// код ошибки -
								snd_cnt=3;
								break;		// ошибка! Используется неизвестная подфункция
							}
						}
					}
					

					if(fl_preJump)																												// Если была команда перехода в другую облать флеш памяти
					{
						MBS_JumpAdres = MBS_WritePage << 16 |res_buffer[4] << 8 | res_buffer[5];
						MBS_Fl_Jump_Region = 1;  
					}
					
				/*}
				else
				{
					write_buffer[0]=res_buffer[0];					// адрес блока
					write_buffer[1]=res_buffer[1]|0x80;						// та-же функция + взведенный бит ошибки
					write_buffer[2]=0x03;				// код ошибки - недопустимая функция -> адрес выходит за пределы заршенного
					snd_cnt=3;
				}*/
				break;
			}	
			default:
			{
				mb_err=1;
				write_buffer[0]=res_buffer[0];					// адрес блока
				write_buffer[1]=res_buffer[1]+0x80;						// та-же функция + взведенный бит ошибки
				write_buffer[2]=0x01;				// код ошибки - недопустимая функция
				snd_cnt=3;

				break;
			}

	  }
			CRCCod=CRC16(write_buffer, snd_cnt);	
			write_buffer[snd_cnt] = CRCCod & 0x00FF;			// мл. байт CRC
			write_buffer[snd_cnt+1] = CRCCod >> 8;				// ст. байт CRC
			HAL_UART_Transmit(&huart2,write_buffer,snd_cnt+2,100);
	  }



  }
//HAL_UART_Transmit(&huart2,res_buffer,res_wr_index,100);
	

  res_wr_index=0;
	 HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
