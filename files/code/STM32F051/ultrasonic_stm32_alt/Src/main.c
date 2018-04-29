
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


PUTCHAR_PROTOTYPE
{
HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

return ch;
}



/* Private variables ---------------------------------------------------------*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define ADC_SAMPLE_WIDTH 1024
#define POT_DELAY 100
#define MAX_PULSE_COUNT 70

#define PULSE_START 0
#define PULSE_END 1
#define PULSE_AMPLITUDE 2

static uint16_t adc_samples[ADC_SAMPLE_WIDTH];
static uint16_t pulses[MAX_PULSE_COUNT][3]; //[[pulse start, pulse end, pulse amplitude],...]
int pulse_number = 0;

static uint16_t digital_pulses[MAX_PULSE_COUNT][2]; //[[pulse start, pulse end],...]
int digital_pulse_number = 0;


volatile int adc_done = 0;
volatile int piezo_state = 0;
volatile int piezo_pulses = 0;

/////////////////////TIMERS////////////////////

void boost_pwm(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void wait_cycles(uint32_t cycles){ //0.19 us per cycle
  volatile int cycle_counter = 0;
  for(cycle_counter = 0;cycle_counter<cycles;cycle_counter++){
  }
}

void break_before(){
  HAL_GPIO_WritePin(PHASE_1_L_GPIO_Port, PHASE_1_L_Pin,0);
  HAL_GPIO_WritePin(PHASE_2_L_GPIO_Port, PHASE_2_L_Pin,0);
  HAL_GPIO_WritePin(PHASE_1_H_GPIO_Port, PHASE_1_H_Pin,0);
  HAL_GPIO_WritePin(PHASE_2_H_GPIO_Port, PHASE_2_H_Pin,0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(piezo_pulses){
    if(piezo_state){
      break_before(); //break before make.
      HAL_GPIO_WritePin(PHASE_1_L_GPIO_Port, PHASE_1_L_Pin,1);
      HAL_GPIO_WritePin(PHASE_2_L_GPIO_Port, PHASE_2_L_Pin,0);
      HAL_GPIO_WritePin(PHASE_1_H_GPIO_Port, PHASE_1_H_Pin,0);
      HAL_GPIO_WritePin(PHASE_2_H_GPIO_Port, PHASE_2_H_Pin,1);
    }
    else{
      break_before();
      HAL_GPIO_WritePin(PHASE_1_L_GPIO_Port, PHASE_1_L_Pin,0);
      HAL_GPIO_WritePin(PHASE_2_L_GPIO_Port, PHASE_2_L_Pin,1);
      HAL_GPIO_WritePin(PHASE_1_H_GPIO_Port, PHASE_1_H_Pin,1);
      HAL_GPIO_WritePin(PHASE_2_H_GPIO_Port, PHASE_2_H_Pin,0);
    }
    piezo_state = !piezo_state;
    piezo_pulses--;
  }
  else{
    HAL_GPIO_WritePin(PHASE_1_L_GPIO_Port, PHASE_1_L_Pin,0);
    HAL_GPIO_WritePin(PHASE_2_L_GPIO_Port, PHASE_2_L_Pin,0);
    HAL_GPIO_WritePin(PHASE_1_H_GPIO_Port, PHASE_1_H_Pin,0);
    HAL_GPIO_WritePin(PHASE_2_H_GPIO_Port, PHASE_2_H_Pin,0); //monitor boost voltage.
  }
}

void set_tx_frequency(uint32_t freq){
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = (24000000UL/freq);
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/////////////////////RESISTORS///////////////////////

//CS_GAIN_Pin
//CS_HP_Pin
//CS_LP_Pin
void reset_resistor(GPIO_TypeDef* CS_Port, uint16_t CS_Pin){
  HAL_GPIO_WritePin(UD_STATE_GPIO_Port, UD_STATE_Pin, 1);
  wait_cycles(POT_DELAY);
  HAL_GPIO_WritePin(CS_Port, CS_Pin, 1);
  wait_cycles(POT_DELAY);
  HAL_GPIO_WritePin(CS_Port, CS_Pin, 0);

  for(int i = 0; i<100; i++){ //overrun the 64 positions to make sure we're all the way up.
    wait_cycles(POT_DELAY);
    HAL_GPIO_WritePin(UD_STATE_GPIO_Port, UD_STATE_Pin, 0);
    wait_cycles(POT_DELAY);
    HAL_GPIO_WritePin(UD_STATE_GPIO_Port, UD_STATE_Pin, 1);
  }
  HAL_GPIO_WritePin(CS_Port, CS_Pin, 1);
  wait_cycles(POT_DELAY);
  HAL_GPIO_WritePin(UD_STATE_GPIO_Port, UD_STATE_Pin, 0);
}

void set_resistor(int value, GPIO_TypeDef* CS_Port, uint16_t CS_Pin){
  reset_resistor(CS_Port,CS_Pin);
  HAL_GPIO_WritePin(UD_STATE_GPIO_Port, UD_STATE_Pin, 0);
  wait_cycles(POT_DELAY);
  HAL_GPIO_WritePin(CS_Port, CS_Pin, 1);
  wait_cycles(POT_DELAY);
  HAL_GPIO_WritePin(CS_Port, CS_Pin, 0);

  for(int i = 0; i<(64-value); i++){ //overrun the 64 positions to make sure we're all the way up.
    wait_cycles(POT_DELAY);
    HAL_GPIO_WritePin(UD_STATE_GPIO_Port, UD_STATE_Pin, 1);
    wait_cycles(POT_DELAY);
    HAL_GPIO_WritePin(UD_STATE_GPIO_Port, UD_STATE_Pin, 0);
  }
  HAL_GPIO_WritePin(CS_Port, CS_Pin, 1);
  wait_cycles(POT_DELAY);
  HAL_GPIO_WritePin(UD_STATE_GPIO_Port, UD_STATE_Pin, 0);
}

////////////////////ADC////////////////////////

void switch_adc_channel(uint32_t channel){
  HAL_ADC_DeInit(&hadc);
  MX_ADC_Init();
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = channel;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  adc_done = 1;
}

void read_adc_channel(uint32_t channel,uint32_t len){
  HAL_ADC_Stop_DMA(&hadc);
  switch_adc_channel(channel);
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_samples, len);
  while(!adc_done);
  HAL_ADC_Stop_DMA(&hadc);
}

float get_boost_voltage(){
  read_adc_channel(ADC_CHANNEL_3,128);
  return (((adc_samples[0]/4096.0)*3.3)/0.0909);
}

void dump_raw_samples(){
  for(int i = 0;i<ADC_SAMPLE_WIDTH;i++){
    printf("%i,%i\r\n",i,adc_samples[i]);
  }
}

////////////////////PROCESSING////////////////////////
uint16_t peak_detect(uint16_t start, uint16_t end){
  uint16_t peak = 0;
  for(start;start<end;start++){
    if(adc_samples[start] > peak){
      peak = adc_samples[start];
    }
  }
  return peak;
}

uint16_t peak_sample(){
  uint16_t peak = 0;
  uint16_t peak_sample = 0;

  for(uint16_t i = 0;i<ADC_SAMPLE_WIDTH;i++){
    if(adc_samples[i] > peak){
      peak = adc_samples[i];
      peak_sample = i;
    }
  }
  return peak_sample;
}

// int reject_pulses(){
//
// }

///////////////////////////PULSES///////////////////////

void set_pulse_amplitudes(){
  for(int i = 0;i<pulse_number;i++){
    pulses[i][PULSE_AMPLITUDE] = peak_detect(pulses[i][PULSE_START],pulses[i][PULSE_END]);
  }
}

void split_into_pulses(){
  uint16_t threshold = 3;
  pulse_number = 0;
  int pulse_found = 0;
  for(uint16_t i = 0;i<ADC_SAMPLE_WIDTH;i++){
    if(pulse_number > MAX_PULSE_COUNT-2){
      break;
    }
    if(adc_samples[i] > threshold){
      if(!pulse_found){
        pulses[pulse_number][PULSE_START] = i;
      }
      pulse_found = 1;
    }
    else{
      if(pulse_found){
        pulses[pulse_number][PULSE_END] = i;
        pulse_number++;
      }
      pulse_found = 0;
    }
  }
  set_pulse_amplitudes();
}

void dump_pulses(){
  for(int i = 0;i<pulse_number;i++){
    printf("%i,%i,%i\r\n",pulses[i][PULSE_START],pulses[i][PULSE_END],pulses[i][PULSE_AMPLITUDE]);
  }
}

void transmit_pulse_train(int power){ //10,20,35,50
  HAL_TIM_Base_Start_IT(&htim16);
  piezo_pulses = power;
  while(piezo_pulses);
  break_before();
  HAL_TIM_Base_Stop_IT(&htim16);
}
/* USER CODE END 4 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  // set_resistor(35,CS_LP_GPIO_Port,CS_LP_Pin);
  // set_resistor(64,CS_HP_GPIO_Port,CS_HP_Pin);
  // set_resistor(0,CS_GAIN_GPIO_Port,CS_GAIN_Pin);

  HAL_TIM_Base_Start(&htim17); // Trying to start the base counter


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADC_Stop_DMA(&hadc);
  switch_adc_channel(ADC_CHANNEL_1);
  while (1){
    // printf("Preparing\r\n");
    // HAL_ADC_Stop_DMA(&hadc);
    // HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_samples, ADC_SAMPLE_WIDTH);
    uint16_t peak_freq = 0;
    uint16_t peak_value = 0;
    uint16_t peak_sample_ = 0;
    uint32_t freq=41000;
    for(uint32_t freq=20000;freq<40000;freq+=50){
      set_tx_frequency(freq);
      boost_pwm(0);
      transmit_pulse_train(5);
      // wait_cycles(500);
      // sample
      // adc_done = 0;
      // HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_samples, ADC_SAMPLE_WIDTH);
      // HAL_GPIO_TogglePin(SAL_ENABLE_GPIO_Port, SAL_ENABLE_Pin);
      //
      digital_pulse_number = 0;
      int pulse_found = 0;
      htim17.Instance->CNT = 0;
      while(digital_pulse_number < MAX_PULSE_COUNT && htim17.Instance->CNT < 60000){
        if(HAL_GPIO_ReadPin(COMPARATOR_GPIO_Port,COMPARATOR_Pin)){
          if(!pulse_found){
            digital_pulses[digital_pulse_number][PULSE_START] = htim17.Instance->CNT;
          }
          pulse_found = 1;
        }
        else{
          if(pulse_found){
            digital_pulses[digital_pulse_number][PULSE_END] = htim17.Instance->CNT;
            digital_pulse_number++;
          }
          pulse_found = 0;
        }
      }
      // while(!(COMPARATOR_GPIO_Port->IDR & COMPARATOR_Pin) && !adc_done){
      //   count++;
      // }
      // while(!adc_done);
      //
      // HAL_GPIO_TogglePin(SAL_ENABLE_GPIO_Port, SAL_ENABLE_Pin);
      // HAL_ADC_Stop_DMA(&hadc);

      boost_pwm(10);
      HAL_Delay(3);

      // split_into_pulses();

      // for(int i = 0;i<pulse_number;i++){
      //   printf("%i,%i,%i,%i\r\n",freq,i,pulses[i][PULSE_START],pulses[i][PULSE_AMPLITUDE]);
      // }
      // printf("%i\r\n",pulses[0][PULSE_START]);
      // }
      // for(int i = 0;i<pulse_number;i++){
      // if(pulse_number > 4){
      //   printf("%i,%i\r\n",freq,digital_pulses[i][PULSE_START],digital_pulses[i][PULSE_END]);
      // }

      // printf("%i\r\n",count);
      //
      for(int i = 0;i<digital_pulse_number;i++){
        printf("%i,%i,%i\r\n",freq,i,digital_pulses[i][PULSE_START]);
      }
      // for(int i = 0;i<pulse_number;i++){
      //   printf("%i,%i,%i,%i,%i,%i\r\n",freq,i,pulses[i][PULSE_START],pulses[i][PULSE_END],pulses[i][PULSE_AMPLITUDE],count);
      // }
      // process
      // printf("%i,%i,%i\r\n",freq,peak_detect(),);
      // int current_peak = peak_detect();
      // if(current_peak > peak_value){
      //   peak_value = current_peak;
      //   peak_freq = freq;
      //   peak_sample_ = peak_sample();
      // }
      // for(int i = 0;i<ADC_SAMPLE_WIDTH;i++){
      //   printf("%i,%i,%i\r\n",freq,i,adc_samples[i]);
      // }
      // HAL_GPIO_TogglePin(SAL_ENABLE_GPIO_Port, SAL_ENABLE_Pin);
      // HAL_Delay(1);

    }
    printf("done\n");

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

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 8;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PHASE_1_L_Pin|PHASE_2_L_Pin|SAL_ENABLE_Pin|PHASE_1_H_Pin
                          |PHASE_2_H_Pin|CS_LP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_GAIN_Pin|UD_STATE_Pin|CS_HP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COMPARATOR_Pin PA4 */
  GPIO_InitStruct.Pin = COMPARATOR_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PHASE_1_L_Pin PHASE_2_L_Pin SAL_ENABLE_Pin PHASE_1_H_Pin
                           PHASE_2_H_Pin CS_LP_Pin */
  GPIO_InitStruct.Pin = PHASE_1_L_Pin|PHASE_2_L_Pin|SAL_ENABLE_Pin|PHASE_1_H_Pin
                          |PHASE_2_H_Pin|CS_LP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_GAIN_Pin UD_STATE_Pin CS_HP_Pin */
  GPIO_InitStruct.Pin = CS_GAIN_Pin|UD_STATE_Pin|CS_HP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
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
    printf("File: %s Line: %i",file,line);
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
