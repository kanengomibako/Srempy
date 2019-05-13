/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2_ext_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile int16_t RX_BUFFER[BLOCK_SIZE*2] = {}; // ��M�o�b�t�@
volatile int16_t TX_BUFFER[BLOCK_SIZE*2] = {}; // ���M�o�b�t�@
volatile uint8_t sw[5] = {};           // �X�C�b�`�I���E�I�t���
volatile uint8_t pot[3] = {10, 50, 0}; // LEVEL RATE MODE �e�p�����[�^
volatile uint8_t mute = 0;             // �f�[�^�ۑ����m�C�Y �~���[�g�p

void writeLEVEL(uint8_t);
void writeRATE(uint8_t);
void writeMODE(uint8_t);
void saveData(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C3_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

  // I2S��DMA�J�n
  HAL_I2SEx_TransmitReceive_DMA(&hi2s2,(uint16_t*)TX_BUFFER,(uint16_t*)RX_BUFFER,BLOCK_SIZE*2);

  // ���荞�݂���M�Ƒ��M��2�񔭐����Ȃ��悤�Е��̊��荞�݂𖳌���
  HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);

  // �N�����
  ssd1306_Init();
  ssd1306_Fill(White);
  ssd1306_SetCursor(9, 9);
  ssd1306_WriteString("S", Font_16x26, Black);
  ssd1306_SetCursor(26, 9);
  ssd1306_WriteString("r", Font_16x26, Black);
  ssd1306_SetCursor(44, 9);
  ssd1306_WriteString("e", Font_16x26, Black);
  ssd1306_SetCursor(64, 9);
  ssd1306_WriteString("m", Font_16x26, Black);
  ssd1306_SetCursor(82, 9);
  ssd1306_WriteString("p", Font_16x26, Black);
  ssd1306_SetCursor(101, 9);
  ssd1306_WriteString("y", Font_16x26, Black);
  ssd1306_SetCursor(9, 40);
  ssd1306_WriteString("Simulated Phaser", Font_7x10, Black);
  ssd1306_SetCursor(45, 52);
  ssd1306_WriteString("v1.00", Font_7x10, Black);
  ssd1306_DrawPixel(102, 34, White);
  ssd1306_DrawPixel(103, 34, White);
  ssd1306_UpdateScreen();

  // I2S�̃t���[���G���[�����̏ꍇ�A�\�t�g���Z�b�g���J��Ԃ�
  HAL_Delay(300);
  if (__HAL_I2S_GET_FLAG(&hi2s2, I2S_FLAG_FRE)) NVIC_SystemReset();

  HAL_Delay(1500);

  // �ۑ��σp�����[�^�Ǎ�
  pot[0] = *(uint8_t *)0x080E0000;
  pot[1] = *(uint8_t *)0x080E0001;
  pot[2] = *(uint8_t *)0x080E0002;

  // �p�����[�^���
  ssd1306_Fill(Black);
  ssd1306_SetCursor(18, 0);
  ssd1306_WriteString("Srempy  v1.00", Font_7x10, White);
  ssd1306_SetCursor(16, 11);
  ssd1306_WriteString("LEVEL", Font_11x18, White);
  ssd1306_SetCursor(16, 29);
  ssd1306_WriteString("RATE", Font_11x18, White);
  ssd1306_SetCursor(16, 47);
  ssd1306_WriteString("MODE", Font_11x18, White);
  writeLEVEL(pot[0]);
  writeRATE(pot[1]);
  writeMODE(pot[2]);
  ssd1306_SetCursor(0, 11);
  ssd1306_WriteString(">", Font_11x18, White);
  ssd1306_UpdateScreen();

  // ����LED�_���i�N���m�F�j
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);

  // �X�C�b�`�p�J�E���^
  uint32_t swcount[5] = {};

  // �J�[�\���ʒu
  uint8_t cursorPosition = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))) // �t�b�gSW
    {
      swcount[0]++;
      if (swcount[0] == 3)
      {
        sw[0] = (sw[0] + 1) % 2;
        if (sw[0]) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET);
        else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET);
      }
    }
    else swcount[0] = 0;
    if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11))) // ���っSW
    {
      swcount[1]++;
      if (swcount[1] == 1)
      {
        ssd1306_SetCursor(0, cursorPosition*18+11);
        ssd1306_WriteString(" ", Font_11x18, White);
        if ( cursorPosition > 0 ) cursorPosition--;
        else cursorPosition = 2;
        ssd1306_SetCursor(0, cursorPosition*18+11);
        ssd1306_WriteString(">", Font_11x18, White);
        ssd1306_UpdateScreen();
      }
      if (swcount[1] == 250) saveData(); // �������Ńf�[�^�ۑ�
    }
    else swcount[1] = 0;
    if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12))) // ������SW
    {
      swcount[2]++;
      if (swcount[2] == 1)
      {
        ssd1306_SetCursor(0, cursorPosition*18+11);
        ssd1306_WriteString(" ", Font_11x18, White);
        if ( cursorPosition < 2 ) cursorPosition++;
        else cursorPosition = 0;
        ssd1306_SetCursor(0, cursorPosition*18+11);
        ssd1306_WriteString(">", Font_11x18, White);
        ssd1306_UpdateScreen();
      }
      if (swcount[2] == 250) saveData(); // �������Ńf�[�^�ۑ�
    }
    else swcount[2] = 0;
    if (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))) // �E��(+)SW
    {
      swcount[3]++;
      if (swcount[3] > 0 && cursorPosition == 0)  // LEVEL -10...+10
      {
        if (pot[0] < 20) pot[0]++; // pot[0] 0...20
        writeLEVEL(pot[0]);
      }
      if (swcount[3] > 0 && cursorPosition == 1) // RATE 0...100
      {
        if (pot[1] < 91 && swcount[3] > 2) pot[1] += 10;
        else if (pot[1] < 100) pot[1]++;
        writeRATE(pot[1]);
      }
      if (swcount[3] % 100 == 1 && cursorPosition == 2) // MODE 0...4
      {
        if (pot[2] < 4) pot[2]++;
        else pot[2] = 0;
        writeMODE(pot[2]);
      }
    }
    else swcount[3] = 0;
    if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))) // �E��(-)SW
    {
      swcount[4]++;
      if (swcount[4] > 0 && cursorPosition == 0)  // LEVEL -10...+10
      {
        if (pot[0] > 0) pot[0]--;  // pot[0] 0...20
        writeLEVEL(pot[0]);
      }
      if (swcount[4] > 0 && cursorPosition == 1) // RATE 0...100
      {
        if (pot[1] > 9 && swcount[4] > 2) pot[1] -= 10;
        else if (pot[1] > 0) pot[1]--;
        writeRATE(pot[1]);
      }
      if (swcount[4] % 100 == 1 && cursorPosition == 2) // MODE 0...4
      {
        if (pot[2] > 0) pot[2]--;
        else pot[2] = 4;
        writeMODE(pot[2]);
      }
    }
    else swcount[4] = 0;

  HAL_Delay(1);
  }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 295;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_SLAVE_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  static float xl[BLOCK_SIZE] = {}; // Lch float�v�Z�p��M�f�[�^ Rch�͖��g�p
  static float yl[BLOCK_SIZE] = {}; // Lch float�v�Z�ϑ��M�f�[�^

  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
  /* �f�[�^����M --------------------*/
  for (int i = 0; i < BLOCK_SIZE; i++)
  {
    // ��M�f�[�^���v�Z�p�f�[�^�z��� �l��-1�`+1(float)�֕ύX
    xl[i] = (float) RX_BUFFER[i*2] / 32768.0f;

    // �v�Z�σf�[�^�𑗐M�o�b�t�@�ցi���ڂ����܂������Ȃ�����1�T���v�����炷�j
    // �l��-32768�`+32767(16�r�b�g����)�֖߂�
    if (i < BLOCK_SIZE - 1) TX_BUFFER[i*2+2] = 32768.0f * yl[i];
    else TX_BUFFER[0] = 32768.0f * yl[i]; // i: BLOCK_SIZE-1
  }

  /* �G�t�F�N�g���� --------------------*/
  for (int i = 0; i < BLOCK_SIZE; i++)
  {
    // �^�񒆂̃f�[�^���珈�����邽�ߏ��Ԃ����ւ���
    // ��F�u���b�N�T�C�Y32���Ə������Ԃ�16��31�A0��15�ƂȂ�
    int j = i + BLOCK_SIZE / 2;
    if (j >= BLOCK_SIZE) j = j - BLOCK_SIZE;

    // �G�t�F�N�g������A�l�𐧌��i-32768�`+32767�͈֖̔͂߂����߁j
    yl[j] = fmaxf(fminf(bypass(xl[j]), 0.99997f), -1.0f);
  }
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
}

void writeLEVEL(uint8_t pot)
{
  char *buf[21] = {"-10", " -9", " -8", " -7", " -6", " -5", " -4",
                   " -3", " -2", " -1", "  0", " +1", " +2", " +3",
                   " +4", " +5", " +6", " +7", " +8", " +9", "+10"};
  ssd1306_SetCursor(82, 11);
  ssd1306_WriteString(buf[pot], Font_11x18, White);
  ssd1306_UpdateScreen();
}

void writeRATE(uint8_t pot)
{
  char buf;
  ssd1306_SetCursor(82, 29);
  if (pot < 100) ssd1306_WriteString(" ", Font_11x18, White);
  if (pot < 10 ) ssd1306_WriteString(" ", Font_11x18, White);
  ssd1306_WriteString(itoa(pot, &buf, 10), Font_11x18, White); // itoa�Ő��l�𕶎����
  ssd1306_UpdateScreen();
}

void writeMODE(uint8_t pot)
{
  char *buf[5] = {"STD ", "SOFT", "HIGH", "LOW ", "INV "};
  ssd1306_SetCursor(82, 47);
  ssd1306_WriteString(buf[pot], Font_11x18, White);
  ssd1306_UpdateScreen();
}

void saveData(void)
{
  mute = 1;
  HAL_Delay(5); // �~���[�g��5ms�҂�
  HAL_FLASH_Unlock(); // �t���b�V�� ���b�N����
  FLASH_EraseInitTypeDef erase;
  uint32_t error = 0;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_11;
  erase.NbSectors = 1;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  HAL_FLASHEx_Erase(&erase, &error); // �t���b�V������
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x080E0000, pot[0]); // �t���b�V������
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x080E0001, pot[1]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x080E0002, pot[2]);
  HAL_FLASH_Lock(); // �t���b�V�� ���b�N
  mute = 0;
  ssd1306_SetCursor(18, 0);
  ssd1306_WriteString("DATA SAVED!  ", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);
  ssd1306_SetCursor(18, 0);
  ssd1306_WriteString("Srempy  v1.00", Font_7x10, White);
  ssd1306_UpdateScreen();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
