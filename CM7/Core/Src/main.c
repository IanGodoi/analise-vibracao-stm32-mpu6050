/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MPU6050 + Moving Average + USART3
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* MPU6050 */
#define MPU6050_ADDR       (0x68 << 1)
#define MPU6050_WHO_AM_I   0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT 0x3B

/* DSP */
#define N_SAMPLES   100
#define FILTER_SIZE 15
#define FFT_SIZE 256

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/*estrutura de configuração da fft */
float hann_window[FFT_SIZE];

arm_rfft_fast_instance_f32 fft_instance;
float fft_magnitude[FFT_SIZE/2];
float fft_mean = 0.0f;
float fft_sum = 0.0f;
uint32_t peak_bin = 0;
float peak_magnitude = 0.0f;
float peak_frequency = 0.0f;
float test_bin_magnitude = 0.0f;
float mag_bin_10 = 0.0f;
float mag_bin_30 = 0.0f;
float mag_bin_60 = 0.0f;
float mag_bin_90 = 0.0f;


/*
 * saida da fft
 * tambem tera ffft_size posicoes
 * */
float fft_output[FFT_SIZE];

float ax_fft_buffer[FFT_SIZE];
uint16_t fft_index = 0;
volatile uint8_t fft_buffer_ready = 0;

volatile uint32_t sample_count= 0;
volatile uint8_t sample_read = 0;

/* MPU6050 */
uint8_t who_am_i = 0;
uint8_t power_mgmt = 0x00;

HAL_StatusTypeDef mpu_status;

uint8_t accel_data[6];

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

/* Buffer com 100 amostras do eixo X */
int16_t ax_buffer[N_SAMPLES];

uint32_t sample_index = 0;

/* Soma das amostras */
int32_t ax_sum = 0;

/* Média das 100 amostras */
float ax_mean = 0.0f;

/* Resultado da média móvel */
float ax_filtered=0.0f;

int16_t ax_window[FILTER_SIZE];
uint8_t ax_window_index = 0;
uint8_t ax_window_count = 0;
int32_t ax_window_sum=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

  /* calculo janela hann*/
  for(uint16_t i = 0; i < FFT_SIZE; i++){
      			hann_window[i]= 0.5f * (1.0f - cosf((2.0f * PI *i)/(FFT_SIZE-1)));
      		}

  HAL_TIM_Base_Start_IT(&htim7);

    /*
     * ============================================================
     * 1. VERIFICA SE O MPU6050 RESPONDE
     * ============================================================
     */

    mpu_status = HAL_I2C_IsDeviceReady(
            &hi2c1,
            MPU6050_ADDR,
            3,
            100
    );

    if (mpu_status == HAL_OK)
    {
        BSP_LED_On(LED_GREEN);
        BSP_LED_Off(LED_RED);
    }
    else
    {
        BSP_LED_Off(LED_GREEN);
        BSP_LED_On(LED_RED);
    }


    /*
     * ============================================================
     * 2. WHO_AM_I
     *
     * Registrador 0x75
     *
     * Esperamos receber:
     *
     * 0x68
     * ============================================================
     */

    mpu_status = HAL_I2C_Mem_Read(
            &hi2c1,
            MPU6050_ADDR,
            MPU6050_WHO_AM_I,
            I2C_MEMADD_SIZE_8BIT,
            &who_am_i,
            1,
            100
    );

    if ((mpu_status == HAL_OK) && (who_am_i == 0x68))
    {
        BSP_LED_On(LED_GREEN);
        BSP_LED_Off(LED_RED);
    }
    else
    {
        BSP_LED_Off(LED_GREEN);
        BSP_LED_On(LED_RED);
    }


    /*
     * ============================================================
     * 3. ACORDA O MPU6050
     *
     * PWR_MGMT_1 = 0x00
     * ============================================================
     */

    power_mgmt = 0x00;

    mpu_status = HAL_I2C_Mem_Write(
            &hi2c1,
            MPU6050_ADDR,
            MPU6050_PWR_MGMT_1,
            I2C_MEMADD_SIZE_8BIT,
            &power_mgmt,
            1,
            100
    );

    if (mpu_status == HAL_OK)
    {
        BSP_LED_On(LED_GREEN);
        BSP_LED_Off(LED_RED);
    }
    else
    {
        BSP_LED_Off(LED_GREEN);
        BSP_LED_On(LED_RED);
    }

    /*
     * Pequeno tempo para o MPU6050 estabilizar
     * depois de sair do modo sleep.
     */
    HAL_Delay(100);

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {
    	/*FFT
    	 * quando 256 amostras estuiverem disponiveis calcula a fft do eixo x*/
    	if(fft_buffer_ready)
    	{
    		/*calcula a media das 256 amostras*/
    		fft_sum= 0.0f;

    		for (uint16_t i = 0; i < FFT_SIZE; i++)
    		{
    			fft_sum += ax_fft_buffer[i];
    		}

    		fft_mean = fft_sum/FFT_SIZE;

    		/* Remove a componente DC - remover media*/
    		for (uint16_t i = 0; i < FFT_SIZE; i++)
    		{
    		    ax_fft_buffer[i] -= fft_mean;
    		}


    		/* janela hann aplicada*/

    		for (uint16_t i = 0; i < FFT_SIZE; i++){
    			ax_fft_buffer[i] *= hann_window[i];
    		}



    		arm_rfft_fast_f32(
    				&fft_instance,
					ax_fft_buffer,
					fft_output, 0);

    		/* Magnitude da componente DC */
    		fft_magnitude[0] = fabsf(fft_output[0]);

    		/* Magnitude dos bins 1 até 127 */
    		arm_cmplx_mag_f32(
    		        &fft_output[2],
    		        &fft_magnitude[1],
    		        (FFT_SIZE / 2) - 1
    		);

    		test_bin_magnitude = fft_magnitude[119];
    		mag_bin_10 = fft_magnitude[10];
    		mag_bin_30 = fft_magnitude[30];
    		mag_bin_60 =fft_magnitude[60];
    		mag_bin_90 = fft_magnitude[90];


    		peak_magnitude = 0.0f;
    		peak_bin = 0;

    		for(uint32_t i = 1; i < FFT_SIZE/2;i++)
    		{

    			if(fft_magnitude[i] > peak_magnitude)
    			{
    				peak_magnitude = fft_magnitude[i];
    				peak_bin = i;
    			}
    		}

    		/*frequencia*/
    		peak_frequency = (float)peak_bin * (200.0f/FFT_SIZE);

    		/*libera o buffer para coletar
    		 * as proximas 256a amostras*/
    		fft_buffer_ready = 0;
    	}
    	/* controle de amostragem com  TIM7
    	 * o tim7 gera uma interrupçao a cada 10ms
    	 * na interrupçao:
    	 * sample_read =1
    	 * Portanto , so fazemos uma nova leitura do MPU6050 quando timer autorizar
    	 * */

    	if(sample_read){

    		sample_read = 0;
    		sample_count++;

        /*

         * LEITURA DO ACELERÔMETRO

         *
         * Começando no registrador 0x3B:
         *
         * 0x3B -> AX HIGH
         * 0x3C -> AX LOW
         *
         * 0x3D -> AY HIGH
         * 0x3E -> AY LOW
         *
         * 0x3F -> AZ HIGH
         * 0x40 -> AZ LOW
         *
         * Total = 6 bytes
         */

        mpu_status = HAL_I2C_Mem_Read(
                &hi2c1,
                MPU6050_ADDR,
                MPU6050_ACCEL_XOUT,
                I2C_MEMADD_SIZE_8BIT,
                accel_data,
                6,
                100
        );


        if (mpu_status == HAL_OK)
        {
            /*
             * Junta HIGH BYTE + LOW BYTE.
             *
             * O resultado é um inteiro com sinal
             * de 16 bits.
             */


            ax = (int16_t)(
                    (accel_data[0] << 8)
                    | accel_data[1]
            );

            ay = (int16_t)(
                    (accel_data[2] << 8)
                    | accel_data[3]
            );

            az = (int16_t)(
                    (accel_data[4] << 8)
                    | accel_data[5]
            );
            /*
             * BUFFER PARA FFT
             *guarda uma nova amostra do eixo x a cada 10 ms.
             *quando chegarmos a 256 fft_buffer_ready = 1
             * */
            if(!fft_buffer_ready)
            {
            	ax_fft_buffer[fft_index] = (float)ax;
            	fft_index++;

            	if(fft_index >= FFT_SIZE)
            	{
            		fft_index = 0;
            		fft_buffer_ready = 1;
            	}

            }

            /*
             * FILTRO MEDIA MOVEL EM TEMPO REAL
             *
             * janela =5 amostras
             * a cada nova leitura
             * 1- remove da soma a amostra mais antiga
             * 2-coloca a nova amostra no buffer circular;
             * 3-adiciona a nova amostra a soma
             * 4-calcula a nova media
             * */

            /*remove a soma do valor antigo desta posiçao*/
            ax_window_sum-=ax_window[ax_window_index];

            /*substitui pela nova posiçao*/
            ax_window[ax_window_index] = ax;
            ax_window_sum+=ax;
            ax_window_index++;
            if(ax_window_index >= FILTER_SIZE)
            {
            	ax_window_index = 0;
            }
            if (ax_window_count < FILTER_SIZE)
            {
            	ax_window_count ++;
            }
            /*resltado atual do filtro*/
            ax_filtered = (float)ax_window_sum / (float)ax_window_count;





            /*

             * COLETA DE 100 AMOSTRAS DO EIXO X

             */

            if (sample_index < N_SAMPLES)
            {
                /*
                 * Guarda a amostra atual.
                 */
                ax_buffer[sample_index] = ax;


                /*
                 * Acumula para calcular a média posteriormente.
                 */
                ax_sum += ax;


                /*
                 * Avança para a próxima posição.
                 */
                sample_index++;


                /*
                 * =================================================
                 * CHEGAMOS A 100 AMOSTRAS
                 * =================================================
                 */

                if (sample_index == N_SAMPLES)
                {


                    ax_mean =
                            (float)ax_sum /
                            (float)N_SAMPLES;




                }
            }


            /*
             * Leitura do MPU6050 funcionou.
             */
            BSP_LED_On(LED_GREEN);
            BSP_LED_Off(LED_RED);
        }
        else
        {
            /*
             * Erro de comunicação I2C.
             */
            BSP_LED_Off(LED_GREEN);
            BSP_LED_On(LED_RED);
        }

    	}



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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 6399;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 49;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == TIM7)
	{
		sample_read = 1;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

    __disable_irq();

    while (1)
    {
    }

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
