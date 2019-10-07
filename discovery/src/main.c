
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm32f3_discovery.h"

#include "sensors.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_system.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int US_G_Echo = 0, US_C_Echo = 0, US_D_Echo = 0;
int US_AV_G_mes = 0, US_AV_C_mes = 0, US_AV_D_mes = 0;
int US_AR_G_mes = 0, US_AR_C_mes = 0, US_AR_D_mes = 0;


int UserPressButton=0;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;

volatile char US_Flag=0;
volatile char AHRS_Flag=0;

float eulerBuffer[3];

typedef union FloatToBufferUnion{
        float val;
        uint8_t buffer[4];
} FloatToBuffer;

const uint32_t CAN_US1_id = 0x000;
const uint32_t CAN_US2_id = 0x001;
const uint32_t CAN_OM1_id = 0x101;
const uint32_t CAN_OM2_id = 0x102;

//const char CAN_AHRS_id_X = 0x05;
//const char CAN_AHRS_id_Y = 0x06;
//const char CAN_AHRS_id_Z = 0x07;


extern const uint16_t LED_PIN[LEDn];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void CAN_FilterConfig(void);
int CAN_read(void);
void CAN_Send(uint8_t* data, const uint32_t id);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM1)
    {
        //resulta en us prescaler 63 (clock 64 Mhz) Pulse positif
        US_G_Echo = HAL_TIM_ReadCapturedValue (&htim1,TIM_CHANNEL_2);//mesure capture Duree pulse
        __HAL_TIM_SET_COUNTER(&htim1,0);// mise a zero compteur apres capture
    }
    if (htim->Instance==TIM2)
    {
        US_C_Echo =	HAL_TIM_ReadCapturedValue (&htim2,TIM_CHANNEL_2);
        __HAL_TIM_SET_COUNTER(&htim2,0);// mise a zero compteur apres capture

    }
    if (htim->Instance==TIM3)
    {
        US_D_Echo =	HAL_TIM_ReadCapturedValue (&htim3,TIM_CHANNEL_2);
        __HAL_TIM_SET_COUNTER(&htim3,0);// mise a zero compteur apres capture

    }

}

void SYS_MicroDelay(uint32_t delay)
{
    volatile uint32_t cnt=(delay*6)+5;

    while (cnt >0) {
        cnt--;
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


    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    hcan.pTxMsg = &TxMessage;
    hcan.pRxMsg = &RxMessage;

    GyroDRDFlag = 1;
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* Calcul du delai */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    MX_CAN_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */

    /* CAPTURE US */
    // Gauche
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    // Centre
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    // Droit
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

    CAN_FilterConfig();

    BSP_LED_Init(LED10);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100000); // Systick a� 10 �s

    Demo_GyroConfig();
    UpdateGyroBias();
    Demo_CompassConfig();
    GyroExtiConfig();
    readAllSensors(GyroTempBuffer, AccTempBuffer, MagTempBuffer);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint8_t CAN_US_mes[8];
    FloatToBuffer CAN_AHRS_mes;
    uint8_t CAN_OMx_mes[8];
    char bufferMsg[100];

    for(int i = 0; i < 8; i++)
    {
        CAN_US_mes[i] = 0;
        CAN_OMx_mes[i] = 0;
        CAN_AHRS_mes.buffer[i]=0;
    }

    while (1)
    {
        if (US_Flag==1)
        {
            US_Flag=0;

            /* US_AV_G D_Trig ; US_AV_D_Trig ; US_AR_C_Trig */
            //pulse quelques 10 aines de us
            HAL_GPIO_WritePin( US_AV_G_Trig_GPIO_Port, US_AV_G_Trig_Pin, GPIO_PIN_SET); //PC0	: US_AV_G_Trig � 1
            HAL_GPIO_WritePin( US_AV_D_Trig_GPIO_Port, US_AV_D_Trig_Pin, GPIO_PIN_SET); //PC2 : US_AV_D_Trig � 1
            HAL_GPIO_WritePin( US_AR_C_Trig_GPIO_Port, US_AR_C_Trig_Pin, GPIO_PIN_SET); //PC4 : US_AR_C_Trig � 1
            //HAL_Delay(1); //10µs
            SYS_MicroDelay(10); // 10µs
            HAL_GPIO_WritePin( US_AV_G_Trig_GPIO_Port, US_AV_G_Trig_Pin, GPIO_PIN_RESET); //PC0	: US_AV_G_Trig � 0
            HAL_GPIO_WritePin( US_AV_D_Trig_GPIO_Port, US_AV_D_Trig_Pin, GPIO_PIN_RESET); //PC2 : US_AV_D_Trig � 0
            HAL_GPIO_WritePin( US_AR_C_Trig_GPIO_Port, US_AR_C_Trig_Pin, GPIO_PIN_RESET); //PC4 : US_AR_C_Trig � 0
            HAL_Delay(30); // 30ms

            US_AV_G_mes = US_G_Echo/58;
            US_AV_D_mes = US_D_Echo/58;
            US_AR_C_mes = US_C_Echo/58;

            CAN_US_mes[0] = (US_AV_G_mes >> 8) & 0xFF;
			CAN_US_mes[1] = US_AV_G_mes & 0xFF;
			CAN_US_mes[2] = (US_AV_D_mes >> 8) & 0xFF;
			CAN_US_mes[3] = US_AV_D_mes & 0xFF;
			CAN_US_mes[4] = (US_AR_C_mes >> 8) & 0xFF;
			CAN_US_mes[5] = US_AR_C_mes & 0xFF;
			CAN_Send(CAN_US_mes, CAN_US1_id);

            /* US_AV_C_Trig ; US_AR_G_Trig ; US_AR_D_Trig */
            //pulse quelques 10 aines de us
            HAL_GPIO_WritePin( US_AV_C_Trig_GPIO_Port, US_AV_C_Trig_Pin, GPIO_PIN_SET); // PC1 : US_AV_C_Trig � 1
            HAL_GPIO_WritePin( US_AR_G_Trig_GPIO_Port, US_AR_G_Trig_Pin, GPIO_PIN_SET); // PC3 : US_AR_G_Trig � 1
            HAL_GPIO_WritePin( US_AR_D_Trig_GPIO_Port, US_AR_D_Trig_Pin, GPIO_PIN_SET); // PC5 : US_AR_D_Trig � 1
            //HAL_Delay(1); //10µs
            SYS_MicroDelay(10); // 10µs
            HAL_GPIO_WritePin( US_AR_D_Trig_GPIO_Port, US_AR_D_Trig_Pin, GPIO_PIN_RESET); // PC1 : US_AV_C_Trig � 0
            HAL_GPIO_WritePin( US_AR_G_Trig_GPIO_Port, US_AR_G_Trig_Pin, GPIO_PIN_RESET); // PC3 : US_AR_G_Trig � 0
            HAL_GPIO_WritePin( US_AV_C_Trig_GPIO_Port, US_AV_C_Trig_Pin, GPIO_PIN_RESET); // PC5 : US_AR_D_Trig � 0
            HAL_Delay(30); // 30ms

            US_AR_G_mes = US_G_Echo/58;
            US_AR_D_mes = US_D_Echo/58;
            US_AV_C_mes = US_C_Echo/58;

            CAN_US_mes[0] = (US_AR_G_mes >> 8) & 0xFF;
			CAN_US_mes[1] = US_AR_G_mes & 0xFF;
			CAN_US_mes[2] = (US_AR_D_mes >> 8) & 0xFF;
			CAN_US_mes[3] = US_AR_D_mes & 0xFF;
			CAN_US_mes[4] = (US_AV_C_mes >> 8) & 0xFF;
			CAN_US_mes[5] = US_AV_C_mes & 0xFF;
			CAN_Send(CAN_US_mes, CAN_US2_id);

            // TODO: Revoir le delai de 1 seconde
            // HAL_Delay(100000);
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (AHRS_Flag)
        {
            AHRS_Flag = 0;

            getEulerAngles(eulerBuffer);

            sprintf (bufferMsg, "Axe X(Pitch): %d\n\r",(int)eulerBuffer[0]);
            HAL_UART_Transmit(&huart2, (uint8_t*)bufferMsg, strlen(bufferMsg), 1000);

            sprintf (bufferMsg, "Axe Y(Roll): %d\n\r",(int)eulerBuffer[2]);
            HAL_UART_Transmit(&huart2, (uint8_t*)bufferMsg, strlen(bufferMsg), 1000);

            sprintf (bufferMsg, "Axe Z(Yaw): %d\n\r\n\r",(int)eulerBuffer[1]);
            HAL_UART_Transmit(&huart2, (uint8_t*)bufferMsg, strlen(bufferMsg), 1000);

            CAN_AHRS_mes.val = eulerBuffer[0];
            //CAN_Send(CAN_AHRS_mes.buffer, CAN_AHRS_id_X);
            CAN_OMx_mes[0] = CAN_AHRS_mes.buffer[3];
            CAN_OMx_mes[1] = CAN_AHRS_mes.buffer[2];
            CAN_OMx_mes[2] = CAN_AHRS_mes.buffer[1];
            CAN_OMx_mes[3] = CAN_AHRS_mes.buffer[0];

            CAN_AHRS_mes.val = eulerBuffer[1];
            CAN_OMx_mes[4] = CAN_AHRS_mes.buffer[3];
            CAN_OMx_mes[5] = CAN_AHRS_mes.buffer[2];
            CAN_OMx_mes[6] = CAN_AHRS_mes.buffer[1];
            CAN_OMx_mes[7] = CAN_AHRS_mes.buffer[0];

            //CAN_Send(CAN_AHRS_mes.buffer, CAN_AHRS_id_Z);
            CAN_Send(CAN_OMx_mes, CAN_OM1_id); // envoi du yaw et pitch

            CAN_AHRS_mes.val = eulerBuffer[2];
            //CAN_Send(CAN_AHRS_mes.buffer, CAN_AHRS_id_Y);
            CAN_OMx_mes[0] = CAN_AHRS_mes.buffer[3];
			CAN_OMx_mes[1] = CAN_AHRS_mes.buffer[2];
            CAN_OMx_mes[2] = CAN_AHRS_mes.buffer[1];
            CAN_OMx_mes[3] = CAN_AHRS_mes.buffer[0];
            CAN_Send(CAN_OMx_mes, CAN_OM2_id);

        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

    /* USER CODE END 3 */
}

void HAL_SYSTICK_Callback()
{
    static uint32_t counter=0;
    static uint32_t counterAHRS=0;

    counter++;
    counterAHRS++;

    if (counterAHRS>=500){
        counterAHRS=0;
        AHRS_Flag=1;
    }

    if (counter>=1000) {
        counter=0;
        US_Flag=1;
    }
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
            |RCC_PERIPHCLK_TIM1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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

/* CAN init function */
static void MX_CAN_Init(void)
{

    hcan.Instance = CAN;
    hcan.Init.Prescaler = 8;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SJW = CAN_SJW_1TQ;
    hcan.Init.BS1 = CAN_BS1_7TQ;
    hcan.Init.BS2 = CAN_BS2_2TQ;
    hcan.Init.TTCM = DISABLE;
    hcan.Init.ABOM = DISABLE;
    hcan.Init.AWUM = DISABLE;
    hcan.Init.NART = DISABLE;
    hcan.Init.RFLM = DISABLE;
    hcan.Init.TXFP = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x2000090E;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Analogue filter 
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Digital filter 
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 63;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 63;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 63;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pins : USART2 TX */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : USART2 RX */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
     PA11   ------> USB_DM
     PA12   ------> USB_DP
 */
static void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
            |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
            |LD6_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, US_AV_G_Trig_Pin|US_AV_C_Trig_Pin|US_AV_D_Trig_Pin|US_AR_G_Trig_Pin
            |US_AR_C_Trig_Pin|US_AR_D_Trig_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
    GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
            |MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
            |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
            |LD6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : US_AV_G_Trig_Pin US_AV_C_Trig_Pin US_AV_D_Trig_Pin US_AR_G_Trig_Pin
                           US_AR_C_Trig_Pin US_AR_D_Trig_Pin */
    GPIO_InitStruct.Pin = US_AV_G_Trig_Pin|US_AV_C_Trig_Pin|US_AV_D_Trig_Pin|US_AR_G_Trig_Pin
            |US_AR_C_Trig_Pin|US_AR_D_Trig_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : DM_Pin DP_Pin */
    GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF14_USB;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_EXTI_InitTypeDef EXTI_InitStruct;

    /**/
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE2);

    /**/
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE4);

    /**/
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE5);

    /**/
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE0);

    /**/
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE1);

    /**/
    LL_GPIO_SetPinPull(DRDY_GPIO_Port, DRDY_Pin, LL_GPIO_PULL_NO);

    /**/
    LL_GPIO_SetPinPull(MEMS_INT3_GPIO_Port, MEMS_INT3_Pin, LL_GPIO_PULL_NO);

    /**/
    LL_GPIO_SetPinPull(MEMS_INT4_GPIO_Port, MEMS_INT4_Pin, LL_GPIO_PULL_NO);

    /**/
    LL_GPIO_SetPinPull(MEMS_INT1_GPIO_Port, MEMS_INT1_Pin, LL_GPIO_PULL_NO);

    /**/
    LL_GPIO_SetPinPull(MEMS_INT2_GPIO_Port, MEMS_INT2_Pin, LL_GPIO_PULL_NO);

    /**/
    LL_GPIO_SetPinMode(DRDY_GPIO_Port, DRDY_Pin, LL_GPIO_MODE_INPUT);

    /**/
    LL_GPIO_SetPinMode(MEMS_INT3_GPIO_Port, MEMS_INT3_Pin, LL_GPIO_MODE_INPUT);

    /**/
    LL_GPIO_SetPinMode(MEMS_INT4_GPIO_Port, MEMS_INT4_Pin, LL_GPIO_MODE_INPUT);

    /**/
    LL_GPIO_SetPinMode(MEMS_INT1_GPIO_Port, MEMS_INT1_Pin, LL_GPIO_MODE_INPUT);

    /**/
    LL_GPIO_SetPinMode(MEMS_INT2_GPIO_Port, MEMS_INT2_Pin, LL_GPIO_MODE_INPUT);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
    EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
    EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_5;
    EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
    EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
    EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (USER_BUTTON_PIN == GPIO_Pin)
    {
        while (BSP_PB_GetState(BUTTON_USER) != RESET);
        UserPressButton = 1;
    } else {
        Sensors_GPIO_EXTI_Callback(GPIO_Pin);
    }
}


int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
    return ch;
}



void CAN_FilterConfig(void)
{
    CAN_FilterConfTypeDef sFilterConfig;

    sFilterConfig.FilterNumber = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.BankNumber = 14;

    if( HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK )
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}



void CAN_Send(uint8_t* data, const uint32_t id)
{
    hcan.pTxMsg->StdId = id;
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->DLC = 8;

    //hcan.pTxMsg->Data[0] = data & 0x000000FF;
    //hcan.pTxMsg->Data[1] = (data >> 8 ) & 0x000000FF;
    //hcan.pTxMsg->Data[2] = (data >> 16) & 0x000000FF;
    //hcan.pTxMsg->Data[3] = (data >> 24) & 0x000000FF;
    hcan.pTxMsg->Data[0] = data[0];
    hcan.pTxMsg->Data[1] = data[1];
    hcan.pTxMsg->Data[2] = data[2];
    hcan.pTxMsg->Data[3] = data[3];
    hcan.pTxMsg->Data[4] = data[4];
    hcan.pTxMsg->Data[5] = data[5];
    hcan.pTxMsg->Data[6] = data[6];
    hcan.pTxMsg->Data[7] = data[7];

    if( HAL_CAN_Transmit(&hcan, 10) != HAL_OK )
    {
        Error_Handler();
    }
}

int CAN_read(void)
{
    if( HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY )
    {
        Error_Handler();
    }

    if( HAL_CAN_Receive(&hcan, CAN_FIFO0, 10) != HAL_OK )
    {
        // Reception error
        Error_Handler();
    }

    if( hcan.pRxMsg->StdId != 0x11 )
        Error_Handler();

    if( hcan.pRxMsg->IDE != CAN_ID_STD )
        Error_Handler();

    if( hcan.pRxMsg->DLC != 2 )
        Error_Handler();

    return (hcan.pRxMsg->Data[0] << 8 | hcan.pRxMsg->Data[1]);

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
    BSP_LED_On(LED10);
    printf("%s | line : %d", file, line);
    /*while(1)
  {
  }*/
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
