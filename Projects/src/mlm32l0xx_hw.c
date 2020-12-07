/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /*******************************************************************************
  * @file    mlm32l0xx_hw.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   system hardware driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "hw.h"
#include "radio.h"
#include "debug.h"
#include "vcom.h"
#include "timeServer.h"

/*!
 *  \brief Unique Devices IDs register set ( STM32L0xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/*!
 * \brief ADC Vbat measurement constants
 */

 /* Internal voltage reference, parameter VREFINT_CAL*/
#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FF80078))
#define LORAWAN_MAX_BAT   254

static ADC_HandleTypeDef hadc;
/*!
 * Flag to indicate if the ADC is Initialized
 */
static bool AdcInitialized = false;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

// hardware comp handle
COMP_HandleTypeDef compHandle;
// comparator callback
void ( *CompOutputLowCallback )( void );

// hv psu state
// true = no clock wiggle  (means enable psu)
// false = wiggle          (means disable psu)
static bool hvState;



// hardware watchdog handle
static IWDG_HandleTypeDef wdgHandle;

//********************************SHADAB************************************
void IMC_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();


    /*Configure GPIO pins : PA10 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

//*********************************SEND**************************************


/**
  * @brief This function initializes the hardware
  * @param None
  * @retval None
  */
void HW_Init( void )
{
  if( McuInitialized == false )
  {
#if defined( USE_BOOTLOADER )
    /* Set the Vector Table base location at 0x3000 */
    NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x3000 );
#endif

    HW_AdcInit( );

    Radio.IoInit( );
    
    HW_SPI_Init( );

    HW_RTC_Init( );

    vcom_Init( );
    
    BSP_LED_Init( LED1 );
    
    BSP_LED_Init( LED2 );
    
    BSP_LED_Init( LED3 );
    
    BSP_LED_Init( LED4 );

    McuInitialized = true;
  }
}

/**
  * @brief This function Deinitializes the hardware
  * @param None
  * @retval None
  */
void HW_DeInit( void )
{
  HW_SPI_DeInit( );
  
  Radio.IoDeInit( );
  
  vcom_DeInit( );
 
  McuInitialized = false;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 6
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */

void SystemClock_Config( void )
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_3;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Set Voltage scale1 as MCU will run at 32MHz */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief This function return a random seed
  * @note based on the device unique ID
  * @param None
  * @retval see
  */
uint32_t HW_GetRandomSeed( void )
{
  return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

/**
  * @brief This function return a unique ID
  * @param unique ID
  * @retval none
  */
void HW_GetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}


uint32_t HW_GetSemiUniqueId( void ) {
	return ( *( uint32_t* )ID3 );
}


void highVoltagePSUInitialize(void) {
    hvState = true;
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_NOCLOCK, RCC_MCODIV_1);
}


bool highVoltagePSUEnable(bool enable) {

    if(hvState != enable) {
        uint32_t clkSource = enable ? RCC_MCO1SOURCE_NOCLOCK : RCC_MCO1SOURCE_LSE;
        // currently using RCC_MCO1 (PA8) output pin
        HAL_RCC_MCOConfig(RCC_MCO1, clkSource, RCC_MCODIV_1);
        // update current state
        hvState = enable;
//PRINTF("HV PSU STATE Changed:  %s\n\r", hvState ? "No Wiggle" : "Wiggle");
        return true;
    } else {
        return false;
    }
}



void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
    // NOTE: the following is deboucing logic, as with a noise signal, many, many interrupts can be generated
    // and need to be handled at source rather than create ev queue events

    // if triggered in low power (wiggle) mode
    if( hvState == false) {
        // if triggered on level low
        if( HAL_COMP_GetOutputLevel(hcomp) == COMP_OUTPUT_LEVEL_LOW) {
            // enable the hv PSU as charge is getting low
            if( highVoltagePSUEnable(true) == true) {
                // call the registered callback which handles the 'on' timer
                CompOutputLowCallback();
            }
        } else {
          // ignore level high's as they are likely due to noise and we will either have just got a low or about to get one
        }
    } else {
        // ignore if we are already powering the hv psu, we must have triggered already, so the timer (or state machine) will turn us off
    }
}


// this is called by HAL_COMP_Init
void HAL_COMP_MspInit(COMP_HandleTypeDef *hcomp) {

    SYSCFGCLK_ENABLE();

    GPIO_InitTypeDef initStruct;

    initStruct.Mode = GPIO_MODE_ANALOG;
    initStruct.Pull = GPIO_NOPULL;
    initStruct.Speed = GPIO_SPEED_HIGH;
    HW_GPIO_Init( COMP_PORT, COMP_PIN, &initStruct );

    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
}


void InputVoltageCompInit(void ( *callback )( void )) {

    CompOutputLowCallback = callback;

    compHandle.Instance = COMP2;
    compHandle.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
    compHandle.Init.Mode = COMP_POWERMODE_ULTRALOWPOWER;
    compHandle.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;  // configured on PA3 for COMP2
    compHandle.Init.InvertingInput = COMP_INPUT_MINUS_VREFINT;
    compHandle.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED; // COMP_OUTPUTPOL_INVERTED
    compHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
    HAL_COMP_Init(&compHandle);

}

void InputVoltageCompStart(void) {
    HAL_COMP_Start_IT(&compHandle);
}

void InputVoltageCompStop(void) {
    HAL_COMP_Stop_IT(&compHandle);
}


void HW_Watchdog_Enable(uint32_t cycleTimeMs) {

    uint32_t reloadValue = 143 * (cycleTimeMs / 1000);

    // 1430 minimum to not trigger with 10 second timer  (143Hz)
    // so clock looks to be around  36.6kHz
    // so max (0xFFF) would be 28.6 seconds

    // initialize and start the watchdog
    //__HAL_RCC_LSI_ENABLE();
    wdgHandle.Instance = IWDG;
    wdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
    wdgHandle.Init.Reload = reloadValue; //0xFFF max
    wdgHandle.Init.Window = IWDG_WINDOW_DISABLE;
    HAL_IWDG_Init(&wdgHandle);
}



void HW_Watchdog_Refresh(void) {
    // refresh hardware watchdog
    HAL_IWDG_Refresh(&wdgHandle);
}

/**
  * @brief This function return the battery level
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t HW_GetBatteryLevel( void ) 
{
  uint8_t batteryLevel = 0;
  uint16_t measuredLevel = 0;
  uint32_t batteryLevelmV;

  measuredLevel = HW_AdcReadChannel( BAT_LEVEL_ADC_CHAN );

  if (measuredLevel == 0)
  {
    batteryLevelmV = 0;
  }
  else
  {
    batteryLevelmV= (( (uint32_t) VDDA_TEMP_CAL * (*VREFINT_CAL ) )/ measuredLevel);
  }

  if (batteryLevelmV > VDD_BAT)
  {
    batteryLevel = LORAWAN_MAX_BAT;
  }
  else if (batteryLevelmV < VDD_MIN)
  {
    batteryLevel = 0;
  }
  else
  {
    batteryLevel = (( (uint32_t) (batteryLevelmV - VDD_MIN)*LORAWAN_MAX_BAT) /(VDD_BAT-VDD_MIN) ); 
  }
  return batteryLevel;
}


uint16_t HW_GetChargeVoltage( void )
{
  return HW_AdcReadChannel( CHARGE_VOLTAGE_ADC_CHAN );
}


/**
  * @brief This function initializes the ADC
  * @param none
  * @retval none
  */
void HW_AdcInit( void )
{
  if( AdcInitialized == false )
  {
    AdcInitialized = true;
    GPIO_InitTypeDef initStruct;
    
    hadc.Instance  = ADC1;
    
    hadc.Init.OversamplingMode      = DISABLE;
  
    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc.Init.LowPowerAutoPowerOff  = DISABLE;
    hadc.Init.LowPowerFrequencyMode = ENABLE;
    hadc.Init.LowPowerAutoWait      = DISABLE;
    
    hadc.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc.Init.SamplingTime          = ADC_SAMPLETIME_7CYCLES_5;
    hadc.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.ContinuousConvMode    = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc.Init.DMAContinuousRequests = DISABLE;

    ADCCLK_ENABLE();
    

    HAL_ADC_Init( &hadc );

    initStruct.Mode = GPIO_MODE_ANALOG;
    initStruct.Pull = GPIO_NOPULL;
    initStruct.Speed = GPIO_SPEED_HIGH;

    // modded for Sentinel hardware
//    HW_GPIO_Init( BAT_LEVEL_PORT, BAT_LEVEL_PIN, &initStruct );
    HW_GPIO_Init( CHARGE_VOLTAGE_PORT, CHARGE_VOLTAGE_PIN, &initStruct );
}

}
/**
  * @brief This function De-initializes the ADC
  * @param none
  * @retval none
  */
void HW_AdcDeInit( void )
{
  AdcInitialized = false;
}

/**
  * @brief This function De-initializes the ADC
  * @param Channel
  * @retval Value
  */
uint16_t HW_AdcReadChannel( uint32_t Channel )
{

  ADC_ChannelConfTypeDef adcConf;
  uint16_t adcData = 0;
  
  if( AdcInitialized == true )
  {
    /* wait the the Vrefint used by adc is set */
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VREFINTRDY) == RESET) {};
      
    ADCCLK_ENABLE();
    
    /*calibrate ADC if any calibraiton hardware*/
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED );
    
    /* Deselects all channels*/
    adcConf.Channel = ADC_CHANNEL_MASK;
    adcConf.Rank = ADC_RANK_NONE; 
    HAL_ADC_ConfigChannel( &hadc, &adcConf);
      
    /* configure adc channel */
    adcConf.Channel = Channel;
    adcConf.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel( &hadc, &adcConf);

    /* Start the conversion process */
    HAL_ADC_Start( &hadc);
      
    /* Wait for the end of conversion */
    HAL_ADC_PollForConversion( &hadc, HAL_MAX_DELAY );
      
    /* Get the converted value of regular channel */
    adcData = HAL_ADC_GetValue ( &hadc);

    __HAL_ADC_DISABLE( &hadc) ;

    ADCCLK_DISABLE();
  }
  return adcData;
}

/**
  * @brief Enters Low Power Stop Mode
  * @note ARM exists the function when waking up
  * @param none
  * @retval none
  */
void LPM_EnterStopMode( void)
{
  BACKUP_PRIMASK();

  DISABLE_IRQ( );

  HW_DeInit( );
  
  /*clear wake up flag*/
  SET_BIT(PWR->CR, PWR_CR_CWUF);
  
  RESTORE_PRIMASK( );

  /* Enter Stop Mode */
  HAL_PWR_EnterSTOPMode ( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
}
/**
  * @brief Exists Low Power Stop Mode
  * @note Enable the pll at 32MHz
  * @param none
  * @retval none
  */
void LPM_ExitStopMode( void)
{
  /* Disable IRQ while the MCU is not running on HSI */

  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );

  /* After wake-up from STOP reconfigure the system clock */
  /* Enable HSI */
  __HAL_RCC_HSI_ENABLE();

  /* Wait till HSI is ready */
  while( __HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET ) {}
  
  /* Enable PLL */
  __HAL_RCC_PLL_ENABLE();
  /* Wait till PLL is ready */
  while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET ) {}
  
  /* Select PLL as system clock source */
  __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );
  
  /* Wait till PLL is used as system clock source */ 
  while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK ) {}
    
  /*initilizes the peripherals*/
  HW_Init( );

  RESTORE_PRIMASK( );
}

/**
  * @brief Enters Low Power Sleep Mode
  * @note ARM exits the function when waking up
  * @param none
  * @retval none
  */
void LPM_EnterSleepMode( void)
{
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

