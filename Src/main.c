/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "openamp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "bhy_support.h"
#include "bhy_uc_driver.h"
#include "fw.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE

/* Command list */
#define LED_ON			"LED ON"
#define LED_OFF			"LED OFF"
#define SENSOR_DATA		"read data"

/* Macro for sensor */
#define FIFO_SIZE                      300
#define ROTATION_VECTOR_SAMPLE_RATE    100
#define MAX_PACKET_LENGTH              18
#define OUT_BUFFER_SIZE                60
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

IPCC_HandleTypeDef hipcc;

/* USER CODE BEGIN PV */

VIRT_UART_HandleTypeDef huart0;

__IO FlagStatus VirtUart0RxMsg = RESET;
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;

char out_buffer[OUT_BUFFER_SIZE] = " W: 0.999  X: 0.999  Y: 0.999  Z: 0.999   \r";
uint8_t fifo[FIFO_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_IPCC_Init(void);
int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);
/* USER CODE BEGIN PFP */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);


static void sensors_callback_rotation_vector(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id);

int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
void Delay_ms(uint32_t ms);

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

	/* BHY Variable*/
	uint8_t                    *fifoptr           = NULL;
	uint8_t                    bytes_left_in_fifo = 0;
	uint16_t                   bytes_remaining    = 0;
	uint16_t                   bytes_read         = 0;
	bhy_data_generic_t         fifo_packet;
	bhy_data_type_t            packet_type;
	BHY_RETURN_FUNCTION_TYPE   result;

	/*
	 * the remapping matrix for BHA or BHI here should be configured according to
	 * its placement on customer's PCB. For details, please check
	 * 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document.
	 * */
	int8_t bhy_mapping_matrix_config[3*3] = {0,1,0,-1,0,0,0,0,1};


	/* the remapping matrix for Magnetometer should be configured according to
	 * its placement on customer's PCB. For details, please check
	 * 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document.
	 * */
	int8_t mag_mapping_matrix_config[3*3] = {0,1,0,1,0,0,0,0,-1};

	/* the sic matrix should be calculated for customer platform
	 * by logging uncalibrated magnetometer data. The sic matrix
	 * here is only an example array (identity matrix). Customer
	 * should generate their own matrix. This affects magnetometer
	 * fusion performance.
	 * */
	float sic_array[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  if(IS_ENGINEERING_BOOT_MODE())
  {
	  SystemClock_Config();
  }

  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();

  /* IPCC initialisation */
   MX_IPCC_Init();
  /* OpenAmp initialisation ---------------------------------*/
  MX_OPENAMP_Init(RPMSG_REMOTE, NULL);

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  GPIO_InitTypeDef   GPIO_InitStruct;

  /* Configure STATUS_LED Pin as output*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  PERIPH_LOCK(LED_STATUS_GPIO_Port);
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);
  PERIPH_UNLOCK(LED_STATUS_GPIO_Port);

  /* Configure ERROR_LED Pin as output*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = LED_ERROR_Pin;
  PERIPH_LOCK(LED_ERROR_GPIO_Port);
  HAL_GPIO_Init(LED_ERROR_GPIO_Port, &GPIO_InitStruct);
  PERIPH_UNLOCK(LED_ERROR_GPIO_Port);

  /* Configure TEST_LED Pin as output*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = LED_TEST_Pin;
  PERIPH_LOCK(LED_TEST_GPIO_Port);
  HAL_GPIO_Init(LED_TEST_GPIO_Port, &GPIO_InitStruct);
  PERIPH_UNLOCK(LED_TEST_GPIO_Port);

  /* Configure input pin for BHI160*/
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = BHI160_IN_Pin;
  PERIPH_LOCK(BHI160_IN_GPIO_Port);
  HAL_GPIO_Init(BHI160_IN_GPIO_Port, &GPIO_InitStruct);
  PERIPH_UNLOCK(BHI160_IN_GPIO_Port);

  if (VIRT_UART_Init(&huart0) != VIRT_UART_OK)
  {
	  Error_Handler();
  }

  if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
  {
	  Error_Handler();
  }

  /* Sensor Initialization ----------------------------------------------------------------------------------------*/
  if(bhy_driver_init(&bhy_firmware_image))
  {
	  Error_Handler();
  }

  /* wait for the bhy trigger the interrupt pin go down and up again */
  while (HAL_GPIO_ReadPin(BHI160_IN_GPIO_Port, BHI160_IN_Pin))
  {
  }

  while (!HAL_GPIO_ReadPin(BHI160_IN_GPIO_Port, BHI160_IN_Pin))
  {
  }

  /*
   * the remapping matrix for BHI and Magmetometer should be configured here
   * to make sure rotation vector is calculated in a correct coordinates system.
   * */
  bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_config);
  bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_MAG, mag_mapping_matrix_config);
  bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_GYRO, bhy_mapping_matrix_config);
  /* This sic matrix setting affects magnetometer fusion performance. */
  bhy_set_sic_matrix(sic_array);

  /* install the callback function for parse fifo data */
  if(bhy_install_sensor_callback(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, sensors_callback_rotation_vector))
  {
	Error_Handler();
  }

  /* install the callback function for parse fifo data */
  if(bhy_enable_virtual_sensor(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, ROTATION_VECTOR_SAMPLE_RATE, 0, VS_FLUSH_NONE, 0, 0))
  {
	Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	OPENAMP_check_for_message();

	if(VirtUart0RxMsg == SET)
	{
		/*if we receive correct command from core A7, send the sensor data*/
		VirtUart0RxMsg = RESET;
		HAL_Delay(200);
		char msg_to_transmit[MAX_BUFFER_SIZE];
		uint16_t msg_size = 0;

		msg_size = snprintf(msg_to_transmit, MAX_BUFFER_SIZE, out_buffer);
		msg_size += snprintf(msg_to_transmit + msg_size, MAX_BUFFER_SIZE, "%s\n", VirtUart0ChannelBuffRx);
		VIRT_UART_Transmit(&huart0, (uint8_t*)msg_to_transmit, msg_size);
		HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
	}

	/*Reading FIFO of the sensor*/
	while (!HAL_GPIO_ReadPin(BHI160_IN_GPIO_Port, BHI160_IN_Pin) && !bytes_remaining)
	{
	}

	bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
	bytes_read           += bytes_left_in_fifo;
	fifoptr              = fifo;
	packet_type          = BHY_DATA_TYPE_PADDING;

	do
	{
		/* this function will call callbacks that are registered */
		result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);

		/* prints all the debug packets */
		if (packet_type == BHY_DATA_TYPE_DEBUG)
		{
			bhy_print_debug_packet(&fifo_packet.data_debug, bhy_printf);
		}

		/* the logic here is that if doing a partial parsing of the fifo, then we should not parse  */
		/* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
		/* packet */
	}
	while ((result == BHY_SUCCESS) && (bytes_read > (bytes_remaining ? MAX_PACKET_LENGTH : 0)));

	bytes_left_in_fifo = 0;

	if (bytes_remaining)
	{
		/* shifts the remaining bytes to the beginning of the buffer */
		while (bytes_left_in_fifo < bytes_read)
		{
			fifo[bytes_left_in_fifo++] = *(fifoptr++);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** RCC Clock Config 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                              |RCC_CLOCKTYPE_PCLK5|RCC_CLOCKTYPE_MPU;
  RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_HSI;
  RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_HSI;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_HSI;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV1;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C12;
  PeriphClkInit.I2c12ClockSelection = RCC_I2C12CLKSOURCE_BCLK;
  PeriphClkInit.EthClockSelection = RCC_ETHCLKSOURCE_OFF;
  PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_OFF;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_OFF;
  PeriphClkInit.Lptim23ClockSelection = RCC_LPTIM23CLKSOURCE_OFF;
  PeriphClkInit.Lptim45ClockSelection = RCC_LPTIM45CLKSOURCE_OFF;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.TIMG1PresSelection = RCC_TIMG1PRES_DEACTIVATED;
  PeriphClkInit.TIMG2PresSelection = RCC_TIMG2PRES_DEACTIVATED;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10707DBC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOZ_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
    if( !strncmp((char *)huart->pRxBuffPtr, LED_ON, strlen(LED_ON)))
    	HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin, GPIO_PIN_SET);

    else if(!strncmp((char *)huart->pRxBuffPtr, LED_OFF, strlen(LED_OFF)))
		HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin, GPIO_PIN_RESET);

    else if(!strncmp((char *)huart->pRxBuffPtr, SENSOR_DATA, strlen(SENSOR_DATA)))
    {
    	VirtUart0RxMsg = SET;
    }
}


/**
  * @brief Call back function of the sensors
  * @param tbd
  * @retval tbd
  */
static void sensors_callback_rotation_vector(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    float temp;
    uint8_t index;

    /*processing the data*/
    temp = sensor_data->data_quaternion.w / 16384.0f; /* change the data unit by dividing 16384 */
    out_buffer[3] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[4] = floor(temp) + '0';

    for (index = 6; index <= 8; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

    temp = sensor_data->data_quaternion.x / 16384.0f;
    out_buffer[13] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[14] = floor(temp) + '0';

    for (index = 16; index <= 18; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

    temp = sensor_data->data_quaternion.y / 16384.0f;
    out_buffer[23] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[24] = floor(temp) + '0';

    for (index = 26; index <= 28; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

    temp = sensor_data->data_quaternion.z / 16384.0f;
    out_buffer[33] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[34] = floor(temp) + '0';

    for (index = 36; index <= 38; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

}

/**
  * @brief 	i2c write function
  * 		this is the implementation of write function in the sensor library
  * 		please see bhy_support.c
  *
  * @param 	addr		i2c address of the sensor
  * @param 	reg			register being writen
  * @param 	*p_buff		pointer to data
  * @param 	szie		size of the data
  * @retval BHY_SUCCES OR BHY_ERROR
  */
int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{
	uint8_t tx_buff[size + 1];
	tx_buff[0] = reg;
	memcpy(tx_buff + 1, p_buf, size * sizeof(uint8_t));
	if(HAL_I2C_Master_Transmit(&hi2c2, addr << 1, tx_buff, size + 1, 0xFF) != HAL_OK)
		return BHY_ERROR;

	return BHY_SUCCESS;
}

/**
  * @brief 	i2c read function
  * 		this is the implementation of read function in the sensor library
  * 		please see bhy_support.c
  *
  * @param 	addr		i2c address of the sensor
  * @param 	reg			register we want to read
  * @param 	*p_buff		pointer to store data
  * @param 	szie		size of the data
  * @retval BHY_SUCCES OR BHY_ERROR
  */
int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{
	if(HAL_I2C_Master_Transmit(&hi2c2, addr << 1, &reg, 1, 0xFF) != HAL_OK)
		return BHY_ERROR;

	if(HAL_I2C_Master_Receive(&hi2c2, ((addr << 1) | 1), p_buf, size, 0xFF) != HAL_OK)
		return BHY_ERROR;

	return BHY_SUCCESS;
}

/**
  * @brief 	Delay function, in milisecond
  * 		This is the implementation of the delay function in the sensor library
  *
  * @param 	ms	 number of miliseconds
  * @retval None
  */
void Delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
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
	while(1)
	{
		/*blocking the application and blink error LED*/
		HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
		HAL_Delay(200);
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
