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
#include "bhy_support.h"
#include "bhy_uc_driver.h"
#include "Bosch_PCB_7183_di01_BMI160-7183_di01.2.1.10836_170103.h"
#include "bmm150.h"
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



#define FIFO_SIZE                      300 //TODO Porting
#define MAX_PACKET_LENGTH              18 //TODO Porting
#define TICKS_IN_ONE_SECOND            32000.0F //TODO Porting
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

IPCC_HandleTypeDef hipcc;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
VIRT_UART_HandleTypeDef huart0;

__IO FlagStatus VirtUart0RxMsg = RESET;
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;


/* system timestamp */
uint32_t bhy_timestamp = 0; //TODO Porting
uint8_t fifo[FIFO_SIZE]; //TODO Porting


typedef enum BMI160_STATE
{
	BMI160_IDLE,
	BMI160_INIT,
	BMI160_READ,
	BMI160_ERROR
}BMI160_State;

volatile BMI160_State bmi_state = BMI160_IDLE;

float   time_stamp    = 0;
uint8_t sensor_type   = 0;
int16_t x_raw         = 0;
int16_t y_raw         = 0;
int16_t z_raw         = 0;
float   x_data        = 0;
float   y_data        = 0;
float   z_data        = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_IPCC_Init(void);
static void MX_SPI2_Init(void);
int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);
/* USER CODE BEGIN PFP */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);

void DirtyDebug(char *msg)
{
	memset(VirtUart0ChannelBuffRx, 0, VirtUart0ChannelRxSize);
	sprintf(VirtUart0ChannelBuffRx, msg);
	VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffRx, VirtUart0ChannelRxSize);
}

static void timestamp_callback(bhy_data_scalar_u16_t *new_timestamp); //TODO Porting
static void sensors_callback_acc(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id); //TODO Porting

int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);

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

	//TODO Porting
	int8_t ret;
	/* BHY Variable*/
	uint8_t                    *fifoptr           = NULL;
	uint8_t                    bytes_left_in_fifo = 0;
	uint16_t                   bytes_remaining    = 0;
	uint16_t                   bytes_read         = 0;
	bhy_data_generic_t         fifo_packet;
	bhy_data_type_t            packet_type;
	BHY_RETURN_FUNCTION_TYPE   result;
	int8_t                    bhy_mapping_matrix_init[3*3]   = {0};
	int8_t                    bhy_mapping_matrix_config[3*3] = {0,1,0,-1,0,0,0,0,1};


	struct bmm150_dev dev;
	int8_t rslt = BMM150_OK;


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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  GPIO_InitTypeDef   GPIO_InitStruct;

  /* Configure STATUS_LED Pin as output*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  PERIPH_LOCK(LED_STATUS_GPIO_Port);
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);
  PERIPH_UNLOCK(LED_STATUS_GPIO_Port);

  /* Configure STATUS_LED Pin as output*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = LED_ERROR_Pin;
  PERIPH_LOCK(LED_ERROR_GPIO_Port);
  HAL_GPIO_Init(LED_ERROR_GPIO_Port, &GPIO_InitStruct);
  PERIPH_UNLOCK(LED_ERROR_GPIO_Port);

  /* Configure STATUS_LED Pin as output*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = LED_TEST_Pin;
  PERIPH_LOCK(LED_TEST_GPIO_Port);
  HAL_GPIO_Init(LED_TEST_GPIO_Port, &GPIO_InitStruct);
  PERIPH_UNLOCK(LED_TEST_GPIO_Port);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = BMP388_CS_Pin;
  PERIPH_LOCK(BMP388_CS_GPIO_Port);
  HAL_GPIO_Init(BMP388_CS_GPIO_Port, &GPIO_InitStruct);
  PERIPH_UNLOCK(BMP388_CS_GPIO_Port);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = BME680_CS_Pin;
  PERIPH_LOCK(BME680_CS_GPIO_Port);
  HAL_GPIO_Init(BME680_CS_GPIO_Port, &GPIO_InitStruct);
  PERIPH_UNLOCK(BME680_CS_GPIO_Port);

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

  //uint8_t I2C_TX[10] = {0};
  //uint8_t I2C_RX[10] = {0};

  //uint8_t SPI_TX[10] = {0};
  //uint8_t SPI_RX[10] = {0};

  HAL_GPIO_WritePin(BMP388_CS_GPIO_Port, BMP388_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BME680_CS_GPIO_Port, BME680_CS_Pin, GPIO_PIN_SET);


  /* Sensor interface over I2C */
  	dev.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
  	dev.intf = BMM150_I2C_INTF;
  	dev.read = sensor_i2c_read;
  	dev.write = sensor_i2c_write;
  	dev.delay_ms = HAL_Delay;

  	rslt = bmm150_init(&dev);


  	/* Setting the power mode as normal */
	dev.settings.pwr_mode = BMM150_NORMAL_MODE;
	rslt = bmm150_set_op_mode(&dev);

	/* Setting the preset mode as Low power mode
	i.e. data rate = 10Hz XY-rep = 1 Z-rep = 2*/
	dev.settings.preset_mode = BMM150_PRESETMODE_LOWPOWER;
	rslt = bmm150_set_presetmode(&dev);

  /*****************************************************/
  if(bhy_driver_init(&bhy1_fw))
	{
		//DirtyDebug("Error Driver Init \r\n");
		//bmi_state = BMI160_ERROR;
	  Error_Handler();
	}

	/* wait for the bhy trigger the interrupt pin go down and up again */
	while (HAL_GPIO_ReadPin(BHI160_IN_GPIO_Port, BHI160_IN_Pin));

	while (!HAL_GPIO_ReadPin(BHI160_IN_GPIO_Port, BHI160_IN_Pin));

	/* To get the customized version number in firmware, it is necessary to read Parameter Page 2, index 125 */
	/* to get this information. This feature is only supported for customized firmware. To get this customized */
	/* firmware, you need to contact your local FAE of Bosch Sensortec. */
	//bhy_read_parameter_page(BHY_PAGE_2, PAGE2_CUS_FIRMWARE_VERSION, (uint8_t*)&bhy_cus_version, sizeof(struct cus_version_t));
	//DEBUG("cus version base:%d major:%d minor:%d\n", bhy_cus_version.base, bhy_cus_version.major, bhy_cus_version.minor);

	/* config mapping matrix, for customer platform, this remapping matrix need to be changed */
	/* according to 'Application Note Axes remapping of BHA250(B) /BHI160(B)' document.       */
	bhy_mapping_matrix_get(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_init);
	bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_config);
	bhy_mapping_matrix_get(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_init);

	/* install time stamp callback */
	bhy_install_timestamp_callback(VS_WAKEUP, timestamp_callback);
	bhy_install_timestamp_callback(VS_NON_WAKEUP, timestamp_callback);


	/* install the callback function for parse fifo data */
	if(bhy_install_sensor_callback(VS_TYPE_ACCELEROMETER, VS_WAKEUP, sensors_callback_acc))
	{
		//DirtyDebug("Fail to install sensor callback\r\n");
		//bmi_state = BMI160_ERROR;
		Error_Handler();
	}

	/* enables the virtual sensor */
	if(bhy_enable_virtual_sensor(VS_TYPE_ACCELEROMETER, VS_WAKEUP, 10, 0, VS_FLUSH_NONE, 0, 0))
	{
		//DirtyDebug("Fail to enable sensor\r\n");
		//bmi_state = BMI160_ERROR;
		Error_Handler();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  rslt = bmm150_read_mag_data(&dev);

	  	/* Print the Mag data */

	OPENAMP_check_for_message();

	if(VirtUart0RxMsg == SET)
	{
		VirtUart0RxMsg = RESET;
		HAL_Delay(200);
		char msg_to_transmit[MAX_BUFFER_SIZE];
		uint16_t msg_size = 0;

		msg_size = snprintf(msg_to_transmit, MAX_BUFFER_SIZE, "acc %.2f %.2f %.2f ", x_data, y_data, z_data);
		msg_size += snprintf(msg_to_transmit + msg_size, MAX_BUFFER_SIZE, "MAG X : %0.2f \t MAG Y : %0.2f \t MAG Z : %0.2f \n" ,dev.data.x, dev.data.y, dev.data.z);
		msg_size += snprintf(msg_to_transmit + msg_size, MAX_BUFFER_SIZE, "%s\n", VirtUart0ChannelBuffRx);
		VIRT_UART_Transmit(&huart0, (uint8_t*)msg_to_transmit, msg_size);
		HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
	}


#if 1
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
	HAL_GPIO_TogglePin(LED_TEST_GPIO_Port, LED_TEST_Pin);
#endif

#if 0
	SPI_TX[0] = 0x80;

	HAL_GPIO_WritePin(BMP388_CS_GPIO_Port, BMP388_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	//HAL_SPI_TransmitReceive(&hspi2, SPI_TX, SPI_RX, 3, 0xFF);
	HAL_SPI_Transmit(&hspi2, SPI_TX, 1, 0xFF);
	while(HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_TX);
	HAL_SPI_Receive(&hspi2, SPI_RX, 2, 0xFF);
	while(HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY_RX);
	HAL_GPIO_WritePin(BMP388_CS_GPIO_Port, BMP388_CS_Pin, GPIO_PIN_SET);

	memset(VirtUart0ChannelBuffRx, 0, VirtUart0ChannelRxSize);
	sprintf(VirtUart0ChannelBuffRx, "SPI ID: 0x%x, 0x%x \r\n", SPI_RX[0], SPI_RX[1]);
	VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffRx, VirtUart0ChannelRxSize);


	SPI_TX[0] = 0xD0;

	HAL_GPIO_WritePin(BME680_CS_GPIO_Port, BME680_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi2, SPI_TX, 1, 0xFF);
	HAL_SPI_Receive(&hspi2, SPI_RX, 1, 0xFF);
	HAL_GPIO_WritePin(BME680_CS_GPIO_Port, BME680_CS_Pin, GPIO_PIN_SET);

	memset(VirtUart0ChannelBuffRx, 0, VirtUart0ChannelRxSize);
	sprintf(VirtUart0ChannelBuffRx, "SPI ID: 0x%x \r\n", SPI_RX[0]);
	VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffRx, VirtUart0ChannelRxSize);

	HAL_Delay(2000);
#endif


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
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSI;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 25;
  RCC_OscInitStruct.PLL4.PLLP = 2;
  RCC_OscInitStruct.PLL4.PLLQ = 2;
  RCC_OscInitStruct.PLL4.PLLR = 2;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_1;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
  RCC_OscInitStruct.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SPI23
                              |RCC_PERIPHCLK_I2C12;
  PeriphClkInit.I2c12ClockSelection = RCC_I2C12CLKSOURCE_BCLK;
  PeriphClkInit.Spi23ClockSelection = RCC_SPI23CLKSOURCE_PLL4;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOZ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    /* copy received msg in a variable to sent it back to master processor in main infinite loop*/
    //VirtUart0ChannelRxSize = huart->RxXferSize < MAX_BUFFER_SIZE? huart->RxXferSize : MAX_BUFFER_SIZE-1;


    if( memcmp(huart->pRxBuffPtr, LED_ON, VirtUart0ChannelRxSize - 1) == 0)
    	HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin, GPIO_PIN_SET);

    else if( memcmp(huart->pRxBuffPtr, LED_OFF, VirtUart0ChannelRxSize - 1) == 0)
		HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin, GPIO_PIN_RESET);

    else if(!strncmp((char *)huart->pRxBuffPtr, "bmi160 read", strlen("bmi160 read")))
    //else if( memcmp(huart->pRxBuffPtr, "bmi160 read", VirtUart0ChannelRxSize - 1) == 0)
    {
    	VirtUart0RxMsg = SET;
    }

    //memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
    //VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffRx, VirtUart0ChannelRxSize);
}




/********************************************************************************/

/*                                 FUNCTIONS                                    */

/********************************************************************************/
static void timestamp_callback(bhy_data_scalar_u16_t *new_timestamp)
{
    /* updates the system timestamp */
    bhy_update_system_timestamp(new_timestamp, &bhy_timestamp);
}


static void sensors_callback_acc(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
	HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    /* Since a timestamp is always sent before every new data, and that the callbacks   */
    /* are called while the parsing is done, then the system timestamp is always equal  */
    /* to the sample timestamp. (in callback mode only)                                 */
    time_stamp = (float)(bhy_timestamp) / TICKS_IN_ONE_SECOND;
    //DEBUG("sensor_id = %d\n", sensor_id);
    //TODO replace
    switch(sensor_id)
    {
        case VS_ID_ACCELEROMETER:

        case VS_ID_ACCELEROMETER_WAKEUP:
            x_raw  = sensor_data->data_vector.x;
            y_raw  = sensor_data->data_vector.y;
            z_raw  = sensor_data->data_vector.z;
            /* The resolution is  15bit ,the default range is 4g, actual acceleration equals: raw_data/(exp(2,15) == 32768) */
            x_data = (float)x_raw / 32768.0f * 4.0f;
            y_data = (float)y_raw / 32768.0f * 4.0f;
            z_data = (float)z_raw / 32768.0f * 4.0f;
            //DEBUG("Time:%6.3fs acc %f %f %f\n", time_stamp, x_data, y_data, z_data);
            //TODO replace
            break;

        default:
            //DEBUG("unknown id = %d\n", sensor_id);
            //TODO replace
        	break;
    }
}

int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{
	//TODO porting I2c code
	uint8_t tx_buff[100];
	tx_buff[0] = reg;
	memcpy(tx_buff + 1, p_buf, size * sizeof(uint8_t));
	HAL_I2C_Master_Transmit(&hi2c2, addr << 1, tx_buff, size + 1, 0xFF);
	return BHY_SUCCESS;
}

int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{
	//TODO porting I2C code
	HAL_I2C_Master_Transmit(&hi2c2, addr << 1, &reg, 1, 0xFF);
	HAL_I2C_Master_Receive(&hi2c2, ((addr << 1) | 1), p_buf, size, 0xFF);
	return BHY_SUCCESS;
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
