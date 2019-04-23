# Create the project from Scratch

## Overview

This user guide will show you a step-by-step to create an application on the Cortex-M side with STM32CubeMX. The application includes the Cortex-M reading the data from sensor thru I2C bus and send the data to the Cortex-A7 thru Inter-Processor communication controller. 

**Toolchains for this guide:**

* STM32CubeMX version 5.1.0
* STM32 Cube MCU package for STM32mp1 series version 1.0.0
* STM32 System Workbench version x.x.x


## Step by Step guide 

### In CubeMX 

* Start a new project and choose the part number: 

![Choosing the controller](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/choosing%20the%20controller.png   "Choosing the controller")

* Go to System Core --> HSEM --> Activated. This will activate the HArdware semaphore. 

![HSEM](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/HSEM.png  "Hardware Semaphore")

* Go to System Core --> IPCC --> Activated. This will activate the Inter-Processor communication controller

![IPCC](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/IPCC.png  "Inter-Processor cmmunication controller")

* In IPCC configuration --> NVIC Settings --> enable IPCC RX1 occupied interrupt and IPCC TX1 free interrupt

![IPCC Interrupt](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/ipcc_interrupt.png   "IPCC Interrupt")

* Go to Connectivity --> I2C --> Choose Cortex-M4 --> choose I2C. 

![I2C bus](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/I2C.png  "I2C bus")

* In the pinout layout, look for pin **PZ0** and set it as **I2C2_SCL**

![I2C pinout](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/I2C_pin.png  "I2C Pinout")

* In the pin layout, look for pin **PA14** and set it as **GPIO_Input**. Right click on the pin --> Enter User Lable --> naming **BHI160_IN**

![Input pin for BHI160](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/BHI_pin.png   "Input pin for BHI160")

![Set User label ](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/BHI_alt_name.png  "Set User label ")

* Repeat the same procedure, look for pin **PG1**, set it as **GPIO_Output** and edit User lable into **LED_TEST**

* Go to Middlewares --> OPENAMP --> Choose Cortex-M4 -->  choose Enable to activate the Open Asymmetric Multi Processing framework. 

![Activate OPENAMP](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/OPENAMP.png  "Activate OPENAMP")

* Go to Project Manager tab --> Enter project name and project location --> Click Generate Code

![Generate Code](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/generate_code.png  "Generate Code")

* You will be prompted to a window, choose **Open Project**, STM32 System Workbench will start.

![Prompted Window](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/prompt_window.png) 

### In STM32 System Workbench

* Build project to check if the generation process is ok

* Copy the folder Remoteproc into the project. Remoteproc is located in For Workshop folder. Inside Remoteproc folder, there is a bash script which help you to run the application on the Cortex.M easier. 

 ![Add Remoteproc folder](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/add_remoteproc.png  "Add Remoteproc folder")
 
* In the folder For Workshop/Bosch_driver, copy .c files into the Src folder and .h files into the Inc folder of the project

 ![Add Bosch sensor driver](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/add_driver.png  "Add Bosch sensor driver")
 
 * In the folder For Workshop, copy lock_resource.c files into the Src folder and lock_resource.h files into the Inc folder of the project
 
 * Open main.h file, add `#include "lock_resource.h"` between `/* USER CODE BEGIN Includes */` and  `/* USER CODE END Includes */`  
 
 * Open main.c file
 
 * Insert the snippet between ``/* USER CODE BEGIN Includes */ `` and  `` /* USER CODE END Includes */  ``.   
 
```C
#include <stdin.h>  
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "bhy_support.h"
#include "bhy_uc_driver.h"
#include "fw.h"
```
 
 * Insert the snippet between `/* USER CODE BEGIN PD */` and `/* USER CODE END PD */` .
 
```C
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
```
 
 *  Insert the snippet between `/* USER CODE BEGIN PV */` and `/* USER CODE END PV */`. Here you will initialize the virtual UART, buffer for the virtual UART and buffer to store sensor data
 
```C
 VIRT_UART_HandleTypeDef huart0;

__IO FlagStatus VirtUart0RxMsg = RESET;
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;

char out_buffer[OUT_BUFFER_SIZE] = " W: 0.999  X: 0.999  Y: 0.999  Z: 0.999   \r";
uint8_t fifo[FIFO_SIZE];
```
 
*  Insert the snippet between `/* USER CODE BEGIN PFP */` and `/* USER CODE END PFP */`. Here we initialize the prototype for the UART call back, sensor callback, i2c write, i2c read, and delay functions. 
  
```C
 void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);


static void sensors_callback_rotation_vector(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id);

int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
void Delay_ms(uint32_t ms);
```
 
 *  Insert the snippet between `/* USER CODE BEGIN 1 */` and `/* USER CODE END 1 */`. Here we initialize the variables for running the sensors.  
 
```C
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
```

*  Insert the snippet between `/* USER CODE BEGIN Init */` and `/* USER CODE END Init */`. Here we check if we are running in Engineering Mode (only Cortex-M is running) or in production mode (noth cores are running). If we are in Engineering mode, we intialize the clock, because normally, the clock will be initialized by Cortex-A. We also enable hardware semaphore.

```C
if(IS_ENGINEERING_BOOT_MODE())
  {
	  SystemClock_Config();
  }

  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
```

* At section `/* Configure the system clock */`, **comment out** the function `SystemClock_Config();` since clock initialization will be taken care by the Cortex-A7.

 *  Insert the snippet between `/* USER CODE BEGIN 2*/` and `/* USER CODE END 2 */`. Here we initialize the GPIO for LED and Sensors, the virtual UART, and the sensors.
 
```C
GPIO_InitTypeDef   GPIO_InitStruct;

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
```

* Insert the snippet in the while(1) loop, between the `/* USER CODE BEGIN WHILE */` and `/* USER CODE BEGIN WHILE */`. Here we poll for the read sensor data commands and send the data to the Cortex-A7. In addition, we read all the available data in the FIFO of the sensor. 

```c
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
``` 
 
 * Finally, insert the snippet between `/* USER CODE BEGIN 4 */` and `/* USER CODE BEGIN 4 */`. Here is the body of the functions in the function prototype section. 
 
```c

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
```

### Build and Flash the application

* Build the application to check for errors

* If you did not setup the ethernet communication between the target and host, please refer [here](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz#building-the-ethernet-interface-between-the-board-and-pc-execute-only-once)

* Go to Run --> Debug Configurations --> Double click on ST's STM32 MPU Debugging

![Debug configuration](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/debug_conf.png  "Debug configuration")

 * In the Debug Configurations, in Startup tab, choose thru Linux core (Production mode) and enter this address 192.168.7.2 into Inet address field.
 
 ![Setup debug interface](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/setup_debug_inf.png  "Setup debug interface")
 
 * Click debug to start debugging the code. 
 
 * If you face the error as below, click OK and skip it because we have no ST Link debugger on board so there is no possibiliy to debug the code. 
 
 ![Error Message](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz/blob/feature/README/Document/pictures/error.png  "Error Message")
 
  * To start running the application, please refer [here](https://github.com/HaiQNguyen/Avenger_Bosch_Mezz#flashing-the-application)

