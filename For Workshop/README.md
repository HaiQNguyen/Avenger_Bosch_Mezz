# Create the project from Scratch

## Overview

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
 
 * Insert the snippet between
 
 ```c
    
#include <stdin.h>  
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "bhy_support.h"
#include "bhy_uc_driver.h"
#include "fw.h"
 
 ```  
 
 * ewqe
 
 ```sh
 sudo 
 ```
 
 