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

