# Avenger96 interfaces with Bosch Sensor Mezzanine

## Overview

TBD

---
## Requirement
### Hardware

* Avenger96 - [link](https://www.arrow.com/en/campaigns/avengers)
* Bosch Sensor Mezzanine - [link](https://www.96boards.org/product/shiratech-bosch/)
* UART Serial - [link](https://www.96boards.org/product/uartserial/) or any other USB-to-Serial supporting 1.8V 
* 2 micro USB cables
* SD card, minimum 8GB
* Power supply: recommended 12V/2A

### Software

* PC with Linux operating system
* STM32CubeMX
* STM32 System Workbench
* Terminal program

---
## Bring up the application

* Power up the board
* Connect the board to the PC with USB-to-Serial interface and start the terminal program
#### Building the ethernet interface between the board and PC (execute only once)
* **On the linux machine, type:**
> sudo gedit /etc/network/interfaces 
* Add the Ethernet interface:
> allow-hotplug ens35u1
> iface ens35u1 inet static
> address 192.168.7.1
> netmask 255.255.255.0
> network 192.168.7.0
> gateway 192.168.7.1
* Disable networkManager for all Ethernet-to-USB: 
> sudo gedit /etc/NetworkManager/conf.d/stm32mp-otg-eth.conf 
* Add those lines below:
> [keyfile]
> unmanaged-devices=interface-name:ens35*;interface-name:usb*
* Reboot linux machine
* After rebooting, on the linux machine we can verify the connetion by:
> ping 192.169.0.2
* **On the Avenger board, we can also verify the connection by:**
> ping 192.168.0.1
#### Flashing the application
* Clone the repository
* Import the project into system workbench and rebuld it. There should be no error and some minor warnings
* Flash the application into the Avenger96
* If the application have not started yet, please go to: 
>cd /usr/local/projects/Avenger_Bosch_Mezz/
> ./fw_cortex_m4.sh start

* Activate the co-proc messaging:
> stty -onlcr -echo -F /dev/ttyRPMSG0
> cat /dev/ttyRPMSG0 &

* The sensor data can be read by using this command: 
> echo "read data > /dev/ttyRPMSG0

* The response message follows:
> W: 0.999  X:-0.025  Y:-0.027  Z: 0.021

* Application can be stopped by:
> ./fw_cortex_m4.sh stop

---
## Starting from scratch

TBD

---
## Change Log

#### Version 1.0.0
I2C bus can be initialize from CubeMX
Integrating BHI160 and BMM150, running Vector Rotation example
Issue with SPI  

---
##References
ST Wiki: [wiki](https://wiki.st.com/stm32mpu/wiki/Main_Page)
DH Wiki: [wiki](https://wiki.dh-electronics.com/index.php/Avenger96)
Shiratech homepage: [page](http://www.shiratech-solutions.com/products/bosch-sensor/)

---
##Contact
Quang Hai Nguyen 
email: qnguyen@arroweurope.com