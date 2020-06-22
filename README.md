# SmartHome_v2_stm32_device
stm32 smarthome device

using STM32 CubeMX 4.27 and Atollic TrueStudio 9.3

HAL Driver, Free RTOS, ....

Implemented:
- IRemote for communicate with ports.
- encoders: for counter traffic humans (fixed in/out) implement with two IR sensors,
- encode for menu navigate.
- visuality info on 7-segment led LCD on TM1638, and keyboard scanning...
- visuality info on ssd1306 LCD.
- implement USB protocol for communicate with host (implement autosearching device function).
- in progress communicate via USART with esp8266 baseboard...

PCB and  schematics on easy-eda:
