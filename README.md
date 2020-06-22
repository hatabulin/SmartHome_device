# SmartHome_v2_stm32_device_controller

Smarthome device controller based on stm32f103ret6 mcu series.

Project tools: HAL_CubeMX_4.27, Atollic TrueStudio 9.3.
Implemented Free RTOS, USB STM32 Virtual com port middlewares.

Main functions:
- IRemote for communicate with ports.
- encoders: for counter traffic humans (fixed in/out) implement with two IR sensors,
- encode for menu navigate.
- visuality info on 7-segment led LCD on TM1638, and keyboard scanning...
- visuality info on ssd1306 LCD.
- implement USB protocol for communicate with host (implement autosearching device function).
- in progress communicate via USART with esp8266 baseboard...

PCB and  schematics on easy-eda:
