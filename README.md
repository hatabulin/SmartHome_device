# SmartHome_v2_stm32_device_controller

Smarthome device controller based on stm32f103ret6 mcu series.

Project tools:
- HAL_CubeMX_4.27
- Atollic TrueStudio 9.3.

Implemented:
- Free RTOS
- USB STM32 Virtual com port 
middlewares.

Main functions:
- IRemote for communicate with ports.
- encoders: for counter traffic humans (fixed in/out) implement with two IR sensors.
- encode for menu navigate.
- visual info on 7-segment led LCD on TM1638.
- keyboard functions on the same chip (TM1638) for control eight (8) output power keys.
- visual info on ssd1306 LCD.
- implement few smart routines (configurable auto on/off output , e.g. timeout sleep mode, ...)
- implement USB protocol for communicate with host (implement autosearching device function).

In progress:
- communicate via USART with esp8266 baseboard - another wireless part this full project.
- PCB and  schematics in easy-eda
