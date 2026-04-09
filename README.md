SUB
---

FILES
-----

src/main.c - initialises all tasks from the component scripts and manages them  
components/*:  
arming - creates and handles the arming mutex  
board - lays out hardware configuration (pin #s, UART channels, bauds)  
control - manages motor mixing from channels and sends to motors  
ibus - handles UART RX/TX from surface side  
mavlink - mavlink library files  
motor - manages PWM and RMT channels for use with ESCs - RMT not implemented  
sensors - handles the collection and processing of sensor data to be sent to the OSD  
telemetry - sends the sensor data to the OSD

SURFACE
-------

Not yet implemented, still in build for debug of UART communication



