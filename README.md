# baseband_sampler
Arduino shield sized baseband sampler. It uses an STM32 F303RE Nucleo board and communicates over USB-UART (230400 baud, 8n1).

<img src="https://github.com/M17-Project/baseband_sampler/blob/main/1.jpg?raw=true" width="700">

## What does it do?
The shield board contains:
- **MCP33131-10-E/MS** 16-bit 1Msps ADC converter,
- **LM4120AIM5-2.5** 2.5V reference,
- **LDL1117S18R** 1.8V low-drop voltage regulator.

The working principle is simple. After being interrogated by the host PC, by sending a single byte over UART, the device collects 19,200 samples of the signal presented at the `Input` port. The sampling trigger signal has to be provided by the user as a TTL signal at the `Trig` port. Every rising edge starts a conversion. After all samples are collected, all the data is sent to a PC as a string of `uint16_t` values.<br>
At the power-up, the ADC performs a calibration sequence taking approximately 500 ms. The blue button on the Nucleo can be used to force re-calibration when the user needs it (significant ambient temperature change etc.). Pressing the button initializes the same sequence.

## Contents
- `CubeIDE` - a complete CubeIDE project, this is where the source code is
- `EAGLE` - the schematic and PCB design files (EAGLE 7.7.0) along with zipped gerbers (tailored for JLCPCB).
- `Matlab` contains a demo Matlab application

## Demo Matlab application
Hit the `Start` button to send the acquisition command to the board. After a few seconds you should see the eye plot.

<img src="https://github.com/M17-Project/baseband_sampler/blob/main/2.png?raw=true" width="500">

## References
- [MCP33111 family datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP33131-MCP33121-MCP33111-Family-Data-Sheet-DS20006122A.pdf)
