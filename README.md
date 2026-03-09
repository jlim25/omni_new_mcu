# OMNI-ROBOT
Modular Robotic Arm

# Development Setup

## C Setup
- STM32CubeIDE (2.0.0)
- STM32CubeMX

## Python Setup
First, download UV (a python management tool).

Windows: `powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"`

MacOS/Linux: `curl -LsSf https://astral.sh/uv/install.sh | sh`

Upon downloading the repo, run `uv sync` and it should download all the dependencies needed.

Ex: Run `test_can.py` -> `uv run test_can.py`

## Raspberry Pi 5 Setup
The CAN transceiver should be hooked up to the PI before powering on the PI because upon bootup, the PI will attempt to initialize the CAN driver through a custom service. GND should be ideally shared throughout the entire low-logic system. On that note, it is imperative for the PI and STM32s to share GND, otherwise CAN may not function (symptom: STM32 won't be able to initialize the CAN driver and the program will hardfault).

To check if the CAN driver was initialized correctly, type `ifconfig` and you should see CAN0 there. If it is not there, double check the wiring and reboot the RPI. Note the INT pin on the RPI CAN transceiver is necessary and it should be connected to [GPIO25](https://pinout.xyz/pinout/pin29_gpio5/).

RPI5 Details:
- Username: roomba
- Password: roomba4161
- Hostname: roombarpi
- SSH and remote connection are both enabled

The initial setup of the RPI5 involved connecting mouse, keyboard, and monitor. From there, I enabled mobile hotspot on my laptop to share internet connection with the RPI5. Given the mobile hotspot is live upon boot up, the RPI5 will automatically connect to it. Be sure that the RPI won't auto connect to any other networks. 

Typical workflow: I use remote SSH in VSC to develop and test on the RPI5. You could also use remote desktop connection on Windows to open up the RPI's Ubuntu GUI.

## RPI Transceiver
The [transceiver](https://www.amazon.ca/MCP2515-Module-TJA1050-Receiver-Controller/dp/B07Z45RLMG) doesn't support a Raspberry Pi out of the box because it expects 5V logic levels for SPI. On any RPI, the logic level 3v3 for the GPIO (including SPI). Thus, the transceiver board must be modified. See [instructions here](https://github.com/tolgakarakurt/CANBus-MCP2515-Raspi) to modify.

## STM32 Setup
To power the STM32 Nucleo without the USB, we can supply 5V to the 5V pin (pin 4 on CN4 [right connector]). However, you must remove SB9 (0 ohm bridge) for it to work. SB9 holds down nRST.

# Miscellaneous
CAN bus is configured at 500 kb/s.

# Configurable Options for STM32s
See `motorSelection.h` and `app_config.h`. By default, the CLI is turned off to save memory space. Given this is a 32-pin Nucleo, memory is a constraint.

# Motor Specs
Motor specifications are defined in `hiwonder_bus_servo.h`.