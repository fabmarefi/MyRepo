# Motivation

Create a cheap nevertheless efficient solution to provide a EFI System for monocilindric two stroke engine based 
on speed-density calculation, focused to improve: drivability, fuel consumption, performance and reduce emissions.

# Project 

Focused in small engines who equip small bikes like: Minibike, Monkey Bike, Pocket Bike, Mopet and related. The idea
is built a basic hardware using a develop board Blue Pill (power and cheap), some basic sensor conditioners, and some 
drives to control fuel pump, injector valves and others. 
Develop a software responsible to perform all calculations and control the actuators/loads in real-time, being calibratable by 
wired serial communication/bluetooth.

## Hardware Specification

To create this project I choose the Blue Pill dev board because:
- Low cost and easy to found it
- Hardware is powerful, available different communication modules like: I2C, SPI, CAN, USART and another important resourses like timers, AD
converters etc
- Development tool to debug and trace software: flash, download, breakpoint debugging, register and memory view, serial wire trace
- Small size and very good design to create external shields using perforated boards (fast prototype)

PS: For more detailed information consult [Datasheet STM32F103C8](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)

Core:
PCB breadboard with socket to connect Blue-Pill dev board

Sensors:

TPS (Throttle position sensor)
MAP (Manifold absolut pressure)
IAT (Intake air temperature)
CTS (Coolant temperature sensor) / OTS (Oil temperature sensor)
OS  (Oxygen Sensor, or Lambda sensor, installed inside the Exaust pipe)
VRS (Variable reluctance speed)

Actuators:

Fuel pump
Fuel injector
Step motor ISC (Idle speed control)

## Software Specification

Prime fuel injection to crank engine easier

Measure in real time battery voltage and use this information to implement some compensation like:
-injector time
-detect low battery voltage (able to diagnostic problem with alternator or battery)

Scheduller to generate tasks: 20ms, 100ms and 1s to hang system tasks

Serial communication drive by interruption

Flash memory management (To emulate a E2PROM memory and store the Calibration)

Calculate the fuel amount based on TPS, MAP, IAT and VRS

Idle control (PI system responsible to manage the Step Motor)

Cranking Management

Fast Acceleration (Mixture enrichment)

Lambda Mixture Compensation (In some specific operating regimes)

Control the fuel amount using a PWM signal with a fixed frequency (20Hz) to simplify the 
signal control implementation (based on intake air flow)






