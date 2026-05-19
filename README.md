<img width="1535" height="1024" alt="Line-Following-Robot-with-Bluetooth-Control" src="https://github.com/user-attachments/assets/825c1928-4a9c-4936-b2d1-21e437cc1042" />


## System Architecture

The robot uses an STM32F401RE microcontroller as the main control unit.  
The TCRT5000 IR sensor array detects the black line and sends digital signals to the STM32.  
The STM32 processes sensor data and sends PWM/DIR control signals to two L298N motor drivers to control the left and right DC motors.  
The HC-05 Bluetooth module communicates with the STM32 through UART for manual control and monitoring.

Power is supplied from the battery pack to the L298N motor drivers first.  
The L298N module then provides 5V/GND to power the STM32 board.  
All modules share a common ground.

```mermaid
flowchart TD
    BAT[Battery Pack<br>Li-ion / AA<br>9V - 12V]

    L298L[L298N Motor Driver L]
    L298R[L298N Motor Driver R]

    STM[STM32F401RE<br>Microcontroller Unit<br>Nucleo-64]

    SENSOR[TCRT5000<br>IR Sensor Array]
    BT[HC-05<br>Bluetooth Module]

    LMOTOR[Left Wheels<br>DC Motors]
    RMOTOR[Right Wheels<br>DC Motors]

    BAT -->|Motor Power<br>+12V/VCC| L298L
    BAT -->|Motor Power<br>+12V/VCC| L298R

    L298L -->|5V/GND Power| STM

    STM -->|PWM / DIR<br>Control Signal| L298L
    STM -->|PWM / DIR<br>Control Signal| L298R

    L298L -->|Motor Output| LMOTOR
    L298R -->|Motor Output| RMOTOR

    SENSOR -->|Digital Input| STM

    STM <-->|UART Data Link| BT

    BAT ---|Common GND| STM
    BAT ---|Common GND| L298L
    BAT ---|Common GND| L298R

POWER FLOW
Battery Pack 9V-12V
        |
        | Motor Power
        v
L298N Motor Drivers
        |
        | 5V / GND
        v
STM32F401RE

TCRT5000 Sensor Array  --->  STM32F401RE
STM32F401RE            --->  L298N Motor Drivers
L298N Motor Drivers    --->  DC Motors
HC-05 Bluetooth Module <-->  STM32F401RE via UART
