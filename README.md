# Embeded Control System
Arm Exoskeleton Control System

## Project Layout
### Layer Layout
```
Control System/
├─ Driver Layer/
│  ├─ GPIO
│  ├─ CAN
│  ├─ I2C
│  ├─ UART
├─ Control Layer/
│  ├─ Posture Analysis/
│  │  ├─ MPU6050 Driver
│  │  ├─ Filter & Algorithm 
│  ├─ Motor Control/
│  │  ├─ Driver
│  ├─ PID
│  ├─ Application (Optional)/
│  │  ├─ Posture Visualization
│  │  ├─ PC Communication
```


### Pin Config
1. System Core
   1. GPIO
      1. PC0: LED light on board
      2. PC4: CAN2 Power Output Enable
      3. PC14: 5V Power Putput Enable
2. Connectivity
   1. CAN2
      1. PB12 PB13: motor driver
   2. I2C1
      1. PC9: SCL
      2. PC8: SDA
   3. UART5
      1. PC12 PD2: serial debug output

## Development Setup & Tutorial (Do it at the beginning)
### STM32 Setup
#### Download
ST-Link utility: https://www.st.com/en/development-tools/stsw-link004.html
STM32CubdeMX: https://www.st.com/en/development-tools/stm32cubemx.html
RealTerm (Serial Debug Software): https://sourceforge.net/projects/realterm/

#### Tutorial
ST-Link utility: https://blog.csdn.net/ybhuangfugui/article/details/52597133
ST-Link in keil: https://blog.csdn.net/zouxin_88/article/details/124103038
Please note: utility and keil cannot use the same st-link at the same timt

#### Step by step set up
Note: Please directly jump to Step5 unless you need to config new pin.
1. Open CtrBoard.ioc.  
2. Click Migrate if it asked. 
3. Config all IO (should already done and nothing need change). 
4. Click GENERATE CODE at the top right.
5. Open keil project file ./MDK-ARM/CtrBoard.nvprojx. 
6. Start dev using keil. Next time you can directly open keil if no IO pin needs modify.

#### Q&A
Keil Loaded Success But Not Running: https://blog.csdn.net/hjl2832/article/details/126951029
https://cloud.tencent.com/developer/article/2038019

## Debug
1. Use ```printf()```, and it will directly print to serial UART5. Please use RealTerm to read output.
Example Debug Print:
```c
printf("Debug uint8_t %d\n\r",myValue);
```
