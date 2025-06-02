# STM32F4-FreeRTOS-ADC-DMA-UART

## Project Summary

This project aims to integrate key components such as **ADC with DMA**, **UART communication**, and **task synchronization using mutexes** in a FreeRTOS environment on an **STM32F4 series** microcontroller. It utilizes three FreeRTOS tasks to process ADC data and handle UART communication.

---

## Components Used

- **Microcontroller:** STM32F407VG  
- **IDE:** STM32CubeIDE  
- **RTOS:** FreeRTOS (configured via CubeMX)  
- **Programming Language:** C  
- **Communication:** UART1 (RX), UART2 (TX)  
- **Analog Inputs:** ADC1 (channels 0, 1, 2)  
- **DMA:** For transferring ADC data directly to memory  
- **GPIO:** PA5 (used for LED control)  

---

## System Architecture

### Tasks Overview

| Task Name         | Description                                                                 |
|-------------------|-----------------------------------------------------------------------------|
| `StartNormalTask` | Receives data over UART1 and echoes it back through UART2.                  |
| `StartHighTask`   | Processes ADC data and calculates voltage using a mutex for safe access.    |
| `StartLowTask`    | Monitors ADC voltage levels and turns on the LED if they exceed a threshold.|

---

## Folder Structure

- `Core/Src/`: Contains source files (`main.c`, task implementations, etc.)  
- `Core/Inc/`: Contains header files  
- `Drivers/`: STM32 HAL and CMSIS drivers  
- `FreeRTOS/`: FreeRTOS kernel files  

---

## How to Use

1. Open the project in **STM32CubeIDE**.  
2. Connect your STM32F407VG board.  
3. Flash the code to your board.  
4. Monitor UART1 and UART2 via serial terminals.  
5. Use analog inputs to test ADC reading and LED triggering.

---

## Notes

- Make sure to configure the clock and peripherals correctly in STM32CubeMX before generating code.  
- DMA must be enabled for ADC1 in circular mode.  
- Mutex usage ensures that ADC data processing is thread-safe.

---

