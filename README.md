# 🧠 STM32 RTOS-Based Sensor & Control System

A modular, event-driven embedded system built on **STM32 + FreeRTOS (CMSIS-RTOS v2)**, designed for efficient real-time sensor acquisition, control processing, and communication.

---

## 🚀 Overview

This project implements a **multi-tasking embedded system** that reads sensor data (IMU, temperature, potentiometer), processes it through a control algorithm, and transmits results via UART — all using a clean, modular architecture.

The system leverages:

* **Interrupt-driven design** (IMU Data Ready / FIFO)
* **RTOS task scheduling**
* **Inter-task communication (queues, semaphores, mutexes)**
* **Hardware abstraction (HAL drivers)**

---

## 🏗️ System Architecture

### 🔹 Tasks

| Task          | Responsibility                                  |
| ------------- | ----------------------------------------------- |
| `sensorTask`  | Waits for IMU interrupt → reads all sensor data |
| `controlTask` | Processes sensor data using control logic       |
| `uartTask`    | Sends formatted data over UART                  |
| `ledTask`     | Handles LED indication                          |

---

### 🔹 RTOS Communication

* **Queues**

  * `sensorQueue` → Sensor → Control
  * `uartSensorQueue` → Sensor → UART

* **Semaphores**

  * `imuInterruptSem` → Triggered by IMU interrupt (data ready)

* **Mutex**

  * `uartMutex` → Ensures safe UART access

---

### 🔹 Event Flow

```
IMU Interrupt (FIFO / DRDY)
        ↓
Semaphore Released (ISR)
        ↓
sensorTask Unblocks
        ↓
Sensor Data Acquisition
        ↓
Queue → controlTask & uartTask
        ↓
Control Processing + UART Output
```

---

## ⚙️ Key Features

### ✅ Modular Design

Code is separated into:

* `sensor.c / sensor.h`
* `control.c / control.h`
* `led.c / led.h`

This improves:

* Maintainability
* Scalability
* Code readability

---

### ✅ Event-Driven Architecture

* No polling delays
* Sensor reads triggered **only when data is ready**
* Efficient CPU utilization

---

### ✅ IMU FIFO + Interrupt Optimization

* FIFO configured to **reduce interrupt frequency**
* Example:

  * ODR = 120 Hz
  * Watermark = 60 samples → **2 Hz interrupt rate**

---

### ✅ Safe Concurrency

* Mutex-protected UART
* Queue-based communication (no shared data corruption)
* ISR → RTOS synchronization via semaphore

---

## 🧩 Peripherals Used

* **ADC** → Potentiometer input
* **I2C** → Temperature sensor
* **SPI** → IMU (Gyroscope)
* **UART** → Data logging / debugging
* **GPIO EXTI** → Interrupt input from IMU

---

## 📡 IMU Configuration Highlights

* Gyroscope enabled (high-performance mode)
* FIFO configured in continuous mode
* Watermark interrupt used for controlled sampling
* Interrupt routed to external pin (EXTI)

---

## 🛠️ Important Code Highlights

### 🔹 Interrupt Handling

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_12){
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        osSemaphoreRelease(imuInterruptSermHandle);
    }
}
```

---

### 🔹 Sensor Task (Event Driven)

```c
osSemaphoreAcquire(imuInterruptSermHandle, osWaitForever);
Sensor_ReadAll(&data);
osMessageQueuePut(sensorQueueHandle, &data, 0, 0);
```

---

### 🔹 UART Protection

```c
osMutexAcquire(uartMutexHandle, osWaitForever);
HAL_UART_Transmit(...);
osMutexRelease(uartMutexHandle);
```

---

## 📁 Project Structure

```
Core/
│── Src/
│   ├── main.c
│   ├── sensor.c
│   ├── control.c
│   ├── led.c
│
│── Inc/
│   ├── sensor.h
│   ├── control.h
│   ├── led.h
```

---

## 🧪 Example Output

```
Potentiometer: 2048
Temperature: 27.45°C
gx: 1.23 dps, gy: -0.45 dps, gz: 0.89 dps
```

---

## ⚡ Design Principles Applied

* Deterministic execution
* Real-time responsiveness
* Modularity
* Efficient resource usage
* Interrupt-driven processing

---

## 🚧 Future Improvements

* Integrate **RTOS timers** for periodic tasks
* Add **power optimization (sleep modes)**
* Expand to **IMU-based motion control (e.g., 90° rotation)**
* Transition to **full RTOS event-driven architecture**

---

## 📌 Notes

* `printf` is used for debugging but is **not ideal in RTOS systems**
* FIFO + interrupt design removes need for blocking delays
* System is scalable for more sensors and control strategies

---

## 👨‍💻 Author

Embedded Systems Developer
Focused on **real-time systems, control, and modular firmware design**

---

## 📜 License

This project is provided as-is for educational and development purposes.

---

### 🔗 Reference

Main implementation available in:
`main.c` 

---
