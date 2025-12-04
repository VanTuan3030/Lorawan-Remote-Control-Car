# Hand Gesture Controlled Car via LoRaWAN (STM32 + MPU6050)

This project builds a remote-controlled robot car system using **Hand Gesture Control**, powered by **LoRa (Long Range)** wireless communication technology. The system allows for stable vehicle control over long distances with excellent obstacle penetration, significantly outperforming traditional RF or Bluetooth solutions.

## 📡 LoRa Technology - Project Highlight

This is the core feature that sets this project apart. Instead of using standard radio waves, the system utilizes the **LoRa Ra-02 (SX1278)** module operating at **433MHz**.

### Why LoRa?
* **Superior Range:** Capable of transmitting signals from several hundred meters to kilometers (in ideal conditions), solving the signal loss issue when the car travels far away.
* **High Interference Immunity:** Uses **CSS (Chirp Spread Spectrum)** modulation technique, ensuring signals are not attenuated or interfered with by other electronic devices.
* **Obstacle Penetration:** The 433MHz LoRa signal has a long wavelength, allowing for vehicle control even when obstructed by walls or large objects.

### Communication Protocol
The system operates on a **Point-to-Point (P2P)** model:
1.  **Data Acquisition:** The STM32 microcontroller at the Transmitter reads tilt angles (Roll, Pitch) from the MPU6050 sensor.
2.  **Packetizing:** Data is packed into a `Struct` to ensure integrity.
3.  **Modulation & Transmission:** The SX1278 module modulates this packet into LoRa waves and transmits it.
4.  **Decoding & Execution:** The Receiver captures the waves, decodes the packet back into motor control signals.

## 🚀 Key Features

* **Gesture Control:**
    * 👋 **Tilt Forward:** Move Forward.
    * 👋 **Tilt Backward:** Move Backward.
    * 👋 **Tilt Left/Right:** Turn Left/Right.
    * 👋 **Keep Level:** Stop.
* **Real-time Processing:** Ultra-fast response speed thanks to the STM32F103 microcontroller (32-bit ARM Cortex-M3).
* **Powerful & Flexible:** The car uses 4 DC motors with an L298N Driver, enabling versatile multi-directional movement.

## 🛠️ Hardware Requirements

### 1. Transmitter (Handheld)
* **Microcontroller:** STM32F103C8T6 (Blue Pill).
* **Sensor:** MPU6050 (6-axis Accelerometer & Gyroscope).
* **Communication:** LoRa Ra-02 Module (SX1278) + 433MHz Antenna (Spring/External).
* **Power:** 3.7V Li-ion Battery or 9V Battery.

### 2. Receiver (Car)
* **Microcontroller:** STM32F103C8T6 (Blue Pill).
* **Communication:** LoRa Ra-02 Module (SX1278).
* **Motors:** L298N Driver + 4 DC Geared Motors + Robot Chassis.
* **Power:** High-capacity Rechargeable Battery (e.g., 2x 18650 cells in series ~7.4V).

## 🔌 Pinout Diagram

Communication between STM32 and LoRa SX1278 uses the **SPI** standard.

### Transmitter Unit

| Device | STM32 Pin | Function |
| :--- | :--- | :--- |
| **LoRa Ra-02 (NSS)** | PA4 | Chip Select (CS) |
| **LoRa Ra-02 (SCK)** | PA5 | SPI Clock |
| **LoRa Ra-02 (MISO)**| PA6 | SPI Data In |
| **LoRa Ra-02 (MOSI)**| PA7 | SPI Data Out |
| **LoRa Ra-02 (RST)** | PB0 | Reset |
| **LoRa Ra-02 (DIO0)**| PB1 | Interrupt (Tx Done) |
| **MPU6050 (SCL)** | PB6 | I2C Clock |
| **MPU6050 (SDA)** | PB7 | I2C Data |

### Receiver Unit

| Device | STM32 Pin | Function |
| :--- | :--- | :--- |
| **LoRa Ra-02** | PA4, PA5, PA6, PA7 | SPI Connection (Same as Transmitter) |
| **L298N (IN1)** | PA0 | Left Motor (+) |
| **L298N (IN2)** | PA1 | Left Motor (-) |
| **L298N (IN3)** | PA2 | Right Motor (+) |
| **L298N (IN4)** | PA3 | Right Motor (-) |

## 💻 Installation and Usage

1.  **Software & Libraries:**
    * IDE: **Keil C V5** or **STM32CubeIDE**.
    * Configuration: **STM32CubeMX** (Configure SPI for LoRa, I2C for MPU6050).
    * Libraries: Import driver libraries for `SX1278` and `MPU6050`.

2.  **Flashing Firmware:**
    * Connect the ST-Link V2 programmer.
    * Upload `Transmitter.hex` code to the handheld circuit.
    * Upload `Receiver.hex` code to the car circuit.

3.  **Operation:**
    * Turn on the car power first, then the handheld controller.
    * Wait 1-2 seconds for the LoRa modules to initialize and synchronize frequencies.
    * Tilt your hand and observe the car's movement.

## 👤 Authors

* **Student Team:**
    * Ngo Huynh Quoc Huy - 6251020057
    * Vo Van Tuan - 6251020094
    * Ha Nhat Chuong - 6251020037
* **Instructor:** Dr. Le Tien Loc
* **Class:** Industrial Electronics & Informatics K62
* **University:** University of Transport and Communications - HCMC Campus

---
*Project for the Course Final Report - IoT & LoRa Applications.*
