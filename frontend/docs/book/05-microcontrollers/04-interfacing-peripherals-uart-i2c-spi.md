---
id: book-05-microcontrollers-04-interfacing-peripherals-uart-i2c-spi
title: 04-interfacing-peripherals-uart-i2c-spi
sidebar_position: 4
---

--- 
sidebar_position: 4
title: Interfacing Peripherals (UART, I2C, SPI)
---

## 04-Interfacing Peripherals (UART, I2C, SPI)

Robots are complex systems comprising many individual components: sensors, actuators, communication modules, displays, and more. Microcontrollers need efficient ways to talk to these various peripherals. **Serial communication protocols** like UART, I2C, and SPI provide standardized methods for data exchange between the microcontroller and other devices. Understanding these protocols is crucial for integrating diverse hardware into your robotic projects.

### 4.1 UART (Universal Asynchronous Receiver-Transmitter)

UART is one of the oldest and most fundamental forms of serial communication. It's an **asynchronous** protocol, meaning there's no shared clock signal between the sender and receiver; instead, both must agree on a common **baud rate** (data transmission speed).

#### 4.1.1 Key Characteristics

*   **Two Wires:** Uses two dedicated wires for data transmission:
    *   **TX (Transmit):** Sends data from the microcontroller.
    *   **RX (Receive):** Receives data into the microcontroller.
*   **Point-to-Point:** Typically used for communication between two devices only.
*   **Full-Duplex:** Can send and receive data simultaneously.
*   **Frame-based:** Data is sent in "frames" that include start bits, data bits (5-9 bits), optional parity bits, and stop bits.
*   **Speed:** Relatively slow (common baud rates: 9600, 115200 bps).
*   **Error Checking:** Optional parity bit provides very basic error checking.

#### 4.1.2 Applications in Robotics

*   **Debugging:** `Serial.print()` in Arduino uses UART to send data to the Serial Monitor.
*   **GPS Modules:** Receive NMEA data from GPS receivers.
*   **Bluetooth Modules (HC-05/06):** Communicate with a smartphone or computer.
*   **Wi-Fi Modules (ESP8266 as a module):** Send AT commands to control the Wi-Fi chip.
*   **Inter-microcontroller Communication:** Two Arduinos can talk to each other.

#### 4.1.3 Arduino Implementation

Arduino boards have at least one hardware UART (on digital pins 0 and 1). Larger boards like Mega have multiple.
*   **Hardware Serial:** Uses `Serial.begin()`, `Serial.print()`, `Serial.read()`.
*   **Software Serial:** The `SoftwareSerial.h` library allows using any digital pins for UART communication, though it's less efficient and can be prone to timing issues.

### 4.2 I2C (Inter-Integrated Circuit)

I2C is a **synchronous, multi-master, multi-slave** serial bus developed by Philips. It uses only two wires, making it very popular for connecting many low-speed peripheral ICs.

#### 4.2.1 Key Characteristics

*   **Two Wires:**
    *   **SDA (Serial Data Line):** Carries the data.
    *   **SCL (Serial Clock Line):** Carries the clock signal (synchronous communication).
*   **Multi-Master, Multi-Slave:** Multiple master devices can control multiple slave devices on the same bus. Each slave device has a unique 7-bit (or 10-bit) address.
*   **Half-Duplex:** Data can only be sent in one direction at a time.
*   **Open-Drain/Open-Collector:** Requires pull-up resistors on both SDA and SCL lines.
*   **Speed:** Moderate (typically 100 kHz, 400 kHz, up to 3.4 MHz).
*   **Addressing:** Master sends a slave's address to initiate communication.

#### 4.2.2 Applications in Robotics

*   **IMUs:** Many accelerometers, gyroscopes, and magnetometers use I2C.
*   **Environmental Sensors:** Temperature, humidity, pressure sensors (e.g., BME280).
*   **Displays:** OLED, character LCDs.
*   **Real-Time Clocks (RTC):** Modules that keep track of time.
*   **EEPROM:** External memory chips.

#### 4.2.3 Arduino Implementation

Arduino uses the `Wire.h` library for I2C communication.
*   **Master:** `Wire.begin()`, `Wire.beginTransmission()`, `Wire.write()`, `Wire.endTransmission()`, `Wire.requestFrom()`, `Wire.read()`.
*   **Slave:** `Wire.begin(address)`, `Wire.onReceive()`, `Wire.onRequest()`.

**Diagram 4.1: I2C Bus Topology**

```mermaid
graph TD
    A[Microcontroller (Master)] -- SDA/SCL --> B(Sensor 1 (Slave))
    A -- SDA/SCL --> C(IMU (Slave))
    A -- SDA/SCL --> D(Display (Slave))
    subgraph I2C Bus
        B -- SDA/SCL --> C
        C -- SDA/SCL --> D
    end
```

*Description: A simplified diagram of an I2C bus showing a master microcontroller communicating with multiple slave devices (sensors, IMU, display) using only two shared data lines (SDA, SCL).*

### 4.3 SPI (Serial Peripheral Interface)

SPI is a **synchronous, master-slave** serial bus that is generally faster and more flexible than I2C.

#### 4.3.1 Key Characteristics

*   **Four Wires (typically):**
    *   **MOSI (Master Out Slave In):** Master sends data to slave.
    *   **MISO (Master In Slave Out):**
    *   **SCK (Serial Clock):** Clock signal from master to synchronize data.
    *   **SS (Slave Select) / CS (Chip Select):** Master uses a separate SS/CS pin for each slave to enable/disable communication with that specific slave.
*   **Multi-Slave:** Can connect multiple slaves (but each needs a dedicated SS/CS pin).
*   **Full-Duplex:** Can send and receive data simultaneously.
*   **Speed:** High speed (up to tens of MHz).
*   **No Addressing:** Slaves are selected by asserting their SS/CS pin LOW.

#### 4.3.2 Applications in Robotics

*   **SD Card Modules:** For logging data.
*   **Displays:** Faster displays (e.g., TFT, some OLEDs).
*   **Flash Memory Chips:** External non-volatile storage.
*   **Radio Transceivers:** For wireless communication (e.g., NRF24L01).
*   **High-Resolution ADCs/DACs:** For precise analog signal conversion.
*   **Other Microcontrollers:** For inter-microcontroller communication.

#### 4.3.3 Arduino Implementation

Arduino uses the `SPI.h` library.
*   **Master:** `SPI.begin()`, `SPI.beginTransaction()`, `SPI.transfer()`, `SPI.endTransaction()`.
*   **Specific Pins:** Typically uses fixed pins (e.g., Uno: 10/SS, 11/MOSI, 12/MISO, 13/SCK).

**Diagram 4.2: SPI Bus Topology**

```mermaid
graph TD
    A[Microcontroller (Master)] -- MOSI/MISO/SCK --> S1(SD Card Module)
    A -- SS1 (Chip Select) --> S1
    A -- MOSI/MISO/SCK --> S2(Display)
    A -- SS2 (Chip Select) --> S2
    A -- MOSI/MISO/SCK --> S3(Radio Transceiver)
    A -- SS3 (Chip Select) --> S3
```

*Description: A simplified diagram of an SPI bus showing a master microcontroller communicating with multiple slave devices. Note the shared MOSI, MISO, SCK lines, but separate Slave Select (SS) lines for each slave.*

### 4.4 Choosing the Right Protocol

| Feature       | UART                               | I2C                                   | SPI                                   |
| :------------ | :--------------------------------- | :------------------------------------ | :------------------------------------ |
| **Wires**     | 2 (TX, RX)                         | 2 (SDA, SCL) + Pull-ups               | 4 (MOSI, MISO, SCK, SS)              |
| **Speed**     | Slow (up to ~1 Mbps)               | Moderate (up to ~3.4 Mbps)            | Fast (up to ~50 Mbps)                 |
| **Topology**  | Point-to-point                     | Multi-master, multi-slave             | Single-master, multi-slave            |
| **Duplex**    | Full-duplex                        | Half-duplex                           | Full-duplex                           |
| **Addressing**| No, direct connection              | 7/10-bit addresses                   | Chip Select (SS/CS) pin per slave     |
| **Complexity**| Simple                             | Moderate (addressing, pull-ups)       | Moderate (chip select logic)          |
| **Use Case**  | GPS, Bluetooth, console debug      | IMUs, RTC, EEPROM, environmental sensors | SD cards, displays, fast ADCs, radios |

By carefully selecting and properly implementing these serial communication protocols, you can seamlessly integrate a wide array of peripherals, greatly expanding your robot's capabilities.

--- 

### C++ Example: Conceptual UART Communication (AT Command)

This C++ example simulates sending AT commands over UART to a hypothetical Bluetooth module.

```cpp
#include <iostream>
#include <string>
#include <queue> // To simulate serial buffers
#include <chrono>
#include <thread>

// Simulate a Serial port for UART communication
class SimulatedSerial {
private:
    std::string port_name;
    int baud_rate;
    std::queue<char> rx_buffer; // Incoming data for the MCU
    std::queue<char> tx_buffer; // Outgoing data from the MCU

public:
    SimulatedSerial(std::string name, int baud) : port_name(name), baud_rate(baud) {
        std::cout << "SimulatedSerial '" << port_name << "' at " << baud_rate << " baud started." << std::endl;
    }

    // Simulate MCU sending data
    void write(const std::string& data) {
        std::cout << "[MCU TX] Sending: '" << data << "'" << std::endl;
        for (char c : data) {
            tx_buffer.push(c);
        }
    }

    // Simulate MCU receiving data
    std::string readStringUntil(char terminator) {
        std::string received_data = "";
        while (!rx_buffer.empty()) {
            char c = rx_buffer.front();
            rx_buffer.pop();
            if (c == terminator) {
                break;
            }
            received_data += c;
        }
        return received_data;
    }

    // Simulate external device (e.g., Bluetooth module) sending data to MCU
    void inject_into_rx_buffer(const std::string& data) {
        std::cout << "[BT RX] Injecting into MCU buffer: '" << data << "'" << std::endl;
        for (char c : data) {
            rx_buffer.push(c);
        }
    }

    // Simulate external device reading from MCU (clearing TX buffer)
    void clear_tx_buffer() {
        while(!tx_buffer.empty()) tx_buffer.pop();
    }
};

int main() {
    SimulatedSerial BluetoothSerial("UART1", 9600);

    std::cout << "--- UART (AT Command) Communication Simulation ---" << std::endl;

    // Simulate sending "AT" command and receiving "OK"
    BluetoothSerial.write("AT\r\n"); // \r\n are carriage return and newline
    BluetoothSerial.clear_tx_buffer(); // External device reads it

    BluetoothSerial.inject_into_rx_buffer("OK\r\n"); // External device responds
    std::string response = BluetoothSerial.readStringUntil('\n');
    std::cout << "[MCU RX] Received: '" << response << "'" << std::endl;

    // Simulate sending "AT+NAME=MyRobot"
    BluetoothSerial.write("AT+NAME=MyRobot\r\n");
    BluetoothSerial.clear_tx_buffer();
    BluetoothSerial.inject_into_rx_buffer("OK\r\n");
    response = BluetoothSerial.readStringUntil('\n');
    std::cout << "[MCU RX] Received: '" << response << "'" << std::endl;
    
    // Simulate sending a command and getting an ERROR
    BluetoothSerial.write("AT+WRONGCMD\r\n");
    BluetoothSerial.clear_tx_buffer();
    BluetoothSerial.inject_into_rx_buffer("ERROR\r\n");
    response = BluetoothSerial.readStringUntil('\n');
    std::cout << "[MCU RX] Received: '" << response << "'" << std::endl;


    std::cout << "\nSimulation finished." << std::endl;
    return 0;
}
```

--- 

### Python Example: Simulating I2C Scan and Data Read

This Python example simulates an I2C bus scan (identifying connected devices by address) and reading data from a hypothetical IMU sensor.

```python
import random
import time

class MockI2CDevice:
    def __init__(self, address, device_type):
        self.address = address
        self.device_type = device_type
        self.data_registers = {} # Simulate internal registers

        if device_type == "IMU":
            self.data_registers = {
                0x00: random.randint(-16000, 16000), # Accel X
                0x01: random.randint(-16000, 16000), # Accel Y
                0x02: random.randint(-16000, 16000), # Accel Z
                0x10: random.randint(-16000, 16000), # Gyro X
                0x11: random.randint(-16000, 16000), # Gyro Y
                0x12: random.randint(-16000, 16000), # Gyro Z
            }
        elif device_type == "TempSensor":
            self.data_registers = {
                0x00: random.randint(2000, 3000), # Temp * 100
                0x01: random.randint(4000, 6000), # Humidity * 100
            }
        print(f"Mock I2C Device '{self.device_type}' at address 0x{self.address:02x} created.")

    def read_register(self, reg_address):
        if reg_address in self.data_registers:
            # Add some noise to the reading
            noise = random.randint(-10, 10)
            return self.data_registers[reg_address] + noise
        return 0xFFFF # Indicate error

    def write_register(self, reg_address, value):
        self.data_registers[reg_address] = value
        # print(f"Device 0x{self.address:02x}: Register 0x{reg_address:02x} written with {value}.")

class MockI2CBus:
    def __init__(self):
        self.connected_devices = {}

    def attach_device(self, device):
        self.connected_devices[device.address] = device

    def beginTransmission(self, address):
        if address in self.connected_devices:
            # print(f"Master: Beginning transmission to 0x{address:02x}")
            return True
        print(f"Master: No device at 0x{address:02x}")
        return False

    def write(self, data):
        # print(f"Master: Writing data {data}")
        pass # In a real bus, this would store data to be sent

    def endTransmission(self):
        # print("Master: Ending transmission.")
        return True # Simulate successful transmission

    def requestFrom(self, address, quantity):
        if address in self.connected_devices:
            # print(f"Master: Requesting {quantity} bytes from 0x{address:02x}")
            device = self.connected_devices[address]
            
            # Simulate reading multiple registers, or a block of data
            # For simplicity, let's just return a random sequence here
            response_data = []
            for _ in range(quantity):
                response_data.append(random.randint(0, 255))
            return response_data
        return []

    def read_register(self, address, reg_address, num_bytes=2):
        if address in self.connected_devices:
            device = self.connected_devices[address]
            return device.read_register(reg_address) # Simplified direct read
        return 0

def i2c_scanner(i2c_bus):
    print("Scanning I2C bus for devices...")
    found_devices = []
    for address in range(1, 127): # I2C addresses are 1-127
        if i2c_bus.beginTransmission(address):
            i2c_bus.endTransmission() # Just to check if device ACKs
            print(f"  Device found at 0x{address:02x}")
            found_devices.append(address)
        # time.sleep(0.001) # Small delay
    return found_devices

if __name__ == "__main__":
    i2c_bus = MockI2CBus()

    imu_sensor = MockI2CDevice(0x68, "IMU") # Common MPU6050 address
    temp_sensor = MockI2CDevice(0x76, "TempSensor") # Common BME280 address
    display_sensor = MockI2CDevice(0x3C, "Display") # Common OLED address

    i2c_bus.attach_device(imu_sensor)
    i2c_bus.attach_device(temp_sensor)
    i2c_bus.attach_device(display_sensor)

    found_addresses = i2c_scanner(i2c_bus)
    print(f"\nFound I2C devices at addresses: {[hex(addr) for addr in found_addresses]}")

    if 0x68 in found_addresses:
        print(f"\nReading data from IMU at 0x68:")
        # In a real scenario, you'd write a register to select what to read, then read
        accel_x = i2c_bus.read_register(0x68, 0x00) # Read Accel X register
        accel_y = i2c_bus.read_register(0x68, 0x01) # Read Accel Y register
        print(f"  Accel X: {accel_x}, Accel Y: {accel_y}")
    
    if 0x76 in found_addresses:
        print(f"\nReading data from TempSensor at 0x76:")
        temp = i2c_bus.read_register(0x76, 0x00) # Read Temp register
        hum = i2c_bus.read_register(0x76, 0x01)  # Read Humidity register
        print(f"  Temperature: {temp/100:.2f}°C, Humidity: {hum/100:.2f}%")

    print("\nI2C communication simulation finished.")
```

--- 

### Arduino Example: SPI Communication with SD Card Module

This Arduino sketch demonstrates basic SPI communication to initialize an SD card module and write data to a file. It uses the built-in `SD.h` library.

```arduino
#include <SPI.h> // Include the SPI library
#include <SD.h>  // Include the SD card library

const int chipSelect = 4; // Chip select pin for the SD card module (can be any digital pin)

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  // See if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // Don't do anything more:
    while (true);
  }
  Serial.println("card initialized.");

  // Make a text file and write to it
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // If the file is available, write to it:
  if (dataFile) {
    dataFile.println("Hello, robot world!");
    dataFile.close(); // Close the file
    Serial.println("datalog.txt written.");
  } else {
    // If the file didn't open, print an error:
    Serial.println("Error opening datalog.txt");
  }

  // Read the file back (optional)
  Serial.println("\nReading datalog.txt:");
  dataFile = SD.open("datalog.txt");
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  } else {
    Serial.println("Error opening datalog.txt for reading.");
  }
}

void loop() {
  // Nothing to do in loop for this simple example
}
```

--- 

### Equations in LaTeX: Data Rate Calculation (Baud Rate)

The **baud rate** (or symbol rate) in UART communication refers to the number of symbol changes per second. In simpler systems, one symbol often corresponds to one bit, so baud rate can be approximately equal to bits per second (bps).

```latex
text{Data Rate (bps)} = text{Baud Rate} times text{Bits per Symbol}
```

For UART, if we consider a full frame (1 start bit + 8 data bits + 1 stop bit = 10 bits per byte):

```latex
text{Bytes per Second} = frac{text{Baud Rate}{10}
```

--- 

### MCQs with Answers

1.  Which serial communication protocol uses only two wires (SDA and SCL) and supports multiple master and slave devices, with each slave having a unique address?
    a) UART
    b) SPI
    c) I2C
    d) Ethernet
    *Answer: c) I2C*

2.  What is the primary advantage of SPI communication over I2C communication?
    a) It uses fewer wires.
    b) It is generally faster.
    c) It supports multiple masters.
    d) It has built-in error correction.
    *Answer: b) It is generally faster.*

3.  In Arduino, which library is typically used for UART communication to send data to the Serial Monitor or a Bluetooth module?
    a) `Wire.h`
    b) `SPI.h`
    c) `Servo.h`
    d) `Serial` (built-in object, no explicit `#include` for basic use)
    *Answer: d) `Serial` (built-in object, no explicit `#include` for basic use - often used with `Serial.begin()`)*

--- 

### Practice Tasks

1.  **Sensor Bus Selection:** You need to connect the following peripherals to an Arduino Mega:
    *   3 IMU sensors (each has 7-bit I2C address, but could use SPI)
    *   1 SD card module
    *   1 GPS module
    *   1 TFT color display (high refresh rate needed)
    For each peripheral, suggest the most suitable communication protocol (UART, I2C, or SPI) and explain your reasoning, considering factors like speed, number of pins, and addressing.
2.  **I2C Address Scanning:** Write an Arduino sketch that scans the I2C bus and prints the addresses of any connected devices to the Serial Monitor. This is a common debugging tool when working with I2C sensors.
3.  **UART Command Parser:** Write an Arduino sketch that continuously reads incoming serial data. If it receives the string "MOTOR_ON", it should turn on an LED. If it receives "MOTOR_OFF", it should turn off the LED. For any other input, it should print "Unknown Command".

--- 

### Notes for Teachers

*   **Hands-on imperative:** Wiring and testing actual sensors and modules with these protocols is crucial.
*   **Show Different Speeds:** If possible, demonstrate the relative speeds of these protocols (e.g., how fast an OLED updates with I2C vs. SPI).
*   **Troubleshooting:** Guide students through common issues like wrong baud rates (UART), missing pull-up resistors (I2C), or incorrect chip select pins (SPI).

### Notes for Students

*   **Wiring Matters:** Pay close attention to the wiring diagrams for each protocol, especially pull-up resistors for I2C.
*   **Baud Rate for UART:** Always ensure that the sender and receiver are configured for the same baud rate in UART communication.
*   **Unique Addresses for I2C:** Remember that each I2C slave device on the bus needs a unique address.
*   **Chip Select for SPI:** Use the Chip Select (SS/CS) pin to specifically enable communication with one SPI slave at a time.
*   **Datasheets:** The datasheets for your peripheral modules will specify which communication protocol they use and how to interface with them.
```