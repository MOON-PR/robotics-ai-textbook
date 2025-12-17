---
id: book-05-microcontrollers-01-microcontroller-architecture
title: 'Part 5: Microcontrollers'
sidebar_position: 1
---

--- 
sidebar_position: 1
title: Microcontroller Architecture
---

# Part 5: Microcontrollers

## 01-Microcontroller Architecture

Microcontrollers are the compact, dedicated computers that serve as the "brains" of most embedded systems, including a vast majority of robots. Unlike general-purpose computers, microcontrollers are designed for specific control tasks, featuring a tightly integrated set of components optimized for real-time operation, low power consumption, and direct hardware interaction. Understanding their basic architecture is fundamental to programming them effectively.

### 1.1 What is a Microcontroller (MCU)?

A **microcontroller** is a complete computer system packed onto a single integrated circuit (IC) or "chip." It combines:
*   A **Central Processing Unit (CPU)**
*   **Memory** (RAM for data, Flash/ROM for program code)
*   **Input/Output (I/O) Peripherals** (for interacting with external devices)

All on a single silicon die. This integration makes them compact, cost-effective, and efficient for dedicated control applications.

**Diagram 1.1: Basic Microcontroller Block Diagram**

```mermaid
graph TD
    A[CPU (Processor Core)] --> B(Memory: RAM)
    A --> C(Memory: Flash/ROM)
    A --> D(Input/Output Peripherals)
    D --> E[Digital I/O (GPIO)]
    D --> F[Analog-to-Digital Converter (ADC)]
    D --> G[Pulse Width Modulation (PWM)]
    D --> H[Timers/Counters]
    D --> I[Communication Interfaces: UART, I2C, SPI]
    D --> J[Interrupt Controller]
```

*Description: A simplified block diagram illustrating the main functional blocks integrated within a typical microcontroller chip.*

### 1.2 Central Processing Unit (CPU)

The CPU is the "brain" of the microcontroller, responsible for executing program instructions, performing calculations, and controlling the flow of data.

*   **Instruction Set:** The set of commands that the CPU can understand and execute.
*   **Registers:** Small, fast memory locations within the CPU used to temporarily store data and instructions during processing.
*   **Clock Speed:** Determines how fast the CPU can execute instructions (measured in MHz).
*   **Architecture:** Common architectures include:
    *   **ARM Cortex-M:** Widely used in modern microcontrollers (e.g., STM32, ESP32, many Arduino-compatible boards). Known for efficiency and power.
    *   **AVR:** Used in popular Arduino Uno boards (ATmega328P). Simple, robust.
    *   **PIC:** Popular in industrial and hobby applications.
    *   **RISC-V:** Emerging open-source instruction set architecture.

### 1.3 Memory

Microcontrollers typically have several types of memory:

*   **Flash Memory (Program Memory / ROM):**
    *   **Purpose:** Stores the program code (the "sketch" in Arduino) and any constant data.
    *   **Characteristics:** Non-volatile (retains data when power is off), can be reprogrammed (flashed) multiple times.
    *   **Size:** Typically ranging from a few KB to several MB.
*   **SRAM (Static Random-Access Memory):**
    *   **Purpose:** Used for temporary data storage during program execution (e.g., variables, stack, heap).
    *   **Characteristics:** Volatile (loses data when power is off), fast access.
    *   **Size:** Generally much smaller than Flash memory (a few KB to hundreds of KB).
*   **EEPROM (Electrically Erasable Programmable Read-Only Memory):**
    *   **Purpose:** Stores small amounts of data that need to persist even when power is off (e.g., configuration settings, calibration data).
    *   **Characteristics:** Non-volatile, can be written to slowly and a limited number of times (wear-leveling often needed).
    *   **Size:** Very small (hundreds of bytes to a few KB).

### 1.4 Input/Output (I/O) Peripherals

These are specialized circuits that allow the microcontroller to interact with the outside world.

*   **GPIO (General Purpose Input/Output):**
    *   **Purpose:** Configurable pins that can be set as inputs (to read logic levels from sensors, buttons) or outputs (to control LEDs, relays, motors).
    *   **Functionality:** Can be read (`digitalRead()`) or written (`digitalWrite()`).
*   **ADC (Analog-to-Digital Converter):**
    *   **Purpose:** Converts continuous analog voltage signals from sensors into discrete digital values that the CPU can process.
    *   **Resolution:** Typically 10-bit (0-1023), 12-bit (0-4095), or higher.
*   **PWM (Pulse Width Modulation):**
    *   **Purpose:** Generates a series of pulses with a variable duty cycle. Used for "analog-like" control of motors (speed), LEDs (brightness), or servo motors (position).
*   **Timers/Counters:**
    *   **Purpose:** Generate precise time delays, measure elapsed time, count external events, or trigger periodic operations. Essential for real-time control.
*   **Communication Interfaces:**
    *   **UART (Universal Asynchronous Receiver-Transmitter):** For serial communication with devices like GPS modules, Bluetooth modules, or a computer (e.g., `Serial` in Arduino).
    *   **I2C (Inter-Integrated Circuit):** A two-wire serial bus for connecting multiple low-speed peripheral ICs (e.g., IMUs, environmental sensors, displays).
    *   **SPI (Serial Peripheral Interface):** A faster, more complex serial bus for connecting higher-speed peripherals (e.g., SD card modules, some displays, flash memory).
*   **Interrupt Controller:**
    *   **Purpose:** Allows the CPU to be immediately notified and temporarily suspend its current task to handle critical external events (e.g., button press, encoder pulse). Essential for responsive systems.

### 1.5 Clock System

*   **Purpose:** Provides the timing signals for all operations within the microcontroller.
*   **Components:** Typically includes a crystal oscillator (for accuracy) and internal phase-locked loops (PLLs) to generate various clock frequencies for different modules.

Understanding this architecture is the first step to truly leveraging the power of microcontrollers in your robotic projects, allowing you to optimize performance, manage memory, and select the right MCU for the job.

---

### C++ Example: Conceptual Microcontroller Simulation (Registers and Memory)

This C++ example conceptually simulates basic microcontroller operations, showing how a "program" might modify "memory" and interact with "registers."

```cpp
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <iomanip> // For std::hex, std::setw, std::setfill

// Simulate basic CPU registers
struct Registers {
    int R0;
    int R1;
    int PC;
    // ... other registers
};

// Simulate Flash memory (for program code)
std::vector<std::string> FLASH_MEMORY = {
    "LOAD R0, 0x10",      // Load immediate value into R0
    "LOAD R1, 0x20",      // Load immediate value into R1
    "ADD R0, R1",         // R0 = R0 + R1
    "STORE R0, 0x2000",   // Store R0 to SRAM address 0x2000
    "JUMP 0x0000",        // Loop back to start (for demo)
    "HALT"                // Stop execution
};

// Simulate SRAM (data memory)
std::map<int, int> SRAM; // Address -> Value

// Simulate I/O Port
int GPIO_PORT_B = 0x00; // 8-bit register for Port B, e.g., for LEDs

// CPU State
Registers cpu_registers = {0, 0, 0}; // Initialize R0, R1, PC to 0

void execute_instruction() {
    if (cpu_registers.PC >= FLASH_MEMORY.size()) {
        std::cout << "Program Counter out of bounds. Halting." << std::endl;
        cpu_registers.PC = -1; // Indicate halt
        return;
    }

    std::string instruction = FLASH_MEMORY[cpu_registers.PC];
    std::cout << "PC: 0x" << std::hex << std::setw(4) << std::setfill('0') << cpu_registers.PC 
              << " | Executing: " << instruction << std::endl;
    
    // Simple instruction decoding (very basic for demo)
    if (instruction.rfind("LOAD R0", 0) == 0) { // Starts with "LOAD R0"
        cpu_registers.R0 = std::stoi(instruction.substr(instruction.find("0x") + 2), nullptr, 16);
    } else if (instruction.rfind("LOAD R1", 0) == 0) {
        cpu_registers.R1 = std::stoi(instruction.substr(instruction.find("0x") + 2), nullptr, 16);
    } else if (instruction == "ADD R0, R1") {
        cpu_registers.R0 = cpu_registers.R0 + cpu_registers.R1;
    } else if (instruction.rfind("STORE R0", 0) == 0) {
        int address = std::stoi(instruction.substr(instruction.find("0x") + 2), nullptr, 16);
        SRAM[address] = cpu_registers.R0;
        std::cout << "  SRAM[0x" << std::hex << std::setw(4) << std::setfill('0') << address << "] = " << std::dec << SRAM[address] << std::endl;
    } else if (instruction.rfind("JUMP", 0) == 0) {
        int address = std::stoi(instruction.substr(instruction.find("0x") + 2), nullptr, 16);
        cpu_registers.PC = address - 1; // -1 because PC increments at end
    } else if (instruction == "HALT") {
        cpu_registers.PC = -1; // Signal halt
    }
    
    cpu_registers.PC++; // Increment Program Counter
    
    std::cout << "  R0: 0x" << std::hex << std::setw(2) << std::setfill('0') << cpu_registers.R0 
              << ", R1: 0x" << std::hex << std::setw(2) << std::setfill('0') << cpu_registers.R1 << std::endl;
    std::cout << std::dec; // Reset to decimal for general output
}

int main() {
    std::cout << "--- Conceptual Microcontroller Simulation ---" << std::endl;

    SRAM[0x2000] = 0; // Initialize a SRAM location

    for (int i = 0; i < 10 && cpu_registers.PC != -1; ++i) { // Run for a few steps
        execute_instruction();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "\nFinal SRAM[0x2000] value: " << SRAM[0x2000] << std::endl;
    std::cout << "Simulation finished." << std::endl;

    return 0;
}
```

--- 

### Python Example: Simulating GPIO and ADC

This Python example simulates a microcontroller's GPIO and ADC functionality.

```python
import random
import time

class MicrocontrollerSimulator:
    def __init__(self):
        self.digital_pins = {} # {pin_number: {'mode': 'INPUT', 'value': 0}}
        self.analog_pins = {}  # {pin_number: {'value': 0, 'voltage_ref': 5.0, 'adc_resolution': 1024}}
        self.serial_buffer = []

    def pinMode(self, pin, mode):
        if pin not in self.digital_pins:
            self.digital_pins[pin] = {'mode': 'INPUT', 'value': 0}
        self.digital_pins[pin]['mode'] = mode
        print(f"Pin {pin} configured as {mode}.")

    def digitalWrite(self, pin, value):
        if pin in self.digital_pins and self.digital_pins[pin]['mode'] == 'OUTPUT':
            self.digital_pins[pin]['value'] = value
            print(f"Pin {pin} set to {value} (digital).")
        else:
            print(f"Error: Pin {pin} not configured as OUTPUT for digitalWrite.")

    def digitalRead(self, pin):
        if pin in self.digital_pins and self.digital_pins[pin]['mode'].startswith('INPUT'):
            # Simulate reading a random digital input
            sim_value = random.choice([0, 1])
            self.digital_pins[pin]['value'] = sim_value
            return sim_value
        else:
            print(f"Error: Pin {pin} not configured as INPUT for digitalRead.")
            return 0

    def analogRead(self, pin):
        if pin not in self.analog_pins:
            self.analog_pins[pin] = {'value': 0, 'voltage_ref': 5.0, 'adc_resolution': 1024}
        
        # Simulate an analog sensor reading (e.g., potentiometer)
        sim_voltage = random.uniform(0, self.analog_pins[pin]['voltage_ref'])
        adc_value = int((sim_voltage / self.analog_pins[pin]['voltage_ref']) * self.analog_pins[pin]['adc_resolution'])
        
        self.analog_pins[pin]['value'] = adc_value
        return adc_value

    def Serial_begin(self, baudrate):
        print(f"Serial communication initialized at {baudrate} baud.")

    def Serial_println(self, message):
        self.serial_buffer.append(message)
        print(f"Serial Output: {message}")

    def get_serial_output(self):
        return self.serial_buffer

# Simulate an Arduino-like program (sketch)
def setup_sketch(mcu):
    mcu.Serial_begin(9600)
    mcu.pinMode(13, 'OUTPUT') # LED
    mcu.pinMode(2, 'INPUT')   # Button
    mcu.Serial_println("Simulated Arduino Ready.")

def loop_sketch(mcu, iteration):
    mcu.Serial_println(f"--- Loop Iteration {iteration} ---")
    
    # Toggle LED
    if iteration % 2 == 0:
        mcu.digitalWrite(13, 1) # HIGH
        mcu.Serial_println("LED ON")
    else:
        mcu.digitalWrite(13, 0) # LOW
        mcu.Serial_println("LED OFF")

    # Read button
    button_state = mcu.digitalRead(2)
    if button_state == 1: # Assuming HIGH when not pressed
        mcu.Serial_println("Button Not Pressed")
    else:
        mcu.Serial_println("Button Pressed!")

    # Read analog sensor
    sensor_value = mcu.analogRead('A0')
    mcu.Serial_println(f"Analog Sensor A0: {sensor_value}")


if __name__ == "__main__":
    mcu = MicrocontrollerSimulator()

    setup_sketch(mcu)

    for i in range(1, 4): # Run loop a few times
        loop_sketch(mcu, i)
        time.sleep(0.5) # Add a small delay for simulation realism
    
    print("\nSimulated Microcontroller execution complete.")
    # print("\nFull Serial Output:\n", "\n".join(mcu.get_serial_output()))
```

--- 

### Arduino Example: Direct Port Manipulation (Advanced GPIO)

This Arduino example demonstrates direct port manipulation to control multiple LEDs simultaneously, showcasing a lower-level interaction with GPIO ports than `digitalWrite()`. This is faster and more compact but less readable and less portable.

```arduino
// Direct Port Manipulation Example
// Controls LEDs on digital pins 8, 9, 10, 11
// On Arduino Uno (ATmega328P), these correspond to PORTB (bits 0, 1, 2, 3)

void setup() {
  Serial.begin(9600);
  Serial.println("Arduino Direct Port Manipulation Demo Ready.");

  // Configure pins 8, 9, 10, 11 as OUTPUT
  // DDRB is the Data Direction Register for Port B
  // DDRB = B00001111; // Set PB0-PB3 (pins 8-11) as output, others as input
  // Or more safely:
  DDRB |= B00001111; // Set bits 0-3 of DDRB to 1 (output) without changing other bits
}

void loop() {
  // Turn on LEDs 8, 9, 10, 11 (corresponds to PB0-PB3)
  // PORTB is the Port B Data Register
  // PORTB = B00001111; // Turn on PB0-PB3
  // Or more safely:
  PORTB |= B00001111; // Set bits 0-3 of PORTB to HIGH
  Serial.println("LEDs ON");
  delay(500);

  // Turn off LEDs 8, 9, 10, 11
  // PORTB = B00000000; // Turn off PB0-PB3
  // Or more safely:
  PORTB &= B11110000; // Set bits 0-3 of PORTB to LOW (clear them)
  Serial.println("LEDs OFF");
  delay(500);
}

/*
Explanation for Arduino Uno (ATmega328P):
- Port D: Digital pins 0-7 (PD0-PD7)
- Port B: Digital pins 8-13 (PB0-PB5)
- Port C: Analog pins A0-A5 (PC0-PC5)

DDRx Register: Data Direction Register for Port x.
  - Set a bit to 1 for OUTPUT.
  - Set a bit to 0 for INPUT.

PORTx Register: Port x Data Register.
  - When pin is OUTPUT: Set a bit to 1 for HIGH, 0 for LOW.
  - When pin is INPUT: Setting a bit to 1 enables internal PULLUP resistor.

PINx Register: Port x Input Pins Register.
  - Reading this register returns the current state of the physical pins.

Example for pin 8 (PB0):
  DDRB |= (1 << PB0); // Set PB0 as OUTPUT
  PORTB |= (1 << PB0); // Set PB0 HIGH
  PORTB &= ~(1 << PB0); // Set PB0 LOW
  (PINB & (1 << PB0)) // Read PB0 state
*/
```

--- 

### Equations in LaTeX: ADC Conversion Formula

The digital value `ADC_{val}` read from an Analog-to-Digital Converter (ADC) for a given analog input voltage `V_{in}` is calculated as:

```latex
ADC_{val} = left lfloor frac{V_{in}{V_{ref} times (2^{N} - 1) right rfloor
```

Where:
*   `V_{in}` is the analog input voltage.
*   `V_{ref}` is the reference voltage of the ADC.
*   `N` is the resolution of the ADC (e.g., 10 for a 10-bit ADC).
*   `2^{N} - 1` is the maximum digital value (e.g., `2^{10} - 1 = 1023`).
*   `lfloor cdot rfloor` denotes the floor function.

To convert `ADC_{val}` back to voltage:

```latex
V_{in} = ADC_{val} times frac{V_{ref}{2^{N} - 1}
```

--- 

### MCQs with Answers

1.  Which type of memory in a microcontroller is typically used to store the program code (firmware) and retains its data when power is off?
    a) SRAM
    b) EEPROM
    c) Flash Memory
    d) CPU Registers
    *Answer: c) Flash Memory*

2.  What is the primary purpose of the ADC (Analog-to-Digital Converter) peripheral in a microcontroller?
    a) To generate PWM signals for motor control.
    b) To store configuration settings permanently.
    c) To convert analog voltage signals into digital values.
    d) To communicate with other digital devices via I2C.
    *Answer: c) To convert analog voltage signals into digital values.*

3.  The `loop()` function in an Arduino sketch is designed to:
    a) Run only once at the beginning of the program.
    b) Run repeatedly, continuously, after the `setup()` function.
    c) Define hardware configurations for pins.
    d) Handle critical external events with high priority.
    *Answer: b) Run repeatedly, continuously, after the `setup()` function.*

--- 

### Practice Tasks

1.  **Microcontroller Resource Management:** Imagine you have an Arduino Uno (ATmega328P) with 32KB Flash, 2KB SRAM, and 1KB EEPROM. You need to store:
    *   A program that takes up 15KB.
    *   An array of 500 floating-point sensor readings.
    *   Robot calibration data (100 bytes) that must persist across power cycles.
    *   Temporary variables for calculations.
    Which memory type would you use for each piece of data, and why? Are there any potential issues with memory limits?
2.  **GPIO State Transition:** Write pseudocode for an Arduino program that configures a digital pin as an output. It should then toggle the state of this pin (HIGH/LOW) every 500 milliseconds without using the `delay()` function (i.e., using `millis()`).
3.  **Communication Interface Selection:** You need to connect two different sensors to an Arduino:
    *   A high-speed SD card module for data logging.
    *   A low-speed ambient temperature and humidity sensor (e.g., DHT11).
    Which serial communication interface (UART, I2C, or SPI) would you likely choose for each, and briefly explain your reasoning?

--- 

### Notes for Teachers

*   **Hands-on imperative:** Working with actual microcontrollers (like Arduino) is essential to grasp these concepts.
*   **Resource Constraints:** Highlight that microcontrollers have limited resources (CPU, memory) compared to desktop computers, which necessitates careful programming.
*   **Trade-offs:** Discuss the trade-offs between ease of use (`digitalWrite()`) and performance/compactness (direct port manipulation).

### Notes for Students

*   **Datasheets are Your Friend:** For any specific microcontroller, its datasheet provides detailed information about its architecture, registers, and peripherals.
*   **Memory Matters:** Be mindful of memory usage, especially SRAM, as it's often the most limited resource. Avoid large global arrays if possible.
*   **Understand `setup()` and `loop()`:** These are the core of Arduino programming. Understand what goes where.
*   **Non-blocking Code:** Prioritize writing non-blocking code using `millis()` for efficient real-time operation in robotics.
