---
sidebar_position: 6
title: Microcontroller Programming Basics (Arduino)
id: book-03-programming-06-microcontroller-programming-basics-arduino
---

## 06-Microcontroller Programming Basics (Arduino)

Microcontrollers are the "brains" of many robots, handling the direct interaction with sensors and actuators. The Arduino platform provides an accessible entry point into microcontroller programming, abstracting much of the low-level complexity while allowing direct hardware control. This chapter covers the fundamental concepts of programming microcontrollers using the Arduino IDE and language.

### 6.1 What is a Microcontroller?

A **microcontroller (MCU)** is a small, low-cost computer on a single integrated circuit. It contains a processor core, memory (RAM, ROM/Flash), and programmable input/output peripherals (GPIOs, ADC, PWM, UART, I2C, SPI).

*   **Key Features for Robotics:**
    *   **GPIO (General Purpose Input/Output):** Pins that can be configured as inputs (to read sensor data) or outputs (to control LEDs, relays).
    *   **ADC (Analog-to-Digital Converter):** Converts analog voltage signals from sensors into digital values the MCU can process.
    *   **PWM (Pulse Width Modulation):** Generates variable-width pulses for controlling motor speed, LED brightness, or servo positions.
    *   **Timers/Counters:** For precise timing and event counting.
    *   **Serial Communication (UART, I2C, SPI):** For communicating with other microcontrollers, sensors, or computers.

### 6.2 The Arduino Platform

Arduino is an open-source electronics platform based on easy-to-use hardware and software. It's particularly popular for hobbyists, artists, and educators due to its simplicity.

*   **Hardware (Arduino Boards):** Microcontroller boards (e.g., Uno, Nano, Mega) with a USB interface for programming and standard pin headers for connections.
*   **Software (Arduino IDE):** An integrated development environment (IDE) for writing, compiling, and uploading code to Arduino boards.
*   **Language:** A simplified C++ with specific functions and libraries for interacting with the hardware.

### 6.3 Arduino Sketch Structure

An Arduino program is called a **sketch**. Every sketch must contain two fundamental functions:

#### 6.3.1 `setup()` Function

*   This function runs once when the sketch starts, after power-up or reset.
*   It's used for initialization tasks:
    *   Setting pin modes (`pinMode()`).
    *   Initializing serial communication (`Serial.begin()`).
    *   Initializing libraries (`myServo.attach()`).
    *   Setting initial states (e.g., `digitalWrite(LED_BUILTIN, LOW)`).

#### 6.3.2 `loop()` Function

*   This function runs repeatedly, continuously, after the `setup()` function has completed.
*   It contains the main logic of your program:
    *   Reading sensors (`analogRead()`, `digitalRead()`).
    *   Controlling actuators (`digitalWrite()`, `analogWrite()`).
    *   Implementing control algorithms.
    *   Managing robot behaviors.

**Diagram 6.1: Arduino Sketch Flow**

```mermaid
flowchart TD
    A[Power On / Reset] --> B(Initialize Microcontroller)
    B --> C{setup() function}
    C --> D{loop() function}
    D -- Loop indefinitely --> D
```

*Description: A simple flowchart showing the execution flow of an Arduino sketch, starting with initialization in `setup()` and then continuously repeating the `loop()` function.*

### 6.4 Basic Arduino I/O Functions

#### 6.4.1 Digital I/O

*   **`pinMode(pin, mode)`:** Configures the specified `pin` to behave either as an input or an output.
    *   `INPUT`: Configures the pin to read digital values (HIGH or LOW).
    *   `OUTPUT`: Configures the pin to output digital values (HIGH or LOW).
    *   `INPUT_PULLUP`: Configures the pin as an input and enables the internal pull-up resistor.
*   **`digitalWrite(pin, value)`:** Writes a HIGH or LOW value to a digital `pin`.
*   **`digitalRead(pin)`:** Reads the value from a specified digital `pin` (returns HIGH or LOW).

#### 6.4.2 Analog I/O

*   **`analogRead(pin)`:** Reads the value from the specified analog input `pin`. Returns an integer between 0 and 1023 for most Arduino boards (10-bit ADC).
*   **`analogWrite(pin, value)`:** Writes an analog value (PWM wave) to a digital `pin`. Used for LED brightness, motor speed. `value` is typically between 0 (off) and 255 (full on). Available only on PWM-enabled pins.

#### 6.4.3 Time Functions

*   **`delay(ms)`:** Pauses the program for the amount of time (in milliseconds) specified as parameter. **Caution:** This stops all program execution, which can be problematic for real-time robotic tasks.
*   **`millis()`:** Returns the number of milliseconds since the Arduino board began running the current program. Useful for non-blocking timing (without `delay()`).
*   **`micros()`:** Returns the number of microseconds since the Arduino board began running the current program.

### 6.5 Serial Communication

The `Serial` object in Arduino allows communication with a computer or other devices via the USB (or UART) connection.

*   **`Serial.begin(baud_rate)`:** Initializes serial communication at a specified `baud_rate` (e.g., 9600, 115200).
*   **`Serial.print(data)` / `Serial.println(data)`:** Prints data to the serial monitor. `println` adds a newline character.
*   **`Serial.available()`:** Returns the number of bytes available for reading from the serial port.
*   **`Serial.read()`:** Reads incoming serial data.

### 6.6 Libraries

Arduino leverages C++ libraries to extend its functionality easily. Libraries simplify complex tasks like controlling servo motors (`Servo.h`), communicating over I2C (`Wire.h`), or using specific sensors.

*   **Installation:** Libraries can be installed through the Arduino IDE's Library Manager or by manually placing files in the `libraries` folder.
*   **Usage:** Included in a sketch using `#include <LibraryName.h>`.

Understanding these basics of Arduino programming provides a solid foundation for directly controlling the hardware aspects of robotic systems.

---

### Arduino Example: Ultrasonic Distance Sensor and Serial Output

This Arduino sketch demonstrates reading data from an ultrasonic sensor and sending it to the serial monitor.

```arduino
// Ultrasonic Sensor Pins
const int trigPin = 9;  // Trigger pin
const int echoPin = 10; // Echo pin

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  Serial.println("Ultrasonic Sensor Demo Ready.");
}

void loop() {
  // Clears the trigPin by setting it LOW for 2 microseconds
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 microseconds to send a pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (Speed of sound in air is approximately 343 meters/second, or 0.0343 cm/µs)
  // Distance = (Duration * Speed of Sound) / 2
  float distanceCm = duration * 0.0343 / 2;
  
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  delay(100); // Wait for 100 milliseconds before the next reading
}
```

---

### C++ Example: Simulating a Simple Microcontroller (Conceptual)

This C++ example provides a conceptual outline of how a microcontroller might execute its `setup()` and `loop()` functions.

```cpp
#include <iostream>
#include <string>
#include <chrono>
#include <thread> // For std::this_thread::sleep_for

// Simulate Arduino-like functions
void pinMode(int pin, std::string mode) {
    std::cout << "Configuring Pin " << pin << " as " << mode << std::endl;
}

void digitalWrite(int pin, std::string value) {
    std::cout << "Writing " << value << " to Pin " << pin << std::endl;
}

int digitalRead(int pin) {
    // Simulate reading a button (LOW = pressed)
    return (rand() % 2 == 0) ? LOW : HIGH; // Random HIGH/LOW
}

void Serial_begin(int baud) {
    std::cout << "Serial communication started at " << baud << " baud." << std::endl;
}

void Serial_println(const std::string& message) {
    std::cout << message << std::endl;
}

// Global "defines" for convenience, mirroring Arduino
const int LOW = 0;
const int HIGH = 1;
const std::string INPUT = "INPUT";
const std::string OUTPUT = "OUTPUT";
const std::string INPUT_PULLUP = "INPUT_PULLUP";

// --- Arduino Sketch Structure Simulation ---
void setup() {
    Serial_begin(9600);
    pinMode(13, OUTPUT); // Built-in LED
    pinMode(2, INPUT_PULLUP); // Button pin
    Serial_println("Simulated Arduino setup() complete.");
}

void loop() {
    // Simulate LED blinking logic
    digitalWrite(13, HIGH);
    Serial_println("LED ON");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    digitalWrite(13, LOW);
    Serial_println("LED OFF");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Simulate button reading
    if (digitalRead(2) == LOW) {
        Serial_println("Button pressed!");
    } else {
        Serial_println("Button released.");
    }
}

int main() {
    std::cout << "--- Starting Microcontroller Simulation ---" << std::endl;
    setup();

    // In a real Arduino, loop() runs indefinitely
    // Here we run it a few times for demonstration
    for (int i = 0; i < 5; ++i) {
        Serial_println("\n--- Loop iteration " + std::to_string(i + 1) + " ---");
        loop();
    }
    std::cout << "\n--- Microcontroller Simulation Finished ---" << std::endl;
    return 0;
}
```

---

### Python Example: Serial Port Communication Simulation

This Python script simulates sending commands to an Arduino-like device via a serial port and receiving responses. It uses the `serial` library (which needs to be installed via `pip install pyserial`).

```python
import serial
import time
import random

# Mock Arduino behavior for testing without physical hardware
class MockArduino:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.simulated_led_state = False
        self.simulated_sensor_value = 0
        self.buffer = []
        print(f"MockArduino on {self.port} initialized.")

    def write_line(self, data):
        self.buffer.append(data)
        if "light_on" in data:
            self.simulated_led_state = True
        elif "light_off" in data:
            self.simulated_led_state = False
        elif "get_status" in data:
            self.buffer.append(f"LED: {'ON' if self.simulated_led_state else 'OFF'}, Sensor: {self.simulated_sensor_value}")
        elif "read_sensor" in data:
            self.simulated_sensor_value = random.randint(0, 1023)
            self.buffer.append(f"SENSOR_VAL:{self.simulated_sensor_value}")

    def read_line(self):
        if self.buffer:
            return self.buffer.pop(0)
        return ""

# This function would interact with a real Arduino
def communicate_with_arduino(port_name, baud_rate):
    """
    Communicates with a physical Arduino using PySerial.
    Ensure 'pyserial' is installed (`pip install pyserial`).
    """
    try:
        # ser = serial.Serial(port_name, baud_rate, timeout=1)
        # time.sleep(2) # Wait for connection
        
        # For demonstration without physical hardware, use MockArduino
        ser = MockArduino(port_name, baud_rate) # Replace with actual serial.Serial for real hw

        print(f"Connected to {port_name} (simulated) at {baud_rate} baud.")
        
        # Arduino setup messages
        ser.write_line("Arduino Ready.")

        commands_to_send = ["light_on", "get_status", "read_sensor", "light_off", "get_status", "read_sensor"]
        for cmd in commands_to_send:
            print(f"\nPython Sending: {cmd}")
            # ser.write(cmd.encode('utf-8') + b'\n') # For real hardware
            ser.write_line(cmd) # For mock
            time.sleep(1)
            
            # Read all available lines from Arduino
            # while ser.in_waiting > 0: # For real hardware
            #    line = ser.readline().decode('utf-8').strip()
            line = ser.read_line() # For mock
            if line:
                print(f"Arduino Responded: {line}")
        
        # ser.close() # For real hardware
        print("\nCommunication simulation finished.")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # This part requires a physical Arduino loaded with an appropriate sketch
    # and its COM port identified.
    # For a purely theoretical execution without hardware, this block won't run as intended.
    
    # Placeholder for a physical Arduino's COM port
    # You'd need to change 'COM3' to your actual Arduino port (e.g., 'COMx' on Windows, '/dev/ttyACM0' or '/dev/ttyUSB0' on Linux)
    # and have an Arduino sketch running that responds to commands like 'light_on', 'light_off', 'get_status'.
    print("This Python script demonstrates serial communication.")
    print("To run it fully, connect an Arduino with a responding sketch and specify its COM port.")
    # simulate_arduino_interaction(port='COM3', baudrate=9600) 
    # Example for conceptual understanding:
    print("\nConceptual example: Imagine sending 'light_on' and getting 'LED ON' response.")
```

---

### Equations in LaTeX: Non-Blocking Timing with `millis()`

To implement non-blocking delays in Arduino using `millis()`, you compare the current time with a `previousMillis` timestamp.

```latex
text{currentMillis} - text{previousMillis} ge text{interval}
```

If this condition is true, it's time to perform an action, and `previousMillis` is updated to `currentMillis`.

---

### MCQs with Answers

1.  Which Arduino function runs only once at the start of the program for initialization tasks?
    a) `loop()`
    b) `setup()`
    c) `main()`
    d) `init()`
    *Answer: b) `setup()`*

2.  What is the typical range of values returned by `analogRead()` on most Arduino boards?
    a) 0 to 255
    b) -1023 to 1023
    c) 0 to 1023
    d) 0 to 5 (Volts)
    *Answer: c) 0 to 1023*

3.  To print text to the Serial Monitor in Arduino, which function would you use after initializing serial communication?
    a) `Serial.begin()`
    b) `Serial.read()`
    c) `Serial.available()`
    d) `Serial.println()`
    *Answer: d) `Serial.println()`*

---

### Practice Tasks

1.  **Arduino Traffic Light:** Write an Arduino sketch to simulate a traffic light sequence (Red, Yellow, Green) using three LEDs. Implement the timing using the `millis()` function to avoid blocking `delay()` calls, allowing other actions to potentially run concurrently.
2.  **Arduino Potentiometer Control:** Connect a potentiometer to an analog input pin on your Arduino. Read the potentiometer's value using `analogRead()` and use `analogWrite()` to control the brightness of an LED connected to a PWM pin. Print the potentiometer value and LED brightness to the Serial Monitor.
3.  **Arduino Button and Serial Input:** Write an Arduino sketch where pressing a button sends a message ("Button Pressed!") to the serial monitor. Additionally, if you type 'H' in the serial monitor and send it, a separate LED should turn ON; typing 'L' should turn it OFF.

---

### Notes for Teachers

*   **Hands-on imperative:** Microcontroller programming *must* be taught with hands-on exercises. Provide students with Arduino boards, breadboards, LEDs, resistors, buttons, and basic sensors (ultrasonic, LDR).
*   **Emphasize `millis()`:** Highlight the importance of non-blocking code using `millis()` for effective robotic control, as `delay()` is problematic in real-time systems.
*   **Troubleshooting:** Guide students through common Arduino errors (e.g., incorrect `pinMode`, missing `Serial.begin`, typos).

### Notes for Students

*   **Start Simple:** Begin with basic LED blinking and button reading before attempting complex sensor integrations.
*   **Consult Arduino Reference:** The official Arduino Reference is an invaluable resource for understanding functions, libraries, and examples.
*   **Breadboard Etiquette:** Learn how to use a breadboard correctly for safe and clear circuit prototyping.
*   **Always Test:** Test small parts of your code and circuit incrementally to isolate issues.