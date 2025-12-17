---
id: book-05-microcontrollers-03-advanced-arduino-features
title: Simulate hardware events
sidebar_position: 3
---

--- 
sidebar_position: 3
title: Advanced Arduino Features (Interrupts, Timers, PWM)
---

## 03-Advanced Arduino Features (Interrupts, Timers, PWM)

Beyond basic digital and analog I/O, Arduino microcontrollers offer powerful hardware features like interrupts, timers, and advanced Pulse Width Modulation (PWM) capabilities. Mastering these allows for more efficient, responsive, and precise control in robotics applications, moving beyond simple blocking delays.

### 3.1 Interrupts

**Interrupts** are mechanisms that allow a microcontroller to respond immediately to external or internal events, temporarily suspending its current task to execute a dedicated piece of code called an **Interrupt Service Routine (ISR)**.

#### 3.1.1 Why use Interrupts?

*   **Responsiveness:** Respond to critical events (e.g., button presses, encoder pulses, sensor triggers) without waiting for the `loop()` function to cycle.
*   **Non-blocking Code:** Avoids using `delay()`, which halts all program execution.
*   **High-Speed Event Capture:** Accurately count fast pulses from encoders or measure precise timings that might be missed by polling in `loop()`.

#### 3.1.2 External Interrupts (Hardware Interrupts)

*   **Principle:** Triggered by a change in the logic level on a specific digital pin.
*   **Arduino Functions:**
    *   `attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)`: Attaches an ISR to an interrupt pin.
        *   `pin`: The Arduino digital pin number.
        *   `ISR`: The function to call when the interrupt occurs. This function must take no arguments and return nothing (`void`).
        *   `mode`: Specifies when the interrupt should trigger:
            *   `LOW`: To trigger whenever the pin is LOW.
            *   `CHANGE`: To trigger whenever the pin changes value (LOW to HIGH or LOW to HIGH).
            *   `RISING`: To trigger when the pin goes from LOW to HIGH.
            *   `FALLING`: To trigger when the pin goes from HIGH to LOW.
    *   `detachInterrupt(digitalPinToInterrupt(pin))`: Disables the interrupt.
*   **Limitations of ISRs:**
    *   **Keep them short and fast:** ISRs should execute as quickly as possible. Avoid `delay()`, `Serial.print()`, or complex calculations inside an ISR.
    *   **`volatile` Keyword:** Variables shared between an ISR and the main code (`loop()`) must be declared as `volatile` to prevent the compiler from optimizing them away unexpectedly.
    *   **Global Interrupts:** Interrupts are automatically disabled during an ISR's execution by default. You can manually disable (`noInterrupts()`) and enable (`interrupts()`) global interrupts if needed, but use with caution.

**Diagram 3.1: Interrupt Flow**

```mermaid
graph TD
    A[Main Program (loop())] --> B(Event Occurs)
    B -- Triggers Interrupt --> C{CPU Halts Main Program}
    C --> D[Execute ISR (Interrupt Service Routine)]
    D --> E{CPU Resumes Main Program}
    E --> A
```

*Description: A flowchart showing how an interrupt diverts the CPU's execution from the main program flow to an Interrupt Service Routine (ISR) before returning to the main program.*

### 3.2 Timers/Counters

Microcontrollers have dedicated hardware **Timers/Counters** that can be configured to perform various timing-related tasks independent of the CPU, generating interrupts or controlling outputs.

#### 3.2.1 Why use Timers?

*   **Precise Timing:** Generate highly accurate delays or intervals.
*   **Non-blocking Delays:** Implement periodic tasks without `delay()` (e.g., update a display every 100ms, read a sensor every 1s).
*   **PWM Generation:** Hardware PWM (like `analogWrite()`) uses timers.
*   **Counting External Events:** Count pulses on a pin.

#### 3.2.2 Timer Modes

*   **Normal Mode:** Counts up to a maximum value, then overflows and resets.
*   **CTC (Clear Timer on Compare Match) Mode:** Counts up to a specified value (Output Compare Register - OCRx), then resets and can trigger an interrupt. Ideal for precise periodic events.
*   **Fast PWM Mode:** Used to generate high-frequency PWM signals.
*   **Phase Correct PWM Mode:** Generates PWM signals optimized for motor control.

#### 3.2.3 Using Timers in Arduino

*   **Basic `millis()`/`micros()`:** These functions are built upon hardware timers.
*   **`analogWrite()`:** Internally uses hardware timers for PWM.
*   **Libraries:** Libraries like `TimerOne.h` or `TimerMs.h` simplify configuring timers without direct register manipulation.
*   **Direct Register Access:** For very fine-grained control or when libraries are not sufficient, you can directly program the ATmega/ESP32 timer registers.

### 3.3 Pulse Width Modulation (PWM) - Advanced

While `analogWrite()` provides a simple way to use PWM, understanding its underlying mechanism with timers allows for more advanced control.

#### 3.3.1 PWM Fundamentals

*   **Frequency:** How often the pulse repeats (cycles per second, Hz). This is determined by the timer's clock source and prescaler.
*   **Duty Cycle:** The percentage of time the signal is HIGH within one period. This is controlled by the `analogWrite()` value (0-255) on Arduino.
*   **Resolution:** The number of distinct duty cycle values available (e.g., 8-bit PWM = 256 steps from 0 to 255).

#### 3.3.2 Advanced PWM Applications

*   **Multiple PWM Frequencies:** Control motors at different speeds and frequencies, or drive specific LED types.
*   **Servo Control:** Hobby servos use a specific pulse width (e.g., 1ms to 2ms) rather than a duty cycle. `Servo.h` library handles this conversion.
*   **Audio Generation:** Simple tones can be generated using PWM.

**Diagram 3.2: PWM Signal Characteristics**

```mermaid
graph TD
    A[Period (T)] -- High --> B(Pulse Width)
    A -- Low --> C(Off Time)
    D[Duty Cycle = Pulse Width / Period]
```

*Description: A diagram illustrating the key characteristics of a PWM signal, including its period, pulse width, off time, and how duty cycle is calculated.*

### 3.4 Putting it Together: Real-time Control

By combining interrupts and timers, robots can achieve robust real-time control:
*   An **interrupt** triggered by an encoder can precisely count motor revolutions.
*   A **timer interrupt** can be configured to run a PID control loop at a fixed, high frequency (e.g., 100Hz) to maintain motor speed or position.
*   **PWM** can then be used to precisely drive the motors based on the control loop's output.

These advanced features move Arduino programming beyond simple on/off control into the realm of sophisticated, high-performance robotic behaviors.

--- 

### C++ Example: Conceptual PID Controller (Utilizing Timer Concept)

This C++ example conceptually shows a PID controller, often implemented within a timer interrupt on a microcontroller.

```cpp
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <numeric> // For std::accumulate
#include <random> // For random number generation
#include <iomanip> // For std::fixed, std::setprecision

class PIDController {
private:
    float Kp, Ki, Kd; // PID gains
    float setpoint;   // Desired value
    float integral;   // Integral sum
    float prev_error; // Previous error
    float dt;         // Time step

public:
    PIDController(float p, float i, float d, float time_step) :
        Kp(p), Ki(i), Kd(d), dt(time_step),
        setpoint(0.0f), integral(0.0f), prev_error(0.0f) {}

    void setSetpoint(float sp) {
        setpoint = sp;
        // Optionally reset integral and prev_error when setpoint changes significantly
        // integral = 0.0f;
        // prev_error = 0.0f;
    }

    float compute(float feedback_value) {
        float error = setpoint - feedback_value;

        // Proportional term
        float Pout = Kp * error;

        // Integral term
        integral += error * dt;
        float Iout = Ki * integral;

        // Derivative term
        float derivative = (error - prev_error) / dt;
        float Dout = Kd * derivative;

        // Calculate total output
        float output = Pout + Iout + Dout;

        // Store error for next iteration
        prev_error = error;

        return output;
    }

    void resetIntegral() {
        integral = 0.0f;
    }
};

// Simulate a motor with some inertia and noise
float simulateMotor(float control_input, float current_speed) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<> noise(0, 0.5);

    // Simple motor model: speed changes based on control input, with some damping
    float motor_response = (control_input * 0.1f) - (current_speed * 0.01f); // Control vs. Damping
    float new_speed = current_speed + motor_response + noise(gen);
    
    // Clamp speed to reasonable limits
    return std::max(0.0f, std::min(100.0f, new_speed));
}

int main() {
    const float dt = 0.1f; // PID loop runs every 100ms
    PIDController motorPID(0.5f, 0.1f, 0.05f, dt); // Kp, Ki, Kd
    
    float current_motor_speed = 0.0f; // Initial speed
    motorPID.setSetpoint(50.0f); // Target speed is 50

    std::cout << "PID Motor Control Simulation" << std::endl;
    std::cout << "Time\tSetpoint\tCurrent\t\tError\t\tOutput\t\tNew Speed" << std::endl;
    std::cout << std::fixed << std::setprecision(2);

    for (int i = 0; i < 50; ++i) { // Simulate for 5 seconds (50 iterations * 0.1s)
        float time_sec = i * dt;
        float error = motorPID.setpoint - current_motor_speed;
        
        float control_output = motorPID.compute(current_motor_speed);
        
        current_motor_speed = simulateMotor(control_output, current_motor_speed);

        std::cout << time_sec << "\t" << motorPID.setpoint << "\t\t" << current_motor_speed << "\t\t"
                  << error << "\t\t" << control_output << "\t\t" << current_motor_speed << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(dt * 1000)));

        // Change setpoint after a few seconds
        if (time_sec > 4.0f && motorPID.setpoint == 50.0f) {
            motorPID.setSetpoint(30.0f);
            std::cout << "\n--- Setpoint changed to 30 RPM ---" << std::endl;
            motorPID.resetIntegral(); // Reset integral for faster response to new setpoint
        }
    }
    std::cout << "\nSimulation finished." << std::endl;

    return 0;
}
```

--- 

### Python Example: Interrupt-Driven Simulation (Event Handling)

This Python example simulates an interrupt-driven system using event queues and threading, mimicking how a microcontroller might react to events.

```python
import threading
import time
import queue

# Simulate hardware events
event_queue = queue.Queue()

def simulate_hardware_events():
    """Function to put events into the queue, mimicking hardware interrupts."""
    print("Hardware event simulator started.")
    time.sleep(1)
    event_queue.put("BUTTON_PRESS")
    time.sleep(0.5)
    event_queue.put("SENSOR_READ_READY")
    time.sleep(2)
    event_queue.put("BUTTON_PRESS")
    time.sleep(1.5)
    event_queue.put("TIMER_OVERFLOW")
    print("Hardware event simulator finished.")

# Interrupt Service Routine (ISR) concept
def isr_button_press():
    print(f"\n[ISR] Button pressed detected! Timestamp: {time.time():.2f}")
    # In a real system, would set a flag or increment a counter
    global button_pressed_flag
    button_pressed_flag = True

def isr_sensor_ready():
    print(f"\n[ISR] Sensor data ready! Timestamp: {time.time():.2f}")
    # In a real system, would read the sensor and set a flag
    global sensor_data_available
    sensor_data_available = True

def isr_timer_overflow():
    print(f"\n[ISR] Timer overflow detected! Timestamp: {time.time():.2f}")
    # In a real system, would increment a time counter or trigger a periodic task
    global periodic_task_needed
    periodic_task_needed = True

# Global flags (equivalent to volatile variables in C/C++)
button_pressed_flag = False
sensor_data_available = False
periodic_task_needed = False

# Main loop (mimics Arduino loop())
def main_loop():
    global button_pressed_flag, sensor_data_available, periodic_task_needed
    
    print("\nMain loop started.")
    counter = 0
    while counter < 10: # Run for a limited number of cycles
        if button_pressed_flag:
            print(f"[LOOP] Processing button press: counter {counter}")
            button_pressed_flag = False # Clear the flag
        
        if sensor_data_available:
            print(f"[LOOP] Processing sensor data: counter {counter}")
            sensor_data_available = False # Clear the flag

        if periodic_task_needed:
            print(f"[LOOP] Executing periodic task: counter {counter}")
            periodic_task_needed = False # Clear the flag

        # Normal main loop operations
        print(f"Main loop running... {counter}")
        time.sleep(0.3) # Simulate some work
        counter += 1
    print("Main loop finished.")

if __name__ == "__main__":
    print("--- Interrupt-Driven Simulation Demo ---")

    # Start the hardware event simulator in a separate thread
    event_thread = threading.Thread(target=simulate_hardware_events)
    event_thread.start()

    # Map conceptual events to ISRs
    isr_map = {
        "BUTTON_PRESS": isr_button_press,
        "SENSOR_READ_READY": isr_sensor_ready,
        "TIMER_OVERFLOW": isr_timer_overflow
    }

    # Monitor the event queue and call appropriate ISRs
    def interrupt_handler_thread():
        print("Interrupt handler thread started.")
        while event_thread.is_alive() or not event_queue.empty():
            try:
                event = event_queue.get(timeout=0.1)
                if event in isr_map:
                    isr_map[event]()
                else:
                    print(f"[ISR Handler] Unknown event: {event}")
            except queue.Empty:
                pass
        print("Interrupt handler thread finished.")

    isr_thread = threading.Thread(target=interrupt_handler_thread)
    isr_thread.start()

    main_loop() # Run the main loop

    event_thread.join()
    isr_thread.join()
    print("\nDemo Complete.")
```

--- 

### Arduino Example: Timer-Based Non-Blocking Blink

This Arduino sketch uses a hardware timer (Timer/Counter1 on Uno) to create a more precise and non-blocking blinking LED, independent of `delay()`.
This typically involves direct register manipulation for advanced timing control.

```arduino
// Timer-Based Non-Blocking Blink Example (ATmega328P - Arduino Uno)
// This uses Timer1 (16-bit timer) in CTC mode to trigger an interrupt periodically.

const int ledPin = 13; // Onboard LED

volatile unsigned long timer_count = 0; // Use volatile for shared variable

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  Serial.println("Arduino Timer-Based Blink Demo Ready.");

  // Configure Timer1 for CTC (Clear Timer on Compare Match) mode
  noInterrupts();           // Disable all interrupts
  
  TCCR1A = 0;               // Clear TCCR1A register
  TCCR1B = 0;               // Clear TCCR1B register
  
  // Set compare match register for 1Hz (1 second) at 16MHz clock, prescaler 1024
  // OCR1A = [ (CPU_CLOCK / (PRESCALER * DESIRED_FREQUENCY)) - 1 ]
  // OCR1A = [ (16,000,000 / (1024 * 1)) - 1 ] = 15625 - 1 = 15624
  OCR1A = 15624; 
  
  // Turn on CTC mode (WGM12 bit set to 1)
  TCCR1B |= (1 << WGM12);
  
  // Set CS10 and CS12 bits for prescaler of 1024
  TCCR1B |= (1 << CS12) | (1 << CS10);
  
  // Enable timer compare interrupt A (OCIE1A)
  TIMSK1 |= (1 << OCIE1A);
  
  interrupts();             // Enable all interrupts
}

void loop() {
  // Main loop can do other things while the timer interrupt handles the blinking.
  Serial.print("Loop running... Timer count: ");
  Serial.println(timer_count);
  delay(500); // Small delay to make Serial output readable
}

// Timer1 Compare Match A Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
  // This code runs every time Timer1 reaches OCR1A (i.e., every 1 second)
  digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED
  timer_count++;
}
```

--- 

### Equations in LaTeX: PWM Frequency Calculation

For an 8-bit timer (like Timer0 or Timer2 on ATmega328P) in Fast PWM mode:

```latex
f_{PWM} = frac{f_{CPU}{text{prescaler} times (255 + 1)}
```

Where:
*   `f_{PWM}` is the PWM frequency.
*   `f_{CPU}` is the CPU clock frequency (e.g., 16 MHz for Arduino Uno).
*   `prescaler` is the value chosen for the timer's clock division (e.g., 1, 8, 64, 256, 1024).

--- 

### MCQs with Answers

1.  What is the primary benefit of using **interrupts** in microcontroller programming for robotics?
    a) To make the code easier to read.
    b) To respond immediately to critical events without waiting for the main `loop()` to process.
    c) To save Flash memory.
    d) To generate precise voltage outputs.
    *Answer: b) To respond immediately to critical events without waiting for the main `loop()` to process.*

2.  When passing data between an Interrupt Service Routine (ISR) and the main `loop()` function, the shared variable must be declared with which keyword?
    a) `static`
    b) `const`
    c) `volatile`
    d) `public`
    *Answer: c) `volatile`*

3.  Which hardware feature is primarily responsible for generating the PWM signals used to control motor speed or LED brightness in Arduino?
    a) ADC
    b) GPIO
    c) Timers/Counters
    d) UART
    *Answer: c) Timers/Counters*

--- 

### Practice Tasks

1.  **Encoder Interrupt Challenge:** Modify the Arduino Rotary Encoder example (from a previous chapter or provided code) to use both `RISING` and `FALLING` edges on `encoderPinA` (or both `encoderPinA` and `encoderPinB`) to increase the resolution of the encoder count.
2.  **Timer-Based PID:** Conceptualize how you would integrate the C++ PID Controller example into an Arduino sketch using a hardware timer. Describe which parts would go in the `ISR` and which in the `loop()`, and what challenges you might face.
3.  **Custom PWM Frequency:** Research how to change the PWM frequency of the `analogWrite()` pins on an Arduino Uno (e.g., for pins 5 and 6, which use Timer0). Write an Arduino sketch that sets a custom, higher PWM frequency for a specific pin and controls an LED with `analogWrite()` at this new frequency.

--- 

### Notes for Teachers

*   **Illustrate Blocking vs. Non-blocking:** Show students the difference between using `delay()` and non-blocking timing with `millis()` or timers for responsiveness.
*   **ISR Constraints:** Emphasize the strict constraints on what can be done inside an ISR (no delays, minimal computations).
*   **Hardware vs. Software PWM:** Explain the difference between hardware PWM (more efficient, uses timers) and software PWM (can be done on any pin, but less precise and CPU intensive).

### Notes for Students

*   **Interrupts for Speed:** Use interrupts for events that require immediate, precise timing, such as encoder readings or critical safety stops.
*   **Timers for Precision:** Use timers for accurate periodic tasks and high-quality PWM signals.
*   **`volatile` is Critical:** Always remember to use the `volatile` keyword for variables shared between an ISR and the main code.
*   **Read the Datasheet:** For advanced timer configurations or specific pin behaviors, consult the ATmega328P datasheet (or the datasheet of your specific microcontroller).
