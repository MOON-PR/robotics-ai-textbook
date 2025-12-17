---
sidebar_position: 2
title: Common Electronic Components Reference
id: book-11-appendix-02-common-electronic-components-reference
---

## 02-Common Electronic Components Reference

This section provides a quick reference guide to common electronic components frequently used in robotics projects. Understanding their symbols, basic function, and typical applications is essential for building and troubleshooting circuits.

---

### Resistor (\u03a9)

*   **Symbol:**
    *   American: `---/\/\/---`
    *   European: `---[ ]---`
*   **Function:** Opposes the flow of electric current. Used to limit current, divide voltage, and generate heat.
*   **Unit:** Ohms (\u03a9).
*   **Key Specs:** Resistance value, power rating (Wattage).
*   **Typical Use:** Current limiting for LEDs, pull-up/pull-down resistors.
*   **Example:** 220\u03a9 for an LED, 10k\u03a9 for a potentiometer voltage divider.

---

### Capacitor (F)

*   **Symbol:**
    *   Non-polarized: `---| |---`
    *   Polarized (Electrolytic): `---| |---` (with one plate curved and a '+' sign)
*   **Function:** Stores electrical energy in an electric field. Blocks DC, passes AC. Used for filtering, timing, and energy storage.
*   **Unit:** Farads (F), typically microfarads (\u00b5F), nanofarads (nF), picofarads (pF).
*   **Key Specs:** Capacitance, voltage rating.
*   **Typical Use:** Smoothing power supply ripple, decoupling noise, timing circuits.
*   **Example:** 100\u00b5F for power supply filtering, 0.1\u00b5F for decoupling.

---

### Inductor (H)

*   **Symbol:** `---(~ ~ ~)---`
*   **Function:** Stores energy in a magnetic field. Blocks AC, passes DC. Used for filtering, energy storage in switching regulators.
*   **Unit:** Henrys (H), typically millihenrys (mH), microhenrys (\u00b5H).
*   **Key Specs:** Inductance, current rating, DC resistance.
*   **Typical Use:** EMI filtering, in buck/boost converters.
*   **Example:** 10\u00b5H inductor in a switching power supply.

---

### Diode (\u2192|-)

*   **Symbol:** `---| >|---` (Anode to Cathode, arrow points to direction of conventional current)
*   **Function:** Allows current to flow primarily in one direction.
*   **Unit:** None (rated by voltage/current).
*   **Key Specs:** Forward voltage drop, maximum reverse voltage, maximum forward current.
*   **Typical Use:** Rectification (AC to DC), protection against reverse polarity, flyback diodes for motor protection.
*   **Example:** 1N4001 for general rectification, 1N4148 for switching, LED for light emission.

---

### Transistor (BJT & MOSFET)

*   **Symbol:**
    *   NPN BJT: (Circle with arrow pointing outwards from emitter, base line, collector line)
    *   N-Channel MOSFET: (Gate line, source line, drain line, channel line)
*   **Function:** Amplifies or switches electronic signals and electrical power.
*   **Unit:** None (rated by voltage/current).
*   **Key Specs:** Max Vce/Vds, Max Ic/Id, Gain (hFE for BJT), Rds(on) (for MOSFET).
*   **Typical Use:** Switching motors, controlling LEDs, amplifying sensor signals.
*   **Example:** 2N2222 (NPN BJT), IRF520 (N-Channel MOSFET).

---

### Integrated Circuit (IC)

*   **Symbol:** Rectangle with pins, often numbered.
*   **Function:** A miniature electronic circuit fabricated on a single piece of semiconductor material, performing a wide range of complex functions.
*   **Unit:** None.
*   **Key Specs:** Function, supply voltage, pinout, package type.
*   **Typical Use:** Microcontrollers (ATmega328P, ESP32), motor drivers (L298N, DRV8833), voltage regulators (LM7805), operational amplifiers (LM358).
*   **Example:** ATmega328P (Arduino Uno), L298N (motor driver), HC-SR04 (ultrasonic sensor module contains an IC).

---

### Microcontroller (MCU)

*   **Symbol:** Often a rectangle with numerous pins, sometimes with a CPU symbol inside.
*   **Function:** A compact computer system on a single IC, containing CPU, memory, and programmable I/O peripherals. The "brain" of most robots.
*   **Unit:** None.
*   **Key Specs:** CPU architecture/speed, Flash memory size, SRAM size, number of GPIOs, ADC/PWM resolution, communication interfaces.
*   **Typical Use:** Controlling motors, reading sensors, making decisions, communicating with other devices.
*   **Example:** ATmega328P (Arduino Uno), ESP32.

---

### Motor, DC

*   **Symbol:** (Circle with an 'M' inside, and two terminals)
*   **Function:** Converts DC electrical energy into mechanical energy (rotation).
*   **Unit:** None.
*   **Key Specs:** Operating voltage, current draw, RPM, torque.
*   **Typical Use:** Driving wheels, robotic joints, fans.
*   **Example:** Small hobby gear motors.

---

### Motor, Servo (Hobby)

*   **Symbol:** Similar to DC motor, often with 3 wires (Power, GND, Signal).
*   **Function:** A geared DC motor with an integrated position sensor and control circuit, allowing it to hold a specific angular position.
*   **Unit:** None.
*   **Key Specs:** Operating voltage, torque, speed, angular range.
*   **Typical Use:** Robotic arms, steering mechanisms, pan/tilt for cameras.
*   **Example:** SG90, MG996R.

---

### Motor, Stepper

*   **Symbol:** Similar to DC motor, but usually with 4, 5, or 6 wires.
*   **Function:** Divides a full rotation into a number of equal steps, allowing for precise position control without a feedback sensor (open-loop).
*   **Unit:** None.
*   **Key Specs:** Step angle, holding torque, operating voltage/current.
*   **Typical Use:** 3D printers, CNC machines, precise positioning mechanisms.
*   **Example:** NEMA 17.

---

### LED (Light Emitting Diode)

*   **Symbol:** `---| >|---` (with two arrows pointing outwards)
*   **Function:** Emits light when current flows through it in the forward direction.
*   **Unit:** None.
*   **Key Specs:** Forward voltage, maximum forward current, color.
*   **Typical Use:** Status indicators, lighting.
*   **Example:** Red, green, blue LEDs.

---

### IR Sensor (Emitter/Receiver)

*   **Symbol:**
    *   IR Emitter: `---| >|---` (with two arrows pointing outwards, labeled IR)
    *   IR Receiver: `---| <|---` (with two arrows pointing inwards)
*   **Function:** Emits/detects infrared light. Used for line following, proximity detection.
*   **Unit:** None.
*   **Key Specs:** Wavelength, detection range.
*   **Typical Use:** HC-SR04 (part of ultrasonic module), line follower modules.
*   **Example:** TCRT5000 (reflective IR sensor).

---

### Ultrasonic Sensor

*   **Symbol:** Often a rectangle with two circles (for transmitter/receiver) and a wavy line radiating outwards.
*   **Function:** Uses high-frequency sound waves to detect objects and measure distance.
*   **Unit:** None.
*   **Key Specs:** Detection range, beam angle, operating voltage.
*   **Typical Use:** Obstacle avoidance, distance measurement.
*   **Example:** HC-SR04.

---

### IMU (Inertial Measurement Unit)

*   **Symbol:** Often a box with 'IMU' text, or separate symbols for accelerometer/gyroscope.
*   **Function:** Measures acceleration, angular velocity, and sometimes magnetic field, to determine orientation and motion.
*   **Unit:** None.
*   **Key Specs:** Number of axes (e.g., 6-DOF, 9-DOF), measurement range, communication protocol (I2C/SPI).
*   **Typical Use:** Robot balancing, drone flight control, attitude estimation.
*   **Example:** MPU6050, BNO055.

---

### Rotary Encoder

*   **Symbol:** Often a circle with output lines (A, B, Z) and power/ground.
*   **Function:** Converts angular position or motion of a shaft into an electrical signal (digital pulses).
*   **Unit:** None.
*   **Key Specs:** Pulses Per Revolution (PPR), number of channels (e.g., quadrature A/B).
*   **Typical Use:** Motor speed feedback, precise position control in robotic joints.
*   **Example:** Incremental quadrature encoder.

---

### Potentiometer

*   **Symbol:** `---/\/\/>---` (Arrow pointing to middle of resistor)
*   **Function:** A three-terminal resistor with a rotating or sliding contact that acts as a variable voltage divider.
*   **Unit:** Ohms (\u03a9).
*   **Key Specs:** Total resistance, power rating.
*   **Typical Use:** User input for adjusting parameters, sensor for measuring angle.
*   **Example:** 10k\u03a9 potentiometer.

---
