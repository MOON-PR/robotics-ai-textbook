---
sidebar_position: 3
title: Arduino and ESP32 Pinout Diagrams
id: book-11-appendix-03-arduino-esp32-pinout-diagrams
---

## 03-Arduino and ESP32 Pinout Diagrams

Understanding the pinout of your microcontroller board is crucial for connecting sensors, actuators, and other peripherals correctly. This section provides reference diagrams and explanations for the most common pins on Arduino Uno and a typical ESP32 Development Board.

### 3.1 Arduino Uno Pinout

The Arduino Uno is based on the ATmega328P microcontroller. It features a good balance of digital I/O, analog inputs, and PWM pins.

**Diagram 3.1: Arduino Uno Pinout Diagram (Conceptual)**

```mermaid
graph TD
    A[USB Connector]
    B[Power Jack]
    C[Reset Button]
    
    subgraph Digital Pins (PWM capable)
        D9(D9~)
        D10(D10~)
        D11(D11~)
    end
    subgraph Digital Pins (General)
        D0(D0/RX)
        D1(D1/TX)
        D2(D2/INT0)
        D3(D3/INT1~)
        D4(D4)
        D5(D5~)
        D6(D6~)
        D7(D7)
        D8(D8)
        D12(D12)
        D13(D13/LED~)
    end
    subgraph Analog Pins
        A0(A0)
        A1(A1)
        A2(A2)
        A3(A3)
        A4(A4/SDA)
        A5(A5/SCL)
    end
    subgraph Power Pins
        VIN(VIN)
        GND_1(GND)
        GND_2(GND)
        GND_3(GND)
        V5(5V)
        V33(3.3V)
        AREF(AREF)
        RESET(RESET)
    end

    A --- D0
    A --- D1
    B --- VIN
    B --- GND_1
    C --- RESET

    D0 -- TX/RX --> D1
    D2 -- Interrupt --> ATMEGA(ATmega328P)
    D3 -- Interrupt --> ATMEGA
    A4 -- I2C --> A5
    V5 -- Power --> D_pins
    V33 -- Power --> D_pins

    D_pins(Digital Pins 0-13)
    A_pins(Analog Pins A0-A5)
    P_pins(Power Pins)

    D0 & D1 --> P_pins
    D2 & D3 --> P_pins
    D4 & D5 & D6 --> P_pins
    D7 & D8 & D9 --> P_pins
    D10 & D11 & D12 & D13 --> P_pins

    A0 & A1 & A2 & A3 & A4 & A5 --> P_pins
    VIN & GND_1 & GND_2 & GND_3 & V5 & V33 & AREF & RESET --> P_pins
```

*Description: A conceptual diagram showing the common pin groups on an Arduino Uno board, highlighting digital I/O (including PWM), analog input, power, and communication pins.*

#### 3.1.1 Key Pin Functions

*   **Digital Pins (0-13):** General Purpose Input/Output.
    *   **PWM (~) Pins (3, 5, 6, 9, 10, 11):** Can be used for `analogWrite()` to control motor speed, LED brightness, etc.
    *   **Serial (0-RX, 1-TX):** Hardware serial communication. Avoid using if you're using `Serial.begin()` to communicate with your computer.
    *   **External Interrupts (2-INT0, 3-INT1):** Can trigger an Interrupt Service Routine (ISR) on a change of state.
*   **Analog Pins (A0-A5):** Can read analog voltage values (0-5V, mapped to 0-1023).
    *   **I2C (A4-SDA, A5-SCL):** Hardware I2C communication.
*   **Power Pins:**
    *   **5V:** Regulated 5V supply, output.
    *   **3.3V:** Regulated 3.3V supply, output (lower current).
    *   **GND:** Ground pins.
    *   **Vin:** Input voltage to Arduino when using an external power source (7-12V recommended).
    *   **AREF:** Analog Reference voltage.

### 3.2 ESP32 Development Board Pinout (Generic)

The ESP32 is a powerful microcontroller with integrated Wi-Fi and Bluetooth. Pinouts can vary slightly between different development boards (e.g., ESP32-WROOM-32 DevKitC, NodeMCU-32). This diagram represents a common arrangement.

**Diagram 3.2: Generic ESP32 Development Board Pinout Diagram (Conceptual)**

```mermaid
graph TD
    A[USB-C/Micro-USB]
    B[EN Button (Reset)]
    C[BOOT Button (Flash)]

    subgraph GPIO Pins (General Purpose)
        GND_1(GND)
        V3V3_1(3V3)
        GPIO36(GPIO36/ADC)
        GPIO39(GPIO39/ADC)
        GPIO34(GPIO34/ADC)
        GPIO35(GPIO35/ADC)
        GPIO32(GPIO32/ADC/Touch)
        GPIO33(GPIO33/ADC/Touch)
        GPIO25(GPIO25/ADC/DAC)
        GPIO26(GPIO26/ADC/DAC)
        GPIO27(GPIO27/ADC/Touch)
        GPIO14(GPIO14/ADC/Touch)
        GPIO12(GPIO12/ADC/Touch)
        GPIO13(GPIO13/ADC/Touch)
        GPIO9(GPIO9)
        GPIO10(GPIO10)
        GPIO11(GPIO11)
        GPIO6(GPIO6)
        GPIO7(GPIO7)
        GPIO8(GPIO8)
    end
    subgraph GPIO Pins (Specific Functions)
        GND_2(GND)
        V3V3_2(3V3)
        GPIO23(GPIO23/SPI MOSI)
        GPIO22(GPIO22/I2C SCL)
        GPIO21(GPIO21/I2C SDA)
        GPIO19(GPIO19/SPI MISO)
        GPIO18(GPIO18/SPI CLK)
        GPIO5(GPIO5/SPI SS)
        GPIO17(GPIO17/UART2 TX)
        GPIO16(GPIO16/UART2 RX)
        GPIO4(GPIO4/ADC/Touch/LED)
        GPIO2(GPIO2/ADC/Touch/LED/BOOT)
        GPIO0(GPIO0/ADC/Touch/BOOT)
        GPIO15(GPIO15/ADC/Touch/BOOT)
        GPIO13_1(GPIO13/ADC/Touch)
        GPIO12_1(GPIO12/ADC/Touch)
        GPIO14_1(GPIO14/ADC/Touch)
        GPIO27_1(GPIO27/ADC/Touch)
    end
    subgraph Power Pins
        VIN_ESP32(VIN)
        GND_3(GND)
        V5_ESP32(5V)
    end

    A --- V5_ESP32
    A --- GND_1
    B --- ESP_CORE(ESP32 Core)
    C --- ESP_CORE

    GPIO21 -- I2C --> GPIO22
    GPIO23 -- SPI --> GPIO19
    GPIO18 -- SPI --> GPIO5

    GND_1 & GND_2 & GND_3 --> ESP_CORE
    V3V3_1 & V3V3_2 --> ESP_CORE
    VIN_ESP32 & V5_ESP32 --> ESP_CORE

    GPIO_pins(GPIOs)
    P_pins(Power Pins)

    GND_1 & V3V3_1 & GPIO36 & GPIO39 & GPIO34 & GPIO35 & GPIO32 & GPIO33 & GPIO25 & GPIO26 & GPIO27 & GPIO14 & GPIO12 & GPIO13 & GPIO9 & GPIO10 & GPIO11 & GPIO6 & GPIO7 & GPIO8 --> GPIO_pins
    GND_2 & V3V3_2 & GPIO23 & GPIO22 & GPIO21 & GPIO19 & GPIO18 & GPIO5 & GPIO17 & GPIO16 & GPIO4 & GPIO2 & GPIO0 & GPIO15 --> GPIO_pins
    VIN_ESP32 & GND_3 & V5_ESP32 --> P_pins
```

*Description: A conceptual diagram showing a generic ESP32 development board pinout, highlighting digital I/O, analog input, touch, DAC, and communication pins, along with power pins.*

#### 3.2.1 Key Pin Functions

*   **GPIO Pins:** Many GPIOs are available (typically 30+). Most can be configured as input/output.
    *   **ADC Pins (Analog-to-Digital Converter):** Many GPIOs support 12-bit ADC (e.g., GPIO 32-36, 39, etc.).
    *   **DAC Pins (Digital-to-Analog Converter):** GPIO 25, 26 can output true analog voltage.
    *   **Touch Pins:** Several GPIOs have built-in capacitive touch sensing.
    *   **PWM:** Any GPIO can generate PWM using the `ledc` peripheral.
    *   **I2C:** Two hardware I2C interfaces (default: SDA=GPIO21, SCL=GPIO22, but configurable).
    *   **SPI:** Two hardware SPI interfaces (HSPI, VSPI). Common pins for VSPI: CLK=GPIO18, MISO=GPIO19, MOSI=GPIO23, SS=GPIO5 (configurable).
    *   **UART:** Three hardware UART interfaces (UART0: TX=GPIO1, RX=GPIO3 (used for serial monitor); UART1: TX=GPIO10, RX=GPIO9; UART2: TX=GPIO17, RX=GPIO16, configurable).
*   **Power Pins:**
    *   **3V3:** Regulated 3.3V supply, output.
    *   **5V:** Input from USB or external 5V source. Can also be output if powered from USB.
    *   **GND:** Ground pins.
    *   **Vin:** Input voltage for the on-board 3.3V regulator (typically 5-12V).
*   **Special Pins:**
    *   **GPIO0, GPIO2, GPIO4, GPIO12, GPIO15:** Have specific roles during boot-up. Be careful using them for inputs or outputs that interfere with the boot process.
    *   **GPIO1 (TX0), GPIO3 (RX0):** Used for USB serial communication (Serial Monitor).

### 3.3 Important Considerations

*   **Logic Levels:** Arduino Uno typically uses 5V logic. ESP32 uses 3.3V logic. When connecting 5V components to ESP32, you might need level shifters for robust communication.
*   **Current Limits:** Microcontroller GPIOs have limited current sourcing/sinking capabilities. Always use transistors or motor drivers for motors and high-current loads.
*   **Input Pull-ups/Pull-downs:** Many digital input pins can be configured with internal pull-up/pull-down resistors, simplifying wiring for buttons or switches.
*   **Analog Reference:** Be aware of the `analogReference()` setting for Arduino and the default ADC voltage range for ESP32 when reading analog sensors.

Always refer to the specific pinout diagram and datasheet for your exact Arduino or ESP32 board, as variations exist.

---
