# Arduino ZC-600 Pulse Generator for Telescope Control

Firmware for an **Arduino Uno** acting as a high-precision hardware pulse generator and RS-485 interface converter for telescope mount tracking control.

## Overview
This project handles real-time pulse generation for telescope tracking drives based on dynamic delay values received over the **serial bus**. It isolates high-level positioning calculations from low-level pulse timing and hardware bus management.

## Key Features
* **Robust Serial Parsing:** Uses line-based reading (`readStringUntil('\n')` with `.trim()`) over the serial bus to completely eliminate buffer errors and phantom zero-commands.
* **Hardware State Monitoring:** Tracks the `pinACT` status line using `INPUT_PULLUP` combined with a robust **5 ms debounce filter** to prevent noise interference.
* **Safe Direction Switching:** Automatically disables pulse generation and enforces a 10 µs bus-settling delay before changing the direction (`DIR`) line for the MAX485 transceiver.
* **Dynamic Serial Control:** Accepts signed integers via the serial interface to control motor speed (`mydelay` in microseconds) and direction on the fly.

## Pin Configuration

| Pin | Function | Description |
| :--- | :--- | :--- |
| **Pin 2** (`pindiren`) | MAX485 DIR TX Enable | Transceiver control |
| **Pin 3** (`pinpulseen`) | MAX485 Pulse TX Enable | Pulse output control |
| **Pin 4** (`pindirrx`) | MAX485 DIR RX | Receiver line |
| **Pin 5** (`pindirtx`) | MAX485 DIR TX | Direction control line |
| **Pin 6** (`pinpulserx`) | MAX485 Pulse RX | Receiver line |
| **Pin 7** (`pinpulsetx`) | MAX485 Pulse TX | Main TTL pulse train output |
| **Pin 12** (`pinACT`) | Input Status (`INPUT_PULLUP`) | Drive activation (`LOW` = Run, `HIGH` = Idle) |
| **Pin 13** (`pinLED`) | Built-in LED | Visual activity indicator |

## Serial Communication Protocol
* **Baud Rate:** 9600 bps
* **Data Source:** Speed and direction parameters (`mydelay`, e.g., `175`, `-200`) are streamed dynamically over the **serial bus**.
* **Command Format:**
  * Positive integer (`+val`): Sets the pulse delay in microseconds and sets direction forward (`pindirtx` HIGH).
  * Negative integer (`-val`): Sets the pulse delay in microseconds and reverses direction (`pindirtx` LOW).
  * `0`: Stops the pulse cycle safely and resets default states.

## License
This project is open-source and intended for astronomical instrumentation and telescope automation.
