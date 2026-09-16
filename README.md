# Arduino ZC-600 Pulse Generator for Telescope Control

Firmware for an **Arduino Uno** acting as a high-precision hardware pulse generator and RS-485 interface converter for telescope mount tracking control via serial commands.

## Overview
This project handles real-time pulse generation for telescope tracking drives based on dynamic delay values received over a serial link. It isolates high-level positioning and tracking calculations from the low-level pulse timing and hardware bus management.

## Key Features
* **Robust Serial Parsing:** Uses line-based reading (`readStringUntil('\n')` with `.trim()`) to completely eliminate buffer errors and phantom zero-commands.
* **Hardware State Monitoring:** Tracks the `pinACT` status line using `INPUT_PULLUP` combined with a robust **5 ms debounce filter** to prevent noise interference.
* **Safe Direction Switching:** Automatically disables pulse generation and enforces a 10 µs bus-settling delay before changing the direction (`DIR`) line for the MAX485 transceiver.
* **Dynamic Tracking Support:** Accepts signed integers to control motor speed (pulse delay in microseconds) and direction on the fly.

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

## Communication Protocol
* **Baud Rate:** 9600 bps
* **Commands:**
  * Positive integer (`+val`): Sets sidereal/custom delay in microseconds and sets direction forward.
  * Negative integer (`-val`): Sets delay in microseconds and reverses direction (`DIR LOW`).
  * `0`: Stops the pulse cycle safely and resets default states.

## License
This project is open-source and intended for astronomical instrumentation and telescope automation.
