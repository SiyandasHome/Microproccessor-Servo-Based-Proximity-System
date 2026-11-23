# Proximity-Based Control System (ATmega328P)

An ultrasonic distance detector that drives a servo motor and three indicator LEDs, built **bare-metal on an Arduino Uno (ATmega328P)** using hardware timers and interrupts only: no `delay()`, no polling, no `pulseIn()`.

![Project build](docs/build-photo.jpg)
<!-- Add your breadboard photo (Figure 2 of the report) as docs/build-photo.jpg -->

## Overview

An HC-SR04 ultrasonic sensor measures the distance to an object. The measured range decides which of three states the system is in, and each state sets an LED and a servo angle. A push-button lets the user override the sensor and cycle through the states manually. This is the same idea used in automatic doors, parking sensors and industrial safety interlocks.

| Item | Detail |
|---|---|
| Course | Microprocessors, School of Electrical and Information Engineering, Wits |
| Microcontroller | ATmega328P (Arduino Uno), 16 MHz |
| Language | Embedded C/C++ (Arduino IDE / AVR registers) |
| Peripherals | HC-SR04 ultrasonic sensor, SG90 servo, 3 LEDs, push-button |
| Status | Complete, all functional requirements met |

## Features

- **Timer-driven distance measurement**: Timer0 generates the trigger pulse, Timer1 timestamps the echo.
- **Hardware PWM servo control**: Timer1 in Fast PWM mode 14 gives a precise 50 Hz signal.
- **Three-state logic**: critical, warning and safe, each with its own LED and servo position.
- **Interrupt-based button override** with a software debounce filter.
- **Built-in self-test** to separate hardware faults from software faults during debugging.

## How it works

### Servo (Timer1, Fast PWM mode 14)
The servo signal is on **D9 (PB1 / OC1A)**. Timer1 runs with `ICR1` as TOP and a prescaler of 8 (0.5 µs per tick):

```
f = f_clk / (TOP+1 × prescaler) = 16 000 000 / (40 000 × 8) = 50 Hz   (20 ms period)
pulse width = OCR1A × 0.5 µs
```

In theory `OCR1A` = 2000 / 3000 / 4000 gives 1.0 / 1.5 / 2.0 ms for 0° / 90° / 180°. In practice, the servo only gave consistent angles at **1350 / 3400 / 5150** (0.68 / 1.70 / 2.58 ms), so these calibrated values are used.

### Ultrasonic sensor (Timer0 trigger + Timer1 echo timing)
- **Trig = D7, Echo = D8 (PB0).**
- Timer0 runs in normal mode with prescaler 1024, overflowing at about **61 Hz** (16.4 ms measurement period).
- Each overflow raises Trig HIGH. The Timer0 compare match (`OCR0A = 1`) fires one tick later and drops it LOW, giving a **64 µs** trigger pulse.
- The echo pulse is timed by Timer1 through **Pin Change Interrupt 0**: `TCNT1` is latched on the rising and falling edges.

```
distance (cm) = echo_width_ticks / 116.6
```

| Distance band | Timer1 ticks |
|---|---|
| Boundary 1 | 1 160 ticks (about 10 cm) |
| Boundary 2 | 3 480 ticks (about 30 cm) |

### Button override (INT0 + debounce)
The button sits between **D2 (PD2)** and GND with the internal pull-up enabled. INT0 triggers on the falling edge. A counter incremented on every Timer0 overflow enforces a minimum gap between accepted presses:

```
tick_count − last_press ≥ 3 ticks (about 50 ms)
```

Edges inside that window are discarded. Each accepted press overrides the sensor and steps through critical, warning and safe, then returns to normal sensor operation.

## Pin map

| Pin | Port | Function |
|---|---|---|
| D2 | PD2 | Push-button (INT0, pull-up) |
| D3, D4, D5 | PD3–PD5 | State indicator LEDs |
| D7 | PD7 | HC-SR04 Trig |
| D8 | PB0 | HC-SR04 Echo (PCINT0) |
| D9 | PB1 | Servo signal (OC1A) |
| 5 V / GND | | Sensor, servo and LED supply |

## Program flow

![Program flowchart](docs/flowchart.png)
<!-- Add the flowchart from Figure 1 of the report as docs/flowchart.png -->

1. Configure I/O, Timer0, Timer1, PCINT0 and INT0, then run the self-test.
2. Enable global interrupts and wait.
3. On a **sensor interrupt**, compute the distance and set the state. On a **button interrupt**, override the state.
4. Apply the state (LED plus servo position) and return to waiting.

## Results

All modules ran in sync. The servo reached the intended angles after calibration, the sensor assigned each object distance to the correct LED and servo state, and the button override cycled cleanly through all three states.

## Lessons learned

- Datasheet servo timings did not match the physical servo, so **calibration against real hardware** was essential.
- Mechanical buttons bounce, so a short **software debounce window** made the override reliable.
- A **self-test routine** cut debugging time substantially by showing whether a fault was hardware or software.

## Repository structure

```
servo-proximity-control/
├── src/
│   └── Project_final.ino          # main firmware
├── build/
│   └── Project_final_ino.hex      # compiled firmware (flash with avrdude)
├── docs/
│   ├── Proximity_Control_Report.pdf
│   ├── build-photo.jpg
│   └── flowchart.png
└── README.md
```

## Build and flash

1. Open `src/Project_final.ino` in the Arduino IDE.
2. Select **Board: Arduino Uno**, pick your port, then **Upload**.
3. Or flash the prebuilt hex: `avrdude -p m328p -c arduino -P <PORT> -b 115200 -U flash:w:build/Project_final_ino.hex:i`

## Documentation

Full write-up with equations, validation and references: [`docs/Proximity_Control_Report.pdf`](docs/Proximity_Control_Report.pdf)

## References

1. Elecfreaks, *Ultrasonic Ranging Module HC-SR04 Datasheet*.
2. TowerPro, *SG90 Micro Servo Motor Datasheet*.
3. Atmel, *ATmega328P 8-bit AVR Microcontroller Datasheet*, 2016.

## Author

**Siyanda Ngwenya**, BSc Electrical Engineering, University of the Witwatersrand
