# Proximity-Based Servo Control System (AVR Assembly, ATmega328P)

An ultrasonic distance detector that drives a servo motor and three indicator LEDs, written **entirely in AVR assembly** for the ATmega328P (Arduino Uno). It uses hardware timers and interrupts only: no `delay()`, no polling, no Arduino libraries. The main loop is a single `rjmp` and all the work happens in interrupt service routines.

![Project build](docs/build-photo.jpg)
<!-- Add your breadboard photo (Figure 2 of the report) as docs/build-photo.jpg -->

## Overview

An HC-SR04 ultrasonic sensor measures the distance to an object. The range selects one of three states, and each state sets an LED colour and a servo angle. A push-button lets the user override the sensor and step through the states manually. The same idea is used in automatic doors, parking sensors and industrial safety interlocks.

| Item | Detail |
|---|---|
| Course | ELEN2021A Microprocessors, School of Electrical and Information Engineering, Wits |
| Microcontroller | ATmega328P (Arduino Uno), 16 MHz |
| Language | AVR assembly (GNU `avr-as`, `.S` file) |
| Peripherals | HC-SR04 ultrasonic sensor, SG90 servo, 3 LEDs, push-button |
| Status | Complete, all functional requirements met |

## States

| State | Distance | LED | Servo angle |
|---|---|---|---|
| **Safe** | more than 30 cm | Green (D3) | 0° |
| **Warning** | 10 to 30 cm | Blue (D4) | 90° |
| **Critical** | less than 10 cm | Red (D5) | 180° |

## Features

- **Timer-driven distance measurement:** Timer0 generates the trigger pulse and Timer1 timestamps the echo.
- **Hardware PWM servo control:** Timer1 in Fast PWM mode 14 gives a precise 50 Hz signal.
- **Interrupt-driven design:** four ISRs (button, echo edge, Timer0 overflow, Timer0 compare) do all the work.
- **Button override** with an overflow-counter debounce.
- **Power-on self-test** that sweeps all LEDs and servo positions to separate hardware faults from software faults.
- **Timer wrap-around correction** so echo measurements stay correct when Timer1 rolls over mid-pulse.

## How it works

### Servo (Timer1, Fast PWM mode 14)
The servo signal is on **D9 (PB1 / OC1A)**. Timer1 uses `ICR1` as TOP (39999) with a prescaler of 8, so one tick is 0.5 µs:

```
f = f_clk / ((TOP + 1) × prescaler) = 16 000 000 / (40 000 × 8) = 50 Hz   (20 ms period)
pulse width = OCR1A × 0.5 µs
```

In theory `OCR1A` = 2000 / 3000 / 4000 gives 1.0 / 1.5 / 2.0 ms for 0° / 90° / 180°. In practice the servo only gave consistent angles at **1350 / 3400 / 5150** (0.68 / 1.70 / 2.58 ms), so those calibrated values are used. All 16-bit register writes follow the AVR rule of writing the high byte first, and reads take the low byte first.

### Ultrasonic sensor (Timer0 trigger, Timer1 echo timing)
- **Trig = D7 (PD7), Echo = D8 (PB0 / PCINT0).**
- Timer0 runs in normal mode with prescaler 1024, overflowing every 16.384 ms (about 61 Hz).
- The overflow ISR raises Trig. The compare-match ISR (`OCR0A = 1`) fires 64 µs later and drops it, giving a 64 µs trigger pulse.
- Pin Change Interrupt 0 fires on both echo edges. On the rising edge `TCNT1` is saved, and on the falling edge the pulse width is computed (with a correction if Timer1 wrapped during the pulse).

```
distance (cm) = echo_width_ticks / 116.6
```

| Boundary | Timer1 ticks |
|---|---|
| Critical / Warning | 1 160 (about 10 cm) |
| Warning / Safe | 3 480 (about 30 cm) |

The echo width is compared with both 16-bit thresholds, and `set_state` drives the LEDs and servo.

### Button override (INT0 and debounce)
The button sits between **D2 (PD2)** and GND with the internal pull-up enabled, and INT0 triggers on the falling edge. A tick counter incremented on every Timer0 overflow enforces a minimum gap between accepted presses:

```
tick_count − last_press ≥ 3 ticks   (about 50 ms, 8-bit subtraction wraps correctly)
```

Each accepted press cycles the mode: **sensor → Safe → Warning → Critical → sensor**. While an override is active the sensor readings are ignored.

### Self-test
For the first 18 timer overflows (about 295 ms) after reset, the system runs a test pattern: green with servo at 0°, then blue at 90°, then red at 180°, then everything off and the servo returns to 0°. During the test the trigger pulse is suppressed and the button is ignored so nothing disturbs the display. If an LED or the servo misbehaves here, the fault is in the hardware or wiring.

## Interrupt map

| ISR | Source | Job |
|---|---|---|
| `__vector_1` | INT0 (D2 button) | Debounce and cycle override state |
| `__vector_3` | PCINT0 (D8 echo) | Time the echo, then classify distance |
| `__vector_16` | Timer0 overflow | Raise Trig, advance tick counter, run self-test |
| `__vector_14` | Timer0 compare A | Drop Trig after 64 µs |

## Pin map

| Pin | Port | Function |
|---|---|---|
| D2 | PD2 | Push-button (INT0, pull-up) |
| D3 | PD3 | Green LED (Safe) |
| D4 | PD4 | Blue LED (Warning) |
| D5 | PD5 | Red LED (Critical) |
| D7 | PD7 | HC-SR04 Trig |
| D8 | PB0 | HC-SR04 Echo (PCINT0) |
| D9 | PB1 | Servo signal (OC1A) |

## SRAM variables (0x0100 to 0x0107)

| Address | Name | Purpose |
|---|---|---|
| 0x0100 | `echo_start` (2 bytes) | `TCNT1` snapshot at echo rising edge |
| 0x0102 | `echo_width` (2 bytes) | Measured pulse width in Timer1 ticks |
| 0x0104 | `override` | 0 = sensor, 1 to 3 = forced state |
| 0x0105 | `tick_count` | Incremented on every Timer0 overflow |
| 0x0106 | `last_press` | `tick_count` at last accepted press |
| 0x0107 | `selftest` | Self-test countdown (18 down to 0) |

## Program flow

![Program flowchart](docs/flowchart.png)
<!-- Add the flowchart from Figure 1 of the report as docs/flowchart.png -->

1. Set up the stack, I/O, Timer1 PWM, Timer0, PCINT0 and INT0, initialise SRAM and start the self-test.
2. Enable global interrupts and idle in `main_loop`.
3. On each interrupt, the matching ISR updates the state, LEDs and servo, then returns.

## Results

All modules ran in sync. The servo reached the intended angles after calibration, the sensor assigned each distance to the correct LED and servo state, and the button override cycled cleanly through all states.

## Lessons learned

- Datasheet servo timings did not match the physical servo, so **calibration against real hardware** was essential.
- Mechanical buttons bounce, and a short **debounce window** made the override reliable.
- The **self-test** cut debugging time by showing whether a fault was hardware or software.
- Writing in assembly made the **timing, register access order and interrupt context saving** explicit, since every ISR saves and restores `SREG` and any registers it uses.

## Repository structure

```
servo-proximity-control/
├── src/
│   └── Project_final/
│       ├── Project_final.ino   # intentionally empty, required by the Arduino IDE
│       └── Project_final.S     # all program logic (AVR assembly)
├── build/
│   └── Project_final_ino.hex   # compiled firmware
├── docs/
│   ├── Proximity_Control_Report.pdf
│   ├── build-photo.jpg
│   └── flowchart.png
└── README.md
```

> The `.ino` file is empty on purpose: the Arduino IDE needs one to treat the folder as a sketch, and it then compiles the `.S` file beside it. The `.S` file defines its own `main`, so the Arduino runtime startup code is not used and Timer0 and Timer1 are fully under the program's control.

## Build and flash

1. Put both files in a folder with the same name as the `.ino` (for example `Project_final/`) and open it in the Arduino IDE.
2. Select **Board: Arduino Uno**, pick your port, then **Upload**.
3. Or flash the prebuilt hex: `avrdude -p m328p -c arduino -P <PORT> -b 115200 -U flash:w:build/Project_final_ino.hex:i`

## Documentation

Full write-up with equations, validation, and references: [`docs/Proximity_Control_Report.pdf`](docs/Proximity_Control_Report.pdf)

## References

1. Elecfreaks, *Ultrasonic Ranging Module HC-SR04 Datasheet*.
2. TowerPro, *SG90 Micro Servo Motor Datasheet*.
3. Atmel, *ATmega328P 8-bit AVR Microcontroller Datasheet*, 2016.

## Author

**Siyanda Ngwenya**, BSc Electrical Engineering, University of the Witwatersrand
