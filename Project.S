;; ===========================================================================
;; ELEN2021A Project: Proximity-Based Servo Control System
;; Target:   Arduino Uno (ATmega328P @ 16 MHz)
;; ===========================================================================

#define __SFR_OFFSET 0
#include <avr/io.h>

;; ---------------------------------------------------------------------------
;; Pin map
;;   PB1 / D9 : Servo PWM             (OC1A)
;;   PB0 / D8 : HC-SR04 Echo          (PCINT0)
;;   PD7 / D7 : HC-SR04 Trig
;;   PD5 / D5 : Red   LED  (Critical)
;;   PD4 / D4 : Blue  LED  (Warning)
;;   PD3 / D3 : Green LED  (Safe)
;;   PD2 / D2 : Push button  (INT0, internal pull-up, other leg to GND)
;; ---------------------------------------------------------------------------

.equ TRIG_BIT,    PD7
.equ ECHO_BIT,    PB0
.equ SERVO_BIT,   PB1
.equ LED_SAFE,    PD3
.equ LED_WARN,    PD4
.equ LED_CRIT,    PD5
.equ BTN_BIT,     PD2

.equ LED_MASK,    (1<<LED_SAFE)|(1<<LED_WARN)|(1<<LED_CRIT)


;; ---------------------------------------------------------------------------
;; Timer1 servo PWM  (Fast PWM mode 14, prescaler /8  ->  0.5 µs per tick)
;;   period = (39999+1) * 8 / 16e6 = 20 ms  (50 Hz)
;;   OCR1A  : 2000 = 1.0 ms =   0°(ideal)
;;            3000 = 1.5 ms =  90°(ideal)
;;            4000 = 2.0 ms = 180°(ideal)
;; ---------------------------------------------------------------------------
.equ TIMER1_TOP,  39999
.equ SERVO_0,      1750 ;;(reality)
.equ SERVO_90,     3500 ;;(reality)
.equ SERVO_180,    5150 ;;(reality)

;; ---------------------------------------------------------------------------
;; Distance thresholds (Timer1 ticks = distance_cm * 116)
;;   10 cm -> 1160 ticks   (Critical / Warning boundary)
;;   30 cm -> 3480 ticks   (Warning  / Safe     boundary)
;; ---------------------------------------------------------------------------
.equ THRESH_CRIT, 1160
.equ THRESH_WARN, 3480


;; State codes
.equ STATE_SAFE,  0
.equ STATE_WARN,  1
.equ STATE_CRIT,  2

;; ---------------------------------------------------------------------------
;; SRAM state  (0x0100 – 0x0107, no .lcomm, no .byte, no BSS)
;; ---------------------------------------------------------------------------
.equ  echo_start, 0x0100   ; 2 B  TCNT1 snapshot at echo rising edge
.equ  echo_width, 0x0102   ; 2 B  measured pulse width (Timer1 ticks)
.equ  override,   0x0104   ; 1 B  0 = sensor, 1–3 = forced state+1
.equ  tick_count, 0x0105   ; 1 B  ++ every Timer0 OVF (~16.4 ms)
.equ  last_press, 0x0106   ; 1 B  tick_count at the last accepted press
.equ  selftest,   0x0107   ; 1 B  countdown: 18..1 = testing, 0 = done

;; ===========================================================================
;; main
;; ===========================================================================
.section .text
.global main

main:
    ;; Stack pointer
    ldi   r16, lo8(RAMEND)
    out   SPL, r16
    ldi   r16, hi8(RAMEND)
    out   SPH, r16

    ;; PORTB: PB1 (servo) output; PB0 (echo) input, no pull-up
    ldi   r16, (1<<SERVO_BIT)
    out   DDRB, r16
    ldi   r16, 0
    out   PORTB, r16

    ;; PORTD: PD3/4/5/7 outputs; PD2 input with internal pull-up
    ldi   r16, (1<<LED_SAFE)|(1<<LED_WARN)|(1<<LED_CRIT)|(1<<TRIG_BIT)
    out   DDRD, r16
    ldi   r16, (1<<BTN_BIT)
    out   PORTD, r16

    ;; Timer1: Fast PWM mode 14, prescaler /8, OC1A non-inverting
    ldi   r16, (1<<COM1A1)|(1<<WGM11)
    sts   TCCR1A, r16
    ldi   r16, (1<<WGM13)|(1<<WGM12)|(1<<CS11)
    sts   TCCR1B, r16
    ldi   r16, hi8(TIMER1_TOP)
    sts   ICR1H, r16
    ldi   r16, lo8(TIMER1_TOP)
    sts   ICR1L, r16
    ;; park servo at 0° (HIGH byte first — §16.3)
    ldi   r16, hi8(SERVO_0)
    sts   OCR1AH, r16
    ldi   r16, lo8(SERVO_0)
    sts   OCR1AL, r16

    ;; Timer0: Normal mode, prescaler /1024
    ;;   OVF  fires every 256*1024/16e6 = 16.384 ms  →  raise Trig + tick
    ;;   COMPA fires 1 tick (64 µs) later              →  drop  Trig
    ldi   r16, 0
    out   TCCR0A, r16
    ldi   r16, (1<<CS02)|(1<<CS00)
    out   TCCR0B, r16
    ldi   r16, 1
    out   OCR0A, r16
    ldi   r16, (1<<TOIE0)|(1<<OCIE0A)
    sts   TIMSK0, r16

    ;; PCINT0 on PB0 (Echo)
    ldi   r16, (1<<PCIE0)
    sts   PCICR, r16
    ldi   r16, (1<<PCINT0)
    sts   PCMSK0, r16

    ;; INT0 on PD2 (button), falling edge
    ldi   r16, (1<<ISC01)
    sts   EICRA, r16
    ldi   r16, (1<<INT0)
    out   EIMSK, r16

    ;; Initialise SRAM state
    clr   r16
    sts   override,    r16
    sts   echo_start,  r16
    sts   echo_start+1,r16
    sts   echo_width,  r16
    sts   echo_width+1,r16
    sts   last_press,  r16
    ;; tick_count = 3: debounce guard (elapsed >= 3) is already satisfied
    ;; the instant selftest clears — no dead-zone on first press.
    ldi   r16, 3
    sts   tick_count,  r16
    ;; selftest countdown (18 OVF ticks ≈ 295 ms total):
    ;;   ticks 17–12 (6)  →  Green + servo   0°
    ;;   ticks 11–6  (6)  →  Blue  + servo  90°
    ;;   ticks  5–1  (5)  →  Red   + servo 180°
    ;;   tick   0         →  all off + servo 0° → normal operation
    ldi   r16, 18
    sts   selftest,    r16

    ;; LEDs off
    in    r16, PORTD
    andi  r16, lo8(~LED_MASK)
    out   PORTD, r16

    sei

main_loop:
    rjmp  main_loop          ; all work happens in ISRs


;; ===========================================================================
;; __vector_1  —  INT0 / push button on PD2
;;   datasheet vector 2  →  avr-libc index 1
;; ===========================================================================
.global __vector_1
__vector_1:
    push  r16
    in    r16, SREG
    push  r16
    push  r17
    push  r18

    ;; Block button during self-test (avoids corrupting the test pattern
    ;; and prevents debounce-counter confusion).
    lds   r17, selftest
    tst   r17
    brne  int0_done

    ;; Software debounce: require >= 3 OVF ticks (~50 ms) since last press.
    ;; 8-bit subtraction wraps mod-256 — safe across tick_count rollover.
    lds   r17, tick_count
    lds   r18, last_press
    sub   r17, r18
    cpi   r17, 3
    brlo  int0_done

    lds   r17, tick_count
    sts   last_press, r17

    ;; Cycle override: 0 (sensor) → 1 (Safe) → 2 (Warn) → 3 (Critical) → 0
    lds   r17, override
    inc   r17
    cpi   r17, 4
    brlo  int0_save
    clr   r17
int0_save:
    sts   override, r17
    rcall apply_override

int0_done:
    pop   r18
    pop   r17
    pop   r16
    out   SREG, r16
    pop   r16
    reti


;; ===========================================================================
;; __vector_3  —  PCINT0 / HC-SR04 Echo on PB0
;;   datasheet vector 4  →  avr-libc index 3
;; ===========================================================================
.global __vector_3
__vector_3:
    push  r16
    in    r16, SREG
    push  r16
    push  r17
    push  r18
    push  r19

    in    r17, PINB
    sbrs  r17, ECHO_BIT
    rjmp  echo_falling

echo_rising:
    lds   r18, TCNT1L        ; read LOW first → latches HIGH into TEMP (§16.3)
    lds   r19, TCNT1H
    sts   echo_start,   r18
    sts   echo_start+1, r19
    rjmp  echo_done

echo_falling:
    lds   r18, TCNT1L
    lds   r19, TCNT1H
    lds   r16, echo_start
    lds   r17, echo_start+1
    sub   r18, r16
    sbc   r19, r17
    brcc  echo_store
    ;; Timer1 wrapped through TOP+1 (40000) during the pulse — correct it.
    subi  r18, lo8(-40000)
    sbci  r19, hi8(-40000)
echo_store:
    sts   echo_width,   r18
    sts   echo_width+1, r19
    rcall classify_distance

echo_done:
    pop   r19
    pop   r18
    pop   r17
    pop   r16
    out   SREG, r16
    pop   r16
    reti


;; ===========================================================================
;; __vector_16  —  Timer0 OVF  (~16.4 ms)
;;   datasheet vector 17  →  avr-libc index 16
;;
;;   While selftest > 0: sequence LEDs/servo through the test pattern and
;;   suppress the Trig pulse so no echo measurements disturb the display.
;;   Once selftest == 0: raise Trig and advance tick_count normally.
;; ===========================================================================
.global __vector_16
__vector_16:
    push  r16
    in    r16, SREG
    push  r16
    push  r17
    push  r18
    push  r19

    lds   r17, selftest
    tst   r17
    breq  ovf_normal

    dec   r17
    sts   selftest, r17

    tst   r17
    breq  st_done_all

    cpi   r17, 12            ; >= 12  →  Green + 0°
    brsh  st_green
    cpi   r17, 6             ; 6..11  →  Blue  + 90°
    brsh  st_blue
    ;; 1..5  →  Red + 180°
    ldi   r17, STATE_CRIT
    rcall set_state
    rjmp  ovf_done

st_green:
    ldi   r17, STATE_SAFE
    rcall set_state
    rjmp  ovf_done

st_blue:
    ldi   r17, STATE_WARN
    rcall set_state
    rjmp  ovf_done

st_done_all:
    in    r16, PORTD
    andi  r16, lo8(~LED_MASK)
    out   PORTD, r16
    ldi   r18, lo8(SERVO_0)
    ldi   r19, hi8(SERVO_0)
    sts   OCR1AH, r19        ; HIGH first (§16.3)
    sts   OCR1AL, r18
    rjmp  ovf_done

ovf_normal:
    sbi   PORTD, TRIG_BIT
    lds   r17, tick_count
    inc   r17
    sts   tick_count, r17

ovf_done:
    pop   r19
    pop   r18
    pop   r17
    pop   r16
    out   SREG, r16
    pop   r16
    reti


;; ===========================================================================
;; __vector_14  —  Timer0 COMPA  (~64 µs after OVF)
;;   datasheet vector 15  →  avr-libc index 14
;; ===========================================================================
.global __vector_14
__vector_14:
    push  r16
    in    r16, SREG
    push  r16

    cbi   PORTD, TRIG_BIT

    pop   r16
    out   SREG, r16
    pop   r16
    reti


;; ===========================================================================
;; classify_distance  —  called from echo_falling
;;   Skips if selftest or manual override is active.
;;   Compares echo_width against the two 16-bit thresholds and calls set_state.
;; ===========================================================================
classify_distance:
    push  r16
    push  r17
    push  r18
    push  r19

    lds   r16, selftest
    tst   r16
    brne  cd_done            ; self-test active → leave display alone

    lds   r16, override
    tst   r16
    brne  cd_done            ; override active  → sensor ignored

    lds   r18, echo_width
    lds   r19, echo_width+1

    ldi   r17, hi8(THRESH_CRIT)
    cpi   r18, lo8(THRESH_CRIT)
    cpc   r19, r17
    brlo  cd_critical

    ldi   r17, hi8(THRESH_WARN)
    cpi   r18, lo8(THRESH_WARN)
    cpc   r19, r17
    brlo  cd_warning

    ldi   r17, STATE_SAFE
    rjmp  cd_apply
cd_warning:
    ldi   r17, STATE_WARN
    rjmp  cd_apply
cd_critical:
    ldi   r17, STATE_CRIT
cd_apply:
    rcall set_state

cd_done:
    pop   r19
    pop   r18
    pop   r17
    pop   r16
    ret


;; ===========================================================================
;; apply_override  —  called from INT0 ISR
;;   If override != 0, maps 1..3 → STATE_SAFE..STATE_CRIT and calls set_state.
;;   If override == 0, returns immediately (sensor will resume on next echo).
;; ===========================================================================
apply_override:
    push  r16
    push  r17

    lds   r16, override
    tst   r16
    breq  ao_done

    mov   r17, r16
    dec   r17                ; 1..3 → 0..2 = STATE_SAFE..STATE_CRIT
    rcall set_state

ao_done:
    pop   r17
    pop   r16
    ret


;; ===========================================================================
;; set_state(r17)  —  drives LEDs (PORTD) and servo (OCR1A)
;;   r17 = STATE_SAFE (0), STATE_WARN (1), or STATE_CRIT (2)
;;   NOTE: OCR1AH must be written before OCR1AL (ATmega328P §16.3).
;; ===========================================================================
set_state:
    push  r16
    push  r18
    push  r19

    in    r16, PORTD
    andi  r16, lo8(~LED_MASK)

    cpi   r17, STATE_SAFE
    brne  ss_check_warn
    ori   r16, (1<<LED_SAFE)
    ldi   r18, lo8(SERVO_0)
    ldi   r19, hi8(SERVO_0)
    rjmp  ss_apply

ss_check_warn:
    cpi   r17, STATE_WARN
    brne  ss_critical
    ori   r16, (1<<LED_WARN)
    ldi   r18, lo8(SERVO_90)
    ldi   r19, hi8(SERVO_90)
    rjmp  ss_apply

ss_critical:
    ori   r16, (1<<LED_CRIT)
    ldi   r18, lo8(SERVO_180)
    ldi   r19, hi8(SERVO_180)

ss_apply:
    out   PORTD, r16
    sts   OCR1AH, r19        ; HIGH byte first  (§16.3)
    sts   OCR1AL, r18        ; LOW byte second  — triggers atomic update

    pop   r19
    pop   r18
    pop   r16
    ret