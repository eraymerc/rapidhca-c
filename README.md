# rapidHCA-c

**rapidHCA-c** is a high-performance, deterministic C implementation of the  
**Harmonic Control Array (HCA)** controller based on parallel PI controllers.

The library is designed for real-time control applications and optimized using
Direct Digital Synthesis (DDS) and LUT-based phasor generation.
It has been validated in real-time execution on a STM32F4 series microcontroller but remains fully portable
and suitable for desktop simulations.

Still under development and will be updated within the next 6 months.
---

## What this repository provides

- A standalone **C implementation** of the Harmonic Control Array Method
- Deterministic O(1) execution time per control step
- LUT-based sine/cosine generation (no runtime trigonometry)
- Parallel harmonic channels with complex PI controllers
- No dynamic memory allocation
- MCU- and desktop-friendly design
- Supports oversampling

This repository focuses on **implementation quality and runtime performance**,
not on proposing a new control algorithm.

---

## Attribution

The Harmonic Control Array (HCA) controller implemented in this library is based on the method
presented in the following publication:

> Murat Doğruel and Hasan Hüseyin Çelik,  
> "Harmonic Control Array Method With a Real Time Application to Periodic Position Control,"  
> *IEEE Transactions on Control Systems Technology*, vol. 19, Issue: 3, May 2011.  
> DOI: 10.1109/TCST.2010.2048110

This repository does **not** propose a new control algorithm.
It provides an optimized and portable C implementation of the method described in the cited work,
with a focus on real-time and embedded execution.

---

## Architecture overview

Each harmonic channel consists of:

1. **DDS-based phase generation** (32-bit phase accumulator)
2. **Disperser block**
   - Modulation into rotating frame
   - Sliding window integration using a circular buffer
3. **Complex PI controller**
4. **Assembler**
   - Inverse modulation
   - Harmonic reconstruction and summation

Multiple harmonic channels are processed in parallel and summed into a single
control output.

![HCA Diagram](./hca.svg)

---

## Why is it fast?

- Precomputed sine LUT (power-of-two size)
- 32-bit DDS phase accumulator
- Branchless inner loop
- Cache-friendly circular buffers
- Multiplication instead of division where possible
- Fixed memory footprint

Execution time is fully deterministic, making the library suitable for
hard real-time control loops.

---

## Usage model

The library exposes a **pure signal-processing core**.

- No assumptions about HAL, RTOS, timers, or interrupts
- The user is responsible for calling `HCA_Process()` at a fixed sampling rate
- Suitable for ISR-driven or scheduler-driven systems

---

## Minimal usage example

```c
HCA_Handle_t hca;

HCA_Init(&hca,
         fundamental_freq,
         switching_freq,
         oversample_ratio,
         output_limit);

Complex_t kp = {0.5f, 0.0f};
Complex_t ki = {0.0f, 0.0f};

HCA_Add_Channel(&hca, 1, kp, ki);  // Fundamental
HCA_Add_Channel(&hca, 3, kp, ki);  // 3rd harmonic

float u = HCA_Process(&hca, input_signal);
