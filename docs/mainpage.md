/**
@mainpage rapidHCA-c Documentation

# rapidHCA-c

**High-performance C implementation of the Harmonic Control Array (HCA) controller**

rapidHCA-c is a standalone, portable C library that implements the Harmonic Control Array (HCA) control method. The HCA approach is a frequency-domain technique that decomposes a periodic tracking error into its harmonic components, regulates each component independently using a complex PI controller operating in a rotating reference frame, and reconstructs the composite control output in the time domain.

## Key Features

The library is designed for demanding real-time control applications:
* **High Performance:** Deterministic, $\mathcal{O}(1)$ execution time per control step.
* **Static Allocation:** No dynamic memory allocation; fixed, compile-time memory footprint.
* **Math Optimized:** LUT-based trigonometry via Direct Digital Synthesis (DDS)—no `sinf`/`cosf` calls at run-time.
* **Scalable:** Supports up to `MAX_HARMONICS` parallel channels.
* **Portable:** Validated on STM32F4 series; fully portable to any C99 platform.

## Theoretical Background

The HCA method controls periodic signals by operating on each harmonic component in its own rotating reference frame. A real-valued error signal is decomposed into $N$ harmonic channels. Inside each channel, the signal is modulated, integrated, and then demodulated back to the time domain. The outputs of all channels are summed to form the control signal.

## Attribution and Licensing

**Note:** This library implements an existing control algorithm. It does not propose a new method. All theoretical credit belongs to the original authors. The library focuses exclusively on implementation quality and runtime efficiency.

* **Based on:** M. Doğruel and H. H. Çelik, "Harmonic Control Arrays Method With a Real Time Application to Periodic Position Control," IEEE Trans. Control Syst. Technol., vol. 19, no. 3, pp. 681-689, May 2011. DOI: 10.1109/TCST.2010.2048110
* **License:** Released under the MIT License.
*/
