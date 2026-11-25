# Runtime-Configurable Parallel GPS C/A Code Acquisition Engine (FPGA)

A Verilog implementation of GPS L1 C/A code acquisition on a Xilinx Artix-7 (Basys 3). Up to **32 parallel correlators** search the 1023 possible code phases. The number of active correlators (N = 1…32) is set **at runtime from the board switches**, so a single bitstream covers every point on the search-time vs. resource/power trade-off.

Developed during a research internship at IIT Madras (May–July 2026).

## Overview
| | |
|---|---|
| **Board / device** | Digilent Basys 3, Artix-7 `xc7a35tcpg236-1`, 100 MHz system clock |
| **Tools** | Vivado (synthesis, implementation, timing, power), MATLAB |
| **Language** | Verilog-2001 |
| **Signal** | GPS L1 C/A Gold code (1023 chips), G1/G2 LFSRs |

## Problem statement
A GPS receiver must find the code phase of each satellite's PRN before it can track it. A single serial correlator has to test all 1023 code phases one after another, which is slow. A fully parallel search is fast but costly in logic and power. This design makes the degree of parallelism a **runtime parameter**, so that trade-off can be measured on one piece of hardware.

## Objectives
1. Generate the C/A code in hardware (G1/G2 LFSR, selectable G2 phase taps per satellite).
2. Split the 1023-chip search space across N parallel correlator slices, with N selectable at runtime.
3. Detect lock with a correlation threshold and display the recovered code offset.
4. Close timing and characterise utilisation and power.

## Architecture
```
            sw[4:0] ──► active_count = sw+1 (1..32) ──► range_size = 1023 / N
                                                        │
 clk 100 MHz ─► clk_divider ─► chip_clk                 ▼
                    │            ┌─────────────── correlator_slice[0..31] ───────────────┐
 prn_generator ─────┼──► local   │ slice i searches offsets [i·range_size, +range_size)  │
 (G1/G2 LFSR)       │    chip    │ XNOR + 1023-chip accumulate, lock if count ≥ 950      │
                    ▼            └───────────────────────────┬───────────────────────────┘
 delay_line (800 chips) ─► incoming chip (test signal)       ▼
                                          priority select of locked slices ─► offset
                                                                         ├─► seven_seg_driver
 btnC ─► debounce ─► reset                                               └─► led_locked
```

| Module | Function |
|---|---|
| `prn_generator.v` | 10-bit G1 (x³+x¹⁰) and G2 (x²+x³+x⁶+x⁸+x⁹+x¹⁰) LFSRs. The chip is G1[10] ⊕ (G2 tap A ⊕ G2 tap B), with taps parameterised per PRN (2/6 = PRN 1). |
| `delay_line.v` / `variable_delay_line.v` | Delays the replica by a known number of chips (800) to create a test "incoming" signal |
| `correlator.v` / `correlator_slice.v` | Counts chip agreements (XNOR) over a 1023-chip period and steps through its assigned offset range. Lock threshold: 950/1023. |
| `correlator_top.v` | Generates 32 slices, enables the first N, partitions the search range, and priority-selects the locked offset |
| `clk_divider.v` | Divides 100 MHz down to a slow chip clock so lock is visible on the board |
| `debounce.v` | Switch and button debouncing (a real hardware bug found during bring-up) |
| `seven_seg_driver.v` | Shows the recovered code offset on the 4-digit display |
| `matlab/` | GPS pseudorange / SISRE calculator (WGS-84 receiver position, L1/L2) and Doppler-shift analysis |

## Hardware I/O (`constraints/basys3.xdc`)
| Port | Pin | Function |
|---|---|---|
| `clk` | W5 | 100 MHz |
| `sw[4:0]` | V17 V16 W16 W17 W15 | Active correlators − 1 |
| `btnC` | U18 | Reset (debounced) |
| `seg[6:0]`, `an[3:0]` | W7…U7 / U2 U4 V4 W4 | Recovered code offset |
| `led_locked` | U16 | Lock indicator |

## Results (full 32-slice build, from `reports/`)
| Metric | Value |
|---|---|
| Slice LUTs | 3,753 / 20,800 (**18.0 %**) |
| Slice registers | 2,049 / 41,600 (4.9 %) |
| WNS @ 100 MHz | **+5.852 ns**, timing met |
| Total on-chip power | 0.138 W (0.069 W dynamic) |

All 32 slices are always present in hardware, and N only enables or disables them. The figures above are therefore the cost of the full runtime-configurable engine. In bring-up the design was characterised at N = 1, 4, 16 and 32. The on-board debug work fixed an index off-by-one in the slice range partitioning, the settling of the runtime range divider, and switch/button bounce.

## Setup and usage
1. In Vivado, create an RTL project for `xc7a35tcpg236-1` and add `rtl/*.v` and `constraints/basys3.xdc`. Set `correlator_top` as top.
2. Generate the bitstream and program the Basys 3.
3. Set `sw[4:0]` for the number of correlators and press `btnC`. When a slice reaches the threshold, `led_locked` lights and the display shows the code offset (expected: the 800-chip test delay).
4. MATLAB: run `matlab/matlab_code.m` (full calculator) or `matlab_short.m`.

## Limitations
- The "incoming" signal is a delayed copy of the local replica (no noise, carrier or Doppler). This validates the correlator architecture, not RF acquisition.
- Only the code-phase search is implemented. There is no frequency (Doppler) bin search.
- The chip clock is slowed down deliberately for visual demonstration.

## Future improvements
- Live L1 samples from a MAX2769 RF front end (scoped), with carrier wipe-off and Doppler bins.
- Add noise and Doppler to the test signal and measure detection probability against C/N₀.
- FFT-based parallel code-phase search as a comparison architecture.
- A self-checking testbench that sweeps all 32 values of N and all PRNs.

## Technologies
`Verilog` `Vivado` `Artix-7` `Basys 3` `GPS / GNSS` `LFSR` `Gold codes` `Parallel correlators` `Timing closure` `MATLAB`
