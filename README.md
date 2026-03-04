# SCPI Multichannel Analyzer for Siglent SDS800X HD

## Project Overview
This project transforms a Raspberry Pi 5 (BCM2712/RP1 architecture) and a Siglent SDS800X HD oscilloscope into a high-speed, hardware-triggered Multichannel Data Acquisition (DAQ) system. By coupling a custom zero-copy Linux Kernel Module (`rpi_fast_irq`) with multithreaded SCPI network communication over TCP, the system captures physical trigger events from the oscilloscope's "Trigger Out" BNC port with nanosecond precision.

Designed for high-performance physics and electronics experiments, the suite features real-time logging, massive burst sequence extraction (up to 500,000 wfm/s), and live GUI histograms powered by the CERN ROOT Framework.

---

## Core Architecture

* **Hardware-Synchronous Triggering**: Eliminates user-space polling jitter. The physical Trigger Out signal generates a kernel-level interrupt on an isolated CPU core, stamping the event time at Ring 0 via `ktime_get_ns()`.
* **Zero-Copy Memory Mapping**: The kernel module shares a lock-free ring buffer directly with the C++ user-space applications via `mmap` (`vmalloc_user`), guaranteeing deterministic microsecond latency.
* **Multi-Threaded I/O Offloading**: Critical acquisition loops are decoupled from SCPI network parsing and disk I/O or X11 GUI rendering, preventing network timeouts from dropping hardware triggers.
* **IEEE 488.2 Binary Block Parsing**: Waveform data is extracted in its native binary format rather than ASCII, maximizing throughput and preserving lossless 16-bit word precision.

---

## Operating Modes

The suite is divided into command-line data loggers and ROOT-integrated graphical analyzers.

### 1. Polling Mode (Continuous Real-Time)
* **Binaries**: `pmode.x` (CLI Logger), `pmode_root.x` (Live GUI)
* **Workflow**: The oscilloscope operates in Single-Shot mode (`TRMD SINGLE`). Upon receiving a hardware trigger, the software enforces a 300ms DSP settling delay, fetches the calculated measurements via SCPI, and immediately re-arms the oscilloscope.
* **Use Case**: Continuous, real-time logging where immediate data availability is required and event rates are low (< 3 Hz).

### 2. Sequence Mode (High-Frequency Burst)
* **Binaries**: `smode.x` (CLI Logger), `smode_root.x` (Batch GUI)
* **Workflow**: Leverages the Siglent's hardware Sequence mode (`ACQ:SEQ ON`). The RPi5 strictly counts hardware interrupts. Once a target capacity or a timeout (`-t`) is reached, the acquisition is safely stopped, and the data is bulk-downloaded via History Mode (for scalars) or direct Sequence RAM extraction (for raw vectors).
* **Use Case**: High-frequency transients or particle detection where dead-time must be zero. Captures bursts at the oscilloscope's maximum trigger rate.

---

## System Setup & Installation

### 1. Hardware Connections
1. Connect the Oscilloscope to the Raspberry Pi 5 via Gigabit Ethernet.
2. Connect the Oscilloscope's **Trigger Out / Pass-Fail BNC** to a logic level shifter (3.3V safe).
3. Connect the shifted logic signal to **GPIO 17** on the RPi5.

### 2. Kernel Isolation & Permissions
To ensure hard real-time performance, isolate CPU 3 on the RPi5:
1. Append `isolcpus=3` to `/boot/firmware/cmdline.txt`.
2. Reboot the system.

ROOT applications (`pmode_root.x`, `smode_root.x`) **must not be run with sudo** to prevent X11/Wayland display server crashes. Grant user access to the IRQ device:
```bash
sudo chmod 666 /dev/rp1_gpio_irq
```
*(Tip: Add a udev rule to persist this permission across reboots).*

### 3. Compilation
Build the kernel module first:
```bash
cd Kernel_module
make
sudo insmod rpi_fast_irq.ko
```

Compile the user-space applications (requires `g++` and CERN ROOT):
```bash
cd Polling_mode && make
cd ../Sequence_mode && make
cd ../Polling_mode_histo && make
cd ../Sequence_mode_histo && make
```

---

## Usage Examples

All modes feature an interactive fallback. If you launch an executable without parameters, it will prompt you for the IP address and bounds dynamically.

### Polling Mode Logging (`pmode.x`)
Log Peak-to-Peak and Maximum voltages, and dump the raw waveform for each event:
```bash
./pmode.x PKPK MAX WAVE -ip 192.168.1.100 -o experiment_1.dat
```

### Polling Mode ROOT GUI (`pmode_root.x`)
Plot a real-time updating histogram of Minimum voltage with dynamic binning:
```bash
./pmode_root.x MIN -ip 192.168.1.100
```
Force strict axis boundaries and fixed bins:
```bash
./pmode_root.x PKPK -ip 192.168.1.100 -min 0.0 -max 5.0 -bin 1000
```

### Sequence Mode Logging (`smode.x`)
Capture an extreme burst of scalar measurements using a 60-second timeout:
```bash
./smode.x AMPL -ip 192.168.1.100 -t 60 -o burst_ampl.dat
```
Dump the raw vector memory blocks for offline processing:
```bash
./smode.x WAVE -ip 192.168.1.100 -t 30 -o raw_capture.wave
```

### Sequence Mode ROOT GUI (`smode_root.x`)
Accumulate Sequence triggers and update the ROOT histogram in batches every 10 seconds:
```bash
./smode_root.x TOP -ip 192.168.1.100 -t 10 -min -1.0 -max 3.3
```

---

## Post-Processing Utilities

* **`analyze_scpi.C`**: A ROOT macro to perform post-acquisition statistical analysis and generate 1-sigma distributions from the generated `.dat` files.
  ```bash
  root -l 'analyze_scpi.C("experiment_1.dat")'
  ```
* **`wave_parser.py`**: A Python script (included in the `Sequence_mode` directory) to decode the massive binary `.wave` dumps. It parses the hardware preamble (VDIV, OFFSET, CODE) and maps the 16-bit raw ADC integers back into absolute voltage readings and timestamps.

---

## License
This project is licensed under the **GNU Affero General Public License v3 (AGPLv3)**. 
See the `LICENSE` file for full details.