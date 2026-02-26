# SCPI Multichannel Analyzer for Siglent SDS800X HD

## Project Overview
This project transforms a Raspberry Pi 5 and a Siglent SDS800X HD oscilloscope into a high-speed, hardware-triggered Multichannel Analyzer. By combining a custom Linux Kernel Module (`rpi_fast_irq`) with SCPI network communication, the system captures physical trigger events from the oscilloscope's "Trigger Out" BNC port with nanosecond precision.

The project features two distinct operational modes to balance real-time processing needs with high-frequency acquisition capabilities.

---

## Operating Modes

### 1. Polling Mode (Real-Time Single Shot)
This mode is designed for continuous, real-time logging where immediate data availability is required.
* **Workflow:** The oscilloscope is placed in Single-Shot mode (`TRMD SINGLE`). When a hardware trigger is detected by the RPi5, the software waits for a necessary 300ms DSP post-processing window before executing SCPI queries to fetch the data. Once downloaded, the application automatically re-arms the oscilloscope for the next event.
* **Advantages:** Data is processed and saved in real-time. Measurement failures or "NaN" responses are handled dynamically. 
* **Limitations:** The oscilloscope experiences a "blind period" (dead-time) during the 300ms DSP wait and the subsequent TCP/IP download. Any triggers occurring during this window are lost. Maximum acquisition rate is strictly limited (typically < 3 Hz).

### 2. Sequence Mode (High-Frequency Burst)
This mode leverages the Siglent SDS800X HD's hardware Sequence mode to capture rapid bursts of events without blind-time overhead. The oscilloscope's 12-bit ADC and specialized architecture support a maximum waveform capture rate of up to 500,000 wfm/s in Sequence mode. 
* **Workflow:** The oscilloscope is armed in Sequence mode for a predefined number of segments ($N$). The RPi5 kernel module tracks the physical triggers hitting the GPIO pin with nanosecond precision. Once the hardware interrupt counter reaches $N$, the RPi5 initiates a bulk SCPI download of the sequenced frames from the oscilloscope's internal memory.
* **Advantages:** Eliminates network and SCPI polling overhead during the active acquisition phase. Captures events at the oscilloscope's maximum hardware trigger rate up to 500,000 wfm/s. Perfect for burst signals and transient phenomena.
* **Limitations:** Data is not available in real-time; it is downloaded and post-processed only after the entire sequence of $N$ events is captured. Maximum $N$ is limited by the oscilloscope's memory depth (up to 100 Mpts).

---

## Core Architecture

* **Hardware-Synchronous Triggering:** Eliminates software polling jitter. The physical Trigger Out signal generates a kernel-level interrupt, stamping the event time at Ring 0 via `ktime_get_ns()`.
* **Multi-Threaded I/O Offloading:** A Lock-Free Ring Buffer decouples the critical acquisition loop (Producer) from disk operations (Consumer). Network transfers and SCPI parsing do not block file writing.
* **IEEE 488.2 Binary Block Parsing:** Waveform data is extracted in its native binary format rather than ASCII, maximizing throughput and minimizing CPU load.
* **Dynamic IP Configuration:** The application queries the user for the oscilloscope's IP address at runtime, allowing flexible deployment without recompilation.

---

## System Requirements
1. **Hardware:** Raspberry Pi 5 (BCM2712) and Siglent SDS800X HD (or compatible SCPI oscilloscope).
2. **OS Configuration:** CPU Core 3 must be isolated (`isolcpus=3` in `/boot/firmware/cmdline.txt`) to ensure deterministic interrupt handling.
3. **Kernel Module:** The `rpi_fast_irq.ko` Linux Kernel Module must be compiled and loaded into the kernel.
4. **Oscilloscope Configuration:**
   * Reachable via LAN (TCP Port `5025`).
   * "Trigger Out" / "Pass/Fail" BNC output configured to emit a pulse on Trigger.
   * Pulse polarity must match the kernel module's edge detection (Rising Edge by default).

---

## Compilation & Installation

Ensure the `RpiFastIrq.hpp` and `RpiFastIrq.cpp` library files are in the same directory as your target application.

Compile the Sequence Mode application:
```bash
g++ -O3 -Wall -pthread main.cpp RpiFastIrq.cpp -o scpi_sequence
```
*(Note: The `rpi_fast_irq` kernel module must be inserted via `sudo insmod rpi_fast_irq.ko` prior to execution).*

---

## Usage Instructions (Sequence Mode)

The program requires root privileges to access the `/dev/rp1_gpio_irq` character device. You must specify the number of segments to capture.

**Syntax:**
```bash
sudo ./scpi_sequence <num_segments> [-o filename.dat]
```

**Execution Examples:**
1. Capture 1000 consecutive triggers and download their waveforms:
   ```bash
   sudo ./scpi_sequence 1000 -o burst_capture.dat
   ```

**Operational Controls:**
* **Start:** Enter the target IP address, wait for the connection test, and press `ENTER` to arm the oscilloscope in Sequence mode. The oscilloscope will wait for physical triggers.
* **Stop:** The application will automatically stop and download data once the specified number of segments is captured. Press `Ctrl+C` to abort early.