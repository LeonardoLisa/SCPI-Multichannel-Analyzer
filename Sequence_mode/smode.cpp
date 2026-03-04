/**
 * @file smode.cpp
 * @version 2.6.0
 * @date 2026-03-04
 * @author Leonardo Lisa
 * @brief SCPI Sequence Mode Analyzer for Siglent Oscilloscopes.
 * @details Configures sequence mode via ACQ:SEQ, captures waveforms up to 90% capacity 
 * or timeout. Splits the architecture based on user input:
 * - MEASUREMENT: Uses standard HISTORy mode logic from v1.9.0 to extract scalar values.
 * - WAVE: Bypasses history mode and directly dumps the hardware Sequence RAM in binary format.
 * Features a dynamic hard real-time CLI interface and custom output filename (-o).
 * Gracefully handles Ctrl+C to stop acquisition early but guarantees full download.
 * @requirements rpi_fast_irq kernel module, RpiFastIrq library.
 *
 * @usage sudo ./smode.x [MEASUREMENT|WAVE] [-ip <address>] [-t <timeout_s>] [-o <filename>]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <csignal>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/tcp.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <algorithm>
#include <iomanip>
#include <sys/stat.h>
#include <poll.h>
#include <cmath>
#include <cstring>

#include "RpiFastIrq.hpp"

#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_CYAN    "\x1b[36m"
#define ANSI_RESET   "\x1b[0m"

const int PORT = 5025;

template <typename T, size_t Size>
class LockFreeRingBuffer {
private:
    T m_data[Size];
    std::atomic<size_t> m_head{0};
    std::atomic<size_t> m_tail{0};

public:
    bool push(const T& item) {
        size_t current_head = m_head.load(std::memory_order_relaxed);
        if (current_head - m_tail.load(std::memory_order_acquire) >= Size) return false;
        m_data[current_head % Size] = item;
        m_head.store(current_head + 1, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t current_tail = m_tail.load(std::memory_order_relaxed);
        if (current_tail == m_head.load(std::memory_order_acquire)) return false;
        item = std::move(m_data[current_tail % Size]);
        m_tail.store(current_tail + 1, std::memory_order_release);
        return true;
    }
};

std::atomic<bool> g_keep_running{true};
LockFreeRingBuffer<GpioIrqEvent, 8192> g_event_buffer;

void signal_handler(int signum) {
    (void)signum;
    // Set global flag to false to safely break the acquisition loop and trigger data extraction
    g_keep_running.store(false, std::memory_order_release);
}

void send_cmd(int sock, const std::string &cmd) {
    if (sock < 0) return;
    std::string packet = cmd + "\n";
    send(sock, packet.c_str(), packet.length(), MSG_NOSIGNAL);
}

std::string read_resp_string(int sock) {
    if (sock < 0) return "";
    char buffer[4096] = {0};
    int valread = read(sock, buffer, 4095);
    if (valread > 0) {
        // Strip trailing newline characters
        while (valread > 0 && (buffer[valread-1] == '\n' || buffer[valread-1] == '\r')) {
            buffer[valread-1] = '\0';
            valread--;
        }
        return std::string(buffer);
    }
    return "";
}

std::vector<char> read_binary_block(int sock) {
    std::vector<char> buffer;
    char c = 0;
    
    // Wait for the IEEE 488.2 block start character ('#')
    while (true) {
        ssize_t n = recv(sock, &c, 1, 0);
        if (n <= 0) return buffer;
        if (c == '#') break;
    }

    // Number of digits representing the payload length
    if (recv(sock, &c, 1, 0) <= 0) return buffer;
    int digits = c - '0';
    if (digits < 0 || digits > 9) return buffer;

    // Read the exact payload length as a string
    std::string len_str;
    for (int i = 0; i < digits; i++) {
        if (recv(sock, &c, 1, 0) > 0) len_str += c;
    }
    
    size_t length = 0;
    try { length = std::stoul(len_str); } catch (...) { return buffer; }

    // Bulk download of the raw binary payload
    buffer.resize(length);
    size_t total_read = 0;
    while (total_read < length) {
        ssize_t n = recv(sock, buffer.data() + total_read, length - total_read, 0);
        if (n <= 0) break;
        total_read += n;
    }

    // Safely flush any trailing whitespaces or '\n' left in the socket stream 
    // to prevent desynchronization of the next SCPI command
    struct pollfd pfd;
    pfd.fd = sock;
    pfd.events = POLLIN;
    while (poll(&pfd, 1, 5) > 0) { // 5ms timeout
        char dump;
        if (recv(sock, &dump, 1, MSG_DONTWAIT) <= 0) break;
    }

    return buffer;
}

std::string extract_value(const std::string &resp) {
    std::string token;
    std::istringstream tokenStream(resp);
    while (std::getline(tokenStream, token, ',')) {
        size_t first = token.find_first_not_of(" \t\r\n");
        if (std::string::npos == first) continue;
        size_t last = token.find_last_not_of(" \t\r\n");
        token = token.substr(first, (last - first + 1));
        if (token.empty()) continue;
        char c = token[0];
        if (isdigit(c) || c == '-' || c == '+' || c == '.') return token;
    }
    return "?";
}

bool is_valid_measurement(const std::string& cmd) {
    const std::vector<std::string> valid_cmds = {"PKPK", "MAX", "MIN", "AMPL", "TOP", "BASE", "WAVE"};
    return std::find(valid_cmds.begin(), valid_cmds.end(), cmd) != valid_cmds.end();
}

void print_usage() {
    std::cout << ANSI_CYAN << "Usage: " << ANSI_RESET << "./smode.x [MEASUREMENT|WAVE] [-ip <address>] [-t <timeout_s>] [-o <filename>]\n\n"
              << "Parameters:\n"
              << "  MEASUREMENT  : Target vertical measurement (PKPK, MAX, MIN, AMPL, TOP, BASE)\n"
              << "  WAVE         : Download the raw waveform data for all active channels (.wave)\n"
              << "  -ip <addr>   : IPv4 address of the Siglent oscilloscope\n"
              << "  -t <timeout> : Timeout in seconds for sequence acquisition (Default: 60s)\n"
              << "  -o <file>    : Custom output filename (Default: MEASUREMENT_DD-MM-YYYY_HH-MM-SS.ext)\n"
              << "  -h, --help   : Display this help message\n\n"
              << "Examples:\n"
              << "  ./smode.x MIN -ip 192.168.1.100 -t 120\n"
              << "  ./smode.x WAVE -t 30 -o output_data.wave\n";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    std::cout << ANSI_CYAN << "========================================================\n";
    std::cout << " SCPI Sequence Mode Analyzer (Hard Real-Time Capture)\n";
    std::cout << "========================================================\n" << ANSI_RESET;

    std::string meas_type;
    std::string scope_ip;
    std::string custom_filename = "";
    int timeout_s = 60;

    // Parse CLI arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage();
            return 0;
        } else if (arg == "-ip" && i + 1 < argc) {
            scope_ip = argv[++i];
        } else if (arg == "-t" && i + 1 < argc) {
            try { timeout_s = std::stoi(argv[++i]); } 
            catch (...) { std::cerr << ANSI_RED << "[Error] Invalid timeout value.\n" << ANSI_RESET; return 1; }
        } else if (arg == "-o" && i + 1 < argc) {
            custom_filename = argv[++i];
        } else {
            std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);
            if (is_valid_measurement(arg)) {
                meas_type = arg;
            } else {
                std::cerr << ANSI_RED << "[Error] Unknown parameter or missing value: " << arg << "\n\n" << ANSI_RESET;
                print_usage();
                return 1;
            }
        }
    }

    // Interactive prompt for measurement type if missing
    if (meas_type.empty()) {
        std::cout << ANSI_CYAN << "Available Modes:\n  PKPK, MAX, MIN, AMPL, TOP, BASE, WAVE\n" << ANSI_RESET;
        std::cout << "Enter measurement type: " << std::flush;
        struct pollfd pfd_m;
        pfd_m.fd = STDIN_FILENO; pfd_m.events = POLLIN;
        while (g_keep_running.load(std::memory_order_acquire)) {
            if (poll(&pfd_m, 1, 100) > 0) {
                std::cin >> meas_type;
                std::transform(meas_type.begin(), meas_type.end(), meas_type.begin(), ::toupper);
                break;
            }
        }
        if (!g_keep_running.load(std::memory_order_acquire)) return 0;
        
        if (!is_valid_measurement(meas_type)) {
            std::cerr << ANSI_RED << "[Error] Invalid measurement type.\n" << ANSI_RESET;
            return 1;
        }
    }

    // Establish TCP connection to the oscilloscope
    int sock = -1;
    while (g_keep_running.load(std::memory_order_acquire)) {
        if (scope_ip.empty()) {
            std::cout << ANSI_CYAN << "[Setup]" << ANSI_RESET << " Enter Oscilloscope IP Address: " << std::flush;
            struct pollfd pfd_ip;
            pfd_ip.fd = STDIN_FILENO; pfd_ip.events = POLLIN;
            while (g_keep_running.load(std::memory_order_acquire)) {
                if (poll(&pfd_ip, 1, 100) > 0) {
                    std::cin >> scope_ip;
                    break;
                }
            }
            if (!g_keep_running.load(std::memory_order_acquire)) break;
        }

        sock = socket(AF_INET, SOCK_STREAM, 0);
        int flag = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
        struct timeval tv = {2, 0}; // 2 seconds socket timeout
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

        sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        
        if (inet_pton(AF_INET, scope_ip.c_str(), &serv_addr.sin_addr) <= 0 || 
            connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << ANSI_RED << "[Error] Connection failed to " << scope_ip << ".\n" << ANSI_RESET;
            close(sock);
            scope_ip = ""; 
            continue;
        }
        std::cout << ANSI_GREEN << "[Success]" << ANSI_RESET << " Connected to " << scope_ip << ".\n";
        break; 
    }

    if (!g_keep_running.load(std::memory_order_acquire)) {
        if (sock >= 0) close(sock);
        return 0;
    }

    // Initialize the kernel module IRQ listener
    RpiFastIrq irq_handler("/dev/rp1_gpio_irq");
    if (!irq_handler.start([](const GpioIrqEvent& event) { g_event_buffer.push(event); })) {
        std::cerr << ANSI_RED << "[Error] IRQ listener failed.\n" << ANSI_RESET;
        close(sock);
        return 1;
    }

    // Configure SCPI header format
    send_cmd(sock, "CHDR OFF");
    
    bool is_wave = (meas_type == "WAVE");
    std::vector<std::string> active_channels;

    if (is_wave) {
        // Automatically detect active analog channels
        for (int i = 1; i <= 4; i++) {
            std::string ch = "C" + std::to_string(i);
            send_cmd(sock, ch + ":TRA?");
            std::string resp = read_resp_string(sock);
            if (resp.find("ON") != std::string::npos) {
                active_channels.push_back(ch);
            }
        }
        if (active_channels.empty()) active_channels.push_back("C1");

        // Force lossless 16-bit word sampling for memory extraction
        send_cmd(sock, "WAV:WID WORD");
        send_cmd(sock, "WAV:STAR 0");
        send_cmd(sock, "WAV:POIN 0"); // Request all allocated points
    } else {
        // Ensure channel 1 is active for scalar parameter measurements
        send_cmd(sock, "C1:TRA ON");
    }

    // File naming logic based on user input or defaults
    std::string filename;
    if (!custom_filename.empty()) {
        filename = custom_filename;
    } else {
        auto now_t = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now_t);
        std::stringstream ss;
        std::string ext = is_wave ? ".wave" : ".dat";
        ss << meas_type << "_" << std::put_time(std::localtime(&in_time_t), "%d-%m-%Y_%H-%M-%S") << ext;
        filename = ss.str();
    }
    
    std::ofstream outfile(filename);

    if (!outfile.is_open()) {
        std::cerr << ANSI_RED << "[Error] Failed to open output file: " << filename << "\n" << ANSI_RESET;
        close(sock);
        return 1;
    }

    // Extract preamble metadata and write file header
    if (is_wave) {
        outfile << "# SCPI Sequence Mode Waveform Dump\n";
        for (const auto& ch : active_channels) {
            send_cmd(sock, "WAV:SOUR " + ch);
            send_cmd(sock, "WAV:SEQ 0,0"); // Initialize sequence bulk read mode
            send_cmd(sock, "WAV:PRE?");
            std::vector<char> pre = read_binary_block(sock);
            if (pre.size() >= 174) {
                float vdiv, offset, code, interval;
                double delay;
                int16_t adc_bit;
                
                // Map the memory block according to Siglent IEEE 488.2 binary structure
                std::memcpy(&vdiv, pre.data() + 156, 4);
                std::memcpy(&offset, pre.data() + 160, 4);
                std::memcpy(&code, pre.data() + 164, 4);
                std::memcpy(&adc_bit, pre.data() + 172, 2);
                std::memcpy(&interval, pre.data() + 176, 4);
                std::memcpy(&delay, pre.data() + 180, 8);
                
                outfile << "# " << ch << "_META: VDIV=" << std::scientific << vdiv 
                        << " OFFSET=" << offset << " CODE=" << code 
                        << " ADC_BIT=" << adc_bit << " INTERVAL=" << interval 
                        << " DELAY=" << delay << "\n";
            } else {
                outfile << "# " << ch << "_META: FAILED TO PARSE PREAMBLE\n";
            }
        }
        // Append description for data blocks mapping
        outfile << "# TIMESTAMP [s]\n";
    } else {
        outfile << "# Time_s " << meas_type << "\n";
    }

    uint64_t global_t0 = 0;
    bool has_t0 = false;
    uint32_t total_saved_events = 0;

    std::cout << ANSI_YELLOW << "[Running]" << ANSI_RESET << " Sequence Mode initialized. Press Ctrl+C to stop.\n";

    // Main acquisition and download loop
    while (g_keep_running.load(std::memory_order_acquire)) {
        // Configure hardware Sequence Mode 
        send_cmd(sock, "ACQ:SEQ ON"); 
        send_cmd(sock, "ACQ:SEQ:COUN 80000"); 
        
        send_cmd(sock, "ACQ:SEQ:COUN?");
        std::string seq_resp = read_resp_string(sock);
        
        int max_waveforms = 0;
        try { max_waveforms = std::stoi(extract_value(seq_resp)); } 
        catch (...) { max_waveforms = 1000; }

        int target_waveforms = max_waveforms * 0.9;
        
        send_cmd(sock, "TRMD SINGLE");
        send_cmd(sock, "ARM");

        // Flush stale IRQs from the ring buffer
        GpioIrqEvent dummy;
        while(g_event_buffer.pop(dummy));

        std::vector<uint64_t> timestamps;
        timestamps.reserve(target_waveforms);

        auto start_wait = std::chrono::steady_clock::now();
        int last_elapsed = -1;
        
        // Hard real-time polling loop for hardware triggers
        while (g_keep_running.load(std::memory_order_acquire)) {
            GpioIrqEvent ev;
            if (g_event_buffer.pop(ev)) {
                timestamps.push_back(ev.timestamp_ns);
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }

            auto current_time = std::chrono::steady_clock::now();
            int elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_wait).count();

            // Dynamic CLI UI update triggered every 1 second or at sequence completion
            if (elapsed != last_elapsed || timestamps.size() >= (size_t)target_waveforms) {
                std::cout << "\r" << ANSI_CYAN << "[Acquiring]" << ANSI_RESET 
                          << " Arming. Target: " << timestamps.size() << "/" << target_waveforms 
                          << " waveforms. Timeout: " << elapsed << "/" << timeout_s << "s    " << std::flush;
                last_elapsed = elapsed;
            }

            // Break loop if target capacity or timeout is reached
            if (timestamps.size() >= (size_t)target_waveforms || elapsed >= timeout_s) {
                break;
            }
        }
        std::cout << "\n";

        // Stop acquisition and flush DSP pipeline
        send_cmd(sock, "STOP");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int frames_to_read = 0;

        if (is_wave) {
            // In WAVE mode, read the preamble to get the exact number of acquired frames
            send_cmd(sock, "WAV:SOUR " + active_channels[0]);
            send_cmd(sock, "WAV:SEQ 0,0"); // Initialize sequence bulk read mode
            send_cmd(sock, "WAV:PRE?");
            std::vector<char> pre = read_binary_block(sock);
            
            int captured_frames = 0;
            if (pre.size() >= 174) {
                uint32_t sum_frame = 0;
                // sum_frame is stored as a 4-byte integer at offset 148 (0x94)
                std::memcpy(&sum_frame, pre.data() + 148, 4); 
                captured_frames = sum_frame;
            }
            frames_to_read = std::min((int)timestamps.size(), captured_frames);
            if (captured_frames <= 0) frames_to_read = timestamps.size(); // Fallback mechanism
            
            std::cout << ANSI_CYAN << "[Download]" << ANSI_RESET 
                      << " Hardware Interrupts: " << timestamps.size() 
                      << " | Scope Frames: " << captured_frames << ". Extracting data...\n";
        } else {
            // Enable History Mode for scalar DSP extraction using the v1.9.0 robust approach
            send_cmd(sock, "HISTORy ON");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Use the strict old methodology: iterate over timestamps and clamp via HISTOR:FRAM?
            frames_to_read = timestamps.size();
            std::cout << ANSI_CYAN << "[Download]" << ANSI_RESET 
                      << " Hardware Interrupts: " << timestamps.size() 
                      << ". Entering History mode and scanning frames...\n";
        }

        int valid_frames = 0;
        int download_timeout_count = 0;

        // Note: The loop condition does NOT check g_keep_running anymore,
        // so a Ctrl+C gracefully waits for the current download operation to finish completely.
        for (int i = 1; i <= frames_to_read; i++) {
            
            // Set global time zero based on the first valid absolute hardware interrupt
            if (!has_t0) {
                global_t0 = timestamps[0];
                has_t0 = true;
            }
            double t_rel_s = (timestamps[i-1] - global_t0) / 1e9;
            
            bool frame_success = false;

            if (is_wave) {
                // Bulk binary extraction logic for WAVE mode 
                outfile << "# " << std::fixed << std::setprecision(6) << t_rel_s << "\n";
                for (const auto& ch : active_channels) {
                    send_cmd(sock, "WAV:SOUR " + ch);
                    // Select specific sequence frame to download directly from the raw buffer
                    send_cmd(sock, "WAV:SEQ " + std::to_string(i) + ",0");
                    send_cmd(sock, "WAV:DATA?");
                    std::vector<char> data = read_binary_block(sock);
                    
                    outfile << ch << ": ";
                    if (!data.empty()) {
                        frame_success = true;
                        // Cast the 16-bit raw binary data block into signed integers
                        int16_t* ptr = reinterpret_cast<int16_t*>(data.data());
                        size_t num_pts = data.size() / 2;
                        for (size_t p = 0; p < num_pts; p++) {
                            outfile << ptr[p] << (p == num_pts - 1 ? "" : " ");
                        }
                    }
                    outfile << "\n";
                }
            } else {
                // Legacy v1.9.0 exact logic: Query actual frame to clamp bounds correctly
                send_cmd(sock, "HISTOR:FRAM " + std::to_string(i));
                send_cmd(sock, "HISTOR:FRAM?");
                std::string check_resp = read_resp_string(sock);
                int actual_frame = 0;
                try { actual_frame = std::stoi(extract_value(check_resp)); } catch (...) {}

                // If hardware clamped the pointer to a lower index, valid memory limit is reached
                if (actual_frame < i) {
                    break;
                }
                
                // Standard ASCII extraction logic for scalar parameters via DSP
                send_cmd(sock, "C1:PAVA? " + meas_type);
                std::string val_str = extract_value(read_resp_string(sock));
                if (val_str != "?") {
                    try {
                        double val = std::abs(std::stod(val_str));
                        outfile << std::fixed << std::setprecision(6) << t_rel_s << " " << val << "\n";
                        frame_success = true;
                    } catch (...) {}
                }
            }

            if (frame_success) {
                total_saved_events++;
                valid_frames++;
                download_timeout_count = 0; // Reset timeout counter on success
            } else {
                download_timeout_count++;
                if (download_timeout_count >= 3) {
                    std::cout << "\n" << ANSI_RED << "[Error]" << ANSI_RESET 
                              << " Safe abort: Repeated download timeouts or empty packets. Saving recovered data.\n";
                    break;
                }
            }
            
            // Update download progress CLI UI
            if (valid_frames % 10 == 0 || valid_frames == 1) {
                std::cout << "\r  Downloading frame " << valid_frames << "    " << std::flush;
            }
        }
        
        // Conclude UI download element based on extracted frames
        if (valid_frames > 0) {
            std::cout << "\r" << ANSI_GREEN << "[Success]" << ANSI_RESET << " Downloaded " << valid_frames << " frames.        \n";
            outfile.flush();
        } else {
            std::cout << "\r" << ANSI_RED << "[Error]" << ANSI_RESET << " No valid frames found for download.        \n";
        }
        
        // Restore hardware context for next sequence loop
        if (!is_wave) {
            send_cmd(sock, "HISTORy OFF");
        }
        send_cmd(sock, "ACQ:SEQ OFF");
    }

    std::cout << ANSI_CYAN << "\n[System]" << ANSI_RESET << " Shutting down. Total events saved: " << total_saved_events << "\n";
    
    // Cleanup IRQ subsystem and sockets
    irq_handler.stop();
    if (outfile.is_open()) outfile.close();
    
    if (sock >= 0) {
        send_cmd(sock, "CHDR SHORT");
        send_cmd(sock, ":SYSTem:REMote OFF");
        close(sock);
    }

    return 0;
}
