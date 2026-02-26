/**
 * @file pmode.cpp
 * @version 1.5.0
 * @date 2026-02-25
 * @author Leonardo Lisa
 * @brief SCPI Multichannel Analyzer - SCPI Polling Mode (Multi-Threaded)
 * @details Fetches hardware triggers via rpi_fast_irq, waits 300ms for DSP
 * processing, executes SCPI queries over TCP, and offloads disk I/O to a 
 * background consumer thread. Features realtime UI tracking of valid acquisitions.
 * @requirements rpi_fast_irq kernel module, RpiFastIrq library, isolated CPU 3.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * * You should have received a copy of the GNU General Public License
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
#include <limits>
#include <sys/time.h>
#include "RpiFastIrq.hpp"

// --- ANSI COLORS ---
#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_CYAN    "\x1b[36m"
#define ANSI_RESET   "\x1b[0m"

// --- CONFIGURATION ---
const int PORT = 5025;

struct AcqData {
    uint64_t timestamp_ns;
    std::vector<std::string> meas_results;
    std::vector<char> wave_data;
};

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

    bool push(T&& item) {
        size_t current_head = m_head.load(std::memory_order_relaxed);
        if (current_head - m_tail.load(std::memory_order_acquire) >= Size) return false;
        m_data[current_head % Size] = std::move(item);
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

    bool is_empty() const {
        return m_head.load(std::memory_order_acquire) == m_tail.load(std::memory_order_acquire);
    }
};

std::atomic<bool> g_keep_running{true};
LockFreeRingBuffer<GpioIrqEvent, 1024> g_event_buffer;
LockFreeRingBuffer<AcqData, 1024> g_write_buffer; 

void signal_handler(int signum) {
    (void)signum;
    g_keep_running = false;
}

void print_header() {
    std::cout << ANSI_CYAN << R"(
  ____                  _             ____                       
 / ___| _ __   ___  ___| |_ _ __ __ _/ ___|  ___ ___  _ __   ___ 
 \___ \| '_ \ / _ \/ __| __| '__/ _` \___ \ / __/ _ \| '_ \ / _ \
  ___) | |_) |  __/ (__| |_| | | (_| |___) | (_| (_) | |_) |  __/
 |____/| .__/ \___|\___|\__|_|  \__,_|____/ \___\___/| .__/ \___|
       |_|                                           |_|         
    )" << ANSI_RESET << "\n";
    std::cout << "v1.5.0 - SCPI Multichannel Analyzer\n";
    std::cout << "--------------------------------------------------------\n";
}

void print_available_commands() {
    std::cout << ANSI_CYAN << "\n[Available SCPI Parameters]\n" << ANSI_RESET;
    std::cout << "Voltage Measurements:\n";
    std::cout << "  PKPK   : Peak-to-Peak Voltage\n";
    std::cout << "  MAX    : Maximum Voltage\n";
    std::cout << "  MIN    : Minimum Voltage\n";
    std::cout << "  AMPL   : Amplitude (Top - Base)\n";
    std::cout << "  TOP    : Top Voltage\n";
    std::cout << "  BASE   : Base Voltage\n";
    std::cout << "  MEAN   : Mean Voltage\n";
    std::cout << "  CMEAN  : Cycle Mean Voltage\n";
    std::cout << "  RMS    : Root Mean Square\n";
    std::cout << "  CRMS   : Cycle Root Mean Square\n\n";
    std::cout << "Time Measurements:\n";
    std::cout << "  FREQ   : Frequency\n";
    std::cout << "  PER    : Period\n";
    std::cout << "  RISE   : Rise Time\n";
    std::cout << "  FALL   : Fall Time\n";
    std::cout << "  PWID   : Positive Pulse Width\n";
    std::cout << "  NWID   : Negative Pulse Width\n";
    std::cout << "  DUTY   : Positive Duty Cycle\n";
    std::cout << "  NDUTY  : Negative Duty Cycle\n\n";
    std::cout << "Special Flags:\n";
    std::cout << "  WAVE   : Download full binary waveform data\n";
}

bool is_valid_scpi_cmd(const std::string& cmd) {
    const std::vector<std::string> valid_cmds = {
        "PKPK", "MAX", "MIN", "AMPL", "TOP", "BASE", "MEAN", "CMEAN", 
        "RMS", "CRMS", "FREQ", "PER", "RISE", "FALL", "PWID", "NWID", 
        "DUTY", "NDUTY"
    };
    return std::find(valid_cmds.begin(), valid_cmds.end(), cmd) != valid_cmds.end();
}

void send_cmd(int sock, const std::string &cmd) {
    std::string packet = cmd + "\n";
    send(sock, packet.c_str(), packet.length(), 0);
}

std::string read_resp_string(int sock) {
    char buffer[4096] = {0};
    int valread = read(sock, buffer, 4095);
    if (valread > 0) {
        while (valread > 0 && (buffer[valread-1] == '\n' || buffer[valread-1] == '\r')) {
            buffer[valread-1] = '\0';
            valread--;
        }
        return std::string(buffer);
    }
    return "";
}

std::vector<char> read_waveform_binary(int sock) {
    send_cmd(sock, "C1:WF? DAT2");
    char c;
    
    while (read(sock, &c, 1) > 0 && c != '#') {
        if (!g_keep_running) return {};
    }
    if (c != '#') return {};
    
    if (read(sock, &c, 1) <= 0) return {};
    int len_digit_count = c - '0';
    if (len_digit_count < 1 || len_digit_count > 9) return {};
    
    std::string len_str(len_digit_count, '\0');
    if (read(sock, &len_str[0], len_digit_count) != len_digit_count) return {};
    size_t payload_size = std::stoull(len_str);
    
    std::vector<char> payload(payload_size);
    size_t total_read = 0;
    while (total_read < payload_size && g_keep_running) {
        ssize_t r = read(sock, payload.data() + total_read, payload_size - total_read);
        if (r <= 0) break;
        total_read += r;
    }
    
    char dump;
    while (recv(sock, &dump, 1, MSG_DONTWAIT) > 0); 
    
    return payload;
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
        
        if (token.find("NAN") != std::string::npos || token.find("INF") != std::string::npos) return "?";
    }
    return "?";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    print_header();

    std::vector<std::string> valid_cmds;
    std::string filename = "";
    bool fetch_wave = false;

    // Parse CLI arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc) {
            filename = argv[++i];
        } else {
            std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);
            if (arg == "WAVE") {
                fetch_wave = true;
            } else {
                if (is_valid_scpi_cmd(arg)) {
                    valid_cmds.push_back(arg);
                } else {
                    std::cerr << ANSI_RED << "[Error] Invalid SCPI parameter: " << arg << ANSI_RESET << "\n";
                    print_available_commands();
                    return 1;
                }
            }
        }
    }

    if (valid_cmds.empty() && !fetch_wave) {
        std::cerr << ANSI_RED << "[Error] No valid parameters provided. Add at least one measurement or WAVE.\n" << ANSI_RESET;
        print_available_commands();
        return 1;
    }

    if (valid_cmds.size() > 4) {
        std::cerr << ANSI_RED << "[Error] Maximum of 4 SCPI measurement parameters exceeded. Provided: " << valid_cmds.size() << ".\n" << ANSI_RESET;
        return 1;
    }

    // TCP Socket Setup & Dynamic IP Configuration
    int sock = -1;
    std::string scope_ip;
    
    while (g_keep_running) {
        std::cout << ANSI_CYAN << "[Setup]" << ANSI_RESET << " Enter Oscilloscope IP Address: ";
        std::cin >> scope_ip;

        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << ANSI_RED << "[Error] Socket creation failed.\n" << ANSI_RESET;
            return 1;
        }
        
        int flag = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

        sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        
        if (inet_pton(AF_INET, scope_ip.c_str(), &serv_addr.sin_addr) <= 0) {
            std::cerr << ANSI_RED << "[Error] Invalid IP address format. Please try again.\n" << ANSI_RESET;
            close(sock);
            continue;
        }

        std::cout << ANSI_CYAN << "[System]" << ANSI_RESET << " Testing connection to " << scope_ip << "...\n";
        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << ANSI_RED << "[Error] Connection failed. Please check the IP and try again.\n" << ANSI_RESET;
            close(sock);
            continue;
        }

        std::cout << ANSI_GREEN << "[Success]" << ANSI_RESET << " Connected to " << scope_ip << ".\n";
        break; 
    }

    if (!g_keep_running) {
        if (sock >= 0) close(sock);
        return 0;
    }

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    send_cmd(sock, "CHDR OFF");        
    send_cmd(sock, "C1:TRA ON");       
    send_cmd(sock, "TRMD SINGLE");     

    RpiFastIrq irq_handler("/dev/rp1_gpio_irq");
    auto my_irq_callback = [](const GpioIrqEvent& event) {
        g_event_buffer.push(event);
    };

    if (!irq_handler.start(my_irq_callback)) {
        std::cerr << ANSI_RED << "[Error] Could not start IRQ listener.\n" << ANSI_RESET;
        close(sock);
        return 1;
    }

    std::cout << ANSI_CYAN << "[System]" << ANSI_RESET << " Ready. Press ENTER to start acquisition and create log files.\n";
    std::cin.get();
    
    if (!g_keep_running) {
        irq_handler.stop();
        close(sock);
        return 0;
    }

    // Flush the event buffer to discard any spurious interrupts generated before arming
    GpioIrqEvent dump_event;
    while (g_event_buffer.pop(dump_event));

    // Auto-generate filename if missing, strict file creation after ENTER
    if (filename.empty()) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "acquisition_" << std::put_time(std::localtime(&in_time_t), "%H-%M-%S_%d-%m-%Y") << ".dat";
        filename = ss.str();
        std::cout << ANSI_CYAN << "[INFO]" << ANSI_RESET << " Filename not specified. Auto-generated: " << filename << "\n";
    }

    std::string base_filename = filename;
    size_t dot_pos = base_filename.find_last_of('.');
    if (dot_pos != std::string::npos) base_filename = base_filename.substr(0, dot_pos);

    std::string wave_folder = base_filename + "_waves";
    if (fetch_wave) {
        mkdir(wave_folder.c_str(), 0777);
    }

    std::thread writer_thread([&]() {
        std::ofstream file;
        if (!valid_cmds.empty()) {
            file.open(filename);
            file << "#";
            for(const auto& cmd : valid_cmds) file << " " << cmd;
            file << " timestamp\n";
        }

        uint64_t w_count = 0;
        AcqData data;
        
        while (g_keep_running || !g_write_buffer.is_empty()) {
            if (g_write_buffer.pop(data)) {
                w_count++;
                
                if (file.is_open()) {
                    for (const auto& res : data.meas_results) file << res << " ";
                    file << data.timestamp_ns << "\n";
                }

                if (fetch_wave && !data.wave_data.empty()) {
                    std::string wave_file = wave_folder + "/" + base_filename + std::to_string(w_count) + ".wave";
                    std::ofstream wf(wave_file, std::ios::binary);
                    wf << "#" << data.timestamp_ns << "\n";
                    wf.write(data.wave_data.data(), data.wave_data.size());
                }
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    });

    std::cout << ANSI_YELLOW << "[Running]" << ANSI_RESET << " Press Ctrl+C to stop.\n";
    send_cmd(sock, "ARM");

    uint64_t total_events = 0;
    std::vector<uint64_t> success_counts(valid_cmds.size(), 0);
    uint64_t wave_success_count = 0;
    GpioIrqEvent event;

    // Producer Thread: Wait 300ms, then poll DSP status
    while (g_keep_running) {
        if (g_event_buffer.pop(event)) {
            AcqData acq;
            acq.timestamp_ns = event.timestamp_ns;
            total_events++;

            // Wait empirically determined 300ms for DSP post-processing 
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            if (!valid_cmds.empty() && g_keep_running) {
                for (size_t i = 0; i < valid_cmds.size(); ++i) {
                    const auto& cmd = valid_cmds[i];
                    std::string val = "?";
                    int retries = 0;
                    const int MAX_RETRIES = 10; // 50ms total polling after 300ms delay

                    while (retries < MAX_RETRIES && g_keep_running) {
                        send_cmd(sock, "C1:PAVA? " + cmd);
                        std::string resp = read_resp_string(sock);
                        
                        if (resp.find("****") == std::string::npos && !resp.empty()) {
                            val = extract_value(resp);
                            if (val != "?") {
                                success_counts[i]++;
                            }
                            break; 
                        }
                        
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        retries++;
                    }
                    acq.meas_results.push_back(val);
                }
            }

            if (fetch_wave && g_keep_running) {
                acq.wave_data = read_waveform_binary(sock);
                if (!acq.wave_data.empty()) {
                    wave_success_count++;
                }
            }

            if (!g_write_buffer.push(std::move(acq))) {
                std::cerr << ANSI_YELLOW << "\n[Warn]" << ANSI_RESET << " Consumer thread overload. Data dropped.\n";
            }
            
            // Realtime terminal status line
            std::cout << "\r" << ANSI_GREEN << "[Acquisition]" << ANSI_RESET << " Events: " << total_events;
            for (size_t i = 0; i < valid_cmds.size(); ++i) {
                std::cout << " | " << valid_cmds[i] << ": " << success_counts[i];
            }
            if (fetch_wave) {
                std::cout << " | WAVE: " << wave_success_count;
            }
            std::cout << "    " << std::flush;

            if (g_keep_running) {
                send_cmd(sock, "ARM");
            }
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    std::cout << ANSI_CYAN << "\n[System]" << ANSI_RESET << " Shutting down. Waiting for background I/O to flush...\n";
    
    irq_handler.stop();
    send_cmd(sock, "CHDR SHORT");
    close(sock);
    
    if (writer_thread.joinable()) {
        writer_thread.join();
    }
    
    return 0;
}