/**
 * @file smode.cpp
 * @version 1.7.0
 * @date 2026-03-03
 * @author Leonardo Lisa
 * @brief SCPI Sequence Mode Analyzer for Siglent Oscilloscopes.
 * @details Configures sequence mode via ACQ:SEQ, captures waveforms up to 90% capacity 
 * or timeout, extracts data via HISTORy mode, and outputs relative timestamps.
 * Download completes even if interrupted via Ctrl+C. A second Ctrl+C aborts download.
 * @requirements rpi_fast_irq kernel module, RpiFastIrq library.
 *
 * @usage sudo ./smode.x [MEASUREMENT] [-ip <address>] [-t <timeout_s>]
 * * Parameters:
 * MEASUREMENT  : Target vertical measurement. Valid options: PKPK, MAX, MIN, AMPL, TOP, BASE.
 * -ip <addr>   : (Optional) IPv4 address of the Siglent oscilloscope.
 * -t <timeout> : (Optional) Timeout in seconds for each sequence block (Default: 60s).
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
 * along with this program.  If not, see [https://www.gnu.org/licenses/](https://www.gnu.org/licenses/).
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
std::atomic<bool> g_abort_download{false};
LockFreeRingBuffer<GpioIrqEvent, 8192> g_event_buffer;

void signal_handler(int signum) {
    (void)signum;
    if (!g_keep_running.load(std::memory_order_acquire)) {
        g_abort_download.store(true, std::memory_order_release);
    }
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
        while (valread > 0 && (buffer[valread-1] == '\n' || buffer[valread-1] == '\r')) {
            buffer[valread-1] = '\0';
            valread--;
        }
        return std::string(buffer);
    }
    return "";
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
    const std::vector<std::string> valid_cmds = {"PKPK", "MAX", "MIN", "AMPL", "TOP", "BASE"};
    return std::find(valid_cmds.begin(), valid_cmds.end(), cmd) != valid_cmds.end();
}

void print_usage() {
    std::cout << ANSI_CYAN << "Usage: " << ANSI_RESET << "./smode.x [MEASUREMENT] [-ip <address>] [-t <timeout_s>]\n\n"
              << "Parameters:\n"
              << "  MEASUREMENT  : Target vertical measurement (PKPK, MAX, MIN, AMPL, TOP, BASE)\n"
              << "  -ip <addr>   : IPv4 address of the Siglent oscilloscope\n"
              << "  -t <timeout> : Timeout in seconds for sequence acquisition (Default: 60s)\n"
              << "  -h, --help   : Display this help message\n\n"
              << "Examples:\n"
              << "  ./smode.x MIN -ip 192.168.1.100 -t 120\n"
              << "  ./smode.x PKPK\n";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    std::cout << ANSI_CYAN << "========================================================\n";
    std::cout << " SCPI Sequence Mode Analyzer (Hard Real-Time Capture)\n";
    std::cout << "========================================================\n" << ANSI_RESET;

    std::string meas_type;
    std::string scope_ip;
    int timeout_s = 60;

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

    if (meas_type.empty()) {
        std::cout << ANSI_CYAN << "Available Measurements:\n  PKPK, MAX, MIN, AMPL, TOP, BASE\n" << ANSI_RESET;
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
        struct timeval tv = {2, 0};
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

    RpiFastIrq irq_handler("/dev/rp1_gpio_irq");
    if (!irq_handler.start([](const GpioIrqEvent& event) { g_event_buffer.push(event); })) {
        std::cerr << ANSI_RED << "[Error] IRQ listener failed.\n" << ANSI_RESET;
        close(sock);
        return 1;
    }

    send_cmd(sock, "CHDR OFF");
    send_cmd(sock, "C1:TRA ON");

    auto now_t = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now_t);
    std::stringstream ss;
    ss << meas_type << "_SMODE_" << std::put_time(std::localtime(&in_time_t), "%d-%m-%Y_%H-%M-%S") << ".dat";
    std::string filename = ss.str();
    
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << ANSI_RED << "[Error] Failed to open output file.\n" << ANSI_RESET;
        close(sock);
        return 1;
    }
    outfile << "# Time_s " << meas_type << "\n";

    uint64_t global_t0 = 0;
    bool has_t0 = false;
    uint32_t total_saved_events = 0;

    std::cout << ANSI_YELLOW << "[Running]" << ANSI_RESET << " Sequence Mode initialized. Press Ctrl+C to stop.\n";

    while (g_keep_running.load(std::memory_order_acquire)) {
        send_cmd(sock, "ACQ:SEQ ON"); 
        send_cmd(sock, "ACQ:SEQ:COUN 80000"); 
        
        send_cmd(sock, "ACQ:SEQ:COUN?");
        std::string seq_resp = read_resp_string(sock);
        
        int max_waveforms = 0;
        try { max_waveforms = std::stoi(extract_value(seq_resp)); } 
        catch (...) { max_waveforms = 1000; }

        int target_waveforms = max_waveforms * 0.9;
        
        std::cout << ANSI_CYAN << "[Sequence]" << ANSI_RESET 
                  << " Arming. Target: " << target_waveforms << " waveforms. Timeout: " << timeout_s << "s\n";

        send_cmd(sock, "TRMD SINGLE");
        send_cmd(sock, "ARM");

        GpioIrqEvent dummy;
        while(g_event_buffer.pop(dummy));

        std::vector<uint64_t> timestamps;
        timestamps.reserve(target_waveforms);

        auto start_wait = std::chrono::steady_clock::now();
        
        while (g_keep_running.load(std::memory_order_acquire)) {
            GpioIrqEvent ev;
            if (g_event_buffer.pop(ev)) {
                timestamps.push_back(ev.timestamp_ns);
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }

            auto current_time = std::chrono::steady_clock::now();
            int elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_wait).count();

            if (timestamps.size() >= (size_t)target_waveforms || elapsed >= timeout_s) {
                break;
            }
        }

        send_cmd(sock, "STOP");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Enable History Mode
        send_cmd(sock, "HISTORy ON");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << ANSI_GREEN << "[Download]" << ANSI_RESET 
                  << " Hardware Interrupts: " << timestamps.size() 
                  << ". Entering History mode and scanning frames...\n";

        int valid_frames = 0;

        for (int i = 1; i <= (int)timestamps.size() && !g_abort_download.load(std::memory_order_acquire); i++) {
            
            // Richiede all'oscilloscopio di puntare al frame i-esimo
            send_cmd(sock, "HISTOR:FRAM " + std::to_string(i));
            
            // Interroga l'oscilloscopio per confermare su quale frame si trova realmente
            send_cmd(sock, "HISTOR:FRAM?");
            std::string check_resp = read_resp_string(sock);
            int actual_frame = 0;
            try { actual_frame = std::stoi(extract_value(check_resp)); } catch (...) {}

            // Se l'hardware ha forzato un frame inferiore a quello richiesto,
            // significa che abbiamo raggiunto e superato il limite di memoria.
            if (actual_frame < i) {
                break;
            }

            // Procedi con la misurazione sul frame confermato
            send_cmd(sock, "C1:PAVA? " + meas_type);
            
            std::string val_str = extract_value(read_resp_string(sock));
            if (val_str != "?") {
                try {
                    double val = std::abs(std::stod(val_str));
                    
                    if (!has_t0) {
                        global_t0 = timestamps[0];
                        has_t0 = true;
                    }

                    double t_rel_s = (timestamps[i-1] - global_t0) / 1e9;
                    outfile << std::fixed << std::setprecision(6) << t_rel_s << " " << val << "\n";
                    total_saved_events++;
                    valid_frames++;
                } catch (...) {}
            }
            
            // Aggiornamento progressivo della UI (ogni 10 frame per ridurre I/O a terminale)
            if (valid_frames % 10 == 0 || valid_frames == 1) {
                std::cout << "\r  Downloading frame " << valid_frames << "    " << std::flush;
            }
        }
        
        if (valid_frames > 0) {
            std::cout << "\r  Completed download of " << valid_frames << " frames.        \n";
            outfile.flush();
        } else {
            std::cout << "  No valid frames found for download.\n";
        }
        
        // Disable History mode
        send_cmd(sock, "HISTORy OFF");
        send_cmd(sock, "SEQ OFF");
    }

    std::cout << ANSI_CYAN << "\n[System]" << ANSI_RESET << " Shutting down. Total events saved: " << total_saved_events << "\n";
    
    irq_handler.stop();
    if (outfile.is_open()) outfile.close();
    
    if (sock >= 0) {
        send_cmd(sock, "CHDR SHORT");
        close(sock);
    }

    return 0;
}
