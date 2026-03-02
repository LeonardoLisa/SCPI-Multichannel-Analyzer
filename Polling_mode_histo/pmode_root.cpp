/**
 * @file pmode_root.cpp
 * @version 2.3.0
 * @date 2026-03-03
 * @author Leonardo Lisa
 * @brief SCPI Real-Time Histogram Analyzer (Multithreaded Architecture) for Siglent SDS800X HD.
 * @details Fetches hardware triggers via rpi_fast_irq. Offloads the 300ms DSP wait and SCPI 
 * TCP querying to a background thread to prevent X11 display server freezing. Utilizes 
 * ROOT::EnableThreadSafety() and std::mutex for atomic histogram GUI updates.
 * Canvas rendering is throttled to a maximum of 1 Hz to reduce X11 overhead.
 * Supports manual bounds (-min, -max) and optional fixed binning (-bin).
 * @requirements rpi_fast_irq kernel module, RpiFastIrq library, CERN ROOT Framework.
 *
 * @usage sudo ./pmode_root.x [MEASUREMENT] [-ip <address>] [-min <value>] [-max <value>] [-bin <value>]
 * * Parameters:
 * MEASUREMENT  : Target vertical measurement. Valid options: PKPK, MAX, MIN, AMPL, TOP, BASE.
 * -ip <addr>   : (Optional) IPv4 address of the Siglent oscilloscope.
 * -min <val>   : (Optional) Minimum boundary for the histogram X-axis (e.g., 0.0). Must be >= 0.
 * -max <val>   : (Optional) Maximum boundary for the histogram X-axis (e.g., 5.0). Must be > min.
 * -bin <val>   : (Optional) Fixed number of bins for the histogram [5 to 4096]. 
 * If omitted, dynamic bin resizing is applied automatically.
 * * Interactive Mode:
 * If any required parameter (MEASUREMENT, -min, -max, or -ip) is omitted from the CLI, 
 * the program will prompt the user to input them interactively via standard input.
 * * Examples:
 * sudo ./pmode_root.x MIN -ip 192.168.1.100 -min 0.0 -max 3.3 -bin 500
 * sudo ./pmode_root.x PKPK -min 0.0 -max 5.0
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
#include <mutex>
#include <cmath>

#include <TROOT.h>
#include <TApplication.h>
#include <TCanvas.h>
#include <TH1D.h>
#include <TSystem.h>
#include <TStyle.h>

#include "RpiFastIrq.hpp"

#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_CYAN    "\x1b[36m"
#define ANSI_RESET   "\x1b[0m"

const int PORT = 5025;
const int MAX_BINS = 4096; 

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

    bool is_empty() const {
        return m_head.load(std::memory_order_acquire) == m_tail.load(std::memory_order_acquire);
    }
};

std::atomic<bool> g_keep_running{true};
LockFreeRingBuffer<GpioIrqEvent, 1024> g_event_buffer;

void signal_handler(int signum) {
    (void)signum;
    g_keep_running.store(false, std::memory_order_release);
}

void send_cmd(int sock, const std::string &cmd) {
    if (!g_keep_running || sock < 0) return;
    std::string packet = cmd + "\n";
    send(sock, packet.c_str(), packet.length(), MSG_NOSIGNAL);
}

std::string read_resp_string(int sock) {
    if (!g_keep_running || sock < 0) return "";
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
    std::cout << ANSI_CYAN << "Usage: " << ANSI_RESET << "./pmode_root.x [MEASUREMENT] [-ip <address>] [-min <value>] [-max <value>] [-bin <value>]\n\n"
              << "Parameters:\n"
              << "  MEASUREMENT  : Target vertical measurement (PKPK, MAX, MIN, AMPL, TOP, BASE)\n"
              << "  -ip <addr>   : IPv4 address of the Siglent oscilloscope\n"
              << "  -min <val>   : Minimum boundary for the histogram X-axis (Must be >= 0)\n"
              << "  -max <val>   : Maximum boundary for the histogram X-axis (Must be > min)\n"
              << "  -bin <val>   : Fixed number of bins for the histogram [5 to 4096]\n"
              << "  -h, --help   : Display this help message\n\n"
              << "Examples:\n"
              << "  ./pmode_root.x MIN -ip 192.168.1.100 -min 0.0 -max 3.3 -bin 500\n"
              << "  ./pmode_root.x PKPK -min 0.0 -max 5.0\n";
}

int main(int argc, char* argv[]) {
    // 1. Thread safety and early GUI initialization
    ROOT::EnableThreadSafety();
    TApplication app("SCPI_HIST_GUI", &argc, argv);
    std::signal(SIGINT, signal_handler);

    std::cout << ANSI_CYAN << "========================================================\n";
    std::cout << " SCPI Real-Time Histogram Analyzer (Vertical Measurements)\n";
    std::cout << "========================================================\n" << ANSI_RESET;

    std::string meas_type;
    std::string scope_ip;
    
    bool has_min = false, has_max = false;
    double min_x = 0.0, max_x = 0.0;
    int current_bins = 10;
    bool fixed_bins = false;

    // Parse CLI arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage();
            return 0;
        } else if (arg == "-ip" && i + 1 < argc) {
            scope_ip = argv[++i];
        } else if (arg == "-min" && i + 1 < argc) {
            try { min_x = std::stod(argv[++i]); has_min = true; } 
            catch (...) { std::cerr << ANSI_RED << "[Error] Invalid -min value.\n" << ANSI_RESET; return 1; }
        } else if (arg == "-max" && i + 1 < argc) {
            try { max_x = std::stod(argv[++i]); has_max = true; } 
            catch (...) { std::cerr << ANSI_RED << "[Error] Invalid -max value.\n" << ANSI_RESET; return 1; }
        } else if (arg == "-bin" && i + 1 < argc) {
            try { 
                current_bins = std::stoi(argv[++i]); 
                fixed_bins = true;
                if (current_bins < 5 || current_bins > 4096) {
                    std::cerr << ANSI_RED << "[Error] -bin parameter must be between 5 and 4096.\n" << ANSI_RESET;
                    return 1;
                }
            } 
            catch (...) { std::cerr << ANSI_RED << "[Error] Invalid -bin value.\n" << ANSI_RESET; return 1; }
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
        while (g_keep_running) {
            gSystem->ProcessEvents();
            if (poll(&pfd_m, 1, 100) > 0) {
                std::cin >> meas_type;
                std::transform(meas_type.begin(), meas_type.end(), meas_type.begin(), ::toupper);
                break;
            }
        }
        if (!g_keep_running) return 0;
        
        if (!is_valid_measurement(meas_type)) {
            std::cerr << ANSI_RED << "[Error] Invalid measurement type.\n" << ANSI_RESET;
            return 1;
        }
    }

    if (!has_min || !has_max) {
        std::cout << ANSI_CYAN << "[Setup]" << ANSI_RESET << " Enter Histogram Min and Max values (e.g., 0.0 5.0): " << std::flush;
        struct pollfd pfd_bounds;
        pfd_bounds.fd = STDIN_FILENO; pfd_bounds.events = POLLIN;
        while (g_keep_running) {
            gSystem->ProcessEvents();
            if (poll(&pfd_bounds, 1, 100) > 0) {
                std::string line;
                std::cin >> min_x >> max_x;
                if (std::cin.fail()) {
                    std::cerr << ANSI_RED << "\n[Error] Invalid numerical input.\n" << ANSI_RESET;
                    return 1;
                }
                has_min = true;
                has_max = true;
                break;
            }
        }
        if (!g_keep_running) return 0;
    }

    if (min_x < 0 || min_x >= max_x) {
        std::cerr << ANSI_RED << "[Error] Min value must be >= 0 and strictly less than Max value.\n" << ANSI_RESET;
        return 1;
    }

    int sock = -1;
    while (g_keep_running) {
        if (scope_ip.empty()) {
            std::cout << ANSI_CYAN << "[Setup]" << ANSI_RESET << " Enter Oscilloscope IP Address: " << std::flush;
            struct pollfd pfd_ip;
            pfd_ip.fd = STDIN_FILENO; pfd_ip.events = POLLIN;
            while (g_keep_running) {
                gSystem->ProcessEvents(); 
                if (poll(&pfd_ip, 1, 100) > 0) {
                    std::cin >> scope_ip;
                    break;
                }
            }
            if (!g_keep_running) break;
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

    if (!g_keep_running) {
        if (sock >= 0) close(sock);
        return 0;
    }

    gStyle->SetOptStat(1111);
    auto canvas = new TCanvas("c_hist", Form("%s Real-Time Monitor", meas_type.c_str()), 900, 600);
    canvas->SetGrid();

    auto hist = new TH1D("h_meas", meas_type.c_str(), current_bins, min_x, max_x);
    hist->SetLineColor(kBlue + 1);
    hist->SetFillColor(kBlue - 9);
    hist->SetLineWidth(2);
    hist->Draw();
    canvas->Update();

    RpiFastIrq irq_handler("/dev/rp1_gpio_irq");
    if (!irq_handler.start([](const GpioIrqEvent& event) { g_event_buffer.push(event); })) {
        std::cerr << ANSI_RED << "[Error] IRQ listener failed.\n" << ANSI_RESET;
        close(sock);
        return 1;
    }

    send_cmd(sock, "CHDR OFF");        
    send_cmd(sock, "C1:TRA ON");       
    send_cmd(sock, "TRMD SINGLE");
    send_cmd(sock, "ARM");

    std::cout << ANSI_YELLOW << "[Running]" << ANSI_RESET << " Waiting for triggers... Press Ctrl+C to stop and save.\n";

    std::vector<std::pair<uint64_t, double>> acquired_data;
    std::vector<double> g_gui_buffer;
    std::mutex hist_mutex; 

    // 2. Background Thread for SCPI I/O
    std::thread scpi_thread([&]() {
        GpioIrqEvent event;
        while (g_keep_running.load(std::memory_order_acquire)) {
            if (g_event_buffer.pop(event)) {
                auto wait_start = std::chrono::steady_clock::now();
                while (std::chrono::steady_clock::now() - wait_start < std::chrono::milliseconds(300)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    if (!g_keep_running.load(std::memory_order_acquire)) break;
                }
                if (!g_keep_running.load(std::memory_order_acquire)) break;

                send_cmd(sock, "C1:PAVA? " + meas_type);
                std::string resp = read_resp_string(sock);
                
                if (resp.find("****") == std::string::npos && !resp.empty()) {
                    std::string val_str = extract_value(resp);
                    if (val_str != "?") {
                        try {
                            double raw_val = std::abs(std::stod(val_str));
                            
                            {
                                std::lock_guard<std::mutex> lock(hist_mutex); 
                                acquired_data.push_back({event.timestamp_ns, raw_val});
                                g_gui_buffer.push_back(raw_val);
                            }
                        } catch (...) {}
                    }
                }
                if (g_keep_running.load(std::memory_order_acquire)) send_cmd(sock, "ARM");
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    });

    // 3. Main GUI loop (Non-Blocking execution at 20 FPS, rendering limited to 1 Hz)
    auto last_update_time = std::chrono::steady_clock::now();
    bool pending_gui_update = false;

    while (g_keep_running.load(std::memory_order_acquire)) {
        gSystem->ProcessEvents(); 
        
        {
            std::lock_guard<std::mutex> lock(hist_mutex); 
            if (!g_gui_buffer.empty()) {
                for (double val : g_gui_buffer) {
                    
                    if (!fixed_bins) {
                        int min_bin_count = -1;
                        for (int i = 1; i <= hist->GetNbinsX(); i++) {
                            int c = hist->GetBinContent(i);
                            if (c > 0) {
                                if (min_bin_count == -1 || c < min_bin_count) min_bin_count = c;
                            }
                        }

                        if (min_bin_count > 10 && current_bins < MAX_BINS) {
                            current_bins = std::min(current_bins * 2, MAX_BINS);
                            hist->Reset();
                            hist->SetBins(current_bins, min_x, max_x);
                            for (const auto& d : acquired_data) hist->Fill(d.second);
                            continue;
                        }
                    }
                    
                    hist->Fill(val);
                }
                
                g_gui_buffer.clear();
                pending_gui_update = true;

                std::cout << "\r" << ANSI_GREEN << "[Acquisition]" << ANSI_RESET 
                          << " Total Events: " << acquired_data.size() 
                          << " | Current Bins: " << current_bins << "    " << std::flush;
            }
        }

        // 1 Hz Rendering Throttle
        auto now = std::chrono::steady_clock::now();
        if (pending_gui_update && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time).count() >= 1000) {
            canvas->Modified();
            canvas->Update();
            last_update_time = now;
            pending_gui_update = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << ANSI_CYAN << "\n[System]" << ANSI_RESET << " Shutting down and exporting data...\n";
    
    if (scpi_thread.joinable()) {
        scpi_thread.join();
    }
    
    irq_handler.stop();
    if (sock >= 0) {
        send_cmd(sock, "CHDR SHORT");
        close(sock);
    }

    if (!acquired_data.empty()) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << meas_type << "_" << std::put_time(std::localtime(&in_time_t), "%d-%m-%Y_%H-%M-%S");
        std::string base_filename = ss.str();

        canvas->SaveAs((base_filename + ".png").c_str());

        std::ofstream outfile(base_filename + ".dat");
        if (outfile.is_open()) {
            outfile << "# timestamp " << meas_type << "\n";
            for (const auto& d : acquired_data) {
                outfile << d.first << " " << d.second << "\n";
            }
            outfile.close();
        }
    }

    return 0;
}
