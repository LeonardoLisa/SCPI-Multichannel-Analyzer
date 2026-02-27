/**
 * @file pmode_root.cpp
 * @version 1.0.0
 * @date 2026-02-27
 * @author Leonardo Lisa
 * @brief SCPI Real-Time Histogram Analyzer (Polling Mode) for Siglent SDS800X HD.
 * @details Fetches hardware triggers via rpi_fast_irq, acquires vertical voltage 
 * measurements via SCPI, and plots an auto-scaling real-time histogram using ROOT.
 * Extent is based on 8*V/div. Max bins bounded to 4096 (12-bit ADC native resolution).
 * @requirements rpi_fast_irq kernel module, RpiFastIrq library, CERN ROOT Framework.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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

#include <TApplication.h>
#include <TCanvas.h>
#include <TH1D.h>
#include <TSystem.h>
#include <TStyle.h>

#include "RpiFastIrq.hpp"

// --- ANSI COLORS ---
#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_CYAN    "\x1b[36m"
#define ANSI_RESET   "\x1b[0m"

const int PORT = 5025;
const int MAX_BINS = 4096; // SDS800X HD 12-bit ADC Native Resolution

std::atomic<bool> g_keep_running{true};
LockFreeRingBuffer<GpioIrqEvent, 1024> g_event_buffer;

void signal_handler(int signum) {
    (void)signum;
    g_keep_running = false;
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

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    TApplication app("SCPI_HIST_GUI", &argc, argv);

    std::cout << ANSI_CYAN << "========================================================\n";
    std::cout << " SCPI Real-Time Histogram Analyzer (Vertical Measurements)\n";
    std::cout << "========================================================\n" << ANSI_RESET;

    std::string meas_type;
    std::string scope_ip;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-ip" && i + 1 < argc) {
            scope_ip = argv[++i];
        } else {
            std::transform(arg.begin(), arg.end(), arg.begin(), ::toupper);
            if (is_valid_measurement(arg)) {
                meas_type = arg;
            }
        }
    }

    if (meas_type.empty()) {
        std::cout << ANSI_CYAN << "Available Measurements:\n  PKPK, MAX, MIN, AMPL, TOP, BASE\n" << ANSI_RESET;
        std::cout << "Enter measurement type: ";
        struct pollfd pfd_m;
        pfd_m.fd = STDIN_FILENO; pfd_m.events = POLLIN;
        while (g_keep_running) {
            if (poll(&pfd_m, 1, 100) > 0) {
                std::cin >> meas_type;
                std::transform(meas_type.begin(), meas_type.end(), meas_type.begin(), ::toupper);
                break;
            }
        }
        if (!is_valid_measurement(meas_type)) {
            std::cerr << ANSI_RED << "[Error] Invalid measurement type.\n" << ANSI_RESET;
            return 1;
        }
    }

    int sock = -1;
    while (g_keep_running) {
        if (scope_ip.empty()) {
            std::cout << ANSI_CYAN << "[Setup]" << ANSI_RESET << " Enter Oscilloscope IP Address: ";
            struct pollfd pfd_ip;
            pfd_ip.fd = STDIN_FILENO; pfd_ip.events = POLLIN;
            while (g_keep_running) {
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

    // Query V/div to determine X-axis limits
    send_cmd(sock, "C1:VDIV?");
    std::string vdiv_resp = read_resp_string(sock);
    double vdiv = 1.0; 
    try {
        vdiv = std::stod(extract_value(vdiv_resp));
    } catch (...) {
        std::cerr << ANSI_YELLOW << "[Warn] Failed to parse V/div. Defaulting to 1.0V\n" << ANSI_RESET;
    }
    double max_x = 8.0 * vdiv;

    // ROOT GUI Initialization
    gStyle->SetOptStat(111111);
    auto canvas = new TCanvas("c_hist", Form("%s Real-Time Monitor", meas_type.c_str()), 900, 600);
    canvas->SetGrid();

    int current_bins = 10;
    auto hist = new TH1D("h_meas", meas_type.c_str(), current_bins, 0, max_x);
    hist->SetLineColor(kBlue + 1);
    hist->SetFillColor(kBlue - 9);
    hist->SetLineWidth(2);

    // Initialize HW Trigger
    RpiFastIrq irq_handler("/dev/rp1_gpio_irq");
    if (!irq_handler.start([](const GpioIrqEvent& event) { g_event_buffer.push(event); })) {
        std::cerr << ANSI_RED << "[Error] IRQ listener failed.\n" << ANSI_RESET;
        close(sock);
        return 1;
    }

    // Configure Oscilloscope
    send_cmd(sock, "CHDR OFF");        
    send_cmd(sock, "C1:TRA ON");       
    send_cmd(sock, "TRMD SINGLE");
    send_cmd(sock, "ARM");

    std::cout << ANSI_YELLOW << "[Running]" << ANSI_RESET << " Waiting for triggers... Press Ctrl+C to stop and save.\n";

    std::vector<std::pair<uint64_t, double>> acquired_data;
    GpioIrqEvent event;

    // Main Polling and GUI Loop
    while (g_keep_running) {
        gSystem->ProcessEvents(); // Non-blocking ROOT GUI update

        if (g_event_buffer.pop(event)) {
            // Non-blocking wait for DSP processing
            auto wait_start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - wait_start < std::chrono::milliseconds(300)) {
                gSystem->ProcessEvents();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                if (!g_keep_running) break;
            }
            if (!g_keep_running) break;

            send_cmd(sock, "C1:PAVA? " + meas_type);
            std::string resp = read_resp_string(sock);
            
            if (resp.find("****") == std::string::npos && !resp.empty()) {
                std::string val_str = extract_value(resp);
                if (val_str != "?") {
                    try {
                        double abs_val = std::abs(std::stod(val_str));
                        acquired_data.push_back({event.timestamp_ns, abs_val});
                        
                        // Dynamic Bin Rescaling Logic
                        int min_bin_count = -1;
                        for (int i = 1; i <= hist->GetNbinsX(); i++) {
                            int c = hist->GetBinContent(i);
                            if (c > 0) {
                                if (min_bin_count == -1 || c < min_bin_count) min_bin_count = c;
                            }
                        }

                        if (min_bin_count > 10 && current_bins < MAX_BINS) {
                            current_bins = std::min(current_bins * 2, MAX_BINS);
                            delete hist;
                            hist = new TH1D("h_meas", meas_type.c_str(), current_bins, 0, max_x);
                            hist->SetLineColor(kBlue + 1);
                            hist->SetFillColor(kBlue - 9);
                            hist->SetLineWidth(2);
                            
                            // Refill history into new high-res bins
                            for (const auto& d : acquired_data) hist->Fill(d.second);
                        } else {
                            hist->Fill(abs_val);
                        }

                        canvas->Modified();
                        canvas->Update();
                        
                        std::cout << "\r" << ANSI_GREEN << "[Acquisition]" << ANSI_RESET 
                                  << " Total Events: " << acquired_data.size() 
                                  << " | Current Bins: " << current_bins << std::flush;
                    } catch (...) {}
                }
            }
            if (g_keep_running) send_cmd(sock, "ARM");
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::cout << ANSI_CYAN << "\n[System]" << ANSI_RESET << " Shutting down and exporting data...\n";
    
    irq_handler.stop();
    if (sock >= 0) {
        send_cmd(sock, "CHDR SHORT");
        close(sock);
    }

    // Export Data & Image
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

    return 0;
}