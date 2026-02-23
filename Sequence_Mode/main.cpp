/**
 * @file main.cpp
 * @version 1.1.0
 * @date 2026-02-23
 * @author Leonardo Lisa
 * @brief SCPI Multichannel Analyzer - SCPI Polling Mode (Multi-Threaded)
 * @details Fetches hardware triggers via rpi_fast_irq, executes SCPI queries
 * over TCP, and offloads disk I/O to a background consumer thread.
 * @requirements rpi_fast_irq kernel module, RpiFastIrq library, isolated CPU 3.
 * * This program is free software: you can redistribute it and/or modify
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
#include "RpiFastIrq.hpp"

// --- CONFIGURATION ---
const char *SCOPE_IP = "192.168.178.20";
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
LockFreeRingBuffer<AcqData, 1024> g_write_buffer; // Producer-Consumer buffer

void signal_handler(int signum) {
    (void)signum;
    g_keep_running = false;
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

// Parses strict IEEE 488.2 Binary Block (e.g. #900001000<data>)
std::vector<char> read_waveform_binary(int sock) {
    send_cmd(sock, "C1:WF? DAT2");
    char c;
    
    // Wait for IEEE block start header
    while (read(sock, &c, 1) > 0 && c != '#');
    if (c != '#') return {};
    
    // Length of the size descriptor
    if (read(sock, &c, 1) <= 0) return {};
    int len_digit_count = c - '0';
    if (len_digit_count < 1 || len_digit_count > 9) return {};
    
    // Payload size
    std::string len_str(len_digit_count, '\0');
    if (read(sock, &len_str[0], len_digit_count) != len_digit_count) return {};
    size_t payload_size = std::stoull(len_str);
    
    // Fetch exact payload bytes
    std::vector<char> payload(payload_size);
    size_t total_read = 0;
    while (total_read < payload_size) {
        ssize_t r = read(sock, payload.data() + total_read, payload_size - total_read);
        if (r <= 0) break;
        total_read += r;
    }
    
    // Flush trailing terminators (\n) from socket buffer
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
        if (token.find("NAN") != std::string::npos || token.find("INF") != std::string::npos) return token;
    }
    return "NaN";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

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
                valid_cmds.push_back(arg);
            }
        }
    }

    if (valid_cmds.empty() && !fetch_wave) {
        std::cerr << "[Error] Missing valid parameters. Add at least one measurement (e.g. pk-pk, min) or wave.\n";
        return 1;
    }

    // Auto-generate filename if missing
    if (filename.empty()) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "acquisition_" << std::put_time(std::localtime(&in_time_t), "%H-%M-%S_%d-%m-%Y") << ".dat";
        filename = ss.str();
        std::cout << "[INFO] Filename not specified. Auto-generated: " << filename << "\n";
    }

    std::string base_filename = filename;
    size_t dot_pos = base_filename.find_last_of('.');
    if (dot_pos != std::string::npos) base_filename = base_filename.substr(0, dot_pos);

    std::string wave_folder = base_filename + "_waves";
    if (fetch_wave) {
        mkdir(wave_folder.c_str(), 0777);
    }

    // Consumer Thread: Handles non-blocking Disk I/O
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

    // TCP Socket Setup
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return 1;
    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, SCOPE_IP, &serv_addr.sin_addr);

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "[Error] Oscilloscope connection failed.\n";
        g_keep_running = false;
        writer_thread.join();
        return 1;
    }

    send_cmd(sock, "CHDR OFF");
    send_cmd(sock, "TRMD SINGLE");

    // Hardware IRQ Setup
    RpiFastIrq irq_handler("/dev/rp1_gpio_irq");
    auto my_irq_callback = [](const GpioIrqEvent& event) {
        g_event_buffer.push(std::move(event));
    };

    if (!irq_handler.start(my_irq_callback)) {
        std::cerr << "[Error] Could not start IRQ listener.\n";
        g_keep_running = false;
        writer_thread.join();
        return 1;
    }

    std::string batch_query = "";
    if (!valid_cmds.empty()) {
        for (size_t i = 0; i < valid_cmds.size(); ++i) {
            batch_query += "C1:PAVA? " + valid_cmds[i];
            if (i < valid_cmds.size() - 1) batch_query += ";";
        }
    }

    std::cout << "[System] Ready. Press ENTER to start acquisition.\n";
    std::cin.get();
    std::cout << "[Running] Press Ctrl+C to stop.\n";
    
    send_cmd(sock, "ARM");

    uint64_t count = 0;
    GpioIrqEvent event;

    // Producer Thread (Main Loop): Synchronous Acquisition
    while (g_keep_running) {
        if (g_event_buffer.pop(event)) {
            AcqData acq;
            acq.timestamp_ns = event.timestamp_ns;

            if (!valid_cmds.empty()) {
                send_cmd(sock, batch_query);
                std::string resp = read_resp_string(sock);
                if (!resp.empty()) {
                    std::istringstream respStream(resp);
                    std::string single_resp;
                    while (std::getline(respStream, single_resp, ';')) {
                        acq.meas_results.push_back(extract_value(single_resp));
                    }
                } else {
                    std::cerr << "\n[Warn] SCPI Measurement read failed.\n";
                }
            }

            if (fetch_wave) {
                acq.wave_data = read_waveform_binary(sock);
                if (acq.wave_data.empty()) {
                    std::cerr << "\n[Warn] SCPI Waveform read failed or empty.\n";
                }
            }

            // Offload data serialization to background thread
            if (!g_write_buffer.push(std::move(acq))) {
                std::cerr << "\n[Warn] Consumer thread overload. Data dropped.\n";
            }
            
            count++;
            std::cout << "\r[Running] Acquisitions: " << count << std::flush;

            // Re-arm oscilloscope strictly after data is retrieved from socket
            send_cmd(sock, "ARM");
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    std::cout << "\n[System] Shutting down. Waiting for background I/O to flush...\n";
    
    irq_handler.stop();
    send_cmd(sock, "CHDR SHORT");
    close(sock);
    
    writer_thread.join();
    
    return 0;
}