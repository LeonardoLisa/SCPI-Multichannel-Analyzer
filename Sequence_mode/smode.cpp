/**
 * @file smode.cpp
 * @version 1.0.0
 * @date 2026-02-26
 * @author Leonardo Lisa
 * @brief SCPI Multichannel Analyzer - Sequence Mode for Siglent SDS800X HD
 * @requirements rpi_fast_irq kernel module, RpiFastIrq library, isolated CPU 3.
 * * @details Arms the oscilloscope in Sequence Mode for N frames. The RPi5 uses 
 * the custom fast IRQ kernel module to count N physical hardware triggers with 
 * nanosecond precision. Once N is reached, the waveform history is dumped via SCPI.
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
#include <csignal>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/tcp.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <iomanip>
#include <sys/stat.h>
#include <limits>
#include "RpiFastIrq.hpp"

// TCP Port for SCPI on Siglent Oscilloscopes
const int PORT = 5025;

std::atomic<bool> g_keep_running{true};
std::atomic<uint32_t> g_trigger_count{0};
std::vector<uint64_t> g_timestamps_ns;

void signal_handler(int signum) {
    (void)signum;
    g_keep_running = false;
}

void send_cmd(int sock, const std::string &cmd) {
    std::string packet = cmd + "\n";
    send(sock, packet.c_str(), packet.length(), 0);
}

// Read binary IEEE 488.2 block containing waveform data
std::vector<char> read_waveform_binary(int sock) {
    send_cmd(sock, "C1:WF? DAT2");
    char c;
    
    // Wait for IEEE 488.2 block header '#'
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
    
    // Discard trailing newline
    char dump;
    while (recv(sock, &dump, 1, MSG_DONTWAIT) > 0); 
    
    return payload;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    if (argc < 2) {
        std::cerr << "[Error] Usage: sudo ./scpi_sequence <num_segments> [-o filename.dat]\n";
        return 1;
    }

    uint32_t num_segments = std::stoul(argv[1]);
    std::string filename = "sequence_capture.dat";

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc) {
            filename = argv[++i];
        }
    }

    int sock = -1;
    std::string scope_ip;
    
    while (g_keep_running) {
        std::cout << "Enter Oscilloscope IP Address: ";
        std::cin >> scope_ip;

        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "[Error] Socket creation failed.\n";
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
            std::cerr << "[Error] Invalid IP address format.\n";
            close(sock);
            continue;
        }

        std::cout << "[System] Testing connection to " << scope_ip << "...\n";
        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "[Error] Connection failed.\n";
            close(sock);
            continue;
        }

        std::cout << "[Success] Connected to " << scope_ip << ".\n";
        break; 
    }

    if (!g_keep_running) {
        if (sock >= 0) close(sock);
        return 0;
    }

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Pre-allocate hardware timestamp vector
    g_timestamps_ns.reserve(num_segments);

    // Initialize the hardware IRQ listener
    RpiFastIrq irq_handler("/dev/rp1_gpio_irq");
    auto my_irq_callback = [&](const GpioIrqEvent& event) {
        if (g_trigger_count < num_segments) {
            g_timestamps_ns.push_back(event.timestamp_ns);
            g_trigger_count.fetch_add(1, std::memory_order_relaxed);
        }
    };

    if (!irq_handler.start(my_irq_callback)) {
        std::cerr << "[Error] Could not start IRQ listener.\n";
        close(sock);
        return 1;
    }

    // Configure Oscilloscope for Sequence Mode
    send_cmd(sock, "CHDR OFF");
    send_cmd(sock, "TRMD SINGLE");
    send_cmd(sock, "SEQ ON");
    send_cmd(sock, "SEQ:COUN " + std::to_string(num_segments));

    std::cout << "[System] Ready. Press ENTER to arm the oscilloscope for " << num_segments << " events.\n";
    std::cin.get();
    
    if (!g_keep_running) {
        irq_handler.stop();
        close(sock);
        return 0;
    }

    std::cout << "[Running] Waiting for hardware triggers... Press Ctrl+C to abort.\n";
    send_cmd(sock, "ARM");

    // Poll until hardware interrupt counter matches the requested segments
    while (g_keep_running && g_trigger_count.load(std::memory_order_relaxed) < num_segments) {
        std::cout << "\r[Acquisition] Captured: " << g_trigger_count.load(std::memory_order_relaxed) 
                  << " / " << num_segments << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "\n[System] Acquisition complete. Halting IRQ listener.\n";
    irq_handler.stop();

    // Proceed to SCPI History Download if running wasn't aborted
    if (g_keep_running) {
        std::string wave_folder = "sequence_waves";
        mkdir(wave_folder.c_str(), 0777);

        std::cout << "[System] Downloading waveform history via SCPI...\n";

        for (uint32_t i = 1; i <= num_segments && g_keep_running; ++i) {
            // Set history frame to retrieve
            send_cmd(sock, "FNUM " + std::to_string(i));
            
            std::vector<char> wave_data = read_waveform_binary(sock);
            
            if (!wave_data.empty()) {
                std::string wave_file = wave_folder + "/frame_" + std::to_string(i) + ".wave";
                std::ofstream wf(wave_file, std::ios::binary);
                
                // Write the corresponding hardware timestamp as ASCII header
                uint64_t ts = (i - 1 < g_timestamps_ns.size()) ? g_timestamps_ns[i - 1] : 0;
                wf << "#" << ts << "\n";
                wf.write(wave_data.data(), wave_data.size());
            }

            if (i % 10 == 0 || i == num_segments) {
                std::cout << "\r[Download] Frames retrieved: " << i << " / " << num_segments << std::flush;
            }
        }
        std::cout << "\n[System] Download complete.\n";
    }

    // Cleanup
    send_cmd(sock, "SEQ OFF");
    send_cmd(sock, "CHDR SHORT");
    close(sock);
    
    return 0;
}