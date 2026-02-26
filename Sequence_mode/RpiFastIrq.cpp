/**
 * @file RpiFastIrq.cpp
 * @version 2.0.0
 * @date 2026-02-25
 * @author Leonardo Lisa
 * @brief Implementation of the zero-copy, lock-free GPIO IRQ user-space library.
 * @requirements C++17, Linux OS with root privileges for SCHED_FIFO.
 * * This program is free software: you can redistribute it and/or modify
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

#include "RpiFastIrq.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <system_error>
#include <cstring>
#include <sys/mman.h>
#include <sched.h>

RpiFastIrq::RpiFastIrq(const std::string& device_path)
    : m_device_path(device_path), m_fd(-1), m_shared_buf(nullptr), m_mmap_size(0), m_running(false) {
}

RpiFastIrq::~RpiFastIrq() {
    stop(); 
}

bool RpiFastIrq::start(IrqCallback user_callback) {
    if (m_running) {
        std::cerr << "[RpiFastIrq] Already running.\n";
        return false;
    }

    // O_RDWR required for PROT_WRITE mmap mapping
    m_fd = ::open(m_device_path.c_str(), O_RDWR);
    if (m_fd < 0) {
        std::cerr << "\033[31m[RpiFastIrq] Failed to open device: " << m_device_path 
                  << " Error: " << std::strerror(errno) << "\033[0m\n";
        return false;
    }

    long page_size = ::sysconf(_SC_PAGESIZE);
    m_mmap_size = (sizeof(SharedRingBuffer) + page_size - 1) & ~(page_size - 1);

    m_shared_buf = static_cast<SharedRingBuffer*>(::mmap(NULL, m_mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0));
    if (m_shared_buf == MAP_FAILED) {
        std::cerr << "\033[31m[RpiFastIrq] mmap failed: " << std::strerror(errno) << "\033[0m\n";
        ::close(m_fd);
        m_fd = -1;
        return false;
    }

    m_callback = std::move(user_callback);
    m_running = true;
    m_listener_thread = std::thread(&RpiFastIrq::listener_thread_func, this);

    return true;
}

void RpiFastIrq::stop() {
    if (!m_running) return;

    m_running = false;

    if (m_listener_thread.joinable()) {
        m_listener_thread.join();
    }

    if (m_shared_buf != nullptr && m_shared_buf != MAP_FAILED) {
        ::munmap(m_shared_buf, m_mmap_size);
        m_shared_buf = nullptr;
    }

    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

void RpiFastIrq::listener_thread_func() {
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::cerr << "\033[33m[RpiFastIrq] Warning: Failed to set SCHED_FIFO priority. Requires root privileges.\033[0m\n";
    }

    struct pollfd pfd;
    pfd.fd = m_fd;
    pfd.events = POLLIN; 

    const int timeout_ms = 100; 

    // Synchronize local tail to prevent processing historical buffer data on startup
    uint32_t local_tail = __atomic_load_n(&m_shared_buf->head, __ATOMIC_ACQUIRE);
    __atomic_store_n(&m_shared_buf->tail, local_tail, __ATOMIC_RELEASE);

    while (m_running) {
        int ret = ::poll(&pfd, 1, timeout_ms);

        if (ret < 0) {
            if (errno != EINTR) {
                std::cerr << "\033[31m[RpiFastIrq] poll() error: " << std::strerror(errno) << "\033[0m\n";
                break;
            }
        } else if (ret > 0) {
            if (pfd.revents & POLLIN) {
                // Lock-free acquire barrier
                uint32_t current_head = __atomic_load_n(&m_shared_buf->head, __ATOMIC_ACQUIRE);
                
                while (local_tail != current_head) {
                    GpioIrqEvent event_data = m_shared_buf->events[local_tail % KBUF_SIZE];
                    
                    if (m_callback) {
                        m_callback(event_data);
                    }
                    
                    local_tail++;
                }

                // Lock-free release barrier updates tail for kernel space
                __atomic_store_n(&m_shared_buf->tail, local_tail, __ATOMIC_RELEASE);
            }
        }
    }
}