/**
 * @file RpiFastIrq.hpp
 * @version 2.0.0
 * @date 2026-02-25
 * @author Leonardo Lisa
 * @brief Header for the zero-copy, lock-free GPIO IRQ user-space library.
 * @requirements C++17
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

#pragma once

#include <functional>
#include <string>
#include <thread>
#include <atomic>
#include <cstdint>
#include <cstddef>

struct GpioIrqEvent {
    uint64_t timestamp_ns;
    uint32_t event_counter;
    uint32_t pin_state;
};

#define KBUF_SIZE 256

struct SharedRingBuffer {
    uint32_t head;
    uint32_t tail;
    //GpioIrqEvent events[KBUF_SIZE];
};

class RpiFastIrq {
public:
    using IrqCallback = std::function<void(const GpioIrqEvent&)>;

    explicit RpiFastIrq(const std::string& device_path = "/dev/rp1_gpio_irq");
    ~RpiFastIrq();

    RpiFastIrq(const RpiFastIrq&) = delete;
    RpiFastIrq& operator=(const RpiFastIrq&) = delete;

    bool start(IrqCallback user_callback);
    void stop();

private:
    std::string m_device_path;
    int m_fd;
    SharedRingBuffer* m_shared_buf;
    size_t m_mmap_size;
    std::atomic<bool> m_running;
    IrqCallback m_callback;
    std::thread m_listener_thread;

    void listener_thread_func();
};