#include "RpiFastIrq.hpp"
#include <iostream>
#include <fcntl.h>      // For open() flags
#include <unistd.h>     // For close(), read()
#include <poll.h>       // For poll()
#include <system_error> // For std::system_category
#include <cstring>      // For strerror

RpiFastIrq::RpiFastIrq(const std::string& device_path)
    : m_device_path(device_path), m_fd(-1), m_running(false) {
}

RpiFastIrq::~RpiFastIrq() {
    stop(); // Ensure resources are cleaned up when the object is destroyed
}

bool RpiFastIrq::start(IrqCallback user_callback) {
    if (m_running) {
        std::cerr << "[RpiFastIrq] Already running.\n";
        return false;
    }

    // Open the character device file created by the kernel module
    m_fd = ::open(m_device_path.c_str(), O_RDONLY);
    if (m_fd < 0) {
        std::cerr << "[RpiFastIrq] Failed to open device: " << m_device_path 
                  << " Error: " << std::strerror(errno) << "\n";
        return false;
    }

    m_callback = std::move(user_callback);
    m_running = true;

    // Launch the listener in a background thread
    m_listener_thread = std::thread(&RpiFastIrq::listener_thread_func, this);

    return true;
}

void RpiFastIrq::stop() {
    if (!m_running) return;

    // Signal the background thread to terminate
    m_running = false;

    // Wait for the thread to finish execution
    if (m_listener_thread.joinable()) {
        m_listener_thread.join();
    }

    // Close the file descriptor
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

void RpiFastIrq::listener_thread_func() {
    struct pollfd pfd;
    pfd.fd = m_fd;
    pfd.events = POLLIN; // We are interested in data ready to be read

    // 100 milliseconds timeout. 
    // This allows poll() to wake up periodically to check the m_running flag 
    // so the thread can exit gracefully when stop() is called.
    const int timeout_ms = 100; 

    while (m_running) {
        // Block until data is available, an error occurs, or timeout expires
        int ret = ::poll(&pfd, 1, timeout_ms);

        if (ret < 0) {
            // Error handling (ignore interrupted system calls)
            if (errno != EINTR) {
                std::cerr << "[RpiFastIrq] poll() error: " << std::strerror(errno) << "\n";
                break;
            }
        } else if (ret > 0) {
            // Data is ready to be read
            if (pfd.revents & POLLIN) {
                GpioIrqEvent event_data;
                ssize_t bytes_read = ::read(m_fd, &event_data, sizeof(event_data));

                if (bytes_read == sizeof(GpioIrqEvent)) {
                    // Execute the user's callback function
                    if (m_callback) {
                        m_callback(event_data);
                    }
                } else if (bytes_read > 0) {
                    std::cerr << "[RpiFastIrq] Partial read occurred. Expected " 
                              << sizeof(GpioIrqEvent) << " bytes, got " << bytes_read << "\n";
                } else {
                    std::cerr << "[RpiFastIrq] Read failed: " << std::strerror(errno) << "\n";
                }
            }
        }
        // If ret == 0, it means the timeout expired. The loop will just check 
        // m_running and call poll() again.
    }
}