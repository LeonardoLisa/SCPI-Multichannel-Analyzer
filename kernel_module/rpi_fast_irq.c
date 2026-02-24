/**
 * @file rpi_fast_irq.c
 * @brief High-Performance GPIO Interrupt Handler for Raspberry Pi 5
 * * ============================================================================
 * INSTALLATION & CONFIGURATION GUIDE
 * ============================================================================
 * * 1. ISOLATE THE CPU (Host OS Configuration):
 * Open /boot/firmware/cmdline.txt and append the following to the end 
 * of the line (do not create a new line):
 * isolcpus=3
 * Reboot the Raspberry Pi: sudo reboot
 * * 2. FIND THE CORRECT GPIO BASE FOR RP1 (Raspberry Pi 5 Specific):
 * On the RPi 5, the GPIOs are handled by the RP1 southbridge. The standard
 * GPIO 17 might not be logically mapped to "17" in the kernel.
 * Run: cat /sys/class/gpio/gpiochip* /label
 * Find the chip labeled "pinctrl-rp1". Let's say its base is 512.
 * Your logical GPIO will be: Base (512) + Pin (17) = 529.
 * Update the GPIO_PIN macro below if necessary.
 * * 3. COMPILE THE MODULE:
 * Run: make
 * * 4. INSTALL THE MODULE:
 * sudo insmod rpi_fast_irq.ko
 * * 5. VERIFY INSTALLATION:
 * dmesg | tail -n 20
 * ls -l /dev/rp1_gpio_irq
 * * 6. UNINSTALL:
 * sudo rmmod rpi_fast_irq
 * ============================================================================
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/ktime.h>
#include <linux/cpumask.h>
#include <linux/spinlock.h>

#define DEVICE_NAME "rp1_gpio_irq"
#define CLASS_NAME  "rp1_irq_class"

// --- CONFIGURATION ---
// Note: Change this to (BASE + 17) if your RP1 gpiochip starts at a different base offset.
#define GPIO_PIN 588 
#define TARGET_CPU 3

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Custom RPi5 LKM");
MODULE_DESCRIPTION("High-Performance GPIO IRQ Handler with CPU Affinity");
MODULE_VERSION("1.0");

// --- SHARED DATA STRUCTURE ---
// Must match the C++ User Space structure exactly
struct GpioIrqEvent {
    u64 timestamp_ns;
    u32 event_counter;
    u32 pin_state;
};

// --- GLOBAL VARIABLES ---
static int major_num;
static struct class* irq_class = NULL;
static struct device* irq_device = NULL;
static struct cdev irq_cdev;

static unsigned int irq_number;
static u32 total_interrupts = 0;

// Communication with User Space
// Kernel-level Ring Buffer to prevent dropped events at high frequencies (e.g., > 500Hz)
#define KBUF_SIZE 256
static struct GpioIrqEvent k_buffer[KBUF_SIZE];
static unsigned int k_head = 0;
static unsigned int k_tail = 0;
static DECLARE_WAIT_QUEUE_HEAD(wq);
static DEFINE_SPINLOCK(event_lock); // Protects k_buffer, k_head, and k_tail

// --- INTERRUPT SERVICE ROUTINE (ISR) ---
static irqreturn_t gpio_isr(int irq, void *dev_id) {
    unsigned long flags;
    u64 ts = ktime_get_ns();
    int state = gpio_get_value(GPIO_PIN);

    // Lock to update the payload safely in atomic context
    spin_lock_irqsave(&event_lock, flags);
    
    total_interrupts++;
    k_buffer[k_head].timestamp_ns = ts;
    k_buffer[k_head].event_counter = total_interrupts;
    k_buffer[k_head].pin_state = state;
    
    // Advance head pointer
    k_head = (k_head + 1) % KBUF_SIZE;
    
    // Handle buffer overflow: drop the oldest event by advancing the tail
    if (k_head == k_tail) {
        k_tail = (k_tail + 1) % KBUF_SIZE; 
    }
    
    spin_unlock_irqrestore(&event_lock, flags);

    // Wake up the user space thread sleeping on poll()
    wake_up_interruptible(&wq);

    return IRQ_HANDLED;
}

// --- FILE OPERATIONS ---

static int dev_open(struct inode *inodep, struct file *filep) {
    pr_info("[%s] Device opened by user space.\n", DEVICE_NAME);
    return 0;
}

static int dev_release(struct inode *inodep, struct file *filep) {
    pr_info("[%s] Device closed by user space.\n", DEVICE_NAME);
    return 0;
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
    int error_count = 0;
    unsigned long flags;
    struct GpioIrqEvent local_event;

    if (len < sizeof(struct GpioIrqEvent)) {
        return -EINVAL;
    }

    // Wait until the buffer is not empty. Sleep otherwise.
    wait_event_interruptible(wq, k_head != k_tail);

    // Lock to read and advance the tail pointer safely
    spin_lock_irqsave(&event_lock, flags);
    local_event = k_buffer[k_tail];
    k_tail = (k_tail + 1) % KBUF_SIZE;
    spin_unlock_irqrestore(&event_lock, flags);

    // Copy data to user space (C++ library)
    error_count = copy_to_user(buffer, &local_event, sizeof(struct GpioIrqEvent));

    if (error_count == 0) {
        return sizeof(struct GpioIrqEvent);
    } else {
        pr_err("[%s] Failed to send %d characters to the user\n", DEVICE_NAME, error_count);
        return -EFAULT;
    }
}

static __poll_t dev_poll(struct file *filep, poll_table *wait) {
    __poll_t mask = 0;
    
    poll_wait(filep, &wq, wait);
    
    // Data is ready to read if the buffer is not empty
    if (k_head != k_tail) {
        mask |= POLLIN | POLLRDNORM; 
    }
    
    return mask;
}

static struct file_operations fops = {
    .open = dev_open,
    .read = dev_read,
    .poll = dev_poll,
    .release = dev_release,
    .owner = THIS_MODULE
};

// --- MODULE INIT & EXIT ---

static int __init rpi_fast_irq_init(void) {
    int result;
    dev_t dev_num;
    struct cpumask affinity_mask;

    pr_info("[%s] Initializing kernel module...\n", DEVICE_NAME);

    // 1. Setup Character Device
    result = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    major_num = MAJOR(dev_num);
    if (result < 0) {
        pr_err("[%s] Failed to allocate major number\n", DEVICE_NAME);
        return result;
    }

    cdev_init(&irq_cdev, &fops);
    irq_cdev.owner = THIS_MODULE;
    cdev_add(&irq_cdev, dev_num, 1);

    irq_class = class_create(CLASS_NAME);
    irq_device = device_create(irq_class, NULL, dev_num, NULL, DEVICE_NAME);
    pr_info("[%s] Created device node /dev/%s\n", DEVICE_NAME, DEVICE_NAME);

    // 2. Setup GPIO
    if (!gpio_is_valid(GPIO_PIN)) {
        pr_err("[%s] Invalid GPIO pin: %d\n", DEVICE_NAME, GPIO_PIN);
        goto r_device;
    }

    gpio_request(GPIO_PIN, "sysfs");
    gpio_direction_input(GPIO_PIN);
    
    irq_number = gpio_to_irq(GPIO_PIN);
    pr_info("[%s] GPIO %d mapped to IRQ: %d\n", DEVICE_NAME, GPIO_PIN, irq_number);

    // 3. Request IRQ (Rising Edge)
    result = request_irq(irq_number,
                         (irq_handler_t) gpio_isr,
                         IRQF_TRIGGER_RISING,
                         "rpi_fast_gpio_handler",
                         NULL);

    if (result) {
        pr_err("[%s] Failed to request IRQ %d\n", DEVICE_NAME, irq_number);
        goto r_gpio;
    }

    // 4. Enforce CPU Affinity (Lock IRQ to CPU 3)
    cpumask_clear(&affinity_mask);
    cpumask_set_cpu(TARGET_CPU, &affinity_mask);
    result = irq_set_affinity_hint(irq_number, &affinity_mask);
    
    if (result) {
        pr_warn("[%s] Failed to set IRQ affinity to CPU %d. Error: %d\n", DEVICE_NAME, TARGET_CPU, result);
    } else {
        pr_info("[%s] Successfully pinned IRQ %d to CPU %d\n", DEVICE_NAME, irq_number, TARGET_CPU);
    }

    return 0;

r_gpio:
    gpio_free(GPIO_PIN);
r_device:
    device_destroy(irq_class, dev_num);
    class_destroy(irq_class);
    cdev_del(&irq_cdev);
    unregister_chrdev_region(dev_num, 1);
    return -1;
}

static void __exit rpi_fast_irq_exit(void) {
    dev_t dev_num = MKDEV(major_num, 0);

    // Clear affinity and free IRQ
    irq_set_affinity_hint(irq_number, NULL);
    free_irq(irq_number, NULL);
    
    // Free GPIO
    gpio_free(GPIO_PIN);

    // Destroy device
    device_destroy(irq_class, dev_num);
    class_destroy(irq_class);
    cdev_del(&irq_cdev);
    unregister_chrdev_region(dev_num, 1);

    pr_info("[%s] Module successfully unloaded.\n", DEVICE_NAME);
}

module_init(rpi_fast_irq_init);
module_exit(rpi_fast_irq_exit);