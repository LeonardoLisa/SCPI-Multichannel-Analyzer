/**
 * @file rpi_fast_irq.c
 * @version 2.0.0
 * @date 2026-02-25
 * @author Leonardo Lisa
 * @brief Zero-copy, lock-free GPIO interrupt handler for RPi5 using mmap and noncached memory.
 * @requirements RPi5 (BCM2712/RP1), isolcpus=3 in cmdline.txt.
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
 * 
 * ============================================================================
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
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/ktime.h>
#include <linux/cpumask.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>

#define DEVICE_NAME "rp1_gpio_irq"
#define CLASS_NAME  "rp1_irq_class"

#define GPIO_PIN 588 
#define TARGET_CPU 3

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Leonardo Lisa");
MODULE_DESCRIPTION("Zero-Copy High-Performance GPIO IRQ Handler");
MODULE_VERSION("2.0");

// Shared payload structure
struct GpioIrqEvent {
    u64 timestamp_ns;
    u32 event_counter;
    // u32 pin_state;
};

#define KBUF_SIZE 256

// Mapped memory structure
struct SharedRingBuffer {
    u32 head;
    u32 tail;
    struct GpioIrqEvent events[KBUF_SIZE];
};

static int major_num;
static struct class* irq_class = NULL;
static struct device* irq_device = NULL;
static struct cdev irq_cdev;

static unsigned int irq_number;
static u32 total_interrupts = 0;

static struct SharedRingBuffer *shared_buf = NULL;
static DECLARE_WAIT_QUEUE_HEAD(wq);

static irqreturn_t gpio_isr(int irq, void *dev_id) {
    u64 ts = ktime_get_ns();
    //int state = gpio_get_value(GPIO_PIN);
    u32 current_head;

    total_interrupts++;

    // Lock-free read of the current head
    current_head = shared_buf->head;
    
    // Write payload
    shared_buf->events[current_head % KBUF_SIZE].timestamp_ns = ts;
    shared_buf->events[current_head % KBUF_SIZE].event_counter = total_interrupts;
    shared_buf->events[current_head % KBUF_SIZE].pin_state = state;
    
    // Memory barrier: ensure payload is written to memory before head is updated
    smp_store_release(&shared_buf->head, current_head + 1);

    // Wake up the user space thread sleeping on poll()
    wake_up_interruptible(&wq);

    return IRQ_HANDLED;
}

static int dev_open(struct inode *inodep, struct file *filep) {
    return 0;
}

static int dev_release(struct inode *inodep, struct file *filep) {
    return 0;
}

static int dev_mmap(struct file *filep, struct vm_area_struct *vma) {
    unsigned long size = vma->vm_end - vma->vm_start;
    unsigned long expected_size = PAGE_ALIGN(sizeof(struct SharedRingBuffer));

    if (size > expected_size) {
        pr_err("[%s] mmap size %lu exceeds allocated size %lu\n", DEVICE_NAME, size, expected_size);
        return -EINVAL;
    }

    // Removing pgprot_noncached ensures the mmap area inherits 
    // the original cacheable attributes of vmalloc_user. Coherency between 
    // CPU 3 (LKM) and the C++ thread is guaranteed at the hardware level.

    // Map the vmalloc area to user space
    if (remap_vmalloc_range(vma, shared_buf, 0)) {
        pr_err("[%s] mmap remap_vmalloc_range failed\n", DEVICE_NAME);
        return -EAGAIN;
    }

    return 0;
}

static __poll_t dev_poll(struct file *filep, poll_table *wait) {
    __poll_t mask = 0;
    
    poll_wait(filep, &wq, wait);
    
    // Data is ready to read if head != tail using lock-free read
    if (smp_load_acquire(&shared_buf->head) != smp_load_acquire(&shared_buf->tail)) {
        mask |= POLLIN | POLLRDNORM; 
    }
    
    return mask;
}

static struct file_operations fops = {
    .open = dev_open,
    .mmap = dev_mmap,
    .poll = dev_poll,
    .release = dev_release,
    .owner = THIS_MODULE
};

static int __init rpi_fast_irq_init(void) {
    int result;
    dev_t dev_num;
    struct cpumask affinity_mask;
    unsigned long buffer_size = PAGE_ALIGN(sizeof(struct SharedRingBuffer));

    shared_buf = vmalloc_user(buffer_size);
    if (!shared_buf) return -ENOMEM;
    
    shared_buf->head = 0;
    shared_buf->tail = 0;

    result = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    major_num = MAJOR(dev_num);
    if (result < 0) goto r_vmalloc;

    cdev_init(&irq_cdev, &fops);
    irq_cdev.owner = THIS_MODULE;
    cdev_add(&irq_cdev, dev_num, 1);

    irq_class = class_create(CLASS_NAME);
    irq_device = device_create(irq_class, NULL, dev_num, NULL, DEVICE_NAME);

    if (!gpio_is_valid(GPIO_PIN)) goto r_device;

    gpio_request(GPIO_PIN, "sysfs");
    gpio_direction_input(GPIO_PIN);
    
    irq_number = gpio_to_irq(GPIO_PIN);

    result = request_irq(irq_number, (irq_handler_t) gpio_isr, IRQF_TRIGGER_RISING, "rpi_fast_gpio_handler", NULL);
    if (result) goto r_gpio;

    cpumask_clear(&affinity_mask);
    cpumask_set_cpu(TARGET_CPU, &affinity_mask);
    irq_set_affinity_hint(irq_number, &affinity_mask);

    return 0;

r_gpio:
    gpio_free(GPIO_PIN);
r_device:
    device_destroy(irq_class, dev_num);
    class_destroy(irq_class);
    cdev_del(&irq_cdev);
    unregister_chrdev_region(dev_num, 1);
r_vmalloc:
    vfree(shared_buf);
    return -1;
}

static void __exit rpi_fast_irq_exit(void) {
    dev_t dev_num = MKDEV(major_num, 0);

    irq_set_affinity_hint(irq_number, NULL);
    free_irq(irq_number, NULL);
    gpio_free(GPIO_PIN);

    device_destroy(irq_class, dev_num);
    class_destroy(irq_class);
    cdev_del(&irq_cdev);
    unregister_chrdev_region(dev_num, 1);

    if (shared_buf) vfree(shared_buf);
}

module_init(rpi_fast_irq_init);
module_exit(rpi_fast_irq_exit);