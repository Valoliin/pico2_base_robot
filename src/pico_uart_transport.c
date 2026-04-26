#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#define UART_ID uart0
#define BAUDRATE 115200
#define UART_TX_PIN 12
#define UART_RX_PIN 13

// --- Buffer circulaire (Ring Buffer) pour ne perdre aucun octet ---
#define RX_RING_BUFFER_SIZE 1024
volatile uint8_t rx_ring_buffer[RX_RING_BUFFER_SIZE];
volatile uint16_t rx_ring_head = 0;
volatile uint16_t rx_ring_tail = 0;

void on_uart_rx()
{
    while (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);
        uint16_t next_head = (rx_ring_head + 1) % RX_RING_BUFFER_SIZE;
        if (next_head != rx_ring_tail)
        {
            rx_ring_buffer[rx_ring_head] = ch;
            rx_ring_head = next_head;
        }
    }
}

bool pico_serial_transport_open(struct uxrCustomTransport *transport)
{
    uart_init(UART_ID, BAUDRATE);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Activation de l'interruption UART en arrière-plan
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport *transport)
{
    return true;
}

// Modifie le prototype pour correspondre au .h et éviter les warnings de compilation
size_t pico_serial_transport_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buf, // Ajout de const ici
    size_t len,
    uint8_t *errcode)
{
    for (size_t i = 0; i < len; i++)
    {
        uart_putc_raw(UART_ID, buf[i]);
    }
    return len;
}

size_t pico_serial_transport_read(
    struct uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *err)
{
    size_t received = 0;
    // On calcule l'heure limite en fonction du timeout (ms)
    absolute_time_t timeout_time = make_timeout_time_ms(timeout);

    while (received < len && absolute_time_diff_us(get_absolute_time(), timeout_time) > 0)
    {
        if (rx_ring_head != rx_ring_tail)
        {
            buf[received++] = rx_ring_buffer[rx_ring_tail];
            rx_ring_tail = (rx_ring_tail + 1) % RX_RING_BUFFER_SIZE;
        }
        else
        {
            sleep_us(10);
        }
    }

    return received;
}

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}