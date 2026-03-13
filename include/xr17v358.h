#ifndef XR17V358_H
#define XR17V358_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum xr17v358_error {
    XR17V358_OK = 0,
    XR17V358_ERROR_INVALID_ARGUMENT = -1,
    XR17V358_ERROR_INVALID_PORT = -2,
    XR17V358_ERROR_INVALID_FRAME = -3,
} xr17v358_error;

typedef enum xr17v358_stop_bits {
    XR17V358_STOP_BITS_1 = 1,
    XR17V358_STOP_BITS_2 = 2,
} xr17v358_stop_bits;

typedef enum xr17v358_parity {
    XR17V358_PARITY_NONE = 0,
    XR17V358_PARITY_EVEN = 1,
    XR17V358_PARITY_ODD = 2,
} xr17v358_parity;

typedef struct xr17v358_port_config {
    uint32_t baud_rate;
    xr17v358_stop_bits stop_bits;
    xr17v358_parity parity;
} xr17v358_port_config;

xr17v358_error xr17v358_initialize_port(
    size_t port_index,
    const xr17v358_port_config *config);
xr17v358_error xr17v358_get_port_config(size_t port_index, xr17v358_port_config *config);
size_t xr17v358_encode_serial_data(
    size_t port_index,
    uint8_t data,
    uint8_t *frame,
    size_t frame_capacity);
xr17v358_error xr17v358_decode_serial_data(
    size_t port_index,
    const uint8_t *frame,
    size_t frame_length,
    uint8_t *data);

size_t xr17v358_get_port_count(void);
const uint32_t *xr17v358_get_port_offsets(void);
xr17v358_error xr17v358_get_uart_base(
    uint32_t device_base,
    size_t port_index,
    uint32_t *uart_base);
xr17v358_error xr17v358_get_tx_fifo_base(
    uint32_t device_base,
    size_t port_index,
    uint32_t *fifo_base);
void xr17v358_print_uart_bases(uint32_t device_base, FILE *out);
int xr17v358_demo_main(int argc, char **argv, FILE *out, FILE *err);

void xr17v358_reset(void);
size_t xr17v358_get_fifo_capacity(void);
size_t xr17v358_get_queue_capacity(void);

size_t xr17v358_fifo_level(size_t port_index);
size_t xr17v358_queue_size(size_t port_index);

xr17v358_error xr17v358_write(
    size_t port_index,
    const uint8_t *data,
    size_t length,
    size_t *bytes_written);
xr17v358_error xr17v358_read(
    size_t port_index,
    uint8_t *data,
    size_t length,
    size_t *bytes_read);
xr17v358_error xr17v358_receive(
    size_t port_index,
    const uint8_t *data,
    size_t length,
    size_t *bytes_received);
xr17v358_error xr17v358_inject_rx_frame_bytes(
    size_t port_index,
    const uint8_t *data,
    size_t length,
    size_t *bytes_written);
xr17v358_error xr17v358_inject_tx_frame_bytes(
    size_t port_index,
    const uint8_t *data,
    size_t length,
    size_t *bytes_written);
xr17v358_error xr17v358_transfer_next_tx_frame(size_t port_index);
xr17v358_error xr17v358_inject_queue_frame_bytes(
    size_t port_index,
    const uint8_t *data,
    size_t length,
    size_t *bytes_written);
xr17v358_error xr17v358_queue_read(
    size_t port_index,
    uint8_t *data,
    size_t length,
    size_t *bytes_read);
xr17v358_error xr17v358_write_tx_fifo(
    volatile void *device_base,
    size_t port_index,
    const uint8_t *data,
    size_t length,
    size_t *bytes_written);

xr17v358_error xr17v358_poll_port(
    size_t port_index,
    const uint8_t *tx_data,
    size_t tx_length,
    size_t *tx_written,
    size_t *queued_count);

#ifdef __cplusplus
}
#endif

#endif
