#include "xr17v358.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static const uint32_t k_port_offsets[] = {
    0x0000U, 0x0400U, 0x0800U, 0x0C00U, 0x1000U, 0x1400U, 0x1800U, 0x1C00U,
};

#define XR17V358_FIFO_REGISTER_OFFSET 0x0100U
#define XR17V358_FIFO_CAPACITY 256U
#define XR17V358_QUEUE_CAPACITY 512U
#define XR17V358_FRAME_DELIMITER 0x7EU
#define XR17V358_FRAME_ESCAPE 0x7DU
#define XR17V358_FRAME_ESCAPE_XOR 0x20U
#define XR17V358_MAX_ENCODED_BYTE_COUNT 4U
#define XR17V358_DEFAULT_BAUD_RATE 115200U

typedef struct xr17v358_ring_buffer {
  uint8_t storage[XR17V358_QUEUE_CAPACITY * XR17V358_MAX_ENCODED_BYTE_COUNT];
  size_t head;
  size_t tail;
  size_t size;
  size_t capacity;
} xr17v358_ring_buffer;

static xr17v358_ring_buffer s_tx_fifo[8];
static xr17v358_ring_buffer s_rx_fifo[8];
static xr17v358_ring_buffer s_queue[8];
static xr17v358_port_config s_port_config[8];
static int s_initialized = 0;

static int is_valid_port(size_t port_index) {
  return port_index < xr17v358_get_port_count();
}

static xr17v358_error validate_port_index(size_t port_index) {
  if (!is_valid_port(port_index)) {
    return XR17V358_ERROR_INVALID_PORT;
  }

  return XR17V358_OK;
}

static xr17v358_error validate_port_config(const xr17v358_port_config *config) {
  if (config == NULL || config->baud_rate == 0U) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  if (config->stop_bits != XR17V358_STOP_BITS_1 &&
      config->stop_bits != XR17V358_STOP_BITS_2) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  if (config->parity != XR17V358_PARITY_NONE &&
      config->parity != XR17V358_PARITY_EVEN &&
      config->parity != XR17V358_PARITY_ODD) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  return XR17V358_OK;
}

static xr17v358_port_config default_port_config(void) {
  xr17v358_port_config config;

  config.baud_rate = XR17V358_DEFAULT_BAUD_RATE;
  config.stop_bits = XR17V358_STOP_BITS_1;
  config.parity = XR17V358_PARITY_NONE;
  return config;
}

static void ring_reset(xr17v358_ring_buffer *rb, size_t capacity) {
  rb->head = 0U;
  rb->tail = 0U;
  rb->size = 0U;
  rb->capacity = capacity;
}

static size_t ring_write_bytes(xr17v358_ring_buffer *rb, const uint8_t *data,
                               size_t length) {
  size_t i = 0U;
  while (i < length && rb->size < rb->capacity) {
    rb->storage[rb->tail] = data[i];
    rb->tail = (rb->tail + 1U) % rb->capacity;
    rb->size++;
    i++;
  }
  return i;
}

static size_t ring_read_bytes(xr17v358_ring_buffer *rb, uint8_t *data,
                              size_t length) {
  size_t i = 0U;
  while (i < length && rb->size > 0U) {
    data[i] = rb->storage[rb->head];
    rb->head = (rb->head + 1U) % rb->capacity;
    rb->size--;
    i++;
  }
  return i;
}

static void ensure_state_initialized(void) {
  if (!s_initialized) {
    xr17v358_reset();
  }
}

static size_t ring_frame_count(const xr17v358_ring_buffer *rb) {
  size_t i = 0U;
  size_t frames = 0U;
  size_t payload_bytes = 0U;
  int in_frame = 0;

  while (i < rb->size) {
    uint8_t value = rb->storage[(rb->head + i) % rb->capacity];
    if (value == XR17V358_FRAME_DELIMITER) {
      if (in_frame && payload_bytes > 0U) {
        frames++;
      }
      in_frame = 1;
      payload_bytes = 0U;
    } else if (in_frame) {
      payload_bytes++;
    }
    i++;
  }

  return frames;
}

static xr17v358_error ring_read_frame(xr17v358_ring_buffer *rb, uint8_t *frame,
                                      size_t frame_capacity,
                                      size_t *frame_length) {
  size_t count = 0U;
  uint8_t byte = 0U;

  if (frame == NULL || frame_length == NULL || frame_capacity == 0U) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  while (rb->size > 0U) {
    (void)ring_read_bytes(rb, &byte, 1U);
    if (count == 0U) {
      if (byte != XR17V358_FRAME_DELIMITER) {
        continue;
      }
      frame[count++] = byte;
      continue;
    }

    if (count >= frame_capacity) {
      return XR17V358_ERROR_INVALID_FRAME;
    }

    frame[count++] = byte;
    if (byte == XR17V358_FRAME_DELIMITER) {
      *frame_length = count;
      return XR17V358_OK;
    }
  }

  return XR17V358_ERROR_INVALID_FRAME;
}

static xr17v358_error ring_transfer_frame(xr17v358_ring_buffer *source,
                                          xr17v358_ring_buffer *destination) {
  uint8_t frame[XR17V358_MAX_ENCODED_BYTE_COUNT];
  size_t frame_length = 0U;
  size_t written = 0U;
  xr17v358_error error;

  error = ring_read_frame(source, frame, sizeof(frame), &frame_length);
  if (error != XR17V358_OK) {
    return error;
  }

  written = ring_write_bytes(destination, frame, frame_length);
  if (written != frame_length) {
    return XR17V358_ERROR_INVALID_FRAME;
  }

  return XR17V358_OK;
}

static xr17v358_error inject_frame_bytes(xr17v358_ring_buffer *buffers,
                                         size_t port_index,
                                         const uint8_t *data, size_t length,
                                         size_t *bytes_written) {
  xr17v358_error error;

  ensure_state_initialized();
  error = validate_port_index(port_index);
  if (error != XR17V358_OK) {
    return error;
  }

  if (bytes_written == NULL || (length > 0U && data == NULL)) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  *bytes_written = ring_write_bytes(&buffers[port_index], data, length);
  return XR17V358_OK;
}

xr17v358_error xr17v358_initialize_port(size_t port_index,
                                        const xr17v358_port_config *config) {
  xr17v358_error error;

  ensure_state_initialized();

  error = validate_port_index(port_index);
  if (error != XR17V358_OK) {
    return error;
  }

  error = validate_port_config(config);
  if (error != XR17V358_OK) {
    return error;
  }

  s_port_config[port_index] = *config;
  return XR17V358_OK;
}

xr17v358_error xr17v358_get_port_config(size_t port_index,
                                        xr17v358_port_config *config) {
  xr17v358_error error;

  ensure_state_initialized();

  error = validate_port_index(port_index);
  if (error != XR17V358_OK) {
    return error;
  }

  if (config == NULL) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  *config = s_port_config[port_index];
  return XR17V358_OK;
}

size_t xr17v358_encode_serial_data(size_t port_index, uint8_t data,
                                   uint8_t *frame, size_t frame_capacity) {
  size_t frame_length = 0U;

  (void)port_index;

  if (frame == NULL || frame_capacity < 3U) {
    return 0U;
  }

  frame[frame_length++] = XR17V358_FRAME_DELIMITER;
  if (data == XR17V358_FRAME_DELIMITER || data == XR17V358_FRAME_ESCAPE) {
    if (frame_capacity < XR17V358_MAX_ENCODED_BYTE_COUNT) {
      return 0U;
    }
    frame[frame_length++] = XR17V358_FRAME_ESCAPE;
    frame[frame_length++] = (uint8_t)(data ^ XR17V358_FRAME_ESCAPE_XOR);
  } else {
    frame[frame_length++] = data;
  }
  frame[frame_length++] = XR17V358_FRAME_DELIMITER;
  return frame_length;
}

xr17v358_error xr17v358_decode_serial_data(size_t port_index,
                                           const uint8_t *frame,
                                           size_t frame_length, uint8_t *data) {
  (void)port_index;

  if (frame == NULL || data == NULL) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  if (frame_length < 3U || frame[0] != XR17V358_FRAME_DELIMITER ||
      frame[frame_length - 1U] != XR17V358_FRAME_DELIMITER) {
    return XR17V358_ERROR_INVALID_FRAME;
  }

  if (frame_length == 3U) {
    if (frame[1] == XR17V358_FRAME_DELIMITER ||
        frame[1] == XR17V358_FRAME_ESCAPE) {
      return XR17V358_ERROR_INVALID_FRAME;
    }
    *data = frame[1];
    return XR17V358_OK;
  }

  if (frame_length == 4U && frame[1] == XR17V358_FRAME_ESCAPE) {
    *data = (uint8_t)(frame[2] ^ XR17V358_FRAME_ESCAPE_XOR);
    if (*data != XR17V358_FRAME_DELIMITER && *data != XR17V358_FRAME_ESCAPE) {
      return XR17V358_ERROR_INVALID_FRAME;
    }
    return XR17V358_OK;
  }

  return XR17V358_ERROR_INVALID_FRAME;
}

size_t xr17v358_get_port_count(void) {
  return sizeof(k_port_offsets) / sizeof(k_port_offsets[0]);
}

const uint32_t *xr17v358_get_port_offsets(void) { return k_port_offsets; }

xr17v358_error xr17v358_get_uart_base(uint32_t device_base, size_t port_index,
                                      uint32_t *uart_base) {
  xr17v358_error error = validate_port_index(port_index);

  if (uart_base == NULL) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  if (error != XR17V358_OK) {
    return error;
  }

  *uart_base = device_base + k_port_offsets[port_index];
  return XR17V358_OK;
}

xr17v358_error xr17v358_get_tx_fifo_base(uint32_t device_base,
                                         size_t port_index,
                                         uint32_t *fifo_base) {
  xr17v358_error error = validate_port_index(port_index);

  if (fifo_base == NULL) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  if (error != XR17V358_OK) {
    return error;
  }

  *fifo_base =
      device_base + k_port_offsets[port_index] + XR17V358_FIFO_REGISTER_OFFSET;
  return XR17V358_OK;
}

void xr17v358_print_uart_bases(uint32_t device_base, FILE *out) {
  size_t i;
  uint32_t uart_base;

  if (out == NULL) {
    return;
  }

  for (i = 0; i < xr17v358_get_port_count(); ++i) {
    if (xr17v358_get_uart_base(device_base, i, &uart_base) == XR17V358_OK) {
      fprintf(out, "UART%zu base: 0x%08X\n", i, uart_base);
    }
  }
}

int xr17v358_demo_main(int argc, char **argv, FILE *out, FILE *err) {
  uint32_t device_base = 0x10000000U;

  if (out == NULL || err == NULL) {
    return 1;
  }

  if (argc > 1) {
    char *end = NULL;
    unsigned long parsed = strtoul(argv[1], &end, 0);
    if (end == argv[1] || *end != '\0') {
      fprintf(err, "Invalid base address: %s\n", argv[1]);
      return 1;
    }
    device_base = (uint32_t)parsed;
  }

  xr17v358_print_uart_bases(device_base, out);
  return 0;
}

void xr17v358_reset(void) {
  size_t i;
  for (i = 0U; i < xr17v358_get_port_count(); ++i) {
    ring_reset(&s_tx_fifo[i],
               XR17V358_FIFO_CAPACITY * XR17V358_MAX_ENCODED_BYTE_COUNT);
    ring_reset(&s_rx_fifo[i],
               XR17V358_FIFO_CAPACITY * XR17V358_MAX_ENCODED_BYTE_COUNT);
    ring_reset(&s_queue[i],
               XR17V358_QUEUE_CAPACITY * XR17V358_MAX_ENCODED_BYTE_COUNT);
    s_port_config[i] = default_port_config();
    memset(s_tx_fifo[i].storage, 0, sizeof(s_tx_fifo[i].storage));
    memset(s_rx_fifo[i].storage, 0, sizeof(s_rx_fifo[i].storage));
    memset(s_queue[i].storage, 0, sizeof(s_queue[i].storage));
  }
  s_initialized = 1;
}

size_t xr17v358_get_fifo_capacity(void) { return XR17V358_FIFO_CAPACITY; }

size_t xr17v358_get_queue_capacity(void) { return XR17V358_QUEUE_CAPACITY; }

size_t xr17v358_fifo_level(size_t port_index) {
  ensure_state_initialized();
  if (!is_valid_port(port_index)) {
    return 0U;
  }
  return ring_frame_count(&s_rx_fifo[port_index]);
}

size_t xr17v358_queue_size(size_t port_index) {
  ensure_state_initialized();
  if (!is_valid_port(port_index)) {
    return 0U;
  }
  return ring_frame_count(&s_queue[port_index]);
}

xr17v358_error xr17v358_write(size_t port_index, const uint8_t *data,
                              size_t length, size_t *bytes_written) {
  size_t count = 0U;
  uint8_t frame[XR17V358_MAX_ENCODED_BYTE_COUNT];
  size_t frame_length = 0U;
  xr17v358_error error;

  ensure_state_initialized();
  error = validate_port_index(port_index);
  if (error != XR17V358_OK) {
    return error;
  }

  if (bytes_written == NULL || (length > 0U && data == NULL)) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  while (count < length) {
    frame_length = xr17v358_encode_serial_data(port_index, data[count], frame,
                                               sizeof(frame));
    if (frame_length == 0U ||
        ring_frame_count(&s_tx_fifo[port_index]) >= XR17V358_FIFO_CAPACITY ||
        s_tx_fifo[port_index].size + frame_length > s_tx_fifo[port_index].capacity) {
      break;
    }
    (void)ring_write_bytes(&s_tx_fifo[port_index], frame, frame_length);
    count++;
  }
  *bytes_written = count;
  return XR17V358_OK;
}

xr17v358_error xr17v358_read(size_t port_index, uint8_t *data, size_t length,
                             size_t *bytes_read) {
  size_t count = 0U;
  uint8_t frame[XR17V358_MAX_ENCODED_BYTE_COUNT];
  size_t frame_length = 0U;
  xr17v358_error error;

  ensure_state_initialized();
  error = validate_port_index(port_index);
  if (error != XR17V358_OK) {
    return error;
  }

  if (bytes_read == NULL || (length > 0U && data == NULL)) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  while (count < length && s_rx_fifo[port_index].size > 0U) {
    error = ring_read_frame(&s_rx_fifo[port_index], frame, sizeof(frame),
                            &frame_length);
    if (error != XR17V358_OK) {
      return error;
    }
    error = xr17v358_decode_serial_data(port_index, frame, frame_length,
                                        &data[count]);
    if (error != XR17V358_OK) {
      return error;
    }
    count++;
  }

  *bytes_read = count;
  return XR17V358_OK;
}

xr17v358_error xr17v358_receive(size_t port_index, const uint8_t *data,
                                size_t length, size_t *bytes_received) {
  size_t count = 0U;
  uint8_t frame[XR17V358_MAX_ENCODED_BYTE_COUNT];
  size_t frame_length = 0U;
  xr17v358_error error;

  ensure_state_initialized();
  error = validate_port_index(port_index);
  if (error != XR17V358_OK) {
    return error;
  }

  if (bytes_received == NULL || (length > 0U && data == NULL)) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  while (count < length) {
    frame_length = xr17v358_encode_serial_data(port_index, data[count], frame,
                                               sizeof(frame));
    if (frame_length == 0U ||
        ring_frame_count(&s_rx_fifo[port_index]) >= XR17V358_FIFO_CAPACITY ||
        s_rx_fifo[port_index].size + frame_length > s_rx_fifo[port_index].capacity) {
      break;
    }
    (void)ring_write_bytes(&s_rx_fifo[port_index], frame, frame_length);
    count++;
  }

  *bytes_received = count;
  return XR17V358_OK;
}

xr17v358_error xr17v358_inject_rx_frame_bytes(size_t port_index,
                                              const uint8_t *data,
                                              size_t length,
                                              size_t *bytes_written) {
  return inject_frame_bytes(s_rx_fifo, port_index, data, length,
                            bytes_written);
}

xr17v358_error xr17v358_inject_tx_frame_bytes(size_t port_index,
                                              const uint8_t *data,
                                              size_t length,
                                              size_t *bytes_written) {
  return inject_frame_bytes(s_tx_fifo, port_index, data, length,
                            bytes_written);
}

xr17v358_error xr17v358_transfer_next_tx_frame(size_t port_index) {
  xr17v358_error error;

  ensure_state_initialized();
  error = validate_port_index(port_index);
  if (error != XR17V358_OK) {
    return error;
  }

  return ring_transfer_frame(&s_tx_fifo[port_index], &s_queue[port_index]);
}

xr17v358_error xr17v358_inject_queue_frame_bytes(size_t port_index,
                                                 const uint8_t *data,
                                                 size_t length,
                                                 size_t *bytes_written) {
  return inject_frame_bytes(s_queue, port_index, data, length,
                            bytes_written);
}

xr17v358_error xr17v358_queue_read(size_t port_index, uint8_t *data,
                                   size_t length, size_t *bytes_read) {
  size_t count = 0U;
  uint8_t frame[XR17V358_MAX_ENCODED_BYTE_COUNT];
  size_t frame_length = 0U;
  xr17v358_error error;

  ensure_state_initialized();
  error = validate_port_index(port_index);
  if (error != XR17V358_OK) {
    return error;
  }

  if (bytes_read == NULL || (length > 0U && data == NULL)) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  while (count < length && s_queue[port_index].size > 0U) {
    error = ring_read_frame(&s_queue[port_index], frame, sizeof(frame),
                            &frame_length);
    if (error != XR17V358_OK) {
      return error;
    }
    error = xr17v358_decode_serial_data(port_index, frame, frame_length,
                                        &data[count]);
    if (error != XR17V358_OK) {
      return error;
    }
    count++;
  }

  *bytes_read = count;
  return XR17V358_OK;
}

xr17v358_error xr17v358_write_tx_fifo(volatile void *device_base,
                                      size_t port_index, const uint8_t *data,
                                      size_t length, size_t *bytes_written) {
  volatile uint8_t *fifo_bytes;
  volatile uint32_t *fifo_words;
  uintptr_t fifo_address;
  size_t count;
  size_t i;
  xr17v358_error error = validate_port_index(port_index);

  if (device_base == NULL || bytes_written == NULL ||
      (length > 0U && data == NULL)) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  if (error != XR17V358_OK) {
    return error;
  }

  count = length;
  if (count > XR17V358_FIFO_CAPACITY) {
    count = XR17V358_FIFO_CAPACITY;
  }

  fifo_address = (uintptr_t)device_base + k_port_offsets[port_index] +
                 XR17V358_FIFO_REGISTER_OFFSET;
  fifo_words = (volatile uint32_t *)fifo_address;
  fifo_bytes = (volatile uint8_t *)fifo_address;

  for (i = 0U; i + sizeof(uint32_t) <= count; i += sizeof(uint32_t)) {
    uint32_t word = (uint32_t)data[i] | ((uint32_t)data[i + 1U] << 8U) |
                    ((uint32_t)data[i + 2U] << 16U) |
                    ((uint32_t)data[i + 3U] << 24U);
    fifo_words[i / sizeof(uint32_t)] = word;
  }

  for (; i < count; ++i) {
    fifo_bytes[i] = data[i];
  }

  *bytes_written = count;
  return XR17V358_OK;
}

xr17v358_error xr17v358_poll_port(size_t port_index, const uint8_t *tx_data,
                                  size_t tx_length, size_t *tx_written,
                                  size_t *queued_count) {
  size_t queued = 0U;
  size_t written = 0U;
  xr17v358_error error;

  ensure_state_initialized();
  error = validate_port_index(port_index);
  if (error != XR17V358_OK) {
    return error;
  }

  if (tx_written == NULL || queued_count == NULL ||
      (tx_length > 0U && tx_data == NULL)) {
    return XR17V358_ERROR_INVALID_ARGUMENT;
  }

  error = xr17v358_write(port_index, tx_data, tx_length, &written);
  if (error != XR17V358_OK) {
    return error;
  }

  while (ring_frame_count(&s_tx_fifo[port_index]) > 0U &&
         ring_frame_count(&s_queue[port_index]) < XR17V358_QUEUE_CAPACITY &&
         s_queue[port_index].size + XR17V358_MAX_ENCODED_BYTE_COUNT <=
             s_queue[port_index].capacity) {
    error = ring_transfer_frame(&s_tx_fifo[port_index], &s_queue[port_index]);
    if (error != XR17V358_OK) {
      return error;
    }
    queued++;
  }

  *tx_written = written;
  *queued_count = queued;
  return XR17V358_OK;
}
