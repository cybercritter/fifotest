#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <string>
#include <vector>

extern "C" {
#include "xr17v358.h"
}

TEST(Xr17v358, PortOffsetsAreExposedInOrder) {
  xr17v358_reset();

  constexpr std::array<uint32_t, 8> kExpectedOffsets = {
      0x0000U, 0x0400U, 0x0800U, 0x0C00U, 0x1000U, 0x1400U, 0x1800U, 0x1C00U,
  };

  ASSERT_EQ(xr17v358_get_port_count(), kExpectedOffsets.size());

  const uint32_t *offsets = xr17v358_get_port_offsets();
  ASSERT_NE(offsets, nullptr);

  for (size_t i = 0; i < kExpectedOffsets.size(); ++i) {
    EXPECT_EQ(offsets[i], kExpectedOffsets[i]);
  }
}

TEST(Xr17v358, ComputesUartBaseForValidPorts) {
  xr17v358_reset();

  constexpr uint32_t kDeviceBase = 0x80000000U;

  for (size_t i = 0; i < xr17v358_get_port_count(); ++i) {
    uint32_t uart_base = 0;
    ASSERT_EQ(xr17v358_get_uart_base(kDeviceBase, i, &uart_base), XR17V358_OK);
    EXPECT_EQ(uart_base, kDeviceBase + xr17v358_get_port_offsets()[i]);
  }
}

TEST(Xr17v358, RejectsInvalidPortIndex) {
  xr17v358_reset();

  uint32_t uart_base = 0;
  EXPECT_EQ(xr17v358_get_uart_base(0x0U, xr17v358_get_port_count(), &uart_base),
            XR17V358_ERROR_INVALID_PORT);
}

TEST(Xr17v358, RejectsNullOutputPointer) {
  xr17v358_reset();

  EXPECT_EQ(xr17v358_get_uart_base(0x0U, 0, nullptr),
            XR17V358_ERROR_INVALID_ARGUMENT);
}

TEST(Xr17v358, PrintsComputedBases) {
  xr17v358_reset();

  constexpr uint32_t kDeviceBase = 0x90000000U;
  std::array<char, 1024> buffer{};

  FILE *file = tmpfile();
  ASSERT_NE(file, nullptr);

  xr17v358_print_uart_bases(kDeviceBase, file);
  rewind(file);

  const size_t bytes = fread(buffer.data(), 1, buffer.size() - 1, file);
  buffer[bytes] = '\0';
  fclose(file);

  std::string output(buffer.data());
  EXPECT_NE(output.find("UART0 base: 0x90000000"), std::string::npos);
  EXPECT_NE(output.find("UART7 base: 0x90001C00"), std::string::npos);
}

TEST(Xr17v358, ComputesTxFifoBaseForValidPorts) {
  xr17v358_reset();

  constexpr uint32_t kDeviceBase = 0x81000000U;

  for (size_t i = 0; i < xr17v358_get_port_count(); ++i) {
    uint32_t fifo_base = 0;
    ASSERT_EQ(xr17v358_get_tx_fifo_base(kDeviceBase, i, &fifo_base),
              XR17V358_OK);
    EXPECT_EQ(fifo_base,
              kDeviceBase + xr17v358_get_port_offsets()[i] + 0x0100U);
  }
}

TEST(Xr17v358, InitializesPortConfigurationPerPort) {
  xr17v358_reset();

  const xr17v358_port_config config = {
      57600U,
      XR17V358_STOP_BITS_2,
      XR17V358_PARITY_ODD,
  };
  xr17v358_port_config stored{};

  ASSERT_EQ(xr17v358_initialize_port(3, &config), XR17V358_OK);
  ASSERT_EQ(xr17v358_get_port_config(3, &stored), XR17V358_OK);
  EXPECT_EQ(stored.baud_rate, config.baud_rate);
  EXPECT_EQ(stored.stop_bits, config.stop_bits);
  EXPECT_EQ(stored.parity, config.parity);
}

TEST(Xr17v358, RejectsInvalidPortConfiguration) {
  xr17v358_reset();

  const xr17v358_port_config invalid_baud = {0U, XR17V358_STOP_BITS_1,
                                             XR17V358_PARITY_NONE};
  const xr17v358_port_config invalid_stop = {
      115200U,
      static_cast<xr17v358_stop_bits>(3),
      XR17V358_PARITY_NONE,
  };

  EXPECT_EQ(xr17v358_initialize_port(0, nullptr),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_initialize_port(0, &invalid_baud),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_initialize_port(0, &invalid_stop),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_get_port_config(xr17v358_get_port_count(), nullptr),
            XR17V358_ERROR_INVALID_PORT);
}

TEST(Xr17v358, LazilyInitializesDefaultPortConfiguration) {
  xr17v358_port_config config{};

  ASSERT_EQ(xr17v358_get_port_config(0, &config), XR17V358_OK);
  EXPECT_EQ(config.baud_rate, 115200U);
  EXPECT_EQ(config.stop_bits, XR17V358_STOP_BITS_1);
  EXPECT_EQ(config.parity, XR17V358_PARITY_NONE);
  EXPECT_EQ(xr17v358_fifo_level(0), 0U);
}

TEST(Xr17v358, EncodesAndDecodesSerialDataUsing7EFraming) {
  xr17v358_reset();

  constexpr uint8_t kPayload = 0xA5U;
  std::array<uint8_t, 4> frame{};
  uint8_t decoded = 0U;
  const size_t frame_length =
      xr17v358_encode_serial_data(0, kPayload, frame.data(), frame.size());

  ASSERT_EQ(frame_length, 3U);
  EXPECT_EQ(frame[0], 0x7EU);
  EXPECT_EQ(frame[1], kPayload);
  EXPECT_EQ(frame[2], 0x7EU);
  ASSERT_EQ(
      xr17v358_decode_serial_data(0, frame.data(), frame_length, &decoded),
      XR17V358_OK);
  EXPECT_EQ(decoded, kPayload);
}

TEST(Xr17v358, EncodesAndDecodesSerialDataUsingEscapedDelimiterFraming) {
  xr17v358_reset();

  constexpr uint8_t kPayload = 0x7EU;
  std::array<uint8_t, 4> frame{};
  uint8_t decoded = 0U;
  const size_t frame_length =
      xr17v358_encode_serial_data(1, kPayload, frame.data(), frame.size());

  ASSERT_EQ(frame_length, 4U);
  EXPECT_EQ(frame[0], 0x7EU);
  EXPECT_EQ(frame[1], 0x7DU);
  EXPECT_EQ(frame[2], 0x5EU);
  EXPECT_EQ(frame[3], 0x7EU);
  ASSERT_EQ(
      xr17v358_decode_serial_data(1, frame.data(), frame_length, &decoded),
      XR17V358_OK);
  EXPECT_EQ(decoded, kPayload);
}

TEST(Xr17v358, RejectsInvalidSerialFrame) {
  xr17v358_reset();

  uint8_t decoded = 0U;
  const std::array<uint8_t, 3> invalid_delimiter = {0x7EU, 0x7EU, 0x7EU};
  const std::array<uint8_t, 3> invalid_escape = {0x7EU, 0x7DU, 0x7EU};

  EXPECT_EQ(xr17v358_decode_serial_data(0, invalid_delimiter.data(),
                                        invalid_delimiter.size(), &decoded),
            XR17V358_ERROR_INVALID_FRAME);
  EXPECT_EQ(xr17v358_decode_serial_data(0, invalid_delimiter.data(),
                                        invalid_delimiter.size(), nullptr),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_decode_serial_data(1, invalid_escape.data(),
                                        invalid_escape.size(), &decoded),
            XR17V358_ERROR_INVALID_FRAME);
}

TEST(Xr17v358, RejectsSerialEncodeEdgeCases) {
  std::array<uint8_t, 4> frame{};

  EXPECT_EQ(xr17v358_encode_serial_data(0, 0x11U, nullptr, frame.size()), 0U);
  EXPECT_EQ(xr17v358_encode_serial_data(0, 0x11U, frame.data(), 2U), 0U);
  EXPECT_EQ(xr17v358_encode_serial_data(0, 0x7EU, frame.data(), 3U), 0U);
}

TEST(Xr17v358, HandlesAdditionalPublicArgumentValidation) {
  xr17v358_reset();

  const xr17v358_port_config invalid_parity = {
      115200U,
      XR17V358_STOP_BITS_1,
      static_cast<xr17v358_parity>(3),
  };
  size_t count = 0U;
  uint8_t byte = 0U;
  uint32_t fifo_base = 0U;

  EXPECT_EQ(
      xr17v358_initialize_port(xr17v358_get_port_count(), &invalid_parity),
      XR17V358_ERROR_INVALID_PORT);
  EXPECT_EQ(xr17v358_initialize_port(0, &invalid_parity),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_get_port_config(0, nullptr),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(
      xr17v358_get_tx_fifo_base(0x0U, xr17v358_get_port_count(), &fifo_base),
      XR17V358_ERROR_INVALID_PORT);
  EXPECT_EQ(xr17v358_write(0, &byte, 1, nullptr),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_write(0, nullptr, 1, &count),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_read(0, &byte, 1, nullptr),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_read(0, nullptr, 1, &count),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_receive(0, &byte, 1, nullptr),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_receive(0, nullptr, 1, &count),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_queue_read(0, &byte, 1, nullptr),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_queue_read(0, nullptr, 1, &count),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_write_tx_fifo(reinterpret_cast<void *>(&byte),
                                   xr17v358_get_port_count(), &byte, 1, &count),
            XR17V358_ERROR_INVALID_PORT);
  EXPECT_EQ(
      xr17v358_poll_port(xr17v358_get_port_count(), &byte, 1, &count, &count),
      XR17V358_ERROR_INVALID_PORT);
  EXPECT_EQ(xr17v358_fifo_level(xr17v358_get_port_count()), 0U);
  EXPECT_EQ(xr17v358_queue_size(xr17v358_get_port_count()), 0U);
  xr17v358_print_uart_bases(0x1000U, nullptr);
}

TEST(Xr17v358, ReadRejectsMalformedInjectedRxFrames) {
  xr17v358_reset();

  const std::array<uint8_t, 2> incomplete = {0x7EU, 0x11U};
  const std::array<uint8_t, 5> oversized = {0x7EU, 0x11U, 0x22U, 0x33U, 0x44U};
  const std::array<uint8_t, 4> bad_escape = {0x7EU, 0x7DU, 0x11U, 0x7EU};
  uint8_t output = 0U;
  size_t count = 0U;

  ASSERT_EQ(xr17v358_inject_rx_frame_bytes(0, incomplete.data(),
                                           incomplete.size(), &count),
            XR17V358_OK);
  EXPECT_EQ(xr17v358_read(0, &output, 1, &count), XR17V358_ERROR_INVALID_FRAME);

  xr17v358_reset();
  ASSERT_EQ(xr17v358_inject_rx_frame_bytes(0, oversized.data(),
                                           oversized.size(), &count),
            XR17V358_OK);
  EXPECT_EQ(xr17v358_read(0, &output, 1, &count), XR17V358_ERROR_INVALID_FRAME);

  xr17v358_reset();
  ASSERT_EQ(xr17v358_inject_rx_frame_bytes(0, bad_escape.data(),
                                           bad_escape.size(), &count),
            XR17V358_OK);
  EXPECT_EQ(xr17v358_read(0, &output, 1, &count), XR17V358_ERROR_INVALID_FRAME);
}

TEST(Xr17v358, QueueReadRejectsMalformedInjectedFrames) {
  xr17v358_reset();

  const std::array<uint8_t, 2> incomplete = {0x7EU, 0x11U};
  const std::array<uint8_t, 4> bad_escape = {0x7EU, 0x7DU, 0x11U, 0x7EU};
  uint8_t output = 0U;
  size_t count = 0U;

  ASSERT_EQ(xr17v358_inject_queue_frame_bytes(0, incomplete.data(),
                                              incomplete.size(), &count),
            XR17V358_OK);
  EXPECT_EQ(xr17v358_queue_read(0, &output, 1, &count),
            XR17V358_ERROR_INVALID_FRAME);

  xr17v358_reset();
  ASSERT_EQ(xr17v358_inject_queue_frame_bytes(0, bad_escape.data(),
                                              bad_escape.size(), &count),
            XR17V358_OK);
  EXPECT_EQ(xr17v358_queue_read(0, &output, 1, &count),
            XR17V358_ERROR_INVALID_FRAME);
}

TEST(Xr17v358, TransferRejectsMalformedInjectedTxFrames) {
  xr17v358_reset();

  const std::array<uint8_t, 6> oversized = {0x7EU, 0x11U, 0x22U,
                                            0x33U, 0x44U, 0x7EU};
  size_t count = 0U;

  ASSERT_EQ(
      xr17v358_inject_tx_frame_bytes(0, oversized.data(), oversized.size(), &count),
      XR17V358_OK);
  EXPECT_EQ(xr17v358_transfer_next_tx_frame(0), XR17V358_ERROR_INVALID_FRAME);
}

TEST(Xr17v358, DemoMainHandlesDefaultValidAndInvalidArguments) {
  std::array<char, 2048> default_out_buffer{};
  std::array<char, 2048> valid_out_buffer{};
  std::array<char, 512> err_buffer{};
  char arg0[] = "demo";
  char valid_arg[] = "0x20000000";
  char invalid_arg[] = "bad";
  char *default_argv[] = {arg0, nullptr};
  char *valid_argv[] = {arg0, valid_arg, nullptr};
  char *invalid_argv[] = {arg0, invalid_arg, nullptr};

  FILE *default_out = tmpfile();
  FILE *valid_out = tmpfile();
  FILE *err = tmpfile();
  ASSERT_NE(default_out, nullptr);
  ASSERT_NE(valid_out, nullptr);
  ASSERT_NE(err, nullptr);

  EXPECT_EQ(xr17v358_demo_main(1, default_argv, default_out, err), 0);
  EXPECT_EQ(xr17v358_demo_main(2, valid_argv, valid_out, err), 0);
  EXPECT_EQ(xr17v358_demo_main(2, invalid_argv, valid_out, err), 1);
  EXPECT_EQ(xr17v358_demo_main(1, default_argv, nullptr, err), 1);
  EXPECT_EQ(xr17v358_demo_main(1, default_argv, default_out, nullptr), 1);

  rewind(default_out);
  rewind(valid_out);
  rewind(err);
  const size_t default_out_bytes = fread(default_out_buffer.data(), 1,
                                         default_out_buffer.size() - 1U, default_out);
  const size_t valid_out_bytes =
      fread(valid_out_buffer.data(), 1, valid_out_buffer.size() - 1U, valid_out);
  const size_t err_bytes =
      fread(err_buffer.data(), 1, err_buffer.size() - 1U, err);
  default_out_buffer[default_out_bytes] = '\0';
  valid_out_buffer[valid_out_bytes] = '\0';
  err_buffer[err_bytes] = '\0';

  fclose(default_out);
  fclose(valid_out);
  fclose(err);

  EXPECT_NE(std::string(default_out_buffer.data()).find("UART0 base: 0x10000000"),
            std::string::npos);
  EXPECT_NE(std::string(valid_out_buffer.data()).find("UART0 base: 0x20000000"),
            std::string::npos);
  EXPECT_NE(std::string(err_buffer.data()).find("Invalid base address: bad"),
            std::string::npos);
}

TEST(Xr17v358, FifoWriteReadPreservesOrder) {
  xr17v358_reset();

  constexpr std::array<uint8_t, 5> kInput = {0x11U, 0x22U, 0x33U, 0x44U, 0x55U};
  std::array<uint8_t, 5> output{};
  size_t received = 0;
  size_t read_count = 0;

  ASSERT_EQ(xr17v358_receive(2, kInput.data(), kInput.size(), &received),
            XR17V358_OK);
  ASSERT_EQ(received, kInput.size());
  ASSERT_EQ(xr17v358_fifo_level(2), kInput.size());

  ASSERT_EQ(xr17v358_read(2, output.data(), output.size(), &read_count),
            XR17V358_OK);
  ASSERT_EQ(read_count, kInput.size());
  EXPECT_EQ(output, kInput);
  EXPECT_EQ(xr17v358_fifo_level(2), 0U);
}

TEST(Xr17v358, ReadPathIsIndependentFromWritePath) {
  xr17v358_reset();

  constexpr std::array<uint8_t, 3> kTxData = {0x11U, 0x22U, 0x33U};
  constexpr std::array<uint8_t, 3> kRxData = {0x44U, 0x55U, 0x66U};
  std::array<uint8_t, 3> output{};
  size_t written = 0U;
  size_t received = 0U;
  size_t read_count = 0U;

  ASSERT_EQ(xr17v358_write(0, kTxData.data(), kTxData.size(), &written),
            XR17V358_OK);
  ASSERT_EQ(written, kTxData.size());
  EXPECT_EQ(xr17v358_fifo_level(0), 0U);

  ASSERT_EQ(xr17v358_receive(0, kRxData.data(), kRxData.size(), &received),
            XR17V358_OK);
  ASSERT_EQ(received, kRxData.size());
  EXPECT_EQ(xr17v358_fifo_level(0), kRxData.size());

  ASSERT_EQ(xr17v358_read(0, output.data(), output.size(), &read_count),
            XR17V358_OK);
  ASSERT_EQ(read_count, kRxData.size());
  EXPECT_EQ(output, kRxData);
}

TEST(Xr17v358, PollerWritesUntilFifoFullThenQueuesData) {
  xr17v358_reset();

  const size_t fifo_capacity = xr17v358_get_fifo_capacity();
  std::vector<uint8_t> tx(fifo_capacity + 20U);
  for (size_t i = 0; i < tx.size(); ++i) {
    tx[i] = static_cast<uint8_t>(i & 0xFFU);
  }

  size_t tx_written = 0;
  size_t queued = 0;
  ASSERT_EQ(xr17v358_poll_port(1, tx.data(), tx.size(), &tx_written, &queued),
            XR17V358_OK);
  EXPECT_EQ(tx_written, fifo_capacity);
  EXPECT_EQ(queued, fifo_capacity);
  EXPECT_EQ(xr17v358_fifo_level(1), 0U);
  EXPECT_EQ(xr17v358_queue_size(1), fifo_capacity);

  std::vector<uint8_t> drained(fifo_capacity);
  size_t popped = 0;
  ASSERT_EQ(xr17v358_queue_read(1, drained.data(), drained.size(), &popped),
            XR17V358_OK);
  ASSERT_EQ(popped, fifo_capacity);

  for (size_t i = 0; i < fifo_capacity; ++i) {
    EXPECT_EQ(drained[i], tx[i]);
  }
}

TEST(Xr17v358, WritesPackedDataToTxFifoRegisterWindow) {
  xr17v358_reset();

  std::array<uint8_t, 0x2000> mmio{};
  constexpr std::array<uint8_t, 6> kPayload = {0x10U, 0x21U, 0x32U,
                                               0x43U, 0x54U, 0x65U};
  size_t written = 0;

  ASSERT_EQ(xr17v358_write_tx_fifo(mmio.data(), 2, kPayload.data(),
                                   kPayload.size(), &written),
            XR17V358_OK);
  ASSERT_EQ(written, kPayload.size());

  const size_t fifo_offset =
      static_cast<size_t>(xr17v358_get_port_offsets()[2] + 0x0100U);
  EXPECT_EQ(mmio[fifo_offset + 0], 0x10U);
  EXPECT_EQ(mmio[fifo_offset + 1], 0x21U);
  EXPECT_EQ(mmio[fifo_offset + 2], 0x32U);
  EXPECT_EQ(mmio[fifo_offset + 3], 0x43U);
  EXPECT_EQ(mmio[fifo_offset + 4], 0x54U);
  EXPECT_EQ(mmio[fifo_offset + 5], 0x65U);
}

TEST(Xr17v358, TxFifoWriteCapsAtHardwareFifoCapacity) {
  xr17v358_reset();

  std::array<uint8_t, 0x2000> mmio{};
  std::vector<uint8_t> payload(xr17v358_get_fifo_capacity() + 9U, 0xA5U);
  size_t written = 0;

  ASSERT_EQ(xr17v358_write_tx_fifo(mmio.data(), 0, payload.data(),
                                   payload.size(), &written),
            XR17V358_OK);
  EXPECT_EQ(written, xr17v358_get_fifo_capacity());
}

TEST(Xr17v358, PollerAndApisRejectInvalidArguments) {
  xr17v358_reset();

  size_t count = 0;
  uint8_t byte = 0;

  EXPECT_EQ(xr17v358_write(xr17v358_get_port_count(), &byte, 1, &count),
            XR17V358_ERROR_INVALID_PORT);
  EXPECT_EQ(xr17v358_receive(xr17v358_get_port_count(), &byte, 1, &count),
            XR17V358_ERROR_INVALID_PORT);
  EXPECT_EQ(xr17v358_read(xr17v358_get_port_count(), &byte, 1, &count),
            XR17V358_ERROR_INVALID_PORT);
  EXPECT_EQ(xr17v358_queue_read(xr17v358_get_port_count(), &byte, 1, &count),
            XR17V358_ERROR_INVALID_PORT);
  EXPECT_EQ(xr17v358_get_tx_fifo_base(0x0U, xr17v358_get_port_count(), nullptr),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_write_tx_fifo(nullptr, 0, &byte, 1, &count),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_poll_port(0, nullptr, 1, &count, &count),
            XR17V358_ERROR_INVALID_ARGUMENT);
  EXPECT_EQ(xr17v358_poll_port(0, &byte, 1, nullptr, &count),
            XR17V358_ERROR_INVALID_ARGUMENT);
}
