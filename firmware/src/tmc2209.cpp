#include "tmc2209.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace fw {

namespace {

constexpr uart_port_t kPort = UART_NUM_1;

// TMC2209 register addresses (write access sets bit 7 in the datagram).
constexpr uint8_t kRegGconf = 0x00;
constexpr uint8_t kRegIholdIrun = 0x10;
constexpr uint8_t kRegChopconf = 0x6C;

// GCONF: pdn_disable (UART stays usable while PDN is driven), mstep_reg_select
// (microstepping from CHOPCONF.MRES, not the MS pins), multistep_filt (default).
constexpr uint32_t kGconfValue = (1u << 6) | (1u << 7) | (1u << 8);

// CHOPCONF reset value (TOFF=3, HSTRT=5, intpol=1); only MRES gets replaced.
constexpr uint32_t kChopconfDefault = 0x10000053;

// CRC8 over the datagram, x^8 + x^2 + x + 1, LSB-first (TMC2209 datasheet §4.2).
uint8_t crc8(const uint8_t* data, int length) {
  uint8_t crc = 0;
  for (int i = 0; i < length; ++i) {
    uint8_t byte = data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (((crc >> 7) ^ (byte & 0x01)) != 0)
        crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      else
        crc = static_cast<uint8_t>(crc << 1);
      byte >>= 1;
    }
  }
  return crc;
}

void writeRegister(uint8_t driverAddress, uint8_t reg, uint32_t value) {
  uint8_t frame[8] = {
      0x05,  // sync + reserved
      driverAddress,
      static_cast<uint8_t>(reg | 0x80),  // write access
      static_cast<uint8_t>(value >> 24),
      static_cast<uint8_t>(value >> 16),
      static_cast<uint8_t>(value >> 8),
      static_cast<uint8_t>(value),
      0,
  };
  frame[7] = crc8(frame, 7);
  uart_write_bytes(kPort, frame, sizeof frame);
  ESP_ERROR_CHECK(uart_wait_tx_done(kPort, pdMS_TO_TICKS(50)));
  vTaskDelay(pdMS_TO_TICKS(2));  // inter-datagram gap so the driver resyncs
}

/** CHOPCONF.MRES code: 0 = 256 µsteps … 8 = full step. */
uint32_t mresCode(double microstepping) {
  const int usteps = static_cast<int>(std::lround(microstepping));
  if (usteps < 1 || usteps > 256 || (usteps & (usteps - 1)) != 0)
    throw std::runtime_error("motor.microstepping must be a power of two in 1..256");
  int log2 = 0;
  while ((1 << log2) < usteps) ++log2;
  return static_cast<uint32_t>(8 - log2);
}

}  // namespace

void configureTmc2209Drivers(const rt::HardwareConfig& hw, const rt::MotorParams& motor) {
  if (!hw.tmcUartEnabled) return;
  if (hw.tmcIrun < 0 || hw.tmcIrun > 31 || hw.tmcIhold < 0 || hw.tmcIhold > 31)
    throw std::runtime_error("tmc_uart irun/ihold must be 0..31");

  uart_config_t cfg = {};
  cfg.baud_rate = hw.tmcUartBaud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(kPort, 256, 256, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(kPort, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(kPort, hw.tmcUartTxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));

  const uint32_t chopconf = (kChopconfDefault & ~(0xFu << 24)) | (mresCode(motor.microstepping) << 24);
  const uint32_t iholdIrun = static_cast<uint32_t>(hw.tmcIhold & 0x1F) |
                             (static_cast<uint32_t>(hw.tmcIrun & 0x1F) << 8) |
                             (8u << 16);  // IHOLDDELAY: gentle power-down ramp

  for (const auto& joint : hw.joints) {
    const auto addr = static_cast<uint8_t>(joint.uartAddress);
    writeRegister(addr, kRegGconf, kGconfValue);
    writeRegister(addr, kRegChopconf, chopconf);
    writeRegister(addr, kRegIholdIrun, iholdIrun);
  }
}

}  // namespace fw
