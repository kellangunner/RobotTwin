#include "gripper_link.hpp"

#include <cstdio>
#include <stdexcept>

#include "driver/uart.h"

#include "math/units.hpp"

namespace fw {

namespace {

// UART0 is the protocol console, UART1 is the TMC2209 bus; 2 is what is left.
constexpr uart_port_t kPort = UART_NUM_2;

// The link only ever carries "G <mm>\n" or "R\n", and nothing is received, so
// the smallest buffer the driver accepts is plenty.
constexpr int kTxBuffer = 256;

// Openings closer than this are the same command as far as the servo is
// concerned — an SG-90's own deadband is wider than 10 um of jaw travel. Keeps
// the 1 kHz control loop from flooding a 19200 baud line.
constexpr double kSendEpsilon = 1e-5;  // m

}  // namespace

void GripperLink::init(const rt::GripperConfig& config) {
  // GPIO34-39 are input-only, so they cannot carry a UART TX.
  if (config.relayPin < 0 || config.relayPin > 33)
    throw std::runtime_error("gripper relay pin must be an output-capable GPIO 0..33");

  uart_config_t cfg = {};
  cfg.baud_rate = config.relayBaud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(kPort, kTxBuffer, kTxBuffer, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(kPort, &cfg));
  // TX only: the Arduino never answers, so RX/RTS/CTS stay unassigned.
  ESP_ERROR_CHECK(uart_set_pin(kPort, config.relayPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  ready_ = true;
}

void GripperLink::send(double openingM) {
  if (!ready_) return;
  // The control loop calls this every tick; only a genuinely new destination is
  // worth a frame.
  if (driving_ && openingM > lastSent_ - kSendEpsilon && openingM < lastSent_ + kSendEpsilon)
    return;

  char frame[24];
  const int n = std::snprintf(frame, sizeof frame, "G %.2f\n", rt::m2mm(openingM));
  if (n > 0) uart_write_bytes(kPort, frame, static_cast<size_t>(n));
  lastSent_ = openingM;
  driving_ = true;
}

void GripperLink::release() {
  if (!ready_ || !driving_) return;
  static constexpr char kFrame[] = "R\n";
  uart_write_bytes(kPort, kFrame, sizeof kFrame - 1);
  // Forget the destination: after a release the Arduino is not holding any
  // particular opening, so the next send must go out even if it repeats.
  lastSent_ = -1;
  driving_ = false;
}

}  // namespace fw
