#include "serial_console.hpp"

#include "driver/uart.h"
#include "esp_timer.h"

#include "hardware/serial_protocol.hpp"

namespace fw {

namespace {

constexpr uart_port_t kPort = UART_NUM_0;
constexpr size_t kMaxLineLength = 200;

}  // namespace

void SerialConsole::init(const rt::HardwareConfig& hw, MotionController& controller) {
  controller_ = &controller;
  firmwareName_ = hw.name;
  telemetryHz_ = hw.telemetryHzDefault;

  uart_config_t cfg = {};
  cfg.baud_rate = hw.serialBaud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(kPort, 1024, 1024, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(kPort, &cfg));
}

void SerialConsole::run() {
  writeLine(rt::proto::formatEvent("BOOT", firmwareName_));

  std::string line;
  uint8_t rx[64];
  for (;;) {
    // Short read timeout so telemetry and events stay responsive while idle.
    const int n = uart_read_bytes(kPort, rx, sizeof rx, pdMS_TO_TICKS(10));
    for (int i = 0; i < n; ++i) {
      const char c = static_cast<char>(rx[i]);
      if (c == '\n' || c == '\r') {
        if (!line.empty()) {
          writeLine(handleLine(line));
          line.clear();
        }
      } else if (line.size() < kMaxLineLength) {
        line.push_back(c);
      }
    }

    Event ev;
    while (controller_->popEvent(ev)) writeLine(rt::proto::formatEvent(ev.name, ev.detail));

    if (telemetryHz_ > 0) {
      const int64_t now = esp_timer_get_time();
      if (now >= nextTelemetryUs_) {
        writeLine(rt::proto::formatState(controller_->state()));
        nextTelemetryUs_ = now + static_cast<int64_t>(1e6 / telemetryHz_);
      }
    }
  }
}

std::string SerialConsole::handleLine(const std::string& line) {
  namespace proto = rt::proto;

  const auto parsed = proto::parseCommandLine(line);
  if (!parsed.ok) return proto::formatError(proto::reason::kParse, parsed.error);

  const auto& cmd = parsed.command;
  using CT = proto::CommandType;

  // Query verbs are answered here; everything else goes to the controller.
  switch (cmd.type) {
    case CT::Ping: return proto::formatPong(firmwareName_);
    case CT::State: return proto::formatState(controller_->state());
    case CT::SetTelemetry:
      telemetryHz_ = cmd.value;
      nextTelemetryUs_ = 0;  // emit the first ST immediately
      return proto::formatOk(proto::verbName(cmd.type));
    default: break;
  }

  CmdResult r;
  switch (cmd.type) {
    case CT::Home: r = controller_->home(); break;
    case CT::Stop: r = controller_->stop(); break;
    case CT::Enable: r = controller_->enable(); break;
    case CT::Disable: r = controller_->disable(); break;
    case CT::MoveJoints: r = controller_->moveJoints(cmd.q); break;
    case CT::MoveLinear: r = controller_->moveLinear(cmd.target); break;
    case CT::SetHome: r = controller_->setHome(cmd.q); break;
    case CT::SetPayload: r = controller_->setPayload(cmd.value); break;
    default: return proto::formatError(proto::reason::kParse, "unhandled command");
  }

  const char* verb = proto::verbName(cmd.type);
  if (!r.ok) return proto::formatError(r.reason, r.detail);
  if (r.isMove) return proto::formatMoveAck(verb, r.duration, r.stretch);
  return proto::formatOk(verb);
}

void SerialConsole::writeLine(const std::string& line) {
  uart_write_bytes(kPort, line.data(), line.size());
  uart_write_bytes(kPort, "\n", 1);
}

}  // namespace fw
