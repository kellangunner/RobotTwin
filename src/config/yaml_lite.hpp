// Minimal YAML-subset reader for config/robot.yaml — deliberately NOT a
// general YAML parser. Supports exactly what the robot config uses:
//   - nested maps by 2-space indentation
//   - scalar values (numbers, plain strings, quoted strings)
//   - inline flat lists of numbers: [a, b]
//   - '#' comments and blank lines
// Anything fancier (anchors, multi-line strings, nested lists) is rejected.
// Native builds parse the YAML here; WASM builds receive parsed values from
// the JavaScript side and never compile this file.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace rt::yaml {

struct Node {
  std::map<std::string, Node> children;
  std::string scalar;
  bool isScalar = false;

  /** Child access; throws std::runtime_error on a missing key. */
  const Node& at(const std::string& key) const;
  bool has(const std::string& key) const { return children.count(key) > 0; }

  double asDouble() const;
  std::string asString() const;
  std::vector<double> asDoubleList() const;
};

/** Parse YAML text into a tree. Throws std::runtime_error on malformed input. */
Node parse(const std::string& text);

/** Read a whole file into a string. Throws std::runtime_error on failure. */
std::string readFile(const std::string& path);

} // namespace rt::yaml
