#include "yaml_lite.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rt::yaml {

namespace {

std::string rtrim(const std::string& s) {
  const auto e = s.find_last_not_of(" \t\r");
  return e == std::string::npos ? "" : s.substr(0, e + 1);
}

std::string ltrim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t");
  return b == std::string::npos ? "" : s.substr(b);
}

/** Strip a trailing comment that is not inside quotes. */
std::string stripComment(const std::string& s) {
  bool inQuote = false;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '"') inQuote = !inQuote;
    if (s[i] == '#' && !inQuote && (i == 0 || s[i - 1] == ' ' || s[i - 1] == '\t')) {
      return rtrim(s.substr(0, i));
    }
  }
  return rtrim(s);
}

std::string unquote(const std::string& s) {
  if (s.size() >= 2 && s.front() == '"' && s.back() == '"') return s.substr(1, s.size() - 2);
  return s;
}

} // namespace

const Node& Node::at(const std::string& key) const {
  const auto it = children.find(key);
  if (it == children.end()) throw std::runtime_error("yaml: missing key '" + key + "'");
  return it->second;
}

double Node::asDouble() const {
  if (!isScalar) throw std::runtime_error("yaml: expected scalar");
  size_t used = 0;
  const double v = std::stod(scalar, &used);
  if (used != scalar.size()) throw std::runtime_error("yaml: not a number: '" + scalar + "'");
  return v;
}

std::string Node::asString() const {
  if (!isScalar) throw std::runtime_error("yaml: expected scalar");
  return unquote(scalar);
}

std::vector<double> Node::asDoubleList() const {
  if (!isScalar || scalar.size() < 2 || scalar.front() != '[' || scalar.back() != ']') {
    throw std::runtime_error("yaml: expected inline list, got '" + scalar + "'");
  }
  std::vector<double> out;
  std::string body = scalar.substr(1, scalar.size() - 2);
  for (char& c : body) {
    if (c == ',') c = ' ';
  }
  std::istringstream ss(body);
  std::string tok;
  while (ss >> tok) {
    size_t used = 0;
    out.push_back(std::stod(tok, &used));
    if (used != tok.size()) throw std::runtime_error("yaml: bad list element '" + tok + "'");
  }
  return out;
}

Node parse(const std::string& text) {
  Node root;
  // stack of (indent, node) — nodes currently open for children
  std::vector<std::pair<int, Node*>> stack{{-1, &root}};

  std::istringstream lines(text);
  std::string raw;
  int lineNo = 0;
  while (std::getline(lines, raw)) {
    ++lineNo;
    const std::string noComment = stripComment(raw);
    if (ltrim(noComment).empty()) continue;

    const size_t indentChars = noComment.find_first_not_of(' ');
    if (noComment[indentChars] == '\t') {
      throw std::runtime_error("yaml: tabs not supported (line " + std::to_string(lineNo) + ")");
    }
    if (indentChars % 2 != 0) {
      throw std::runtime_error("yaml: odd indentation (line " + std::to_string(lineNo) + ")");
    }
    const int indent = static_cast<int>(indentChars / 2);

    const std::string body = noComment.substr(indentChars);
    if (body[0] == '-') {
      throw std::runtime_error("yaml: block lists not supported (line " +
                               std::to_string(lineNo) + ")");
    }
    const size_t colon = body.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error("yaml: expected 'key:' (line " + std::to_string(lineNo) + ")");
    }
    const std::string key = rtrim(body.substr(0, colon));
    const std::string value = ltrim(rtrim(body.substr(colon + 1)));

    while (stack.size() > 1 && stack.back().first >= indent) stack.pop_back();
    Node& parent = *stack.back().second;
    Node& node = parent.children[key];
    if (value.empty()) {
      stack.emplace_back(indent, &node); // map node: children follow
    } else {
      node.scalar = value;
      node.isScalar = true;
    }
  }
  return root;
}

std::string readFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open file: " + path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

} // namespace rt::yaml
