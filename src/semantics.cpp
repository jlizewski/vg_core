#include "vg_core/semantics.hpp"

namespace vg {
namespace {

struct ClassInfo {
  std::string_view name;
  std::array<std::uint8_t, 3> color;
};

constexpr std::array<ClassInfo, kSemanticClasses> kClasses = {{
    {"unknown", {128, 128, 128}},
    {"ground", {150, 110, 60}},
    {"plant", {90, 200, 60}},
    {"tree", {20, 110, 30}},
    {"structure", {200, 60, 50}},
    {"sky", {110, 170, 240}},
    {"dynamic", {240, 200, 0}},
}};

}  // namespace

std::string_view semantic_class_name(SemanticClass c) {
  const auto i = static_cast<std::size_t>(c);
  return i < kClasses.size() ? kClasses[i].name : "unknown";
}

std::optional<SemanticClass> parse_semantic_class(std::string_view name) {
  for (std::size_t i = 0; i < kClasses.size(); ++i) {
    if (kClasses[i].name == name) {
      return static_cast<SemanticClass>(i);
    }
  }
  return std::nullopt;
}

std::array<std::uint8_t, 3> semantic_class_color(SemanticClass c) {
  const auto i = static_cast<std::size_t>(c);
  return i < kClasses.size() ? kClasses[i].color : kClasses[0].color;
}

}  // namespace vg
