#pragma once

// The semantic classes the map knows. A segmentation model (on the phone, or
// offline) labels each depth pixel with one of these; fusion counts the
// labels each voxel was seen with, and the stages ask the map by class (the
// ground stage for ground, a structures stage for structure, ...).
//
// The set is small on purpose: what mapping and sun modeling need to tell
// apart, not everything a model can name. A model's own classes (e.g.
// ADE20K's 150) are mapped onto these where it runs.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace vg {

enum class SemanticClass : std::uint8_t {
  Unknown = 0,    // Not labeled, or nothing the model knows.
  Ground = 1,     // Lawn, soil, mulch, paths: the terrain surface.
  Plant = 2,      // Plants, flowers, shrubs: low vegetation.
  Tree = 3,       // Trunks and canopy.
  Structure = 4,  // House, walls, fences, decks, sheds.
  Sky = 5,
  Dynamic = 6,    // People and animals: not part of the map.
};

inline constexpr std::size_t kSemanticClasses = 7;

// Lower-case name ("ground"), as used in settings files.
std::string_view semantic_class_name(SemanticClass c);
std::optional<SemanticClass> parse_semantic_class(std::string_view name);

// Color for viewing, e.g. in labels.ply.
std::array<std::uint8_t, 3> semantic_class_color(SemanticClass c);

// A set of classes as a bit mask (bit i is class i).
using SemanticMask = std::uint32_t;
inline constexpr SemanticMask mask_of(SemanticClass c) {
  return SemanticMask{1} << static_cast<unsigned>(c);
}

}  // namespace vg
