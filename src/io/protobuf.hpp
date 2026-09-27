#pragma once

// Minimal protobuf wire-format encoding and decoding, enough for the handful of
// capture-format messages. Avoids a libprotobuf dependency on device.
// Assumes a little-endian host (all supported platforms are).

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace vg::io::detail {

enum class WireType : std::uint8_t { Varint = 0, Fixed64 = 1, Bytes = 2, Fixed32 = 5 };

class ProtoWriter {
 public:
  void varint_field(std::uint32_t field, std::uint64_t value);
  void int64_field(std::uint32_t field, std::int64_t value);
  void double_field(std::uint32_t field, double value);
  void fixed32_field(std::uint32_t field, std::uint32_t value);
  void bytes_field(std::uint32_t field, std::string_view value);
  void message_field(std::uint32_t field, const ProtoWriter& message);
  void packed_doubles(std::uint32_t field, const double* values, std::size_t count);

  // google.protobuf.Timestamp from nanoseconds.
  void timestamp_field(std::uint32_t field, std::int64_t nanoseconds);

  const std::string& data() const { return data_; }

 private:
  void tag(std::uint32_t field, WireType type);
  void varint(std::uint64_t value);
  std::string data_;
};

struct ProtoField {
  std::uint32_t number = 0;
  WireType type = WireType::Varint;
  std::uint64_t value = 0;  // Varint, Fixed64 and Fixed32 payloads.
  std::string_view bytes;   // Bytes payload.

  double as_double() const;
  std::int64_t as_int64() const { return static_cast<std::int64_t>(value); }
  std::uint32_t as_uint32() const { return static_cast<std::uint32_t>(value); }
  // Accepts packed (Bytes) or a single unpacked (Fixed64) value.
  void append_doubles(std::vector<double>& out) const;
};

// Iterates the fields of one encoded message. Throws std::runtime_error on
// malformed input.
class ProtoReader {
 public:
  explicit ProtoReader(std::string_view data) : data_(data) {}
  bool next(ProtoField& field);

 private:
  std::uint64_t varint();
  std::string_view data_;
  std::size_t pos_ = 0;
};

// Decodes a google.protobuf.Timestamp to nanoseconds.
std::int64_t parse_timestamp(std::string_view message);

}  // namespace vg::io::detail
