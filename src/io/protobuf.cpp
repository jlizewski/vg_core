#include "protobuf.hpp"

#include <cstring>
#include <stdexcept>

namespace vg::io::detail {
namespace {

constexpr std::int64_t kNanosPerSecond = 1'000'000'000;

template <typename T>
void append_raw(std::string& out, T value) {
  char bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  out.append(bytes, sizeof(T));
}

}  // namespace

void ProtoWriter::varint(std::uint64_t value) {
  while (value >= 0x80) {
    data_.push_back(static_cast<char>((value & 0x7f) | 0x80));
    value >>= 7;
  }
  data_.push_back(static_cast<char>(value));
}

void ProtoWriter::tag(std::uint32_t field, WireType type) {
  varint((static_cast<std::uint64_t>(field) << 3) | static_cast<std::uint64_t>(type));
}

void ProtoWriter::varint_field(std::uint32_t field, std::uint64_t value) {
  tag(field, WireType::Varint);
  varint(value);
}

void ProtoWriter::int64_field(std::uint32_t field, std::int64_t value) {
  varint_field(field, static_cast<std::uint64_t>(value));
}

void ProtoWriter::double_field(std::uint32_t field, double value) {
  tag(field, WireType::Fixed64);
  append_raw(data_, value);
}

void ProtoWriter::fixed32_field(std::uint32_t field, std::uint32_t value) {
  tag(field, WireType::Fixed32);
  append_raw(data_, value);
}

void ProtoWriter::bytes_field(std::uint32_t field, std::string_view value) {
  tag(field, WireType::Bytes);
  varint(value.size());
  data_.append(value.data(), value.size());
}

void ProtoWriter::message_field(std::uint32_t field, const ProtoWriter& message) {
  bytes_field(field, message.data());
}

void ProtoWriter::packed_doubles(std::uint32_t field, const double* values, std::size_t count) {
  if (count == 0) {
    return;
  }
  tag(field, WireType::Bytes);
  varint(count * sizeof(double));
  for (std::size_t i = 0; i < count; ++i) {
    append_raw(data_, values[i]);
  }
}

void ProtoWriter::timestamp_field(std::uint32_t field, std::int64_t nanoseconds) {
  std::int64_t seconds = nanoseconds / kNanosPerSecond;
  std::int64_t nanos = nanoseconds % kNanosPerSecond;
  if (nanos < 0) {  // Timestamp nanos must be non-negative.
    nanos += kNanosPerSecond;
    seconds -= 1;
  }
  ProtoWriter timestamp;
  timestamp.int64_field(1, seconds);
  timestamp.int64_field(2, nanos);
  message_field(field, timestamp);
}

double ProtoField::as_double() const {
  double result = 0.0;
  std::memcpy(&result, &value, sizeof(double));
  return result;
}

void ProtoField::append_doubles(std::vector<double>& out) const {
  if (type == WireType::Fixed64) {
    out.push_back(as_double());
    return;
  }
  if (type != WireType::Bytes || bytes.size() % sizeof(double) != 0) {
    throw std::runtime_error("protobuf: bad repeated double field");
  }
  for (std::size_t i = 0; i < bytes.size(); i += sizeof(double)) {
    double v = 0.0;
    std::memcpy(&v, bytes.data() + i, sizeof(double));
    out.push_back(v);
  }
}

std::uint64_t ProtoReader::varint() {
  std::uint64_t result = 0;
  for (unsigned shift = 0; shift < 64; shift += 7) {
    if (pos_ >= data_.size()) {
      throw std::runtime_error("protobuf: truncated varint");
    }
    const auto byte = static_cast<std::uint8_t>(data_[pos_++]);
    result |= static_cast<std::uint64_t>(byte & 0x7f) << shift;
    if ((byte & 0x80) == 0) {
      return result;
    }
  }
  throw std::runtime_error("protobuf: varint too long");
}

bool ProtoReader::next(ProtoField& field) {
  if (pos_ >= data_.size()) {
    return false;
  }
  const std::uint64_t key = varint();
  field.number = static_cast<std::uint32_t>(key >> 3);
  field.bytes = {};
  field.value = 0;
  const auto type = static_cast<std::uint8_t>(key & 0x7);
  auto take = [this](std::size_t n) {
    if (n > data_.size() - pos_) {
      throw std::runtime_error("protobuf: truncated field");
    }
    const std::string_view out = data_.substr(pos_, n);
    pos_ += n;
    return out;
  };
  switch (type) {
    case 0:
      field.type = WireType::Varint;
      field.value = varint();
      break;
    case 1: {
      field.type = WireType::Fixed64;
      std::memcpy(&field.value, take(8).data(), 8);
      break;
    }
    case 2:
      field.type = WireType::Bytes;
      field.bytes = take(static_cast<std::size_t>(varint()));
      break;
    case 5: {
      field.type = WireType::Fixed32;
      std::uint32_t v = 0;
      std::memcpy(&v, take(4).data(), 4);
      field.value = v;
      break;
    }
    default:
      throw std::runtime_error("protobuf: unsupported wire type");
  }
  return true;
}

std::int64_t parse_timestamp(std::string_view message) {
  std::int64_t seconds = 0;
  std::int64_t nanos = 0;
  ProtoReader reader(message);
  ProtoField f;
  while (reader.next(f)) {
    if (f.number == 1) {
      seconds = f.as_int64();
    } else if (f.number == 2) {
      nanos = f.as_int64();
    }
  }
  return seconds * kNanosPerSecond + nanos;
}

}  // namespace vg::io::detail
