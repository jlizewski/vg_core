#include "../src/io/protobuf.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace {

using vg::io::detail::ProtoField;
using vg::io::detail::ProtoReader;
using vg::io::detail::ProtoWriter;
using vg::io::detail::WireType;

TEST(Protobuf, EncodesKnownBytes) {
  // Examples from the protobuf encoding guide.
  ProtoWriter w;
  w.varint_field(1, 150);
  w.bytes_field(2, "testing");
  EXPECT_EQ(w.data(), std::string("\x08\x96\x01\x12\x07testing", 12));
}

TEST(Protobuf, RoundTripsFields) {
  const double values[3] = {1.5, -2.25, 1e-9};
  ProtoWriter nested;
  nested.double_field(1, 3.0);
  ProtoWriter w;
  w.int64_field(1, -5);
  w.double_field(2, 0.1);
  w.fixed32_field(3, 640);
  w.message_field(4, nested);
  w.packed_doubles(5, values, 3);
  w.timestamp_field(6, 12'345'678'901);

  ProtoReader r(w.data());
  ProtoField f;
  ASSERT_TRUE(r.next(f));
  EXPECT_EQ(f.as_int64(), -5);
  ASSERT_TRUE(r.next(f));
  EXPECT_DOUBLE_EQ(f.as_double(), 0.1);
  ASSERT_TRUE(r.next(f));
  EXPECT_EQ(f.type, WireType::Fixed32);
  EXPECT_EQ(f.as_uint32(), 640u);
  ASSERT_TRUE(r.next(f));
  EXPECT_EQ(f.bytes, nested.data());
  ASSERT_TRUE(r.next(f));
  std::vector<double> decoded;
  f.append_doubles(decoded);
  EXPECT_EQ(decoded, std::vector<double>(values, values + 3));
  ASSERT_TRUE(r.next(f));
  EXPECT_EQ(vg::io::detail::parse_timestamp(f.bytes), 12'345'678'901);
  EXPECT_FALSE(r.next(f));
}

TEST(Protobuf, NegativeTimestampKeepsNanosPositive) {
  ProtoWriter w;
  w.timestamp_field(1, -1);
  ProtoReader r(w.data());
  ProtoField f;
  ASSERT_TRUE(r.next(f));
  EXPECT_EQ(vg::io::detail::parse_timestamp(f.bytes), -1);
}

TEST(Protobuf, RejectsTruncatedInput) {
  ProtoWriter w;
  w.bytes_field(1, "hello");
  ProtoReader r(std::string_view(w.data()).substr(0, 4));
  ProtoField f;
  EXPECT_THROW(r.next(f), std::runtime_error);
}

}  // namespace
