#pragma once

// Writing protobuf messages to an MCAP file by topic: each topic's channel and
// each message type's schema (from schema_descriptors.cpp) are added the
// first time they're used.

#include <mcap/writer.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "schema_descriptors.hpp"
#include "vg_core/sensor_data.hpp"

namespace vg::io::detail {

class McapChannels {
 public:
  explicit McapChannels(mcap::McapWriter& writer) : writer_(writer) {}

  // Writes one message; what names the file in errors, e.g. "session".
  void write(const std::string& topic, const std::string& schema_name, Timestamp timestamp,
             const std::string& data, const char* what) {
    if (timestamp < 0) {
      throw std::invalid_argument(std::string(what) + ": negative timestamp on " + topic);
    }
    mcap::Message message;
    message.channelId = channel(topic, schema_name, what);
    message.sequence = 0;
    message.logTime = static_cast<mcap::Timestamp>(timestamp);
    message.publishTime = message.logTime;
    message.data = reinterpret_cast<const std::byte*>(data.data());
    message.dataSize = data.size();
    const mcap::Status status = writer_.write(message);
    if (!status.ok()) {
      throw std::runtime_error(std::string(what) + ": writing " + topic + ": " + status.message);
    }
  }

 private:
  mcap::ChannelId channel(const std::string& topic, const std::string& schema_name,
                          const char* what) {
    if (auto it = channels_.find(topic); it != channels_.end()) {
      return it->second;
    }
    auto schema_it = schemas_.find(schema_name);
    if (schema_it == schemas_.end()) {
      const std::string_view descriptor = schema_descriptor(schema_name);
      if (descriptor.empty()) {
        throw std::logic_error(std::string(what) + ": no descriptor for " + schema_name);
      }
      mcap::Schema schema(schema_name, "protobuf", descriptor);
      writer_.addSchema(schema);
      schema_it = schemas_.emplace(schema_name, schema.id).first;
    }
    mcap::Channel ch(topic, "protobuf", schema_it->second);
    writer_.addChannel(ch);
    return channels_.emplace(topic, ch.id).first->second;
  }

  mcap::McapWriter& writer_;
  std::unordered_map<std::string, mcap::SchemaId> schemas_;
  std::unordered_map<std::string, mcap::ChannelId> channels_;
};

}  // namespace vg::io::detail
