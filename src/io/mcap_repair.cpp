#include "vg_core/io/mcap_repair.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
#include <stdexcept>
#include <unordered_map>

namespace vg::io {

McapRepairResult repair_mcap(const std::filesystem::path& input,
                             const std::filesystem::path& output) {
  std::error_code ec;
  if (std::filesystem::equivalent(input, output, ec)) {
    throw std::runtime_error("mcap repair: output " + output.string() + " is the input file");
  }

  std::ifstream file(input, std::ios::binary);
  if (!file) {
    throw std::runtime_error("mcap repair: cannot open " + input.string());
  }
  mcap::FileStreamReader source(file);
  std::byte* magic = nullptr;
  if (source.read(&magic, 0, sizeof(mcap::Magic)) != sizeof(mcap::Magic) ||
      std::memcmp(magic, mcap::Magic, sizeof(mcap::Magic)) != 0) {
    throw std::runtime_error("mcap repair: " + input.string() + " is not an MCAP file");
  }

  mcap::McapWriter writer;
  mcap::McapWriterOptions options("");
  options.compression = mcap::Compression::Zstd;
  bool writer_open = false;
  McapRepairResult result;
  // The input's schema and channel ids, mapped to the output's.
  std::unordered_map<mcap::SchemaId, mcap::SchemaId> schemas;
  std::unordered_map<mcap::ChannelId, mcap::ChannelId> channels;
  std::unordered_map<mcap::ChannelId, mcap::ChannelPtr> pending_channels;
  mcap::Status write_status;

  auto open_writer = [&] {
    if (!writer_open) {
      const mcap::Status status = writer.open(output.string(), options);
      if (!status.ok()) {
        throw std::runtime_error("mcap repair: cannot write " + output.string() + ": " +
                                 status.message);
      }
      writer_open = true;
    }
  };
  auto keep = [&](const mcap::Status& status) {
    if (!status.ok() && write_status.ok()) {
      write_status = status;
    }
  };

  mcap::TypedRecordReader reader(source, sizeof(mcap::Magic));
  reader.onHeader = [&](const mcap::Header& header, mcap::ByteOffset) {
    options.profile = header.profile;
    open_writer();
  };
  reader.onSchema = [&](const mcap::SchemaPtr schema, mcap::ByteOffset,
                        std::optional<mcap::ByteOffset>) {
    open_writer();
    if (schemas.count(schema->id) == 0) {
      mcap::Schema copy = *schema;
      writer.addSchema(copy);
      schemas[schema->id] = copy.id;
    }
  };
  // Channels are added when their first message arrives, so channels that
  // never got a message (or whose messages were all lost) aren't copied.
  reader.onChannel = [&](const mcap::ChannelPtr channel, mcap::ByteOffset,
                         std::optional<mcap::ByteOffset>) {
    pending_channels.emplace(channel->id, channel);
  };
  reader.onMessage = [&](const mcap::Message& message, mcap::ByteOffset,
                         std::optional<mcap::ByteOffset>) {
    open_writer();
    auto it = channels.find(message.channelId);
    if (it == channels.end()) {
      auto pending = pending_channels.find(message.channelId);
      if (pending == pending_channels.end()) {
        return;  // Message on a channel that was never defined.
      }
      mcap::Channel copy = *pending->second;
      if (copy.schemaId != 0) {
        auto schema = schemas.find(copy.schemaId);
        if (schema == schemas.end()) {
          return;  // Channel whose schema was never defined.
        }
        copy.schemaId = schema->second;
      }
      writer.addChannel(copy);
      it = channels.emplace(message.channelId, copy.id).first;
      ++result.channels;
    }
    mcap::Message copy = message;
    copy.channelId = it->second;
    keep(writer.write(copy));
    if (result.messages == 0) {
      result.start_time = static_cast<Timestamp>(message.logTime);
      result.end_time = result.start_time;
    }
    result.start_time = std::min(result.start_time, static_cast<Timestamp>(message.logTime));
    result.end_time = std::max(result.end_time, static_cast<Timestamp>(message.logTime));
    ++result.messages;
  };
  reader.onMetadata = [&](const mcap::Metadata& metadata, mcap::ByteOffset) {
    open_writer();
    keep(writer.write(metadata));
    ++result.metadata;
  };
  reader.onAttachment = [&](const mcap::Attachment& attachment, mcap::ByteOffset) {
    open_writer();
    mcap::Attachment copy = attachment;
    keep(writer.write(copy));
    ++result.attachments;
  };
  // Data End is followed by the summary, which only repeats what was read.
  bool data_end = false;
  reader.onDataEnd = [&](const mcap::DataEnd&, mcap::ByteOffset) { data_end = true; };

  // Stop at the first damaged record: past it, records can't be trusted to
  // be what they appear to be.
  while (!data_end && write_status.ok() && reader.next() && reader.status().ok()) {
  }
  open_writer();
  writer.close();
  if (!write_status.ok()) {
    throw std::runtime_error("mcap repair: writing " + output.string() + ": " +
                             write_status.message);
  }
  if (!reader.status().ok()) {
    result.complete = false;
    result.problem = reader.status().message;
  }
  return result;
}

}  // namespace vg::io
