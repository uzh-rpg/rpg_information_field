#pragma once

#include <fstream>
#include <glog/logging.h>
#include <google/protobuf/message.h>
#include <google/protobuf/message_lite.h>

namespace act_map {

namespace voxblox {

namespace utils {
bool readProtoMsgCountToStream(std::fstream* stream_in, uint32_t* message_count,
                               uint32_t* byte_offset);

bool writeProtoMsgCountToStream(uint32_t message_count,
                                std::fstream* stream_out);

bool readProtoMsgFromStream(std::fstream* stream_in,
                            google::protobuf::Message* message,
                            uint32_t* byte_offset);

bool writeProtoMsgToStream(const google::protobuf::Message& message,
                           std::fstream* stream_out);

}  // namespace utils
}  // namespace voxblox
}  // namespace act_map

