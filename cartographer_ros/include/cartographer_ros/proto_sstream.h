#ifndef CARTOGRAPHER_ROS_IO_PROTO_STRINGSTREAM_H_
#define CARTOGRAPHER_ROS_IO_PROTO_STRINGSTREAM_H_

#include <fstream>
#include <sstream>

#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream_interface.h"
#include "google/protobuf/message.h"

namespace cartographer
{
namespace io
{

class ProtoSStreamWriter : public ProtoStreamWriterInterface
{
  public:
    explicit ProtoSStreamWriter(std::ostream& stream);
    ~ProtoSStreamWriter() = default;

    ProtoSStreamWriter(const ProtoSStreamWriter&) = delete;
    ProtoSStreamWriter& operator=(const ProtoSStreamWriter&) = delete;

    void WriteProto(const google::protobuf::Message& proto) override;
    bool Close() override;

  private:
    void Write(const std::string& uncompressed_data);

    std::ostream& out_;
};

// A reader of the format produced by ProtoStreamWriter.
class ProtoSStreamReader : public ProtoStreamReaderInterface
{
  public:
    explicit ProtoSStreamReader(std::istream& stream);
    ~ProtoSStreamReader() = default;

    ProtoSStreamReader(const ProtoSStreamReader&) = delete;
    ProtoSStreamReader& operator=(const ProtoSStreamReader&) = delete;

    bool ReadProto(google::protobuf::Message* proto) override;
    bool eof() const override;

  private:
    bool Read(std::string* decompressed_data);

    std::istream& in_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_ROS_IO_PROTO_STRINGSTREAM_H_
