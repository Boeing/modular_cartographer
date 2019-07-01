#include "cartographer_ros/proto_sstream.h"
#include "glog/logging.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>

namespace cartographer
{
namespace io
{

namespace
{

// First eight bytes to identify our proto stream format.
const uint64 kMagic = 0x7b1d1f7b5bf501db;

void WriteSizeAsLittleEndian(uint64 size, std::ostream* out)
{
    for (int i = 0; i != 8; ++i)
    {
        out->put(size & 0xff);
        size >>= 8;
    }
}

bool ReadSizeAsLittleEndian(std::istream* in, uint64* size)
{
    *size = 0;
    for (int i = 0; i != 8; ++i)
    {
        *size >>= 8;
        *size += static_cast<uint64>(in->get()) << 56;
    }
    return !in->fail();
}

}  // namespace

ProtoSStreamWriter::ProtoSStreamWriter(std::ostream& stream) : out_(stream)
{
    WriteSizeAsLittleEndian(kMagic, &out_);
}

void ProtoSStreamWriter::Write(const std::string& uncompressed_data)
{
    std::string compressed_data;
    common::FastGzipString(uncompressed_data, &compressed_data);
    WriteSizeAsLittleEndian(compressed_data.size(), &out_);
    out_.write(compressed_data.data(), compressed_data.size());
}

void ProtoSStreamWriter::WriteProto(const google::protobuf::Message& proto)
{
    std::string uncompressed_data;
    proto.SerializeToString(&uncompressed_data);
    Write(uncompressed_data);
    //  LOG(INFO) << "full data size: " << uncompressed_data.size();
}

bool ProtoSStreamWriter::Close()
{
    return true;
}

ProtoSStreamReader::ProtoSStreamReader(std::istream& stream) : in_(stream)
{
    uint64 magic;
    if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic)
    {
        in_.setstate(std::ios::failbit);
    }
    if (!in_.good())
    {
        throw std::runtime_error("Failed to read pbstream");
    }
}

bool ProtoSStreamReader::Read(std::string* decompressed_data)
{
    uint64 compressed_size;
    if (!ReadSizeAsLittleEndian(&in_, &compressed_size))
    {
        return false;
    }
    std::string compressed_data(compressed_size, '\0');
    if (!in_.read(&compressed_data.front(), compressed_size))
    {
        return false;
    }
    common::FastGunzipString(compressed_data, decompressed_data);
    //  LOG(INFO) << "reading: " << compressed_size << " uncompressed: " << decompressed_data->size();
    return true;
}

bool ProtoSStreamReader::ReadProto(google::protobuf::Message* proto)
{
    std::string decompressed_data;
    return Read(&decompressed_data) && proto->ParseFromString(decompressed_data);
}

bool ProtoSStreamReader::eof() const
{
    return in_.eof();
}

}  // namespace io
}  // namespace cartographer
