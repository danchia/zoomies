// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/nav_msgs/OccupancyGrid.proto

#include "ros/nav_msgs/OccupancyGrid.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace ros {
namespace nav_msgs {
constexpr OccupancyGrid::OccupancyGrid(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : data_()
  , _data_cached_byte_size_(0)
  , header_(nullptr)
  , info_(nullptr){}
struct OccupancyGridDefaultTypeInternal {
  constexpr OccupancyGridDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~OccupancyGridDefaultTypeInternal() {}
  union {
    OccupancyGrid _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT OccupancyGridDefaultTypeInternal _OccupancyGrid_default_instance_;
}  // namespace nav_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto = nullptr;

const uint32_t TableStruct_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::OccupancyGrid, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::OccupancyGrid, header_),
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::OccupancyGrid, info_),
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::OccupancyGrid, data_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::nav_msgs::OccupancyGrid)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::nav_msgs::_OccupancyGrid_default_instance_),
};

const char descriptor_table_protodef_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n ros/nav_msgs/OccupancyGrid.proto\022\014ros."
  "nav_msgs\032\036ros/nav_msgs/MapMetaData.proto"
  "\032\031ros/std_msgs/Header.proto\"l\n\rOccupancy"
  "Grid\022$\n\006header\030\001 \001(\0132\024.ros.std_msgs.Head"
  "er\022\'\n\004info\030\002 \001(\0132\031.ros.nav_msgs.MapMetaD"
  "ata\022\014\n\004data\030\003 \003(\021b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto_deps[2] = {
  &::descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto = {
  false, false, 225, descriptor_table_protodef_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto, "ros/nav_msgs/OccupancyGrid.proto", 
  &descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto_once, descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto::offsets,
  file_level_metadata_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto, file_level_enum_descriptors_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto, file_level_service_descriptors_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto_getter() {
  return &descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto(&descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto);
namespace ros {
namespace nav_msgs {

// ===================================================================

class OccupancyGrid::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const OccupancyGrid* msg);
  static const ::ros::nav_msgs::MapMetaData& info(const OccupancyGrid* msg);
};

const ::ros::std_msgs::Header&
OccupancyGrid::_Internal::header(const OccupancyGrid* msg) {
  return *msg->header_;
}
const ::ros::nav_msgs::MapMetaData&
OccupancyGrid::_Internal::info(const OccupancyGrid* msg) {
  return *msg->info_;
}
void OccupancyGrid::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void OccupancyGrid::clear_info() {
  if (GetArenaForAllocation() == nullptr && info_ != nullptr) {
    delete info_;
  }
  info_ = nullptr;
}
OccupancyGrid::OccupancyGrid(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  data_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.nav_msgs.OccupancyGrid)
}
OccupancyGrid::OccupancyGrid(const OccupancyGrid& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      data_(from.data_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_info()) {
    info_ = new ::ros::nav_msgs::MapMetaData(*from.info_);
  } else {
    info_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.nav_msgs.OccupancyGrid)
}

inline void OccupancyGrid::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&info_) -
    reinterpret_cast<char*>(&header_)) + sizeof(info_));
}

OccupancyGrid::~OccupancyGrid() {
  // @@protoc_insertion_point(destructor:ros.nav_msgs.OccupancyGrid)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void OccupancyGrid::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete info_;
}

void OccupancyGrid::ArenaDtor(void* object) {
  OccupancyGrid* _this = reinterpret_cast< OccupancyGrid* >(object);
  (void)_this;
}
void OccupancyGrid::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void OccupancyGrid::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void OccupancyGrid::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.nav_msgs.OccupancyGrid)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  if (GetArenaForAllocation() == nullptr && info_ != nullptr) {
    delete info_;
  }
  info_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* OccupancyGrid::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .ros.std_msgs.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .ros.nav_msgs.MapMetaData info = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_info(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated sint32 data = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedSInt32Parser(_internal_mutable_data(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 24) {
          _internal_add_data(::PROTOBUF_NAMESPACE_ID::internal::ReadVarintZigZag32(&ptr));
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* OccupancyGrid::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.nav_msgs.OccupancyGrid)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // .ros.nav_msgs.MapMetaData info = 2;
  if (this->_internal_has_info()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::info(this), target, stream);
  }

  // repeated sint32 data = 3;
  {
    int byte_size = _data_cached_byte_size_.load(std::memory_order_relaxed);
    if (byte_size > 0) {
      target = stream->WriteSInt32Packed(
          3, _internal_data(), byte_size, target);
    }
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.nav_msgs.OccupancyGrid)
  return target;
}

size_t OccupancyGrid::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.nav_msgs.OccupancyGrid)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated sint32 data = 3;
  {
    size_t data_size = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      SInt32Size(this->data_);
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<int32_t>(data_size));
    }
    int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(data_size);
    _data_cached_byte_size_.store(cached_size,
                                    std::memory_order_relaxed);
    total_size += data_size;
  }

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // .ros.nav_msgs.MapMetaData info = 2;
  if (this->_internal_has_info()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *info_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData OccupancyGrid::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    OccupancyGrid::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*OccupancyGrid::GetClassData() const { return &_class_data_; }

void OccupancyGrid::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<OccupancyGrid *>(to)->MergeFrom(
      static_cast<const OccupancyGrid &>(from));
}


void OccupancyGrid::MergeFrom(const OccupancyGrid& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.nav_msgs.OccupancyGrid)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  if (from._internal_has_info()) {
    _internal_mutable_info()->::ros::nav_msgs::MapMetaData::MergeFrom(from._internal_info());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void OccupancyGrid::CopyFrom(const OccupancyGrid& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.nav_msgs.OccupancyGrid)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool OccupancyGrid::IsInitialized() const {
  return true;
}

void OccupancyGrid::InternalSwap(OccupancyGrid* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  data_.InternalSwap(&other->data_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(OccupancyGrid, info_)
      + sizeof(OccupancyGrid::info_)
      - PROTOBUF_FIELD_OFFSET(OccupancyGrid, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata OccupancyGrid::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto_getter, &descriptor_table_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto_once,
      file_level_metadata_ros_2fnav_5fmsgs_2fOccupancyGrid_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace nav_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::nav_msgs::OccupancyGrid* Arena::CreateMaybeMessage< ::ros::nav_msgs::OccupancyGrid >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::nav_msgs::OccupancyGrid >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>