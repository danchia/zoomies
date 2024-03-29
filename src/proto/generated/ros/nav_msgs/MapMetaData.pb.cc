// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/nav_msgs/MapMetaData.proto

#include "ros/nav_msgs/MapMetaData.pb.h"

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
constexpr MapMetaData::MapMetaData(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : map_load_time_(nullptr)
  , origin_(nullptr)
  , resolution_(0)
  , width_(0u)
  , height_(0u){}
struct MapMetaDataDefaultTypeInternal {
  constexpr MapMetaDataDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~MapMetaDataDefaultTypeInternal() {}
  union {
    MapMetaData _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT MapMetaDataDefaultTypeInternal _MapMetaData_default_instance_;
}  // namespace nav_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fnav_5fmsgs_2fMapMetaData_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fnav_5fmsgs_2fMapMetaData_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fnav_5fmsgs_2fMapMetaData_2eproto = nullptr;

const uint32_t TableStruct_ros_2fnav_5fmsgs_2fMapMetaData_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::MapMetaData, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::MapMetaData, map_load_time_),
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::MapMetaData, resolution_),
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::MapMetaData, width_),
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::MapMetaData, height_),
  PROTOBUF_FIELD_OFFSET(::ros::nav_msgs::MapMetaData, origin_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::nav_msgs::MapMetaData)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::nav_msgs::_MapMetaData_default_instance_),
};

const char descriptor_table_protodef_ros_2fnav_5fmsgs_2fMapMetaData_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\036ros/nav_msgs/MapMetaData.proto\022\014ros.na"
  "v_msgs\032\022ros/builtins.proto\032\034ros/geometry"
  "_msgs/Pose.proto\"\213\001\n\013MapMetaData\022 \n\rmap_"
  "load_time\030\001 \001(\0132\t.ros.Time\022\022\n\nresolution"
  "\030\002 \001(\002\022\r\n\005width\030\003 \001(\r\022\016\n\006height\030\004 \001(\r\022\'\n"
  "\006origin\030\005 \001(\0132\027.ros.geometry_msgs.Poseb\006"
  "proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto_deps[2] = {
  &::descriptor_table_ros_2fbuiltins_2eproto,
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto = {
  false, false, 246, descriptor_table_protodef_ros_2fnav_5fmsgs_2fMapMetaData_2eproto, "ros/nav_msgs/MapMetaData.proto", 
  &descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto_once, descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fnav_5fmsgs_2fMapMetaData_2eproto::offsets,
  file_level_metadata_ros_2fnav_5fmsgs_2fMapMetaData_2eproto, file_level_enum_descriptors_ros_2fnav_5fmsgs_2fMapMetaData_2eproto, file_level_service_descriptors_ros_2fnav_5fmsgs_2fMapMetaData_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto_getter() {
  return &descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fnav_5fmsgs_2fMapMetaData_2eproto(&descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto);
namespace ros {
namespace nav_msgs {

// ===================================================================

class MapMetaData::_Internal {
 public:
  static const ::ros::Time& map_load_time(const MapMetaData* msg);
  static const ::ros::geometry_msgs::Pose& origin(const MapMetaData* msg);
};

const ::ros::Time&
MapMetaData::_Internal::map_load_time(const MapMetaData* msg) {
  return *msg->map_load_time_;
}
const ::ros::geometry_msgs::Pose&
MapMetaData::_Internal::origin(const MapMetaData* msg) {
  return *msg->origin_;
}
void MapMetaData::clear_map_load_time() {
  if (GetArenaForAllocation() == nullptr && map_load_time_ != nullptr) {
    delete map_load_time_;
  }
  map_load_time_ = nullptr;
}
void MapMetaData::clear_origin() {
  if (GetArenaForAllocation() == nullptr && origin_ != nullptr) {
    delete origin_;
  }
  origin_ = nullptr;
}
MapMetaData::MapMetaData(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.nav_msgs.MapMetaData)
}
MapMetaData::MapMetaData(const MapMetaData& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_map_load_time()) {
    map_load_time_ = new ::ros::Time(*from.map_load_time_);
  } else {
    map_load_time_ = nullptr;
  }
  if (from._internal_has_origin()) {
    origin_ = new ::ros::geometry_msgs::Pose(*from.origin_);
  } else {
    origin_ = nullptr;
  }
  ::memcpy(&resolution_, &from.resolution_,
    static_cast<size_t>(reinterpret_cast<char*>(&height_) -
    reinterpret_cast<char*>(&resolution_)) + sizeof(height_));
  // @@protoc_insertion_point(copy_constructor:ros.nav_msgs.MapMetaData)
}

inline void MapMetaData::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&map_load_time_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&height_) -
    reinterpret_cast<char*>(&map_load_time_)) + sizeof(height_));
}

MapMetaData::~MapMetaData() {
  // @@protoc_insertion_point(destructor:ros.nav_msgs.MapMetaData)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void MapMetaData::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete map_load_time_;
  if (this != internal_default_instance()) delete origin_;
}

void MapMetaData::ArenaDtor(void* object) {
  MapMetaData* _this = reinterpret_cast< MapMetaData* >(object);
  (void)_this;
}
void MapMetaData::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void MapMetaData::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void MapMetaData::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.nav_msgs.MapMetaData)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && map_load_time_ != nullptr) {
    delete map_load_time_;
  }
  map_load_time_ = nullptr;
  if (GetArenaForAllocation() == nullptr && origin_ != nullptr) {
    delete origin_;
  }
  origin_ = nullptr;
  ::memset(&resolution_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&height_) -
      reinterpret_cast<char*>(&resolution_)) + sizeof(height_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* MapMetaData::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .ros.Time map_load_time = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_map_load_time(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // float resolution = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 21)) {
          resolution_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // uint32 width = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 24)) {
          width_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // uint32 height = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 32)) {
          height_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .ros.geometry_msgs.Pose origin = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 42)) {
          ptr = ctx->ParseMessage(_internal_mutable_origin(), ptr);
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

uint8_t* MapMetaData::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.nav_msgs.MapMetaData)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.Time map_load_time = 1;
  if (this->_internal_has_map_load_time()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::map_load_time(this), target, stream);
  }

  // float resolution = 2;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_resolution = this->_internal_resolution();
  uint32_t raw_resolution;
  memcpy(&raw_resolution, &tmp_resolution, sizeof(tmp_resolution));
  if (raw_resolution != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2, this->_internal_resolution(), target);
  }

  // uint32 width = 3;
  if (this->_internal_width() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3, this->_internal_width(), target);
  }

  // uint32 height = 4;
  if (this->_internal_height() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(4, this->_internal_height(), target);
  }

  // .ros.geometry_msgs.Pose origin = 5;
  if (this->_internal_has_origin()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        5, _Internal::origin(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.nav_msgs.MapMetaData)
  return target;
}

size_t MapMetaData::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.nav_msgs.MapMetaData)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .ros.Time map_load_time = 1;
  if (this->_internal_has_map_load_time()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *map_load_time_);
  }

  // .ros.geometry_msgs.Pose origin = 5;
  if (this->_internal_has_origin()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *origin_);
  }

  // float resolution = 2;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_resolution = this->_internal_resolution();
  uint32_t raw_resolution;
  memcpy(&raw_resolution, &tmp_resolution, sizeof(tmp_resolution));
  if (raw_resolution != 0) {
    total_size += 1 + 4;
  }

  // uint32 width = 3;
  if (this->_internal_width() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32SizePlusOne(this->_internal_width());
  }

  // uint32 height = 4;
  if (this->_internal_height() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32SizePlusOne(this->_internal_height());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData MapMetaData::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    MapMetaData::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*MapMetaData::GetClassData() const { return &_class_data_; }

void MapMetaData::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<MapMetaData *>(to)->MergeFrom(
      static_cast<const MapMetaData &>(from));
}


void MapMetaData::MergeFrom(const MapMetaData& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.nav_msgs.MapMetaData)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_map_load_time()) {
    _internal_mutable_map_load_time()->::ros::Time::MergeFrom(from._internal_map_load_time());
  }
  if (from._internal_has_origin()) {
    _internal_mutable_origin()->::ros::geometry_msgs::Pose::MergeFrom(from._internal_origin());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_resolution = from._internal_resolution();
  uint32_t raw_resolution;
  memcpy(&raw_resolution, &tmp_resolution, sizeof(tmp_resolution));
  if (raw_resolution != 0) {
    _internal_set_resolution(from._internal_resolution());
  }
  if (from._internal_width() != 0) {
    _internal_set_width(from._internal_width());
  }
  if (from._internal_height() != 0) {
    _internal_set_height(from._internal_height());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void MapMetaData::CopyFrom(const MapMetaData& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.nav_msgs.MapMetaData)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MapMetaData::IsInitialized() const {
  return true;
}

void MapMetaData::InternalSwap(MapMetaData* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(MapMetaData, height_)
      + sizeof(MapMetaData::height_)
      - PROTOBUF_FIELD_OFFSET(MapMetaData, map_load_time_)>(
          reinterpret_cast<char*>(&map_load_time_),
          reinterpret_cast<char*>(&other->map_load_time_));
}

::PROTOBUF_NAMESPACE_ID::Metadata MapMetaData::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto_getter, &descriptor_table_ros_2fnav_5fmsgs_2fMapMetaData_2eproto_once,
      file_level_metadata_ros_2fnav_5fmsgs_2fMapMetaData_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace nav_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::nav_msgs::MapMetaData* Arena::CreateMaybeMessage< ::ros::nav_msgs::MapMetaData >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::nav_msgs::MapMetaData >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
