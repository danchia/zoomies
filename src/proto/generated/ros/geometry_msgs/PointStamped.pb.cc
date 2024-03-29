// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/PointStamped.proto

#include "ros/geometry_msgs/PointStamped.pb.h"

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
namespace geometry_msgs {
constexpr PointStamped::PointStamped(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : header_(nullptr)
  , point_(nullptr){}
struct PointStampedDefaultTypeInternal {
  constexpr PointStampedDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~PointStampedDefaultTypeInternal() {}
  union {
    PointStamped _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PointStampedDefaultTypeInternal _PointStamped_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto = nullptr;

const uint32_t TableStruct_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::PointStamped, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::PointStamped, header_),
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::PointStamped, point_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::geometry_msgs::PointStamped)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::geometry_msgs::_PointStamped_default_instance_),
};

const char descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n$ros/geometry_msgs/PointStamped.proto\022\021"
  "ros.geometry_msgs\032\035ros/geometry_msgs/Poi"
  "nt.proto\032\031ros/std_msgs/Header.proto\"]\n\014P"
  "ointStamped\022$\n\006header\030\001 \001(\0132\024.ros.std_ms"
  "gs.Header\022\'\n\005point\030\002 \001(\0132\030.ros.geometry_"
  "msgs.Pointb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto_deps[2] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fPoint_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto = {
  false, false, 218, descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto, "ros/geometry_msgs/PointStamped.proto", 
  &descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto_once, descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto::offsets,
  file_level_metadata_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto, file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto, file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto_getter() {
  return &descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto(&descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto);
namespace ros {
namespace geometry_msgs {

// ===================================================================

class PointStamped::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const PointStamped* msg);
  static const ::ros::geometry_msgs::Point& point(const PointStamped* msg);
};

const ::ros::std_msgs::Header&
PointStamped::_Internal::header(const PointStamped* msg) {
  return *msg->header_;
}
const ::ros::geometry_msgs::Point&
PointStamped::_Internal::point(const PointStamped* msg) {
  return *msg->point_;
}
void PointStamped::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void PointStamped::clear_point() {
  if (GetArenaForAllocation() == nullptr && point_ != nullptr) {
    delete point_;
  }
  point_ = nullptr;
}
PointStamped::PointStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.geometry_msgs.PointStamped)
}
PointStamped::PointStamped(const PointStamped& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_point()) {
    point_ = new ::ros::geometry_msgs::Point(*from.point_);
  } else {
    point_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.geometry_msgs.PointStamped)
}

inline void PointStamped::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&point_) -
    reinterpret_cast<char*>(&header_)) + sizeof(point_));
}

PointStamped::~PointStamped() {
  // @@protoc_insertion_point(destructor:ros.geometry_msgs.PointStamped)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void PointStamped::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete point_;
}

void PointStamped::ArenaDtor(void* object) {
  PointStamped* _this = reinterpret_cast< PointStamped* >(object);
  (void)_this;
}
void PointStamped::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void PointStamped::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void PointStamped::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.geometry_msgs.PointStamped)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  if (GetArenaForAllocation() == nullptr && point_ != nullptr) {
    delete point_;
  }
  point_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* PointStamped::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // .ros.geometry_msgs.Point point = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_point(), ptr);
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

uint8_t* PointStamped::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.geometry_msgs.PointStamped)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // .ros.geometry_msgs.Point point = 2;
  if (this->_internal_has_point()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::point(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.geometry_msgs.PointStamped)
  return target;
}

size_t PointStamped::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.geometry_msgs.PointStamped)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // .ros.geometry_msgs.Point point = 2;
  if (this->_internal_has_point()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *point_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData PointStamped::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    PointStamped::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*PointStamped::GetClassData() const { return &_class_data_; }

void PointStamped::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<PointStamped *>(to)->MergeFrom(
      static_cast<const PointStamped &>(from));
}


void PointStamped::MergeFrom(const PointStamped& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.geometry_msgs.PointStamped)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  if (from._internal_has_point()) {
    _internal_mutable_point()->::ros::geometry_msgs::Point::MergeFrom(from._internal_point());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void PointStamped::CopyFrom(const PointStamped& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.geometry_msgs.PointStamped)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PointStamped::IsInitialized() const {
  return true;
}

void PointStamped::InternalSwap(PointStamped* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(PointStamped, point_)
      + sizeof(PointStamped::point_)
      - PROTOBUF_FIELD_OFFSET(PointStamped, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata PointStamped::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto_getter, &descriptor_table_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto_once,
      file_level_metadata_ros_2fgeometry_5fmsgs_2fPointStamped_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::geometry_msgs::PointStamped* Arena::CreateMaybeMessage< ::ros::geometry_msgs::PointStamped >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::geometry_msgs::PointStamped >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
