// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/AccelStamped.proto

#include "ros/geometry_msgs/AccelStamped.pb.h"

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
constexpr AccelStamped::AccelStamped(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : header_(nullptr)
  , accel_(nullptr){}
struct AccelStampedDefaultTypeInternal {
  constexpr AccelStampedDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~AccelStampedDefaultTypeInternal() {}
  union {
    AccelStamped _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT AccelStampedDefaultTypeInternal _AccelStamped_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto = nullptr;

const uint32_t TableStruct_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::AccelStamped, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::AccelStamped, header_),
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::AccelStamped, accel_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::geometry_msgs::AccelStamped)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::geometry_msgs::_AccelStamped_default_instance_),
};

const char descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n$ros/geometry_msgs/AccelStamped.proto\022\021"
  "ros.geometry_msgs\032\035ros/geometry_msgs/Acc"
  "el.proto\032\031ros/std_msgs/Header.proto\"]\n\014A"
  "ccelStamped\022$\n\006header\030\001 \001(\0132\024.ros.std_ms"
  "gs.Header\022\'\n\005accel\030\002 \001(\0132\030.ros.geometry_"
  "msgs.Accelb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto_deps[2] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fAccel_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto = {
  false, false, 218, descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto, "ros/geometry_msgs/AccelStamped.proto", 
  &descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto_once, descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto::offsets,
  file_level_metadata_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto, file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto, file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto_getter() {
  return &descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto(&descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto);
namespace ros {
namespace geometry_msgs {

// ===================================================================

class AccelStamped::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const AccelStamped* msg);
  static const ::ros::geometry_msgs::Accel& accel(const AccelStamped* msg);
};

const ::ros::std_msgs::Header&
AccelStamped::_Internal::header(const AccelStamped* msg) {
  return *msg->header_;
}
const ::ros::geometry_msgs::Accel&
AccelStamped::_Internal::accel(const AccelStamped* msg) {
  return *msg->accel_;
}
void AccelStamped::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void AccelStamped::clear_accel() {
  if (GetArenaForAllocation() == nullptr && accel_ != nullptr) {
    delete accel_;
  }
  accel_ = nullptr;
}
AccelStamped::AccelStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.geometry_msgs.AccelStamped)
}
AccelStamped::AccelStamped(const AccelStamped& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_accel()) {
    accel_ = new ::ros::geometry_msgs::Accel(*from.accel_);
  } else {
    accel_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.geometry_msgs.AccelStamped)
}

inline void AccelStamped::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&accel_) -
    reinterpret_cast<char*>(&header_)) + sizeof(accel_));
}

AccelStamped::~AccelStamped() {
  // @@protoc_insertion_point(destructor:ros.geometry_msgs.AccelStamped)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void AccelStamped::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete accel_;
}

void AccelStamped::ArenaDtor(void* object) {
  AccelStamped* _this = reinterpret_cast< AccelStamped* >(object);
  (void)_this;
}
void AccelStamped::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void AccelStamped::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void AccelStamped::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.geometry_msgs.AccelStamped)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  if (GetArenaForAllocation() == nullptr && accel_ != nullptr) {
    delete accel_;
  }
  accel_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* AccelStamped::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // .ros.geometry_msgs.Accel accel = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_accel(), ptr);
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

uint8_t* AccelStamped::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.geometry_msgs.AccelStamped)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // .ros.geometry_msgs.Accel accel = 2;
  if (this->_internal_has_accel()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::accel(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.geometry_msgs.AccelStamped)
  return target;
}

size_t AccelStamped::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.geometry_msgs.AccelStamped)
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

  // .ros.geometry_msgs.Accel accel = 2;
  if (this->_internal_has_accel()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *accel_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData AccelStamped::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    AccelStamped::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*AccelStamped::GetClassData() const { return &_class_data_; }

void AccelStamped::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<AccelStamped *>(to)->MergeFrom(
      static_cast<const AccelStamped &>(from));
}


void AccelStamped::MergeFrom(const AccelStamped& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.geometry_msgs.AccelStamped)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  if (from._internal_has_accel()) {
    _internal_mutable_accel()->::ros::geometry_msgs::Accel::MergeFrom(from._internal_accel());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void AccelStamped::CopyFrom(const AccelStamped& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.geometry_msgs.AccelStamped)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AccelStamped::IsInitialized() const {
  return true;
}

void AccelStamped::InternalSwap(AccelStamped* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(AccelStamped, accel_)
      + sizeof(AccelStamped::accel_)
      - PROTOBUF_FIELD_OFFSET(AccelStamped, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata AccelStamped::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto_getter, &descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto_once,
      file_level_metadata_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::geometry_msgs::AccelStamped* Arena::CreateMaybeMessage< ::ros::geometry_msgs::AccelStamped >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::geometry_msgs::AccelStamped >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>