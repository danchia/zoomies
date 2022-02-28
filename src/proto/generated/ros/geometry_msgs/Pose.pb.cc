// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/Pose.proto

#include "ros/geometry_msgs/Pose.pb.h"

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
constexpr Pose::Pose(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : position_(nullptr)
  , orientation_(nullptr){}
struct PoseDefaultTypeInternal {
  constexpr PoseDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~PoseDefaultTypeInternal() {}
  union {
    Pose _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PoseDefaultTypeInternal _Pose_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fgeometry_5fmsgs_2fPose_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fPose_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fPose_2eproto = nullptr;

const uint32_t TableStruct_ros_2fgeometry_5fmsgs_2fPose_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::Pose, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::Pose, position_),
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::Pose, orientation_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::geometry_msgs::Pose)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::geometry_msgs::_Pose_default_instance_),
};

const char descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fPose_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\034ros/geometry_msgs/Pose.proto\022\021ros.geom"
  "etry_msgs\032\035ros/geometry_msgs/Point.proto"
  "\032\"ros/geometry_msgs/Quaternion.proto\"f\n\004"
  "Pose\022*\n\010position\030\001 \001(\0132\030.ros.geometry_ms"
  "gs.Point\0222\n\013orientation\030\002 \001(\0132\035.ros.geom"
  "etry_msgs.Quaternionb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto_deps[2] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fPoint_2eproto,
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fQuaternion_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto = {
  false, false, 228, descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fPose_2eproto, "ros/geometry_msgs/Pose.proto", 
  &descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto_once, descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fgeometry_5fmsgs_2fPose_2eproto::offsets,
  file_level_metadata_ros_2fgeometry_5fmsgs_2fPose_2eproto, file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fPose_2eproto, file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fPose_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto_getter() {
  return &descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fgeometry_5fmsgs_2fPose_2eproto(&descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto);
namespace ros {
namespace geometry_msgs {

// ===================================================================

class Pose::_Internal {
 public:
  static const ::ros::geometry_msgs::Point& position(const Pose* msg);
  static const ::ros::geometry_msgs::Quaternion& orientation(const Pose* msg);
};

const ::ros::geometry_msgs::Point&
Pose::_Internal::position(const Pose* msg) {
  return *msg->position_;
}
const ::ros::geometry_msgs::Quaternion&
Pose::_Internal::orientation(const Pose* msg) {
  return *msg->orientation_;
}
void Pose::clear_position() {
  if (GetArenaForAllocation() == nullptr && position_ != nullptr) {
    delete position_;
  }
  position_ = nullptr;
}
void Pose::clear_orientation() {
  if (GetArenaForAllocation() == nullptr && orientation_ != nullptr) {
    delete orientation_;
  }
  orientation_ = nullptr;
}
Pose::Pose(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.geometry_msgs.Pose)
}
Pose::Pose(const Pose& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_position()) {
    position_ = new ::ros::geometry_msgs::Point(*from.position_);
  } else {
    position_ = nullptr;
  }
  if (from._internal_has_orientation()) {
    orientation_ = new ::ros::geometry_msgs::Quaternion(*from.orientation_);
  } else {
    orientation_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.geometry_msgs.Pose)
}

inline void Pose::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&position_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&orientation_) -
    reinterpret_cast<char*>(&position_)) + sizeof(orientation_));
}

Pose::~Pose() {
  // @@protoc_insertion_point(destructor:ros.geometry_msgs.Pose)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Pose::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete position_;
  if (this != internal_default_instance()) delete orientation_;
}

void Pose::ArenaDtor(void* object) {
  Pose* _this = reinterpret_cast< Pose* >(object);
  (void)_this;
}
void Pose::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Pose::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Pose::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.geometry_msgs.Pose)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && position_ != nullptr) {
    delete position_;
  }
  position_ = nullptr;
  if (GetArenaForAllocation() == nullptr && orientation_ != nullptr) {
    delete orientation_;
  }
  orientation_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Pose::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .ros.geometry_msgs.Point position = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_position(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .ros.geometry_msgs.Quaternion orientation = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_orientation(), ptr);
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

uint8_t* Pose::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.geometry_msgs.Pose)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.geometry_msgs.Point position = 1;
  if (this->_internal_has_position()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::position(this), target, stream);
  }

  // .ros.geometry_msgs.Quaternion orientation = 2;
  if (this->_internal_has_orientation()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::orientation(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.geometry_msgs.Pose)
  return target;
}

size_t Pose::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.geometry_msgs.Pose)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .ros.geometry_msgs.Point position = 1;
  if (this->_internal_has_position()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *position_);
  }

  // .ros.geometry_msgs.Quaternion orientation = 2;
  if (this->_internal_has_orientation()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *orientation_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Pose::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Pose::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Pose::GetClassData() const { return &_class_data_; }

void Pose::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Pose *>(to)->MergeFrom(
      static_cast<const Pose &>(from));
}


void Pose::MergeFrom(const Pose& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.geometry_msgs.Pose)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_position()) {
    _internal_mutable_position()->::ros::geometry_msgs::Point::MergeFrom(from._internal_position());
  }
  if (from._internal_has_orientation()) {
    _internal_mutable_orientation()->::ros::geometry_msgs::Quaternion::MergeFrom(from._internal_orientation());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Pose::CopyFrom(const Pose& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.geometry_msgs.Pose)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Pose::IsInitialized() const {
  return true;
}

void Pose::InternalSwap(Pose* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Pose, orientation_)
      + sizeof(Pose::orientation_)
      - PROTOBUF_FIELD_OFFSET(Pose, position_)>(
          reinterpret_cast<char*>(&position_),
          reinterpret_cast<char*>(&other->position_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Pose::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto_getter, &descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto_once,
      file_level_metadata_ros_2fgeometry_5fmsgs_2fPose_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::geometry_msgs::Pose* Arena::CreateMaybeMessage< ::ros::geometry_msgs::Pose >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::geometry_msgs::Pose >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>