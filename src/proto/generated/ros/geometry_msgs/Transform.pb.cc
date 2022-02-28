// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/Transform.proto

#include "ros/geometry_msgs/Transform.pb.h"

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
constexpr Transform::Transform(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : translation_(nullptr)
  , rotation_(nullptr){}
struct TransformDefaultTypeInternal {
  constexpr TransformDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~TransformDefaultTypeInternal() {}
  union {
    Transform _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT TransformDefaultTypeInternal _Transform_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fgeometry_5fmsgs_2fTransform_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fTransform_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fTransform_2eproto = nullptr;

const uint32_t TableStruct_ros_2fgeometry_5fmsgs_2fTransform_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::Transform, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::Transform, translation_),
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::Transform, rotation_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::geometry_msgs::Transform)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::geometry_msgs::_Transform_default_instance_),
};

const char descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fTransform_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n!ros/geometry_msgs/Transform.proto\022\021ros"
  ".geometry_msgs\032\"ros/geometry_msgs/Quater"
  "nion.proto\032\037ros/geometry_msgs/Vector3.pr"
  "oto\"m\n\tTransform\022/\n\013translation\030\001 \001(\0132\032."
  "ros.geometry_msgs.Vector3\022/\n\010rotation\030\002 "
  "\001(\0132\035.ros.geometry_msgs.Quaternionb\006prot"
  "o3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto_deps[2] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fQuaternion_2eproto,
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fVector3_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto = {
  false, false, 242, descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fTransform_2eproto, "ros/geometry_msgs/Transform.proto", 
  &descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto_once, descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fgeometry_5fmsgs_2fTransform_2eproto::offsets,
  file_level_metadata_ros_2fgeometry_5fmsgs_2fTransform_2eproto, file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fTransform_2eproto, file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fTransform_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto_getter() {
  return &descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fgeometry_5fmsgs_2fTransform_2eproto(&descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto);
namespace ros {
namespace geometry_msgs {

// ===================================================================

class Transform::_Internal {
 public:
  static const ::ros::geometry_msgs::Vector3& translation(const Transform* msg);
  static const ::ros::geometry_msgs::Quaternion& rotation(const Transform* msg);
};

const ::ros::geometry_msgs::Vector3&
Transform::_Internal::translation(const Transform* msg) {
  return *msg->translation_;
}
const ::ros::geometry_msgs::Quaternion&
Transform::_Internal::rotation(const Transform* msg) {
  return *msg->rotation_;
}
void Transform::clear_translation() {
  if (GetArenaForAllocation() == nullptr && translation_ != nullptr) {
    delete translation_;
  }
  translation_ = nullptr;
}
void Transform::clear_rotation() {
  if (GetArenaForAllocation() == nullptr && rotation_ != nullptr) {
    delete rotation_;
  }
  rotation_ = nullptr;
}
Transform::Transform(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.geometry_msgs.Transform)
}
Transform::Transform(const Transform& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_translation()) {
    translation_ = new ::ros::geometry_msgs::Vector3(*from.translation_);
  } else {
    translation_ = nullptr;
  }
  if (from._internal_has_rotation()) {
    rotation_ = new ::ros::geometry_msgs::Quaternion(*from.rotation_);
  } else {
    rotation_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.geometry_msgs.Transform)
}

inline void Transform::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&translation_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&rotation_) -
    reinterpret_cast<char*>(&translation_)) + sizeof(rotation_));
}

Transform::~Transform() {
  // @@protoc_insertion_point(destructor:ros.geometry_msgs.Transform)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Transform::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete translation_;
  if (this != internal_default_instance()) delete rotation_;
}

void Transform::ArenaDtor(void* object) {
  Transform* _this = reinterpret_cast< Transform* >(object);
  (void)_this;
}
void Transform::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Transform::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Transform::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.geometry_msgs.Transform)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && translation_ != nullptr) {
    delete translation_;
  }
  translation_ = nullptr;
  if (GetArenaForAllocation() == nullptr && rotation_ != nullptr) {
    delete rotation_;
  }
  rotation_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Transform::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .ros.geometry_msgs.Vector3 translation = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_translation(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .ros.geometry_msgs.Quaternion rotation = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_rotation(), ptr);
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

uint8_t* Transform::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.geometry_msgs.Transform)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.geometry_msgs.Vector3 translation = 1;
  if (this->_internal_has_translation()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::translation(this), target, stream);
  }

  // .ros.geometry_msgs.Quaternion rotation = 2;
  if (this->_internal_has_rotation()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::rotation(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.geometry_msgs.Transform)
  return target;
}

size_t Transform::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.geometry_msgs.Transform)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .ros.geometry_msgs.Vector3 translation = 1;
  if (this->_internal_has_translation()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *translation_);
  }

  // .ros.geometry_msgs.Quaternion rotation = 2;
  if (this->_internal_has_rotation()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *rotation_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Transform::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Transform::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Transform::GetClassData() const { return &_class_data_; }

void Transform::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Transform *>(to)->MergeFrom(
      static_cast<const Transform &>(from));
}


void Transform::MergeFrom(const Transform& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.geometry_msgs.Transform)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_translation()) {
    _internal_mutable_translation()->::ros::geometry_msgs::Vector3::MergeFrom(from._internal_translation());
  }
  if (from._internal_has_rotation()) {
    _internal_mutable_rotation()->::ros::geometry_msgs::Quaternion::MergeFrom(from._internal_rotation());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Transform::CopyFrom(const Transform& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.geometry_msgs.Transform)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Transform::IsInitialized() const {
  return true;
}

void Transform::InternalSwap(Transform* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Transform, rotation_)
      + sizeof(Transform::rotation_)
      - PROTOBUF_FIELD_OFFSET(Transform, translation_)>(
          reinterpret_cast<char*>(&translation_),
          reinterpret_cast<char*>(&other->translation_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Transform::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto_getter, &descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto_once,
      file_level_metadata_ros_2fgeometry_5fmsgs_2fTransform_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::geometry_msgs::Transform* Arena::CreateMaybeMessage< ::ros::geometry_msgs::Transform >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::geometry_msgs::Transform >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>