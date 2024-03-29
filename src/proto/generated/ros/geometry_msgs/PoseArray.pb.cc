// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/PoseArray.proto

#include "ros/geometry_msgs/PoseArray.pb.h"

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
constexpr PoseArray::PoseArray(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : poses_()
  , header_(nullptr){}
struct PoseArrayDefaultTypeInternal {
  constexpr PoseArrayDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~PoseArrayDefaultTypeInternal() {}
  union {
    PoseArray _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PoseArrayDefaultTypeInternal _PoseArray_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto = nullptr;

const uint32_t TableStruct_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::PoseArray, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::PoseArray, header_),
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::PoseArray, poses_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::geometry_msgs::PoseArray)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::geometry_msgs::_PoseArray_default_instance_),
};

const char descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n!ros/geometry_msgs/PoseArray.proto\022\021ros"
  ".geometry_msgs\032\034ros/geometry_msgs/Pose.p"
  "roto\032\031ros/std_msgs/Header.proto\"Y\n\tPoseA"
  "rray\022$\n\006header\030\001 \001(\0132\024.ros.std_msgs.Head"
  "er\022&\n\005poses\030\002 \003(\0132\027.ros.geometry_msgs.Po"
  "seb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto_deps[2] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto = {
  false, false, 210, descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto, "ros/geometry_msgs/PoseArray.proto", 
  &descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto_once, descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto::offsets,
  file_level_metadata_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto, file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto, file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto_getter() {
  return &descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto(&descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto);
namespace ros {
namespace geometry_msgs {

// ===================================================================

class PoseArray::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const PoseArray* msg);
};

const ::ros::std_msgs::Header&
PoseArray::_Internal::header(const PoseArray* msg) {
  return *msg->header_;
}
void PoseArray::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void PoseArray::clear_poses() {
  poses_.Clear();
}
PoseArray::PoseArray(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  poses_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.geometry_msgs.PoseArray)
}
PoseArray::PoseArray(const PoseArray& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      poses_(from.poses_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.geometry_msgs.PoseArray)
}

inline void PoseArray::SharedCtor() {
header_ = nullptr;
}

PoseArray::~PoseArray() {
  // @@protoc_insertion_point(destructor:ros.geometry_msgs.PoseArray)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void PoseArray::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
}

void PoseArray::ArenaDtor(void* object) {
  PoseArray* _this = reinterpret_cast< PoseArray* >(object);
  (void)_this;
}
void PoseArray::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void PoseArray::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void PoseArray::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.geometry_msgs.PoseArray)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  poses_.Clear();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* PoseArray::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // repeated .ros.geometry_msgs.Pose poses = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_poses(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
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

uint8_t* PoseArray::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.geometry_msgs.PoseArray)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // repeated .ros.geometry_msgs.Pose poses = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_poses_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_poses(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.geometry_msgs.PoseArray)
  return target;
}

size_t PoseArray::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.geometry_msgs.PoseArray)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .ros.geometry_msgs.Pose poses = 2;
  total_size += 1UL * this->_internal_poses_size();
  for (const auto& msg : this->poses_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData PoseArray::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    PoseArray::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*PoseArray::GetClassData() const { return &_class_data_; }

void PoseArray::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<PoseArray *>(to)->MergeFrom(
      static_cast<const PoseArray &>(from));
}


void PoseArray::MergeFrom(const PoseArray& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.geometry_msgs.PoseArray)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  poses_.MergeFrom(from.poses_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void PoseArray::CopyFrom(const PoseArray& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.geometry_msgs.PoseArray)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PoseArray::IsInitialized() const {
  return true;
}

void PoseArray::InternalSwap(PoseArray* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  poses_.InternalSwap(&other->poses_);
  swap(header_, other->header_);
}

::PROTOBUF_NAMESPACE_ID::Metadata PoseArray::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto_getter, &descriptor_table_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto_once,
      file_level_metadata_ros_2fgeometry_5fmsgs_2fPoseArray_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::geometry_msgs::PoseArray* Arena::CreateMaybeMessage< ::ros::geometry_msgs::PoseArray >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::geometry_msgs::PoseArray >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
