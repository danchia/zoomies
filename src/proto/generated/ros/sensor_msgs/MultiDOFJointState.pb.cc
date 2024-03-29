// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/MultiDOFJointState.proto

#include "ros/sensor_msgs/MultiDOFJointState.pb.h"

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
namespace sensor_msgs {
constexpr MultiDOFJointState::MultiDOFJointState(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : joint_names_()
  , transforms_()
  , twist_()
  , wrench_()
  , header_(nullptr){}
struct MultiDOFJointStateDefaultTypeInternal {
  constexpr MultiDOFJointStateDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~MultiDOFJointStateDefaultTypeInternal() {}
  union {
    MultiDOFJointState _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT MultiDOFJointStateDefaultTypeInternal _MultiDOFJointState_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto = nullptr;

const uint32_t TableStruct_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::MultiDOFJointState, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::MultiDOFJointState, header_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::MultiDOFJointState, joint_names_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::MultiDOFJointState, transforms_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::MultiDOFJointState, twist_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::MultiDOFJointState, wrench_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::sensor_msgs::MultiDOFJointState)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::sensor_msgs::_MultiDOFJointState_default_instance_),
};

const char descriptor_table_protodef_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n(ros/sensor_msgs/MultiDOFJointState.pro"
  "to\022\017ros.sensor_msgs\032!ros/geometry_msgs/T"
  "ransform.proto\032\035ros/geometry_msgs/Twist."
  "proto\032\036ros/geometry_msgs/Wrench.proto\032\031r"
  "os/std_msgs/Header.proto\"\325\001\n\022MultiDOFJoi"
  "ntState\022$\n\006header\030\001 \001(\0132\024.ros.std_msgs.H"
  "eader\022\023\n\013joint_names\030\002 \003(\t\0220\n\ntransforms"
  "\030\003 \003(\0132\034.ros.geometry_msgs.Transform\022\'\n\005"
  "twist\030\004 \003(\0132\030.ros.geometry_msgs.Twist\022)\n"
  "\006wrench\030\005 \003(\0132\031.ros.geometry_msgs.Wrench"
  "b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto_deps[4] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fTransform_2eproto,
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fTwist_2eproto,
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fWrench_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto = {
  false, false, 408, descriptor_table_protodef_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto, "ros/sensor_msgs/MultiDOFJointState.proto", 
  &descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto_once, descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto_deps, 4, 1,
  schemas, file_default_instances, TableStruct_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto::offsets,
  file_level_metadata_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto, file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto, file_level_service_descriptors_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto_getter() {
  return &descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto(&descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto);
namespace ros {
namespace sensor_msgs {

// ===================================================================

class MultiDOFJointState::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const MultiDOFJointState* msg);
};

const ::ros::std_msgs::Header&
MultiDOFJointState::_Internal::header(const MultiDOFJointState* msg) {
  return *msg->header_;
}
void MultiDOFJointState::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void MultiDOFJointState::clear_transforms() {
  transforms_.Clear();
}
void MultiDOFJointState::clear_twist() {
  twist_.Clear();
}
void MultiDOFJointState::clear_wrench() {
  wrench_.Clear();
}
MultiDOFJointState::MultiDOFJointState(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  joint_names_(arena),
  transforms_(arena),
  twist_(arena),
  wrench_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.sensor_msgs.MultiDOFJointState)
}
MultiDOFJointState::MultiDOFJointState(const MultiDOFJointState& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      joint_names_(from.joint_names_),
      transforms_(from.transforms_),
      twist_(from.twist_),
      wrench_(from.wrench_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.sensor_msgs.MultiDOFJointState)
}

inline void MultiDOFJointState::SharedCtor() {
header_ = nullptr;
}

MultiDOFJointState::~MultiDOFJointState() {
  // @@protoc_insertion_point(destructor:ros.sensor_msgs.MultiDOFJointState)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void MultiDOFJointState::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
}

void MultiDOFJointState::ArenaDtor(void* object) {
  MultiDOFJointState* _this = reinterpret_cast< MultiDOFJointState* >(object);
  (void)_this;
}
void MultiDOFJointState::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void MultiDOFJointState::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void MultiDOFJointState::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.sensor_msgs.MultiDOFJointState)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  joint_names_.Clear();
  transforms_.Clear();
  twist_.Clear();
  wrench_.Clear();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* MultiDOFJointState::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // repeated string joint_names = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            auto str = _internal_add_joint_names();
            ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
            CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "ros.sensor_msgs.MultiDOFJointState.joint_names"));
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .ros.geometry_msgs.Transform transforms = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_transforms(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<26>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .ros.geometry_msgs.Twist twist = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 34)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_twist(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<34>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .ros.geometry_msgs.Wrench wrench = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 42)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_wrench(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<42>(ptr));
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

uint8_t* MultiDOFJointState::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.sensor_msgs.MultiDOFJointState)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // repeated string joint_names = 2;
  for (int i = 0, n = this->_internal_joint_names_size(); i < n; i++) {
    const auto& s = this->_internal_joint_names(i);
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      s.data(), static_cast<int>(s.length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "ros.sensor_msgs.MultiDOFJointState.joint_names");
    target = stream->WriteString(2, s, target);
  }

  // repeated .ros.geometry_msgs.Transform transforms = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_transforms_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, this->_internal_transforms(i), target, stream);
  }

  // repeated .ros.geometry_msgs.Twist twist = 4;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_twist_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(4, this->_internal_twist(i), target, stream);
  }

  // repeated .ros.geometry_msgs.Wrench wrench = 5;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_wrench_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(5, this->_internal_wrench(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.sensor_msgs.MultiDOFJointState)
  return target;
}

size_t MultiDOFJointState::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.sensor_msgs.MultiDOFJointState)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated string joint_names = 2;
  total_size += 1 *
      ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(joint_names_.size());
  for (int i = 0, n = joint_names_.size(); i < n; i++) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
      joint_names_.Get(i));
  }

  // repeated .ros.geometry_msgs.Transform transforms = 3;
  total_size += 1UL * this->_internal_transforms_size();
  for (const auto& msg : this->transforms_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .ros.geometry_msgs.Twist twist = 4;
  total_size += 1UL * this->_internal_twist_size();
  for (const auto& msg : this->twist_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .ros.geometry_msgs.Wrench wrench = 5;
  total_size += 1UL * this->_internal_wrench_size();
  for (const auto& msg : this->wrench_) {
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

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData MultiDOFJointState::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    MultiDOFJointState::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*MultiDOFJointState::GetClassData() const { return &_class_data_; }

void MultiDOFJointState::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<MultiDOFJointState *>(to)->MergeFrom(
      static_cast<const MultiDOFJointState &>(from));
}


void MultiDOFJointState::MergeFrom(const MultiDOFJointState& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.sensor_msgs.MultiDOFJointState)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  joint_names_.MergeFrom(from.joint_names_);
  transforms_.MergeFrom(from.transforms_);
  twist_.MergeFrom(from.twist_);
  wrench_.MergeFrom(from.wrench_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void MultiDOFJointState::CopyFrom(const MultiDOFJointState& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.sensor_msgs.MultiDOFJointState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MultiDOFJointState::IsInitialized() const {
  return true;
}

void MultiDOFJointState::InternalSwap(MultiDOFJointState* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  joint_names_.InternalSwap(&other->joint_names_);
  transforms_.InternalSwap(&other->transforms_);
  twist_.InternalSwap(&other->twist_);
  wrench_.InternalSwap(&other->wrench_);
  swap(header_, other->header_);
}

::PROTOBUF_NAMESPACE_ID::Metadata MultiDOFJointState::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto_getter, &descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto_once,
      file_level_metadata_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::sensor_msgs::MultiDOFJointState* Arena::CreateMaybeMessage< ::ros::sensor_msgs::MultiDOFJointState >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::sensor_msgs::MultiDOFJointState >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
