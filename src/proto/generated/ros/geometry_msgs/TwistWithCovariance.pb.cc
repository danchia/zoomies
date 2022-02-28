// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/TwistWithCovariance.proto

#include "ros/geometry_msgs/TwistWithCovariance.pb.h"

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
constexpr TwistWithCovariance::TwistWithCovariance(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : covariance_()
  , twist_(nullptr){}
struct TwistWithCovarianceDefaultTypeInternal {
  constexpr TwistWithCovarianceDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~TwistWithCovarianceDefaultTypeInternal() {}
  union {
    TwistWithCovariance _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT TwistWithCovarianceDefaultTypeInternal _TwistWithCovariance_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto = nullptr;

const uint32_t TableStruct_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::TwistWithCovariance, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::TwistWithCovariance, twist_),
  PROTOBUF_FIELD_OFFSET(::ros::geometry_msgs::TwistWithCovariance, covariance_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::geometry_msgs::TwistWithCovariance)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::geometry_msgs::_TwistWithCovariance_default_instance_),
};

const char descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n+ros/geometry_msgs/TwistWithCovariance."
  "proto\022\021ros.geometry_msgs\032\035ros/geometry_m"
  "sgs/Twist.proto\"R\n\023TwistWithCovariance\022\'"
  "\n\005twist\030\001 \001(\0132\030.ros.geometry_msgs.Twist\022"
  "\022\n\ncovariance\030\002 \003(\001b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto_deps[1] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fTwist_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto = {
  false, false, 187, descriptor_table_protodef_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto, "ros/geometry_msgs/TwistWithCovariance.proto", 
  &descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto_once, descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto::offsets,
  file_level_metadata_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto, file_level_enum_descriptors_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto, file_level_service_descriptors_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto_getter() {
  return &descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto(&descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto);
namespace ros {
namespace geometry_msgs {

// ===================================================================

class TwistWithCovariance::_Internal {
 public:
  static const ::ros::geometry_msgs::Twist& twist(const TwistWithCovariance* msg);
};

const ::ros::geometry_msgs::Twist&
TwistWithCovariance::_Internal::twist(const TwistWithCovariance* msg) {
  return *msg->twist_;
}
void TwistWithCovariance::clear_twist() {
  if (GetArenaForAllocation() == nullptr && twist_ != nullptr) {
    delete twist_;
  }
  twist_ = nullptr;
}
TwistWithCovariance::TwistWithCovariance(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  covariance_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.geometry_msgs.TwistWithCovariance)
}
TwistWithCovariance::TwistWithCovariance(const TwistWithCovariance& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      covariance_(from.covariance_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_twist()) {
    twist_ = new ::ros::geometry_msgs::Twist(*from.twist_);
  } else {
    twist_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.geometry_msgs.TwistWithCovariance)
}

inline void TwistWithCovariance::SharedCtor() {
twist_ = nullptr;
}

TwistWithCovariance::~TwistWithCovariance() {
  // @@protoc_insertion_point(destructor:ros.geometry_msgs.TwistWithCovariance)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void TwistWithCovariance::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete twist_;
}

void TwistWithCovariance::ArenaDtor(void* object) {
  TwistWithCovariance* _this = reinterpret_cast< TwistWithCovariance* >(object);
  (void)_this;
}
void TwistWithCovariance::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void TwistWithCovariance::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void TwistWithCovariance::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.geometry_msgs.TwistWithCovariance)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  covariance_.Clear();
  if (GetArenaForAllocation() == nullptr && twist_ != nullptr) {
    delete twist_;
  }
  twist_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* TwistWithCovariance::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .ros.geometry_msgs.Twist twist = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_twist(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated double covariance = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_covariance(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 17) {
          _internal_add_covariance(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
          ptr += sizeof(double);
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

uint8_t* TwistWithCovariance::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.geometry_msgs.TwistWithCovariance)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.geometry_msgs.Twist twist = 1;
  if (this->_internal_has_twist()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::twist(this), target, stream);
  }

  // repeated double covariance = 2;
  if (this->_internal_covariance_size() > 0) {
    target = stream->WriteFixedPacked(2, _internal_covariance(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.geometry_msgs.TwistWithCovariance)
  return target;
}

size_t TwistWithCovariance::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.geometry_msgs.TwistWithCovariance)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double covariance = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_covariance_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<int32_t>(data_size));
    }
    total_size += data_size;
  }

  // .ros.geometry_msgs.Twist twist = 1;
  if (this->_internal_has_twist()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *twist_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData TwistWithCovariance::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    TwistWithCovariance::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*TwistWithCovariance::GetClassData() const { return &_class_data_; }

void TwistWithCovariance::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<TwistWithCovariance *>(to)->MergeFrom(
      static_cast<const TwistWithCovariance &>(from));
}


void TwistWithCovariance::MergeFrom(const TwistWithCovariance& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.geometry_msgs.TwistWithCovariance)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  covariance_.MergeFrom(from.covariance_);
  if (from._internal_has_twist()) {
    _internal_mutable_twist()->::ros::geometry_msgs::Twist::MergeFrom(from._internal_twist());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void TwistWithCovariance::CopyFrom(const TwistWithCovariance& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.geometry_msgs.TwistWithCovariance)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TwistWithCovariance::IsInitialized() const {
  return true;
}

void TwistWithCovariance::InternalSwap(TwistWithCovariance* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  covariance_.InternalSwap(&other->covariance_);
  swap(twist_, other->twist_);
}

::PROTOBUF_NAMESPACE_ID::Metadata TwistWithCovariance::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto_getter, &descriptor_table_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto_once,
      file_level_metadata_ros_2fgeometry_5fmsgs_2fTwistWithCovariance_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::geometry_msgs::TwistWithCovariance* Arena::CreateMaybeMessage< ::ros::geometry_msgs::TwistWithCovariance >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::geometry_msgs::TwistWithCovariance >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>