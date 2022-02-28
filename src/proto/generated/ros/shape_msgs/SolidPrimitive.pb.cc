// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/shape_msgs/SolidPrimitive.proto

#include "ros/shape_msgs/SolidPrimitive.pb.h"

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
namespace shape_msgs {
constexpr SolidPrimitive::SolidPrimitive(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : dimensions_()
  , type_(0){}
struct SolidPrimitiveDefaultTypeInternal {
  constexpr SolidPrimitiveDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~SolidPrimitiveDefaultTypeInternal() {}
  union {
    SolidPrimitive _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT SolidPrimitiveDefaultTypeInternal _SolidPrimitive_default_instance_;
}  // namespace shape_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto = nullptr;

const uint32_t TableStruct_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::shape_msgs::SolidPrimitive, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::shape_msgs::SolidPrimitive, type_),
  PROTOBUF_FIELD_OFFSET(::ros::shape_msgs::SolidPrimitive, dimensions_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::shape_msgs::SolidPrimitive)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::shape_msgs::_SolidPrimitive_default_instance_),
};

const char descriptor_table_protodef_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n#ros/shape_msgs/SolidPrimitive.proto\022\016r"
  "os.shape_msgs\"2\n\016SolidPrimitive\022\014\n\004type\030"
  "\001 \001(\005\022\022\n\ndimensions\030\002 \003(\001b\006proto3"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto = {
  false, false, 113, descriptor_table_protodef_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto, "ros/shape_msgs/SolidPrimitive.proto", 
  &descriptor_table_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto::offsets,
  file_level_metadata_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto, file_level_enum_descriptors_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto, file_level_service_descriptors_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto_getter() {
  return &descriptor_table_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto(&descriptor_table_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto);
namespace ros {
namespace shape_msgs {

// ===================================================================

class SolidPrimitive::_Internal {
 public:
};

SolidPrimitive::SolidPrimitive(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  dimensions_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.shape_msgs.SolidPrimitive)
}
SolidPrimitive::SolidPrimitive(const SolidPrimitive& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      dimensions_(from.dimensions_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  type_ = from.type_;
  // @@protoc_insertion_point(copy_constructor:ros.shape_msgs.SolidPrimitive)
}

inline void SolidPrimitive::SharedCtor() {
type_ = 0;
}

SolidPrimitive::~SolidPrimitive() {
  // @@protoc_insertion_point(destructor:ros.shape_msgs.SolidPrimitive)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void SolidPrimitive::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void SolidPrimitive::ArenaDtor(void* object) {
  SolidPrimitive* _this = reinterpret_cast< SolidPrimitive* >(object);
  (void)_this;
}
void SolidPrimitive::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SolidPrimitive::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void SolidPrimitive::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.shape_msgs.SolidPrimitive)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  dimensions_.Clear();
  type_ = 0;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SolidPrimitive::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // int32 type = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          type_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated double dimensions = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_dimensions(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 17) {
          _internal_add_dimensions(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
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

uint8_t* SolidPrimitive::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.shape_msgs.SolidPrimitive)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // int32 type = 1;
  if (this->_internal_type() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(1, this->_internal_type(), target);
  }

  // repeated double dimensions = 2;
  if (this->_internal_dimensions_size() > 0) {
    target = stream->WriteFixedPacked(2, _internal_dimensions(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.shape_msgs.SolidPrimitive)
  return target;
}

size_t SolidPrimitive::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.shape_msgs.SolidPrimitive)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double dimensions = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_dimensions_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<int32_t>(data_size));
    }
    total_size += data_size;
  }

  // int32 type = 1;
  if (this->_internal_type() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_type());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SolidPrimitive::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    SolidPrimitive::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SolidPrimitive::GetClassData() const { return &_class_data_; }

void SolidPrimitive::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<SolidPrimitive *>(to)->MergeFrom(
      static_cast<const SolidPrimitive &>(from));
}


void SolidPrimitive::MergeFrom(const SolidPrimitive& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.shape_msgs.SolidPrimitive)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  dimensions_.MergeFrom(from.dimensions_);
  if (from._internal_type() != 0) {
    _internal_set_type(from._internal_type());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void SolidPrimitive::CopyFrom(const SolidPrimitive& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.shape_msgs.SolidPrimitive)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SolidPrimitive::IsInitialized() const {
  return true;
}

void SolidPrimitive::InternalSwap(SolidPrimitive* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  dimensions_.InternalSwap(&other->dimensions_);
  swap(type_, other->type_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SolidPrimitive::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto_getter, &descriptor_table_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto_once,
      file_level_metadata_ros_2fshape_5fmsgs_2fSolidPrimitive_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace shape_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::shape_msgs::SolidPrimitive* Arena::CreateMaybeMessage< ::ros::shape_msgs::SolidPrimitive >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::shape_msgs::SolidPrimitive >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>