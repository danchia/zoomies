// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/std_msgs/Bool.proto

#include "ros/std_msgs/Bool.pb.h"

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
namespace std_msgs {
constexpr Bool::Bool(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : data_(false){}
struct BoolDefaultTypeInternal {
  constexpr BoolDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~BoolDefaultTypeInternal() {}
  union {
    Bool _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT BoolDefaultTypeInternal _Bool_default_instance_;
}  // namespace std_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fstd_5fmsgs_2fBool_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fstd_5fmsgs_2fBool_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fstd_5fmsgs_2fBool_2eproto = nullptr;

const uint32_t TableStruct_ros_2fstd_5fmsgs_2fBool_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::std_msgs::Bool, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::std_msgs::Bool, data_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::std_msgs::Bool)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::std_msgs::_Bool_default_instance_),
};

const char descriptor_table_protodef_ros_2fstd_5fmsgs_2fBool_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\027ros/std_msgs/Bool.proto\022\014ros.std_msgs\""
  "\024\n\004Bool\022\014\n\004data\030\001 \001(\010b\006proto3"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fstd_5fmsgs_2fBool_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fstd_5fmsgs_2fBool_2eproto = {
  false, false, 69, descriptor_table_protodef_ros_2fstd_5fmsgs_2fBool_2eproto, "ros/std_msgs/Bool.proto", 
  &descriptor_table_ros_2fstd_5fmsgs_2fBool_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_ros_2fstd_5fmsgs_2fBool_2eproto::offsets,
  file_level_metadata_ros_2fstd_5fmsgs_2fBool_2eproto, file_level_enum_descriptors_ros_2fstd_5fmsgs_2fBool_2eproto, file_level_service_descriptors_ros_2fstd_5fmsgs_2fBool_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fstd_5fmsgs_2fBool_2eproto_getter() {
  return &descriptor_table_ros_2fstd_5fmsgs_2fBool_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fstd_5fmsgs_2fBool_2eproto(&descriptor_table_ros_2fstd_5fmsgs_2fBool_2eproto);
namespace ros {
namespace std_msgs {

// ===================================================================

class Bool::_Internal {
 public:
};

Bool::Bool(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.std_msgs.Bool)
}
Bool::Bool(const Bool& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  data_ = from.data_;
  // @@protoc_insertion_point(copy_constructor:ros.std_msgs.Bool)
}

inline void Bool::SharedCtor() {
data_ = false;
}

Bool::~Bool() {
  // @@protoc_insertion_point(destructor:ros.std_msgs.Bool)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Bool::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void Bool::ArenaDtor(void* object) {
  Bool* _this = reinterpret_cast< Bool* >(object);
  (void)_this;
}
void Bool::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Bool::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Bool::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.std_msgs.Bool)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_ = false;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Bool::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // bool data = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          data_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

uint8_t* Bool::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.std_msgs.Bool)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // bool data = 1;
  if (this->_internal_data() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(1, this->_internal_data(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.std_msgs.Bool)
  return target;
}

size_t Bool::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.std_msgs.Bool)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // bool data = 1;
  if (this->_internal_data() != 0) {
    total_size += 1 + 1;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Bool::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Bool::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Bool::GetClassData() const { return &_class_data_; }

void Bool::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Bool *>(to)->MergeFrom(
      static_cast<const Bool &>(from));
}


void Bool::MergeFrom(const Bool& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.std_msgs.Bool)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_data() != 0) {
    _internal_set_data(from._internal_data());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Bool::CopyFrom(const Bool& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.std_msgs.Bool)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Bool::IsInitialized() const {
  return true;
}

void Bool::InternalSwap(Bool* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(data_, other->data_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Bool::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fstd_5fmsgs_2fBool_2eproto_getter, &descriptor_table_ros_2fstd_5fmsgs_2fBool_2eproto_once,
      file_level_metadata_ros_2fstd_5fmsgs_2fBool_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace std_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::std_msgs::Bool* Arena::CreateMaybeMessage< ::ros::std_msgs::Bool >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::std_msgs::Bool >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
